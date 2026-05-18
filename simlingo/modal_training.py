"""Modal training app for SimLingo fine-tuning on NVIDIA real-world data.

This trains SimLingo VLA on NVIDIA PhysicalAI-AV data, warm-starting from
the pretrained SimLingo checkpoint with LoRA fine-tuning.

Setup:
    1. Configure Modal secrets:
       modal secret create wandb WANDB_API_KEY=<your-key>
       modal secret create huggingface HF_TOKEN=<your-token>

    2. Prepare data (run extraction first):
       modal run modal_training.py::prepare_nvidia_data --scale tiny

    3. Run training:
       modal run modal_training.py::train --config config/nvidia_finetune.yaml

Usage:
    # Prepare tiny dataset for validation
    modal run modal_training.py::prepare_nvidia_data --scale tiny

    # Run training
    modal run modal_training.py::train

    # Run training with W&B and HF checkpointing
    modal run modal_training.py::train --wandb-project simlingo-nvidia --hf-repo your-name/simlingo-nvidia

    # Evaluate on held-out data
    modal run modal_training.py::evaluate
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import modal

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

APP_NAME = "simlingo-nvidia-training"

# Upstream SimLingo
SIMLINGO_REPO_URL = "https://github.com/RenzKa/simlingo.git"
SIMLINGO_REPO_DIR = "/opt/simlingo"

# HF model artifacts
HF_MODEL_REPO = "RenzKa/simlingo"
HF_CKPT_FILE = "simlingo/checkpoints/epoch=013.ckpt/pytorch_model.pt"
HF_HYDRA_CONFIG_FILE = "simlingo/.hydra/config.yaml"

# Volume mountpoints
CACHE_DIR = "/cache"
DATA_DIR = "/data"
NVIDIA_DATA_DIR = "/nvidia_data"
OUTPUTS_DIR = "/outputs"
CHECKPOINTS_DIR = "/checkpoints"
CKPT_DIR = f"{DATA_DIR}/checkpoint"

# ---------------------------------------------------------------------------
# Image
# ---------------------------------------------------------------------------

training_image = (
    modal.Image.from_registry(
        "nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04",
        add_python="3.11",  # Python 3.11 for physical_ai_av
    )
    .apt_install(
        "git",
        "git-lfs",
        "build-essential",
        "ninja-build",
        "libgl1",
        "libglib2.0-0",
    )
    # PyTorch with CUDA 12.1
    .pip_install(
        "torch==2.2.0",
        "torchvision==0.17.0",
        "torchaudio==2.2.0",
        index_url="https://download.pytorch.org/whl/cu121",
    )
    # Training dependencies
    .pip_install(
        "transformers==4.46.3",
        "tokenizers==0.20.3",
        "sentencepiece",
        "peft==0.13.2",
        "accelerate==1.0.1",
        "huggingface_hub==0.27.0",
        "pytorch-lightning==2.4.0",
        "lightning==2.3.3",
        "hydra-core==1.3.2",
        "hydra-zen==0.12.1",
        "omegaconf==2.3.0",
        "einops==0.7.0",
        "timm==0.9.16",
        "scipy==1.10.1",
        "scikit-image==0.21.0",
        "imgaug==0.4.0",
        "Pillow==10.2.0",
        "filterpy==1.4.5",
        "ujson==5.9.0",
        "matplotlib==3.7.5",
        "tqdm",
        "numpy<2",
        "opencv-python-headless==4.10.0.84",
        "line_profiler",
        # W&B for experiment tracking
        "wandb",
        # NVIDIA PhysicalAI-AV SDK
        "physical_ai_av",
    )
    # Flash attention (prebuilt wheel)
    .pip_install(
        "https://github.com/Dao-AILab/flash-attention/releases/download/v2.7.0.post2/flash_attn-2.7.0.post2+cu12torch2.2cxx11abiFALSE-cp311-cp311-linux_x86_64.whl"
    )
    .pip_install("deepspeed==0.16.2")
    # Clone SimLingo repo
    .run_commands(
        f"git clone --depth 1 {SIMLINGO_REPO_URL} {SIMLINGO_REPO_DIR}",
    )
    .env(
        {
            "PYTHONPATH": SIMLINGO_REPO_DIR,
            "HF_HOME": f"{CACHE_DIR}/hf",
            "HUGGINGFACE_HUB_CACHE": f"{CACHE_DIR}/hf/hub",
            "TRANSFORMERS_CACHE": f"{CACHE_DIR}/hf/transformers",
            "TRUST_REMOTE_CODE": "1",
            "TOKENIZERS_PARALLELISM": "false",
        }
    )
    .add_local_python_source("scripts")
    .add_local_dir("config", remote_path="/app/config")
)

# ---------------------------------------------------------------------------
# Volumes
# ---------------------------------------------------------------------------

cache_volume = modal.Volume.from_name("simlingo-cache", create_if_missing=True)
data_volume = modal.Volume.from_name("simlingo-data", create_if_missing=True)
nvidia_data_volume = modal.Volume.from_name("simlingo-nvidia-data", create_if_missing=True)
output_volume = modal.Volume.from_name("simlingo-outputs", create_if_missing=True)
checkpoint_volume = modal.Volume.from_name("simlingo-checkpoints", create_if_missing=True)

VOLUMES = {
    CACHE_DIR: cache_volume,
    DATA_DIR: data_volume,
    NVIDIA_DATA_DIR: nvidia_data_volume,
    OUTPUTS_DIR: output_volume,
    CHECKPOINTS_DIR: checkpoint_volume,
}

# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

app = modal.App(APP_NAME)


# ---------------------------------------------------------------------------
# Data Preparation
# ---------------------------------------------------------------------------


@app.function(
    image=training_image,
    volumes=VOLUMES,
    timeout=60 * 60 * 6,  # 6 hours for large datasets
    cpu=8,
    memory=32 * 1024,
    secrets=[modal.Secret.from_name("huggingface")],
)
def prepare_nvidia_data(
    scale: str = "tiny",
    output_subdir: str = "extracted",
    create_split: bool = True,
    val_ratio: float = 0.1,
) -> dict:
    """Download and extract NVIDIA data for training.

    Args:
        scale: Dataset scale (tiny/small/medium/large).
        output_subdir: Subdirectory within NVIDIA_DATA_DIR.
        create_split: Whether to create train/val split.
        val_ratio: Validation set ratio.

    Returns:
        Summary of extraction.
    """
    sys.path.insert(0, os.path.dirname(__file__))

    import physical_ai_av

    from scripts.extract_nvidia import (
        SCALE_CONFIGS,
        create_train_val_split,
        extract_dataset,
        sample_clip_ids_from_api,
    )

    output_dir = Path(NVIDIA_DATA_DIR) / output_subdir
    output_dir.mkdir(parents=True, exist_ok=True)

    # Get HF token from secret
    token = os.environ.get("HF_TOKEN")
    if not token:
        raise ValueError("HF_TOKEN not found in environment")

    print(f"Initializing NVIDIA PhysicalAI-AV with scale={scale}")
    avdi = physical_ai_av.PhysicalAIAVDatasetInterface(token=token)

    # Get clip IDs
    config = SCALE_CONFIGS[scale]
    print(f"Scale: {scale} - {config['description']}")

    clip_ids = sample_clip_ids_from_api(avdi, config["num_clips"])
    if not clip_ids:
        raise ValueError("No clips available from API")

    # Extract data
    summary = extract_dataset(
        avdi=avdi,
        clip_ids=clip_ids,
        output_dir=output_dir,
        frames_per_clip=config["frames_per_clip"],
    )

    # Create train/val split
    if create_split:
        create_train_val_split(output_dir, val_ratio=val_ratio)

    nvidia_data_volume.commit()

    return summary


@app.function(
    image=training_image,
    volumes=VOLUMES,
    timeout=60 * 60 * 3,
    cpu=4,
    secrets=[modal.Secret.from_name("huggingface")],
)
def prepare_base_model(force: bool = False) -> dict:
    """Download SimLingo base model and InternVL2 weights."""
    from huggingface_hub import hf_hub_download, snapshot_download

    Path(CKPT_DIR).mkdir(parents=True, exist_ok=True)

    # Download SimLingo checkpoint
    for filename in (HF_CKPT_FILE, HF_HYDRA_CONFIG_FILE):
        dst = Path(CKPT_DIR) / filename
        if dst.exists() and not force:
            print(f"  cached: {dst}")
            continue
        print(f"  downloading: {filename}")
        hf_hub_download(
            repo_id=HF_MODEL_REPO,
            filename=filename,
            local_dir=CKPT_DIR,
            repo_type="model",
        )

    # Download InternVL2-1B
    print("  downloading: OpenGVLab/InternVL2-1B")
    snapshot_download(
        repo_id="OpenGVLab/InternVL2-1B",
        local_dir=f"{CACHE_DIR}/hf/snapshots/InternVL2-1B",
        ignore_patterns=["*.bin"],
    )

    cache_volume.commit()
    data_volume.commit()

    return {"ckpt_dir": CKPT_DIR, "status": "ready"}


# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------


@app.function(
    image=training_image,
    volumes=VOLUMES,
    gpu="H100",  # Use H100 for fastest training
    timeout=60 * 60 * 24,  # 24 hours max
    cpu=8,
    memory=64 * 1024,
    secrets=[
        modal.Secret.from_name("huggingface"),
        modal.Secret.from_name("wandb"),
    ],
)
def train(
    config_path: str = "/app/config/nvidia_finetune.yaml",
    data_dir: str | None = None,
    wandb_project: str = "simlingo-nvidia-finetune",
    wandb_entity: str | None = None,
    hf_repo: str | None = None,
    resume_from: str | None = None,
    epochs: int | None = None,
    batch_size: int | None = None,
    learning_rate: float | None = None,
    lora_rank: int | None = None,
) -> dict:
    """Train SimLingo on NVIDIA data with LoRA fine-tuning.

    Args:
        config_path: Path to training config YAML.
        data_dir: Override data directory.
        wandb_project: W&B project name.
        wandb_entity: W&B entity (team/user).
        hf_repo: HuggingFace repo for checkpoint uploads (e.g., "user/simlingo-nvidia").
        resume_from: Checkpoint path to resume from.
        epochs: Override number of epochs.
        batch_size: Override batch size.
        learning_rate: Override learning rate.
        lora_rank: Override LoRA rank.

    Returns:
        Training summary with metrics.
    """
    import torch
    import wandb
    import yaml
    from omegaconf import OmegaConf

    sys.path.insert(0, SIMLINGO_REPO_DIR)
    sys.path.insert(0, os.path.dirname(__file__))

    # Load config
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Apply overrides
    if data_dir:
        config["data"]["train_dir"] = data_dir
    if epochs:
        config["training"]["epochs"] = epochs
    if batch_size:
        config["training"]["batch_size"] = batch_size
    if learning_rate:
        config["training"]["learning_rate"] = learning_rate
    if lora_rank:
        config["lora"]["rank"] = lora_rank

    # Set default data directory if not specified
    if not config["data"].get("train_dir"):
        config["data"]["train_dir"] = f"{NVIDIA_DATA_DIR}/extracted"

    print("Training config:")
    print(yaml.dump(config, default_flow_style=False))

    # Initialize W&B
    wandb.init(
        project=wandb_project,
        entity=wandb_entity,
        config=config,
        name=f"nvidia-finetune-lr{config['training']['learning_rate']}-lora{config['lora']['rank']}",
    )

    # Import training utilities
    from scripts.nvidia_trainer import NVIDIASimLingoTrainer

    # Initialize trainer
    trainer = NVIDIASimLingoTrainer(
        config=config,
        ckpt_dir=CKPT_DIR,
        output_dir=CHECKPOINTS_DIR,
        hf_repo=hf_repo,
    )

    # Resume if specified
    if resume_from:
        print(f"Resuming from: {resume_from}")
        trainer.load_checkpoint(resume_from)

    # Train
    print("Starting training...")
    metrics = trainer.train()

    # Save final checkpoint
    final_ckpt = trainer.save_checkpoint("final")
    print(f"Saved final checkpoint: {final_ckpt}")

    # Push to HuggingFace if configured
    if hf_repo:
        print(f"Pushing checkpoint to HuggingFace: {hf_repo}")
        trainer.push_to_hub(hf_repo)

    wandb.finish()
    checkpoint_volume.commit()

    return {
        "metrics": metrics,
        "checkpoint": str(final_ckpt),
        "config": config,
    }


@app.function(
    image=training_image,
    volumes=VOLUMES,
    gpu="H100:2",  # Multi-GPU for faster training
    timeout=60 * 60 * 48,  # 48 hours for full training
    cpu=16,
    memory=128 * 1024,
    secrets=[
        modal.Secret.from_name("huggingface"),
        modal.Secret.from_name("wandb"),
    ],
)
def train_multigpu(
    config_path: str = "/app/config/nvidia_finetune.yaml",
    wandb_project: str = "simlingo-nvidia-finetune",
    hf_repo: str | None = None,
) -> dict:
    """Multi-GPU training with DDP.

    Same as `train` but uses 2x H100 GPUs for faster training on larger datasets.
    """
    import subprocess

    # Use torchrun for DDP
    cmd = [
        "torchrun",
        "--nproc_per_node=2",
        "--master_port=29500",
        "-m", "scripts.nvidia_trainer",
        "--config", config_path,
        "--wandb-project", wandb_project,
        "--output-dir", CHECKPOINTS_DIR,
    ]
    if hf_repo:
        cmd.extend(["--hf-repo", hf_repo])

    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, text=True)

    if result.returncode != 0:
        print(f"Training failed: {result.stderr}")
        raise RuntimeError(result.stderr)

    checkpoint_volume.commit()

    return {"status": "completed", "output": result.stdout}


# ---------------------------------------------------------------------------
# Evaluation
# ---------------------------------------------------------------------------


@app.function(
    image=training_image,
    volumes=VOLUMES,
    gpu="L4",
    timeout=60 * 60 * 2,
    cpu=4,
    memory=24 * 1024,
    secrets=[modal.Secret.from_name("huggingface")],
)
def evaluate(
    checkpoint_path: str | None = None,
    data_dir: str | None = None,
    num_samples: int = 100,
    save_overlays: bool = True,
) -> dict:
    """Evaluate fine-tuned model on held-out data.

    Args:
        checkpoint_path: Path to fine-tuned checkpoint. If None, uses base SimLingo.
        data_dir: Path to evaluation data.
        num_samples: Number of samples to evaluate.
        save_overlays: Whether to save visualization overlays.

    Returns:
        Evaluation metrics.
    """
    sys.path.insert(0, SIMLINGO_REPO_DIR)
    sys.path.insert(0, os.path.dirname(__file__))

    from scripts import inference
    from scripts.nvidia_loader import ExternalSample

    # Set paths
    if checkpoint_path is None:
        ckpt_path = str(Path(CKPT_DIR) / HF_CKPT_FILE)
    else:
        ckpt_path = checkpoint_path

    hydra_cfg_path = str(Path(CKPT_DIR) / HF_HYDRA_CONFIG_FILE)

    if data_dir is None:
        data_dir = f"{NVIDIA_DATA_DIR}/extracted"

    out_dir = Path(OUTPUTS_DIR) / "nvidia_eval"
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load evaluation samples from extracted data
    samples = load_extracted_samples(data_dir, num_samples, split="val")

    print(f"Evaluating on {len(samples)} samples...")

    summary = inference.run_external_samples(
        ckpt_path=ckpt_path,
        hydra_cfg_path=hydra_cfg_path,
        out_dir=str(out_dir),
        samples=samples,
        use_cot=True,
        save_overlays=save_overlays,
        label="nvidia_eval",
    )

    output_volume.commit()

    print("Evaluation metrics:", summary["metrics"])
    return summary


def load_extracted_samples(
    data_dir: str,
    num_samples: int,
    split: str = "val",
) -> list:
    """Load ExternalSample objects from extracted data on disk.

    Args:
        data_dir: Path to extracted data directory.
        num_samples: Number of samples to load.
        split: Which split to use (train/val).

    Returns:
        List of ExternalSample objects.
    """
    import gzip
    import json
    import random

    from scripts.nuscenes_loader import ExternalSample

    data_dir = Path(data_dir)

    # Load split file if exists
    split_file = data_dir / "train_val_split.json"
    if split_file.exists():
        with open(split_file) as f:
            split_info = json.load(f)
        clip_ids = split_info.get(split, [])
    else:
        # Use all clips
        clip_ids = [d.name for d in data_dir.iterdir() if d.is_dir() and (d / "rgb").exists()]

    random.shuffle(clip_ids)
    samples = []

    for clip_id in clip_ids:
        if len(samples) >= num_samples:
            break

        clip_dir = data_dir / clip_id
        rgb_dir = clip_dir / "rgb"
        meas_dir = clip_dir / "measurements"

        if not rgb_dir.exists():
            continue

        frame_files = sorted(rgb_dir.glob("*.jpg"))

        for frame_file in frame_files:
            if len(samples) >= num_samples:
                break

            frame_idx = int(frame_file.stem)
            meas_file = meas_dir / f"{frame_idx:04d}.json.gz"

            if not meas_file.exists():
                continue

            with gzip.open(meas_file, "rt") as f:
                meas = json.load(f)

            samples.append(ExternalSample(
                rgb_path=frame_file,
                speed_mps=meas["speed"],
                target_points=np.array(
                    [meas["target_point"], meas["target_point_next"]],
                    dtype=np.float32,
                ),
                intrinsics=np.array(meas["intrinsics"], dtype=np.float32) if "intrinsics" in meas else None,
                fov_deg=meas.get("fov_deg", 120.0),
                cam_translation_xyz=(0.0, 1.5, 2.0),  # Default for NVIDIA
                crop_bottom=False,
                gt_wps=np.array(meas["waypoints"], dtype=np.float32),
                gt_route=np.array(meas["route"], dtype=np.float32),
                gt_commentary=None,
                meta={
                    "clip_id": clip_id,
                    "frame_idx": frame_idx,
                },
            ))

    return samples


# Import numpy for load_extracted_samples
import numpy as np


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------


@app.function(
    image=training_image,
    volumes=VOLUMES,
    gpu="L4",
    timeout=60 * 60,
    cpu=4,
    memory=24 * 1024,
    secrets=[modal.Secret.from_name("huggingface")],
)
def visualize_nvidia_clip(
    clip_id: str,
    num_frames: int = 20,
    make_video: bool = True,
    fps: int = 10,
) -> dict:
    """Generate visualization of a single NVIDIA clip with waypoints.

    This is useful for validating the data pipeline before training.

    Args:
        clip_id: NVIDIA clip ID to visualize.
        num_frames: Number of frames to visualize.
        make_video: Whether to create MP4 from frames.
        fps: Video frame rate.

    Returns:
        Path to output files.
    """
    sys.path.insert(0, os.path.dirname(__file__))

    import physical_ai_av

    from scripts.nvidia_viz import (
        create_visualization_video,
        visualize_clip_waypoints,
    )

    token = os.environ.get("HF_TOKEN")
    if not token:
        raise ValueError("HF_TOKEN not found")

    avdi = physical_ai_av.PhysicalAIAVDatasetInterface(token=token)

    output_dir = Path(OUTPUTS_DIR) / "nvidia_viz" / clip_id
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Visualizing clip: {clip_id}")
    visualize_clip_waypoints(
        avdi=avdi,
        clip_id=clip_id,
        output_dir=output_dir,
        num_frames=num_frames,
    )

    if make_video:
        video_path = output_dir / "visualization.mp4"
        create_visualization_video(output_dir, video_path, fps=fps)
        print(f"Created video: {video_path}")

    output_volume.commit()

    return {
        "output_dir": str(output_dir),
        "num_frames": num_frames,
        "clip_id": clip_id,
    }


# ---------------------------------------------------------------------------
# Test / Debug Functions
# ---------------------------------------------------------------------------


@app.function(
    image=training_image,
    volumes=VOLUMES,
    timeout=60 * 30,  # 30 minutes
    cpu=4,
    memory=16 * 1024,
    secrets=[modal.Secret.from_name("huggingface")],
)
def test_nvidia_connection() -> dict:
    """Test connection to NVIDIA PhysicalAI-AV API and list available clips.

    This is a quick sanity check to verify HF token and API access.

    Returns:
        dict with connection status and sample clip IDs.
    """
    import physical_ai_av

    token = os.environ.get("HF_TOKEN")
    if not token:
        return {"status": "error", "message": "HF_TOKEN not found"}

    print("Initializing NVIDIA PhysicalAI-AV interface...")
    try:
        avdi = physical_ai_av.PhysicalAIAVDatasetInterface(token=token)
        print("Connection successful!")

        # Try to list clips
        print("Listing available clips...")
        # The actual API may vary - this is a best guess
        try:
            clips = avdi.list_clips()
            clip_count = len(clips) if clips else 0
            sample_clips = clips[:5] if clips else []
        except AttributeError:
            # Try alternative methods
            try:
                # Some APIs use different methods
                clips = list(avdi.clips())[:100]
                clip_count = len(clips)
                sample_clips = clips[:5]
            except Exception:
                clip_count = "unknown"
                sample_clips = []

        return {
            "status": "success",
            "clip_count": clip_count,
            "sample_clips": sample_clips,
            "message": "API connection successful",
        }

    except Exception as e:
        return {
            "status": "error",
            "message": str(e),
        }


@app.function(
    image=training_image,
    volumes=VOLUMES,
    timeout=60 * 30,
    cpu=4,
    memory=24 * 1024,
    secrets=[modal.Secret.from_name("huggingface")],
)
def test_waypoint_visualization(
    clip_id: str | None = None,
    num_frames: int = 10,
) -> dict:
    """Quick test: extract a few frames from one clip and visualize waypoints.

    If no clip_id is provided, will try to get the first available clip.

    Args:
        clip_id: Specific clip ID to test, or None to auto-select.
        num_frames: Number of frames to visualize.

    Returns:
        dict with output paths and status.
    """
    import physical_ai_av
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont

    token = os.environ.get("HF_TOKEN")
    if not token:
        return {"status": "error", "message": "HF_TOKEN not found"}

    print("Initializing NVIDIA PhysicalAI-AV interface...")
    avdi = physical_ai_av.PhysicalAIAVDatasetInterface(token=token)

    # Get a clip ID if not provided
    if clip_id is None:
        print("No clip_id provided, fetching first available clip...")
        try:
            clips = avdi.list_clips()
            if not clips:
                clips = list(avdi.clips())[:1]
            clip_id = clips[0] if clips else None
        except Exception as e:
            return {"status": "error", "message": f"Could not list clips: {e}"}

    if clip_id is None:
        return {"status": "error", "message": "No clips available"}

    print(f"Using clip: {clip_id}")

    # Create output directory
    output_dir = Path(OUTPUTS_DIR) / "waypoint_test" / clip_id[:20]
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        # Load clip data
        print("Loading video data...")
        camera_feature = avdi.features.CAMERA.CAMERA_FRONT_WIDE_120FOV
        video = avdi.get_clip_feature(clip_id, camera_feature)

        print("Loading egomotion data...")
        egomotion = avdi.get_clip_feature(clip_id, avdi.features.LABELS.EGOMOTION)

        print("Loading calibration...")
        intrinsics_data = avdi.get_clip_feature(
            clip_id, avdi.features.CALIBRATION.CAMERA_INTRINSICS
        )
        K = intrinsics_data.get_camera_matrix("camera_front_wide_120fov")

        # Create interpolator
        interpolator = physical_ai_av.utils.interpolation.Interpolator([egomotion])

        # Sample timestamps
        max_ts_us = 17_000_000  # 17 seconds
        timestamps_us = np.linspace(0, max_ts_us, num_frames).astype(int)

        print(f"Decoding {num_frames} frames...")
        frames, actual_ts = video.decode_images_from_timestamps(timestamps_us)

        print("Generating visualizations with waypoints...")
        saved_frames = []

        for i, (frame, ts_us) in enumerate(zip(frames, actual_ts)):
            # Get current pose
            current_state = interpolator(ts_us)
            current_pose = current_state.pose

            # Compute waypoints (11 points, 0.25s spacing)
            waypoints = []
            for j in range(1, 12):
                future_us = ts_us + int(j * 0.25 * 1_000_000)
                try:
                    future_state = interpolator(future_us)
                    relative_pos = current_pose.inverse().apply(
                        future_state.pose.translation
                    )
                    waypoints.append([relative_pos[0], relative_pos[1]])
                except Exception:
                    if waypoints:
                        waypoints.append(waypoints[-1])
                    else:
                        waypoints.append([0.0, 0.0])

            waypoints = np.array(waypoints, dtype=np.float32)

            # Get speed
            try:
                velocity = current_state.velocity
                speed_mps = float(np.linalg.norm(velocity[:2]))
            except Exception:
                speed_mps = 0.0

            # Create visualization
            h, w = frame.shape[:2]
            pil_img = Image.fromarray(frame)
            draw = ImageDraw.Draw(pil_img)

            # Project waypoints to image
            # Simple pinhole projection
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            cam_height = 1.5  # Approximate camera height

            pixels = []
            for x_fwd, y_left in waypoints:
                if x_fwd > 0.1:  # In front of camera
                    # Project to image
                    u = cx - (y_left * fx) / x_fwd
                    v = cy + (cam_height * fy) / x_fwd
                    if 0 <= u < w and 0 <= v < h:
                        pixels.append((int(u), int(v)))

            # Draw waypoints with color gradient
            for j, (u, v) in enumerate(pixels):
                progress = j / max(len(waypoints) - 1, 1)
                r = int(255 * progress)
                g = int(255 * (1 - progress))
                radius = 8
                draw.ellipse(
                    (u - radius, v - radius, u + radius, v + radius),
                    fill=(r, g, 0),
                    outline=(255, 255, 255),
                    width=2,
                )
                # Connect with lines
                if j > 0:
                    prev_u, prev_v = pixels[j - 1]
                    draw.line([(prev_u, prev_v), (u, v)], fill=(r, g, 0), width=3)

            # Add info overlay
            try:
                font = ImageFont.truetype(
                    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20
                )
            except Exception:
                font = ImageFont.load_default()

            # Semi-transparent background
            overlay = Image.new("RGBA", (w, 100), (0, 0, 0, 200))
            pil_img.paste(overlay, (0, 0), overlay)

            info_text = (
                f"Clip: {clip_id[:30]}...\n"
                f"Frame {i+1}/{num_frames} | Time: {ts_us/1e6:.2f}s | "
                f"Speed: {speed_mps:.1f} m/s ({speed_mps*2.237:.1f} mph)\n"
                f"Waypoints: {len(pixels)}/11 visible | "
                f"Green=0.25s ahead, Red=2.75s ahead"
            )
            draw.text((10, 10), info_text, fill=(255, 255, 255), font=font)

            # Save frame
            frame_path = output_dir / f"frame_{i:04d}.png"
            pil_img.save(frame_path)
            saved_frames.append(str(frame_path))
            print(f"  Saved frame {i+1}: speed={speed_mps:.1f} m/s, "
                  f"waypoints visible={len(pixels)}/11")

        # Create video
        import cv2

        video_path = output_dir / "waypoint_test.mp4"
        first = cv2.imread(saved_frames[0])
        h, w = first.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(video_path), fourcc, 2.0, (w, h))
        for fp in saved_frames:
            writer.write(cv2.imread(fp))
        writer.release()

        print(f"\nVisualization complete!")
        print(f"Output directory: {output_dir}")
        print(f"Video: {video_path}")

        output_volume.commit()

        return {
            "status": "success",
            "clip_id": clip_id,
            "output_dir": str(output_dir),
            "video_path": str(video_path),
            "num_frames": len(saved_frames),
            "message": "Waypoint visualization created successfully",
        }

    except Exception as e:
        import traceback
        return {
            "status": "error",
            "clip_id": clip_id,
            "message": str(e),
            "traceback": traceback.format_exc(),
        }


# ---------------------------------------------------------------------------
# Local entrypoint
# ---------------------------------------------------------------------------


@app.local_entrypoint()
def main(
    action: str = "prepare",
    scale: str = "tiny",
    config: str = "/app/config/nvidia_finetune.yaml",
    wandb_project: str = "simlingo-nvidia-finetune",
):
    """Main entrypoint for training pipeline.

    Actions:
        prepare: Download base model and prepare data
        train: Run training
        evaluate: Evaluate model

    Examples:
        modal run modal_training.py --action prepare --scale tiny
        modal run modal_training.py --action train
        modal run modal_training.py --action evaluate
    """
    if action == "prepare":
        print("Preparing base model...")
        prepare_base_model.remote()
        print("Preparing NVIDIA data...")
        prepare_nvidia_data.remote(scale=scale)
    elif action == "train":
        train.remote(config_path=config, wandb_project=wandb_project)
    elif action == "evaluate":
        evaluate.remote()
    else:
        print(f"Unknown action: {action}")
