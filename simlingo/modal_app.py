"""Modal app for offline SimLingo inference + sanity-check eval.

Run order:
    modal run modal_app.py::prepare_assets    # one-time download + extract
    modal run modal_app.py::run --num-samples 64

See README.md for the design.
"""

from __future__ import annotations

import os
import shlex
import subprocess
import sys
import tarfile
from pathlib import Path

import modal

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

APP_NAME = "simlingo-sandbox"

# Upstream repo + release artifact names.
SIMLINGO_REPO_URL = "https://github.com/RenzKa/simlingo.git"
SIMLINGO_REPO_DIR = "/opt/simlingo"

HF_MODEL_REPO = "RenzKa/simlingo"
HF_CKPT_FILE = "simlingo/checkpoints/epoch=013.ckpt/pytorch_model.pt"
HF_HYDRA_CONFIG_FILE = "simlingo/.hydra/config.yaml"

HF_DATA_REPO = "RenzKa/simlingo"
# A single 1-scenario validation chunk + matching commentary annotations.
# (Each chunk is a few GB; we just need *some* held-out frames.)
HF_DATA_FILES = [
    "data_simlingo_validation_1_scenario_routes_validation_random_weather_seed_2_balanced_150_chunk_001.tar.gz",
    "commentary_simlingo_validation_1_scenario_routes_validation_random_weather_seed_2_balanced_150_chunk_001.tar.gz",
]

# Volume mountpoints.
CACHE_DIR = "/cache"
DATA_DIR = "/data"
OUTPUTS_DIR = "/outputs"
EXTRACTED_DIR = f"{DATA_DIR}/extracted"
CKPT_DIR = f"{DATA_DIR}/checkpoint"

# ---------------------------------------------------------------------------
# Image
# ---------------------------------------------------------------------------

# CUDA dev image so flash-attn can build against the runtime. Add Python 3.10
# because the upstream env pins torch 2.2 / transformers 4.46.
simlingo_image = (
    modal.Image.from_registry(
        "nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04",
        add_python="3.10",
    )
    .apt_install(
        "git",
        "git-lfs",
        "build-essential",
        "ninja-build",
        "libgl1",
        "libglib2.0-0",
    )
    # torch first so flash-attn can link against it later.
    .pip_install(
        "torch==2.2.0",
        "torchvision==0.17.0",
        "torchaudio==2.2.0",
        index_url="https://download.pytorch.org/whl/cu121",
    )
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
        # Pin numpy<2 because SimLingo + torch 2.2 + opencv build against numpy 1.x.
        "numpy<2",
        # Headless opencv to avoid pulling libGL into the runtime layer.
        "opencv-python-headless==4.10.0.84",
        # line_profiler is imported unconditionally by datamodule.py upstream.
        "line_profiler",
    )
    # Prebuilt flash-attn wheel — much faster than the 20–30min from-source
    # compile that `pip install flash-attn` triggers by default. The cu122
    # wheel works against torch built for cu121 (CUDA is forward-compatible
    # within a major version).
    .pip_install(
        "https://github.com/Dao-AILab/flash-attention/releases/download/v2.7.0.post2/flash_attn-2.7.0.post2+cu12torch2.2cxx11abiFALSE-cp310-cp310-linux_x86_64.whl"
    )
    .pip_install("deepspeed==0.16.2")
    .run_commands(
        f"git clone --depth 1 {SIMLINGO_REPO_URL} {SIMLINGO_REPO_DIR}",
        # Pin to a known-good commit so reruns are reproducible.
        f"cd {SIMLINGO_REPO_DIR} && git rev-parse HEAD",
    )
    .env(
        {
            "PYTHONPATH": SIMLINGO_REPO_DIR,
            "HF_HOME": f"{CACHE_DIR}/hf",
            "HUGGINGFACE_HUB_CACHE": f"{CACHE_DIR}/hf/hub",
            "TRANSFORMERS_CACHE": f"{CACHE_DIR}/hf/transformers",
            # InternVL2 modeling code uses trust_remote_code; pre-accept.
            "TRUST_REMOTE_CODE": "1",
            # Quiet some noisy import-time warnings.
            "TOKENIZERS_PARALLELISM": "false",
        }
    )
    # Mount our local scripts/ dir into the image so we don't have to rebuild it
    # every time we tweak inference logic.
    .add_local_python_source("scripts")
)

# ---------------------------------------------------------------------------
# Volumes
# ---------------------------------------------------------------------------

cache_volume = modal.Volume.from_name("simlingo-cache", create_if_missing=True)
data_volume = modal.Volume.from_name("simlingo-data", create_if_missing=True)
output_volume = modal.Volume.from_name("simlingo-outputs", create_if_missing=True)

VOLUMES = {
    CACHE_DIR: cache_volume,
    DATA_DIR: data_volume,
    OUTPUTS_DIR: output_volume,
}

# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

app = modal.App(APP_NAME)


# ----- helpers -------------------------------------------------------------


def _hf_download(repo_id: str, filename: str, local_dir: str, repo_type: str = "model") -> str:
    """Download one file from the HF hub into local_dir, return the local path."""
    from huggingface_hub import hf_hub_download

    return hf_hub_download(
        repo_id=repo_id,
        filename=filename,
        local_dir=local_dir,
        repo_type=repo_type,
    )


def _extract_tar(tar_path: str, dst_dir: str) -> None:
    print(f"  extract: {tar_path} -> {dst_dir}", flush=True)
    Path(dst_dir).mkdir(parents=True, exist_ok=True)
    # Use the streaming tarfile module so we can handle multi-GB archives without
    # buffering them in RAM.
    with tarfile.open(tar_path, "r:gz") as tf:
        tf.extractall(dst_dir)


# ----- entrypoints --------------------------------------------------------


@app.function(
    image=simlingo_image,
    volumes=VOLUMES,
    timeout=60 * 60 * 3,  # 3h to download big tarballs
    cpu=4,
)
def prepare_assets(skip_data: bool = False, force: bool = False) -> dict:
    """Download model weights + one validation chunk into Modal volumes."""
    print(">> preparing assets", flush=True)

    Path(CKPT_DIR).mkdir(parents=True, exist_ok=True)
    Path(EXTRACTED_DIR).mkdir(parents=True, exist_ok=True)

    # 1) Model weights + hydra config.
    for filename in (HF_CKPT_FILE, HF_HYDRA_CONFIG_FILE):
        dst = Path(CKPT_DIR) / filename
        if dst.exists() and not force:
            print(f"  cached: {dst}", flush=True)
            continue
        print(f"  hf model: {filename}", flush=True)
        _hf_download(HF_MODEL_REPO, filename, CKPT_DIR, repo_type="model")

    # Also snapshot the InternVL2-1B base into the HF cache. Its
    # `conversation.py` is needed to build the InternLM2 chat template, and
    # `trust_remote_code` requires the files to be present locally.
    from huggingface_hub import snapshot_download

    print("  hf snapshot: OpenGVLab/InternVL2-1B", flush=True)
    snapshot_download(
        repo_id="OpenGVLab/InternVL2-1B",
        local_dir=f"{CACHE_DIR}/hf/snapshots/InternVL2-1B",
        ignore_patterns=["*.bin"],  # safetensors only, keep it small
    )

    cache_volume.commit()

    # 2) Validation data chunk(s).
    if not skip_data:
        for filename in HF_DATA_FILES:
            tar_dst = Path(DATA_DIR) / "downloads" / filename
            tar_dst.parent.mkdir(parents=True, exist_ok=True)
            if not tar_dst.exists() or force:
                print(f"  hf data: {filename}", flush=True)
                # hf_hub_download streams to a tmp dir; we then copy.
                downloaded = _hf_download(
                    HF_DATA_REPO,
                    filename,
                    str(tar_dst.parent),
                    repo_type="dataset",
                )
                if Path(downloaded) != tar_dst:
                    Path(downloaded).rename(tar_dst)
            else:
                print(f"  cached: {tar_dst}", flush=True)
            _extract_tar(str(tar_dst), EXTRACTED_DIR)

    data_volume.commit()

    inventory = subprocess.check_output(
        ["du", "-sh", CKPT_DIR, EXTRACTED_DIR],
        text=True,
    )
    print(inventory, flush=True)
    return {"ckpt_dir": CKPT_DIR, "data_dir": EXTRACTED_DIR}


@app.function(
    image=simlingo_image,
    volumes=VOLUMES,
    gpu="L4",
    timeout=60 * 60,
    cpu=4,
    memory=24 * 1024,
)
def run(
    num_samples: int = 32,
    frame_stride: int = 20,
    use_cot: bool = True,
    save_overlays: bool = True,
    seed: int = 0,
) -> dict:
    """Load SimLingo, run inference on a few validation frames, dump metrics + overlays."""
    # Late imports so they only happen inside the Modal container.
    sys.path.insert(0, SIMLINGO_REPO_DIR)
    sys.path.insert(0, os.path.dirname(__file__))

    from scripts import inference

    ckpt_path = str(Path(CKPT_DIR) / HF_CKPT_FILE)
    hydra_cfg_path = str(Path(CKPT_DIR) / HF_HYDRA_CONFIG_FILE)
    data_root = EXTRACTED_DIR
    out_dir = Path(OUTPUTS_DIR) / f"run_n{num_samples}_s{seed}"
    out_dir.mkdir(parents=True, exist_ok=True)

    summary = inference.run_inference(
        ckpt_path=ckpt_path,
        hydra_cfg_path=hydra_cfg_path,
        data_root=data_root,
        out_dir=str(out_dir),
        num_samples=num_samples,
        frame_stride=frame_stride,
        use_cot=use_cot,
        save_overlays=save_overlays,
        seed=seed,
    )

    output_volume.commit()

    print(">> metrics:", summary["metrics"], flush=True)
    print(f">> wrote {summary['num_written']} overlays to {out_dir}", flush=True)
    return summary


@app.function(image=simlingo_image, volumes=VOLUMES, timeout=600)
def peek(path: str = EXTRACTED_DIR, max_depth: int = 4) -> str:
    """Cheap `find -maxdepth N` for debugging volume contents."""
    cmd = f"find {shlex.quote(path)} -maxdepth {max_depth} | head -200"
    out = subprocess.check_output(cmd, shell=True, text=True)
    print(out)
    return out


@app.local_entrypoint()
def main(
    num_samples: int = 32,
    frame_stride: int = 20,
    skip_data: bool = False,
):
    """Convenience: prepare assets then run inference."""
    prepare_assets.remote(skip_data=skip_data)
    run.remote(num_samples=num_samples, frame_stride=frame_stride)
