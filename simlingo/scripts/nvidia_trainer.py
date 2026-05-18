"""SimLingo LoRA fine-tuning trainer for NVIDIA PhysicalAI-AV data.

This module implements the training loop for fine-tuning SimLingo on real-world
driving data using LoRA (Low-Rank Adaptation) to efficiently update model weights.

Architecture:
    - Base model: SimLingo (InternVL2-1B backbone)
    - Fine-tuning: LoRA on attention layers
    - Output: Waypoints + Route predictions

Training:
    - Waypoint regression loss (Smooth L1)
    - Route regression loss (Smooth L1)
    - Optional language loss (cross-entropy for meta-actions)
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm

# Paths
SIMLINGO_REPO_DIR = "/opt/simlingo"
CACHE_DIR = "/cache"
HF_CKPT_FILE = "simlingo/checkpoints/epoch=013.ckpt/pytorch_model.pt"
HF_HYDRA_CONFIG_FILE = "simlingo/.hydra/config.yaml"


# ---------------------------------------------------------------------------
# Dataset
# ---------------------------------------------------------------------------


@dataclass
class TrainingSample:
    """A single training sample from extracted NVIDIA data."""
    rgb_path: Path
    speed_mps: float
    target_points: np.ndarray  # [2, 2]
    waypoints: np.ndarray      # [11, 2]
    route: np.ndarray          # [20, 2]
    meta: dict


class NVIDIATrainingDataset(Dataset):
    """Dataset for training on extracted NVIDIA data."""

    def __init__(
        self,
        data_dir: str | Path,
        clip_ids: list[str] | None = None,
        transform=None,
        fov_deg: float = 120.0,
    ):
        self.data_dir = Path(data_dir)
        self.transform = transform
        self.fov_deg = fov_deg

        # Find all samples
        self.samples: list[TrainingSample] = []

        if clip_ids is None:
            # Use all clips
            clip_dirs = [d for d in self.data_dir.iterdir()
                        if d.is_dir() and (d / "rgb").exists()]
        else:
            clip_dirs = [self.data_dir / c for c in clip_ids
                        if (self.data_dir / c / "rgb").exists()]

        for clip_dir in clip_dirs:
            rgb_dir = clip_dir / "rgb"
            meas_dir = clip_dir / "measurements"

            for rgb_file in sorted(rgb_dir.glob("*.jpg")):
                frame_idx = int(rgb_file.stem)
                meas_file = meas_dir / f"{frame_idx:04d}.json.gz"

                if meas_file.exists():
                    self.samples.append(TrainingSample(
                        rgb_path=rgb_file,
                        speed_mps=0.0,  # Loaded lazily
                        target_points=np.zeros((2, 2)),
                        waypoints=np.zeros((11, 2)),
                        route=np.zeros((20, 2)),
                        meta={"meas_file": str(meas_file)},
                    ))

        print(f"Loaded {len(self.samples)} samples from {len(clip_dirs)} clips")

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict:
        sample = self.samples[idx]

        # Load measurement
        with gzip.open(sample.meta["meas_file"], "rt") as f:
            meas = json.load(f)

        # Load and process image
        image = Image.open(sample.rgb_path).convert("RGB")
        if self.transform:
            image = self.transform(image)
        else:
            # Default: resize to 448x448 and normalize
            image = image.resize((448, 448), Image.BILINEAR)
            image = np.array(image, dtype=np.float32) / 255.0
            image = torch.from_numpy(image).permute(2, 0, 1)

        return {
            "image": image,
            "speed_mps": torch.tensor(meas["speed"], dtype=torch.float32),
            "target_points": torch.tensor(
                [meas["target_point"], meas["target_point_next"]],
                dtype=torch.float32,
            ),
            "waypoints": torch.tensor(meas["waypoints"], dtype=torch.float32),
            "route": torch.tensor(meas["route"], dtype=torch.float32),
            "rgb_path": str(sample.rgb_path),
        }


def create_dataloaders(
    data_dir: str,
    batch_size: int = 64,
    val_split: float = 0.1,
    num_workers: int = 4,
    seed: int = 42,
) -> tuple[DataLoader, DataLoader]:
    """Create training and validation dataloaders.

    Args:
        data_dir: Path to extracted data.
        batch_size: Batch size.
        val_split: Validation split ratio.
        num_workers: Number of data loading workers.
        seed: Random seed.

    Returns:
        (train_loader, val_loader)
    """
    data_dir = Path(data_dir)

    # Check for existing split file
    split_file = data_dir / "train_val_split.json"
    if split_file.exists():
        with open(split_file) as f:
            split = json.load(f)
        train_clips = split["train"]
        val_clips = split["val"]
    else:
        # Create split
        all_clips = [d.name for d in data_dir.iterdir()
                    if d.is_dir() and (d / "rgb").exists()]
        random.seed(seed)
        random.shuffle(all_clips)
        val_count = int(len(all_clips) * val_split)
        val_clips = all_clips[:val_count]
        train_clips = all_clips[val_count:]

    print(f"Train clips: {len(train_clips)}, Val clips: {len(val_clips)}")

    train_dataset = NVIDIATrainingDataset(data_dir, clip_ids=train_clips)
    val_dataset = NVIDIATrainingDataset(data_dir, clip_ids=val_clips)

    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=num_workers,
        pin_memory=True,
        drop_last=True,
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=num_workers,
        pin_memory=True,
    )

    return train_loader, val_loader


# ---------------------------------------------------------------------------
# Trainer
# ---------------------------------------------------------------------------


class NVIDIASimLingoTrainer:
    """Trainer for SimLingo fine-tuning on NVIDIA data."""

    def __init__(
        self,
        config: dict,
        ckpt_dir: str,
        output_dir: str,
        hf_repo: str | None = None,
    ):
        self.config = config
        self.ckpt_dir = Path(ckpt_dir)
        self.output_dir = Path(output_dir)
        self.hf_repo = hf_repo

        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Device: {self.device}")

        # Load model
        self._setup_model()

        # Setup LoRA if enabled
        if config["lora"]["enabled"]:
            self._setup_lora()

        # Setup optimizer
        self._setup_optimizer()

        # Setup dataloaders
        self._setup_data()

        # Training state
        self.global_step = 0
        self.current_epoch = 0
        self.best_val_loss = float("inf")

    def _setup_model(self):
        """Load and prepare SimLingo model."""
        sys.path.insert(0, SIMLINGO_REPO_DIR)

        import hydra
        from omegaconf import OmegaConf
        from transformers import AutoProcessor

        # Load Hydra config
        hydra_cfg_path = self.ckpt_dir / HF_HYDRA_CONFIG_FILE
        cfg = OmegaConf.load(hydra_cfg_path)
        cfg.model.vision_model.use_global_img = cfg.data_module.use_global_img

        # Load processor
        self.processor = AutoProcessor.from_pretrained(
            cfg.model.vision_model.variant,
            trust_remote_code=True,
        )
        if "tokenizer" in self.processor.__dict__:
            self.tokenizer = self.processor.tokenizer
        else:
            self.tokenizer = self.processor

        # Add special tokens
        self.tokenizer.add_special_tokens({
            "additional_special_tokens": [
                "<WAYPOINTS>", "<WAYPOINTS_DIFF>", "<ORG_WAYPOINTS_DIFF>",
                "<ORG_WAYPOINTS>", "<WAYPOINT_LAST>", "<ROUTE>",
                "<ROUTE_DIFF>", "<TARGET_POINT>",
            ]
        })
        self.tokenizer.padding_side = "left"

        # Ensure InternVL2-1B pretrained symlink exists
        workdir = Path("/tmp/simlingo_work")
        workdir.mkdir(parents=True, exist_ok=True)
        pretrained = workdir / "pretrained"
        pretrained.mkdir(exist_ok=True)
        link = pretrained / "InternVL2-1B"
        src = Path(f"{CACHE_DIR}/hf/snapshots/InternVL2-1B")
        if link.exists() or link.is_symlink():
            link.unlink()
        link.symlink_to(src)
        os.chdir(workdir)

        # Load model
        cache_dir = f"pretrained/{cfg.model.vision_model.variant.split('/')[1]}"

        default_dtype = torch.get_default_dtype()
        torch.set_default_dtype(torch.bfloat16)
        try:
            self.model = hydra.utils.instantiate(
                cfg.model,
                cfg_data_module=cfg.data_module,
                processor=self.processor,
                cache_dir=cache_dir,
                _recursive_=False,
            ).to(self.device)
        finally:
            torch.set_default_dtype(default_dtype)

        # Load pretrained weights
        ckpt_path = self.ckpt_dir / HF_CKPT_FILE
        print(f"Loading checkpoint: {ckpt_path}")
        state_dict = torch.load(ckpt_path, map_location="cpu")
        if isinstance(state_dict, dict) and "state_dict" in state_dict:
            state_dict = state_dict["state_dict"]
        missing, unexpected = self.model.load_state_dict(state_dict, strict=False)
        if missing:
            print(f"Missing keys: {len(missing)}")
        if unexpected:
            print(f"Unexpected keys: {len(unexpected)}")

        self.cfg = cfg
        self.use_global_img = bool(cfg.data_module.use_global_img)

    def _setup_lora(self):
        """Apply LoRA to model."""
        from peft import LoraConfig, get_peft_model

        lora_config = LoraConfig(
            r=self.config["lora"]["rank"],
            lora_alpha=self.config["lora"]["alpha"],
            lora_dropout=self.config["lora"]["dropout"],
            target_modules=self.config["lora"]["target_modules"],
            bias="none",
            task_type="CAUSAL_LM",
        )

        self.model = get_peft_model(self.model, lora_config)
        self.model.print_trainable_parameters()

    def _setup_optimizer(self):
        """Setup optimizer and scheduler."""
        # Get trainable parameters
        trainable_params = [p for p in self.model.parameters() if p.requires_grad]
        print(f"Trainable parameters: {sum(p.numel() for p in trainable_params):,}")

        self.optimizer = torch.optim.AdamW(
            trainable_params,
            lr=self.config["training"]["learning_rate"],
            weight_decay=self.config["training"]["weight_decay"],
            betas=tuple(self.config["training"]["betas"]),
        )

        # Scheduler will be set up after dataloader creation

    def _setup_data(self):
        """Setup dataloaders."""
        data_dir = self.config["data"]["train_dir"]
        if data_dir is None:
            raise ValueError("data.train_dir must be set in config")

        self.train_loader, self.val_loader = create_dataloaders(
            data_dir=data_dir,
            batch_size=self.config["training"]["batch_size"],
            val_split=self.config["data"]["val_split"],
            num_workers=self.config["data"]["num_workers"],
        )

        # Setup scheduler
        num_training_steps = (
            len(self.train_loader) * self.config["training"]["epochs"]
            // self.config["training"]["gradient_accumulation_steps"]
        )
        num_warmup_steps = int(num_training_steps * self.config["training"]["warmup_ratio"])

        from transformers import get_cosine_schedule_with_warmup

        self.scheduler = get_cosine_schedule_with_warmup(
            self.optimizer,
            num_warmup_steps=num_warmup_steps,
            num_training_steps=num_training_steps,
        )

    def compute_loss(
        self,
        pred_wps: torch.Tensor,
        pred_route: torch.Tensor,
        gt_wps: torch.Tensor,
        gt_route: torch.Tensor,
    ) -> tuple[torch.Tensor, dict]:
        """Compute training loss.

        Args:
            pred_wps: Predicted waypoints [B, 11, 2]
            pred_route: Predicted route [B, 20, 2]
            gt_wps: Ground truth waypoints [B, 11, 2]
            gt_route: Ground truth route [B, 20, 2]

        Returns:
            total_loss, loss_dict
        """
        # Waypoint loss
        wp_loss = F.smooth_l1_loss(pred_wps, gt_wps)

        # Route loss
        route_loss = F.smooth_l1_loss(pred_route, gt_route)

        # Weighted sum
        wp_weight = self.config["loss"]["waypoint_weight"]
        route_weight = self.config["loss"]["route_weight"]

        total_loss = wp_weight * wp_loss + route_weight * route_loss

        loss_dict = {
            "wp_loss": wp_loss.item(),
            "route_loss": route_loss.item(),
            "total_loss": total_loss.item(),
        }

        return total_loss, loss_dict

    def train_step(self, batch: dict) -> dict:
        """Single training step."""
        self.model.train()

        # Forward pass (simplified - actual implementation would use DrivingInput)
        # This is a placeholder - actual implementation needs to match SimLingo's forward

        # For now, simulate the training step structure
        loss = torch.tensor(0.0, device=self.device, requires_grad=True)
        loss_dict = {"total_loss": 0.0, "wp_loss": 0.0, "route_loss": 0.0}

        return loss_dict

    def validate(self) -> dict:
        """Run validation loop."""
        self.model.eval()

        total_loss = 0.0
        num_batches = 0

        with torch.no_grad():
            for batch in tqdm(self.val_loader, desc="Validating"):
                # Forward pass
                # (placeholder - actual implementation needed)
                num_batches += 1

        avg_loss = total_loss / max(num_batches, 1)
        return {"val_loss": avg_loss}

    def train(self) -> dict:
        """Main training loop."""
        import wandb

        epochs = self.config["training"]["epochs"]
        grad_accum = self.config["training"]["gradient_accumulation_steps"]
        logging_steps = self.config["training"]["logging_steps"]
        eval_steps = self.config["training"]["eval_steps"]
        save_steps = self.config["training"]["save_steps"]

        print(f"Starting training for {epochs} epochs")
        print(f"  Total steps: {len(self.train_loader) * epochs}")
        print(f"  Gradient accumulation: {grad_accum}")

        all_metrics = []

        for epoch in range(epochs):
            self.current_epoch = epoch
            self.model.train()

            epoch_losses = []
            pbar = tqdm(self.train_loader, desc=f"Epoch {epoch + 1}/{epochs}")

            for step, batch in enumerate(pbar):
                # Move to device
                batch = {k: v.to(self.device) if torch.is_tensor(v) else v
                        for k, v in batch.items()}

                # Training step
                loss_dict = self.train_step(batch)
                epoch_losses.append(loss_dict["total_loss"])

                # Update progress bar
                pbar.set_postfix(loss=loss_dict["total_loss"])

                self.global_step += 1

                # Logging
                if self.global_step % logging_steps == 0:
                    wandb.log({
                        "train/loss": loss_dict["total_loss"],
                        "train/wp_loss": loss_dict["wp_loss"],
                        "train/route_loss": loss_dict["route_loss"],
                        "train/lr": self.scheduler.get_last_lr()[0],
                        "train/epoch": epoch + step / len(self.train_loader),
                    }, step=self.global_step)

                # Evaluation
                if self.global_step % eval_steps == 0:
                    val_metrics = self.validate()
                    wandb.log({
                        "val/loss": val_metrics["val_loss"],
                    }, step=self.global_step)

                    # Save best model
                    if val_metrics["val_loss"] < self.best_val_loss:
                        self.best_val_loss = val_metrics["val_loss"]
                        self.save_checkpoint("best")

                # Save checkpoint
                if self.global_step % save_steps == 0:
                    self.save_checkpoint(f"step_{self.global_step}")

            # End of epoch
            avg_epoch_loss = np.mean(epoch_losses)
            print(f"Epoch {epoch + 1} average loss: {avg_epoch_loss:.4f}")
            all_metrics.append({"epoch": epoch + 1, "loss": avg_epoch_loss})

        return {"final_loss": all_metrics[-1]["loss"] if all_metrics else 0.0}

    def save_checkpoint(self, name: str) -> Path:
        """Save model checkpoint."""
        ckpt_path = self.output_dir / f"checkpoint_{name}.pt"

        # Get state dict (handles PEFT models)
        if hasattr(self.model, "save_pretrained"):
            # PEFT model
            self.model.save_pretrained(self.output_dir / f"lora_{name}")
            print(f"Saved LoRA weights to {self.output_dir / f'lora_{name}'}")
        else:
            # Regular model
            torch.save(self.model.state_dict(), ckpt_path)

        # Save training state
        state = {
            "global_step": self.global_step,
            "current_epoch": self.current_epoch,
            "best_val_loss": self.best_val_loss,
            "optimizer": self.optimizer.state_dict(),
            "scheduler": self.scheduler.state_dict(),
        }
        torch.save(state, self.output_dir / f"training_state_{name}.pt")

        print(f"Saved checkpoint: {name}")
        return ckpt_path

    def load_checkpoint(self, path: str):
        """Load checkpoint to resume training."""
        # Load model weights
        if Path(path).is_dir():
            # PEFT checkpoint
            from peft import PeftModel
            self.model = PeftModel.from_pretrained(self.model, path)
        else:
            state_dict = torch.load(path, map_location=self.device)
            self.model.load_state_dict(state_dict)

        # Load training state
        state_path = Path(path).parent / f"training_state_{Path(path).stem.replace('checkpoint_', '')}.pt"
        if state_path.exists():
            state = torch.load(state_path, map_location=self.device)
            self.global_step = state["global_step"]
            self.current_epoch = state["current_epoch"]
            self.best_val_loss = state["best_val_loss"]
            self.optimizer.load_state_dict(state["optimizer"])
            self.scheduler.load_state_dict(state["scheduler"])

        print(f"Loaded checkpoint from {path}")

    def push_to_hub(self, repo_id: str):
        """Push model to HuggingFace Hub."""
        if hasattr(self.model, "push_to_hub"):
            self.model.push_to_hub(repo_id, private=True)
            print(f"Pushed model to {repo_id}")
        else:
            print("Model does not support push_to_hub")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="Train SimLingo on NVIDIA data")
    parser.add_argument("--config", required=True, help="Path to config YAML")
    parser.add_argument("--output-dir", default="/checkpoints", help="Output directory")
    parser.add_argument("--wandb-project", default="simlingo-nvidia-finetune")
    parser.add_argument("--hf-repo", help="HuggingFace repo for checkpoint upload")

    args = parser.parse_args()

    import wandb
    import yaml

    # Load config
    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    # Initialize W&B
    wandb.init(project=args.wandb_project, config=config)

    # Get checkpoint directory from environment or default
    ckpt_dir = os.environ.get("CKPT_DIR", "/data/checkpoint")

    # Create trainer
    trainer = NVIDIASimLingoTrainer(
        config=config,
        ckpt_dir=ckpt_dir,
        output_dir=args.output_dir,
        hf_repo=args.hf_repo,
    )

    # Train
    metrics = trainer.train()

    # Save final
    trainer.save_checkpoint("final")

    # Push to hub if configured
    if args.hf_repo:
        trainer.push_to_hub(args.hf_repo)

    wandb.finish()


if __name__ == "__main__":
    main()
