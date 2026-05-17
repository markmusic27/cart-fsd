"""Offline SimLingo inference + metrics.

This runs *inside* the Modal container (see modal_app.py). The job:

1. Load the released SimLingo checkpoint using the upstream Hydra config.
2. Walk a few CARLA validation routes (extracted from the HF dataset tarballs).
3. For each sampled frame, reconstruct the same `DrivingInput` the live CARLA
   agent would assemble (image patches, ego-frame target points, prompt with
   `<TARGET_POINT>` placeholders) and run the model.
4. Compute waypoint ADE/FDE against ground-truth waypoints derived from each
   frame's `ego_matrix`, plus a couple of cheap commentary checks.

Most of the heavy lifting (chat template assembly, image patching, placeholder
substitution) is delegated to the upstream `simlingo_training.utils.*` modules
so behaviour stays in lock-step with the trained model.
"""

from __future__ import annotations

import glob
import gzip
import json
import math
import os
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import torch


# ---------------------------------------------------------------------------
# Path / env setup helpers
# ---------------------------------------------------------------------------


def _ensure_internvl_pretrained_symlink(cache_root: str = "/cache/hf/snapshots") -> Path:
    """Upstream `get_custom_chat_template` expects to find
    `pretrained/InternVL2-1B/conversation.py` relative to cwd.

    We pre-snapshot the model into the HF cache during `prepare_assets`, so all
    we need to do here is point a `pretrained/` dir at it.
    """
    workdir = Path("/tmp/simlingo_work")
    workdir.mkdir(parents=True, exist_ok=True)
    os.chdir(workdir)

    pretrained = workdir / "pretrained"
    pretrained.mkdir(exist_ok=True)
    link = pretrained / "InternVL2-1B"
    src = Path(cache_root) / "InternVL2-1B"
    if link.exists() or link.is_symlink():
        link.unlink()
    link.symlink_to(src)
    return workdir


# ---------------------------------------------------------------------------
# Dataset walking (raw, without the upstream `BaseDataset` bucket machinery)
# ---------------------------------------------------------------------------


@dataclass
class FrameSample:
    route_dir: Path  # ".../Town12/route_xx"
    frame_idx: int  # 0-indexed; matches the filename digits (e.g. 0042 -> 42)
    rgb_path: Path
    measurements_dir: Path
    commentary_path: Path | None  # may not exist


def _find_route_dirs(data_root: str) -> list[Path]:
    # Routes are nested as ".../Town<N>/<route_id>/". Look one level above an
    # `rgb/` subdir so we don't depend on the parent prefix structure.
    candidates = glob.glob(os.path.join(data_root, "**", "rgb"), recursive=True)
    return sorted({Path(c).parent for c in candidates})


def _collect_samples(
    data_root: str,
    num_samples: int,
    frame_stride: int,
    pred_len: int = 11,
    seed: int = 0,
) -> list[FrameSample]:
    routes = _find_route_dirs(data_root)
    print(f"  found {len(routes)} route dirs under {data_root}", flush=True)
    rng = random.Random(seed)
    rng.shuffle(routes)

    samples: list[FrameSample] = []
    for route_dir in routes:
        rgb_dir = route_dir / "rgb"
        meas_dir = route_dir / "measurements"
        if not (rgb_dir.exists() and meas_dir.exists()):
            continue
        # Match the file naming convention used by the data agent: NNNN.jpg.
        frame_files = sorted(rgb_dir.glob("[0-9][0-9][0-9][0-9].jpg"))
        # We need future measurements to be available for the GT waypoints.
        usable = [
            int(p.stem)
            for p in frame_files
            if (meas_dir / f"{int(p.stem) + pred_len:04d}.json.gz").exists()
        ]
        if not usable:
            continue
        # Stride to spread samples across the route.
        for i in range(0, len(usable), frame_stride):
            idx = usable[i]
            # Optional commentary annotation lives in a parallel `commentary/`
            # tree the dataset publishes alongside `data/`.
            comm_path = _commentary_path_for(route_dir, idx)
            samples.append(
                FrameSample(
                    route_dir=route_dir,
                    frame_idx=idx,
                    rgb_path=rgb_dir / f"{idx:04d}.jpg",
                    measurements_dir=meas_dir,
                    commentary_path=comm_path,
                )
            )
            if len(samples) >= num_samples:
                return samples
    return samples


def _commentary_path_for(route_dir: Path, frame_idx: int) -> Path | None:
    """The upstream dataset stores commentary at the same path with `data/` ->
    `commentary/` and `measurements/` -> `commentary/`."""
    parts = list(route_dir.parts)
    try:
        # rewrite the first `data` segment, like the upstream loader does.
        idx_data = parts.index("data")
        parts[idx_data] = "commentary"
    except ValueError:
        return None
    candidate = Path(*parts) / "commentary" / f"{frame_idx:04d}.json.gz"
    return candidate if candidate.exists() else None


def _load_json_gz(path: Path) -> dict:
    import ujson

    with gzip.open(path, "rt") as fh:
        return ujson.load(fh)


# ---------------------------------------------------------------------------
# Building a DrivingInput from raw files
# ---------------------------------------------------------------------------


def _crop_bottom(img: np.ndarray) -> np.ndarray:
    """Drop the bottom ~30% of the image to hide the bonnet. The exact ratio
    comes from upstream data collection (`(h * 4.8) // 16`)."""
    h = img.shape[0]
    new_h = int(h - (h * 4.8) // 16)
    return img[:new_h, :, :]


def _compute_waypoints_from_ego_matrix(
    measurements: list[dict],
) -> np.ndarray:
    """Replicates `BaseDataset.get_waypoints`: future positions in the current
    ego frame, dropping the height dimension. Returns shape [F, 2]."""
    origin = np.asarray(measurements[0]["ego_matrix"])[:3]
    origin_t = origin[:, 3:4]
    origin_R = origin[:, :3]
    wps = []
    for m in measurements[1:]:
        p = np.asarray(m["ego_matrix"])[:3, 3:4]
        ego = origin_R.T @ (p - origin_t)
        wps.append(ego[:2, 0])
    return np.asarray(wps, dtype=np.float32)


def _equal_spacing_route(points: np.ndarray, num: int = 20) -> np.ndarray:
    """Mirror of `BaseDataset.equal_spacing_route`: 1m-spaced interpolation
    starting at the origin, padded out to `num` points."""
    pts = np.concatenate((np.zeros_like(points[:1]), points))
    shift = np.roll(pts, 1, axis=0)
    shift[0] = shift[1]
    dists = np.linalg.norm(pts - shift, axis=1)
    dists = np.cumsum(dists)
    dists += np.arange(len(dists)) * 1e-4
    x = np.arange(0, num, 1)
    return np.stack(
        [np.interp(x, dists, pts[:, 0]), np.interp(x, dists, pts[:, 1])],
        axis=-1,
    ).astype(np.float32)


def _build_prompt(speed_mps: float, use_cot: bool) -> str:
    speed_rounded = round(speed_mps, 1)
    # The placeholder appears twice because the model embeds *both* target
    # points (current and next) by replacing each placeholder slot. See
    # `team_code/agent_simlingo.py::tick`.
    target_segment = "Target waypoint: <TARGET_POINT><TARGET_POINT>."
    if use_cot:
        return f"Current speed: {speed_rounded} m/s. {target_segment} What should the ego do next?"
    return f"Current speed: {speed_rounded} m/s. {target_segment} Predict the waypoints."


def _process_image(
    rgb_path: Path,
    use_global_img: bool,
) -> tuple[torch.Tensor, tuple[int, int]]:
    """Run the upstream InternVL2 dynamic patching on a single front-cam frame.

    Returns:
        pixel_values: shape [1, T=1, num_patches, 3, 448, 448], float32.
        (H, W): the size of the cropped image fed to the patcher.
    """
    import cv2
    from PIL import Image

    from simlingo_training.utils.internvl2_utils import build_transform, dynamic_preprocess

    bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    if bgr is None:
        raise FileNotFoundError(rgb_path)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    # Live agent does an extra JPEG roundtrip; on already-jpeg dataset frames
    # this is essentially a no-op so we skip it for speed.
    rgb = _crop_bottom(rgb)
    H, W, _ = rgb.shape

    transform = build_transform(input_size=448)
    pil = Image.fromarray(rgb)
    patches = dynamic_preprocess(
        pil,
        image_size=448,
        use_thumbnail=use_global_img,
        max_num=2,
    )
    pixel_values = torch.stack([transform(p) for p in patches])  # [P, 3, 448, 448]
    pixel_values = pixel_values.unsqueeze(0).unsqueeze(0)  # [1, T=1, P, 3, 448, 448]
    return pixel_values, (H, W)


def _build_language_label(
    prompt_text: str,
    target_points_np: np.ndarray,
    tokenizer,
    encoder_variant: str,
    num_image_tokens_total: int,
    device: torch.device,
):
    """Tokenize the prompt, slot in image tokens, and build two LanguageLabel
    objects (the upstream forward expects both `prompt` and `prompt_inference`
    to be present)."""
    from simlingo_training.utils.custom_types import LanguageLabel
    from simlingo_training.utils.internvl2_utils import get_custom_chat_template

    # The upstream code expects each conversation as a list-of-dict messages
    # with a list-of-content blocks. We mimic the agent's two-turn shape so the
    # InternLM2 chat template renders identically to training.
    conversation_all = [
        [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt_text},
                    {"type": "image"},
                ],
            },
            {
                "role": "assistant",
                "content": [
                    {"type": "text", "text": "Waypoints:"},
                ],
            },
        ]
    ]

    conv_dict, question_dict = get_custom_chat_template(
        conversation_all,
        tokenizer,
        encoder_variant=encoder_variant,
        num_image_tokens_total=num_image_tokens_total,
    )

    placeholder_token_id = tokenizer.convert_tokens_to_ids("<TARGET_POINT>")
    placeholder_batch_list = [{placeholder_token_id: target_points_np}]

    def _to_ll(d):
        return LanguageLabel(
            phrase_ids=d["phrase_ids"].to(device),
            phrase_valid=d["phrase_valid"].to(device),
            phrase_mask=d["phrase_mask"].to(device),
            placeholder_values=placeholder_batch_list,
            language_string=d["language_string"],
            loss_masking=d.get("loss_masking"),
        )

    return _to_ll(conv_dict), _to_ll(question_dict)


def _build_driving_input(
    pixel_values: torch.Tensor,
    speed_mps: float,
    target_points_np: np.ndarray,
    prompt_ll,
    prompt_inference_ll,
    HW: tuple[int, int],
    device: torch.device,
):
    """Assemble the upstream `DrivingInput` namedtuple."""
    from simlingo_training.utils.custom_types import DrivingInput
    from simlingo_training.utils.projection import (
        get_camera_extrinsics,
        get_camera_intrinsics,
    )

    H, W = HW
    intrinsics = get_camera_intrinsics(W, H, 110).unsqueeze(0).to(device).float()
    extrinsics = get_camera_extrinsics().unsqueeze(0).to(device).float()
    return DrivingInput(
        camera_images=pixel_values.to(device).bfloat16(),
        image_sizes=None,
        camera_intrinsics=intrinsics,
        camera_extrinsics=extrinsics,
        vehicle_speed=torch.tensor([[speed_mps]], dtype=torch.float32, device=device),
        target_point=torch.tensor(
            target_points_np[0:1], dtype=torch.float32, device=device
        ),
        prompt=prompt_ll,
        prompt_inference=prompt_inference_ll,
    )


# ---------------------------------------------------------------------------
# Model loading
# ---------------------------------------------------------------------------


def _load_model(ckpt_path: str, hydra_cfg_path: str, device: torch.device):
    """Instantiate the released SimLingo checkpoint exactly the way Hydra would."""
    import hydra
    from omegaconf import OmegaConf
    from transformers import AutoProcessor

    cfg = OmegaConf.load(hydra_cfg_path)
    # The agent's live setup forwards `use_global_img` from the data module to
    # the vision model; replicate that to keep grid sizes consistent.
    cfg.model.vision_model.use_global_img = cfg.data_module.use_global_img

    print(f"  load processor: {cfg.model.vision_model.variant}", flush=True)
    processor = AutoProcessor.from_pretrained(
        cfg.model.vision_model.variant,
        trust_remote_code=True,
    )
    if "tokenizer" in processor.__dict__:
        tokenizer = processor.tokenizer
    else:
        tokenizer = processor
    # These match `team_code/agent_simlingo.py` exactly. Without them the
    # placeholder substitution in the model won't find <TARGET_POINT>.
    tokenizer.add_special_tokens(
        {
            "additional_special_tokens": [
                "<WAYPOINTS>",
                "<WAYPOINTS_DIFF>",
                "<ORG_WAYPOINTS_DIFF>",
                "<ORG_WAYPOINTS>",
                "<WAYPOINT_LAST>",
                "<ROUTE>",
                "<ROUTE_DIFF>",
                "<TARGET_POINT>",
            ]
        }
    )
    tokenizer.padding_side = "left"

    cache_dir = f"pretrained/{cfg.model.vision_model.variant.split('/')[1]}"

    default_dtype = torch.get_default_dtype()
    torch.set_default_dtype(torch.bfloat16)
    try:
        model = hydra.utils.instantiate(
            cfg.model,
            cfg_data_module=cfg.data_module,
            processor=processor,
            cache_dir=cache_dir,
            _recursive_=False,
        ).to(device)
    finally:
        torch.set_default_dtype(default_dtype)

    print(f"  load state dict: {ckpt_path}", flush=True)
    state_dict = torch.load(ckpt_path, map_location="cpu")
    if isinstance(state_dict, dict) and "state_dict" in state_dict:
        state_dict = state_dict["state_dict"]
    missing, unexpected = model.load_state_dict(state_dict, strict=False)
    if missing:
        print(f"  WARN: missing keys in state_dict (n={len(missing)})", flush=True)
        for k in missing[:5]:
            print(f"        {k}", flush=True)
    if unexpected:
        print(f"  WARN: unexpected keys in state_dict (n={len(unexpected)})", flush=True)
        for k in unexpected[:5]:
            print(f"        {k}", flush=True)

    model.eval()
    return model, tokenizer, cfg


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------


def _ade_fde(pred: np.ndarray, gt: np.ndarray) -> tuple[float, float]:
    """L2 displacement averaged over the trajectory (ADE) and final-step
    displacement (FDE), in meters."""
    n = min(len(pred), len(gt))
    if n == 0:
        return float("nan"), float("nan")
    d = np.linalg.norm(pred[:n] - gt[:n], axis=-1)
    return float(d.mean()), float(d[-1])


def _implied_speed(wps: np.ndarray, wp_freq: int = 5, carla_fps: int = 20) -> float:
    """Same as upstream PID: distance between wps half a second apart, *2."""
    one_sec = int(carla_fps // wp_freq)
    half_sec = one_sec // 2
    if len(wps) <= one_sec:
        return float("nan")
    return float(np.linalg.norm(wps[half_sec - 2] - wps[one_sec - 2]) * 2.0)


# ---------------------------------------------------------------------------
# Public entrypoint (called from modal_app.py::run)
# ---------------------------------------------------------------------------


def run_inference(
    *,
    ckpt_path: str,
    hydra_cfg_path: str,
    data_root: str,
    out_dir: str,
    num_samples: int,
    frame_stride: int,
    use_cot: bool,
    save_overlays: bool,
    seed: int,
) -> dict[str, Any]:
    sys.path.insert(0, "/opt/simlingo")
    _ensure_internvl_pretrained_symlink()

    # Heavy imports happen after we've patched sys.path + cwd.
    from simlingo_training.utils.internvl2_utils import get_num_image_tokens_per_patch

    from . import viz  # local

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f">> device: {device}", flush=True)
    print(">> loading model", flush=True)
    t0 = time.time()
    model, tokenizer, cfg = _load_model(ckpt_path, hydra_cfg_path, device)
    print(f"   loaded in {time.time() - t0:.1f}s", flush=True)

    use_global_img = bool(cfg.data_module.use_global_img)
    # Upstream `DataModule` and `team_code/agent_simlingo.py` both hardcode
    # NUM_IMAGE_PATCHES=2 here regardless of `use_global_img` (the thumbnail,
    # when present, is consumed inside the vision encoder, not surfaced as
    # extra context tokens in the prompt).
    NUM_IMAGE_PATCHES = 2
    num_image_tokens_total = (
        get_num_image_tokens_per_patch(cfg.model.vision_model.variant) * NUM_IMAGE_PATCHES
    )

    print(">> collecting samples", flush=True)
    samples = _collect_samples(
        data_root,
        num_samples=num_samples,
        frame_stride=frame_stride,
        seed=seed,
    )
    print(f"   collected {len(samples)} samples", flush=True)

    out_dir_p = Path(out_dir)
    out_dir_p.mkdir(parents=True, exist_ok=True)
    overlays_dir = out_dir_p / "overlays"
    if save_overlays:
        overlays_dir.mkdir(exist_ok=True)

    per_sample: list[dict] = []
    n_written = 0
    for i, s in enumerate(samples):
        try:
            measurements_now_to_future = []
            for fi in range(s.frame_idx, s.frame_idx + 12):
                measurements_now_to_future.append(
                    _load_json_gz(s.measurements_dir / f"{fi:04d}.json.gz")
                )
            current = measurements_now_to_future[0]

            speed_mps = float(current["speed"])
            target_point = np.asarray(current["target_point"], dtype=np.float32)
            next_target_point = np.asarray(current["target_point_next"], dtype=np.float32)
            target_points = np.stack([target_point, next_target_point], axis=0)

            gt_wps = _compute_waypoints_from_ego_matrix(measurements_now_to_future)
            gt_path = _equal_spacing_route(
                np.asarray(current["route"], dtype=np.float32)
            )

            pixel_values, HW = _process_image(s.rgb_path, use_global_img=use_global_img)

            prompt_text = _build_prompt(speed_mps, use_cot=use_cot)
            prompt_ll, prompt_inf_ll = _build_language_label(
                prompt_text,
                target_points_np=target_points,
                tokenizer=tokenizer,
                encoder_variant=cfg.model.vision_model.variant,
                num_image_tokens_total=num_image_tokens_total,
                device=device,
            )

            driving_input = _build_driving_input(
                pixel_values,
                speed_mps=speed_mps,
                target_points_np=target_points,
                prompt_ll=prompt_ll,
                prompt_inference_ll=prompt_inf_ll,
                HW=HW,
                device=device,
            )

            with torch.inference_mode():
                speed_wps, route_wps, language = model(driving_input)

            pred_wps = speed_wps[0].float().cpu().numpy() if speed_wps is not None else None
            # Upstream `predict_step` resamples the predicted route onto a 1m
            # grid before computing metrics — replicate that so ADE/FDE on the
            # path is comparable to the dataset's `route_adjusted` GT.
            pred_route = (
                _equal_spacing_route(route_wps[0].float().cpu().numpy(), num=20)
                if route_wps is not None
                else None
            )
            pred_text = language[0] if language else ""

            ade_wp, fde_wp = (
                _ade_fde(pred_wps, gt_wps)
                if pred_wps is not None
                else (float("nan"), float("nan"))
            )
            ade_path, fde_path = (
                _ade_fde(pred_route, gt_path)
                if pred_route is not None
                else (float("nan"), float("nan"))
            )

            gt_commentary = None
            if s.commentary_path is not None:
                try:
                    gt_commentary = _load_json_gz(s.commentary_path).get("commentary")
                except Exception:
                    gt_commentary = None

            entry = {
                "route": str(s.route_dir),
                "frame": s.frame_idx,
                "speed_mps": speed_mps,
                "target_point": target_point.tolist(),
                "next_target_point": next_target_point.tolist(),
                "ade_wp": ade_wp,
                "fde_wp": fde_wp,
                "ade_path": ade_path,
                "fde_path": fde_path,
                "implied_speed_pred": _implied_speed(pred_wps) if pred_wps is not None else None,
                "implied_speed_gt": _implied_speed(gt_wps),
                "pred_commentary": pred_text,
                "gt_commentary": gt_commentary,
                "prompt": prompt_text,
            }
            per_sample.append(entry)

            if save_overlays:
                overlay_path = overlays_dir / f"{i:04d}.png"
                viz.write_overlay(
                    rgb_path=s.rgb_path,
                    pred_wps=pred_wps,
                    pred_route=pred_route,
                    gt_wps=gt_wps,
                    gt_route=gt_path,
                    pred_text=pred_text,
                    gt_text=gt_commentary,
                    out_path=overlay_path,
                )
                n_written += 1

            if (i + 1) % 5 == 0 or i == len(samples) - 1:
                print(
                    f"  [{i + 1}/{len(samples)}] ADE_wp={ade_wp:.2f} FDE_wp={fde_wp:.2f}"
                    f"  ADE_path={ade_path:.2f} pred='{pred_text[:60]}'",
                    flush=True,
                )

        except Exception as exc:
            print(f"  [{i}] ERROR: {exc!r}", flush=True)
            per_sample.append({"error": repr(exc), "route": str(s.route_dir), "frame": s.frame_idx})

    metrics = _aggregate(per_sample)
    summary = {
        "metrics": metrics,
        "num_samples": len(samples),
        "num_written": n_written,
        "out_dir": str(out_dir_p),
        "ckpt": ckpt_path,
    }
    with open(out_dir_p / "predictions.json", "w") as fh:
        json.dump({"summary": summary, "per_sample": per_sample}, fh, indent=2, default=str)
    return summary


def _aggregate(rows: Iterable[dict]) -> dict[str, float]:
    keys = ["ade_wp", "fde_wp", "ade_path", "fde_path"]
    agg: dict[str, float] = {}
    for k in keys:
        vals = [r[k] for r in rows if isinstance(r.get(k), float) and not math.isnan(r[k])]
        agg[k] = float(np.mean(vals)) if vals else float("nan")
        agg[f"{k}_p50"] = float(np.median(vals)) if vals else float("nan")
        agg[f"{k}_p90"] = float(np.percentile(vals, 90)) if vals else float("nan")
    agg["num_evaluated"] = float(sum(1 for r in rows if "ade_wp" in r))
    return agg
