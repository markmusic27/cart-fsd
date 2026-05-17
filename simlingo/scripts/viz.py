"""Side-by-side overlays of GT vs predicted waypoints on the front-cam image.

We reuse the same camera intrinsics + extrinsics SimLingo trains with, so the
projection matches what the model "sees".
"""

from __future__ import annotations

import textwrap
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def _project_points_to_image(points: np.ndarray, K: np.ndarray) -> list[tuple[int, int]]:
    """Project (x_forward, y_right) ego-frame points to image pixels.

    Matches `team_code/simlingo_utils.py::project_points`: the rear-axle
    extrinsic is folded into a fixed tvec, no rotation.
    """
    tvec = np.array([[0.0, 2.0, 1.5]], dtype=np.float32)  # camera ~2m up, 1.5m back
    rvec = np.zeros((3, 1), dtype=np.float32)
    dist = np.zeros((5, 1), dtype=np.float32)
    pixels: list[tuple[int, int]] = []
    for x_fwd, y_right in points:
        # World-to-camera convention used upstream: (X=y, Y=0, Z=x+offset).
        pos_3d = np.array([y_right, 0.0, x_fwd + tvec[0][2]], dtype=np.float32)
        p2d, _ = cv2.projectPoints(pos_3d, rvec=rvec, tvec=tvec, cameraMatrix=K, distCoeffs=dist)
        pixels.append((int(p2d[0][0][0]), int(p2d[0][0][1])))
    return pixels


def _camera_intrinsics(w: int, h: int, fov_deg: float = 110.0) -> np.ndarray:
    focal = w / (2.0 * np.tan(fov_deg * np.pi / 360.0))
    K = np.eye(3, dtype=np.float32)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def _draw_polyline(
    draw: ImageDraw.ImageDraw,
    pixels: list[tuple[int, int]],
    color: tuple[int, int, int],
    radius: int = 3,
) -> None:
    for x, y in pixels:
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=color)


def _safe_font(size: int = 14) -> ImageFont.ImageFont:
    # The base image we'll Pillow-draw on inside Modal won't have arial; fall
    # back to the default bitmap font which is fine for a sanity check.
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", size)
    except Exception:
        return ImageFont.load_default()


def write_overlay(
    *,
    rgb_path: Path,
    pred_wps: np.ndarray | None,
    pred_route: np.ndarray | None,
    gt_wps: np.ndarray | None,
    gt_route: np.ndarray | None,
    pred_text: str,
    gt_text: str | None,
    out_path: Path,
) -> None:
    bgr = cv2.imread(str(rgb_path))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    # Match the training-time crop so projection lands in the same image space.
    h = rgb.shape[0]
    crop_h = int(h - (h * 4.8) // 16)
    rgb = rgb[:crop_h, :, :]

    H, W, _ = rgb.shape
    K = _camera_intrinsics(W, H, 110.0)

    image = Image.fromarray(rgb)
    draw = ImageDraw.Draw(image)

    if gt_route is not None:
        _draw_polyline(draw, _project_points_to_image(gt_route, K), (0, 180, 0), radius=2)
    if gt_wps is not None:
        _draw_polyline(draw, _project_points_to_image(gt_wps, K), (0, 255, 0), radius=3)
    if pred_route is not None:
        _draw_polyline(draw, _project_points_to_image(pred_route, K), (180, 0, 0), radius=2)
    if pred_wps is not None:
        _draw_polyline(draw, _project_points_to_image(pred_wps, K), (255, 64, 64), radius=3)

    # Caption: predicted commentary on top of a black strip for readability.
    caption_height = 120
    canvas = Image.new("RGB", (W, H + caption_height), (0, 0, 0))
    canvas.paste(image, (0, 0))
    caption_draw = ImageDraw.Draw(canvas)
    font = _safe_font(14)
    y = H + 6
    for label, text, color in [
        ("PRED", pred_text or "<no language>", (255, 200, 200)),
        ("GT  ", gt_text or "<no commentary annotation>", (200, 255, 200)),
    ]:
        wrapped = textwrap.wrap(f"{label}: {text}", width=max(1, W // 8))
        for line in wrapped[:3]:
            caption_draw.text((6, y), line, fill=color, font=font)
            y += 16
        y += 2

    out_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_path)
