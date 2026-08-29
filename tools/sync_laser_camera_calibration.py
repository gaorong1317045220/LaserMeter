# SPDX-License-Identifier: MIT
"""Generate the firmware laser/camera calibration header from PC JSON data."""
from __future__ import annotations

import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
CALIBRATION = ROOT / "calibration_capture"
INTRINSICS = CALIBRATION / "camera_intrinsics.json"
EXTRINSICS = CALIBRATION / "laser_camera_extrinsics.json"
OUTPUT = ROOT / "main" / "laser_camera_calibration.h"


def finite(value: object, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def vector(data: dict, key: str, length: int) -> list[float]:
    values = data.get(key)
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f"{key} must contain {length} values")
    return [finite(value, f"{key}[{index}]") for index, value in enumerate(values)]


def c_float(value: float) -> str:
    text = f"{value:.15g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return f"{text}f"


def main() -> int:
    intrinsics = json.loads(INTRINSICS.read_text(encoding="utf-8"))
    extrinsics = json.loads(EXTRINSICS.read_text(encoding="utf-8"))
    camera_matrix = intrinsics.get("camera_matrix")
    if not isinstance(camera_matrix, list) or len(camera_matrix) != 3:
        raise ValueError("camera_matrix must be 3x3")
    if any(not isinstance(row, list) or len(row) != 3 for row in camera_matrix):
        raise ValueError("camera_matrix must be 3x3")
    image_size = intrinsics.get("image_size")
    if not isinstance(image_size, list) or len(image_size) != 2:
        raise ValueError("image_size must contain width and height")
    width = finite(image_size[0], "image_size[0]")
    height = finite(image_size[1], "image_size[1]")
    if width <= 0 or height <= 0:
        raise ValueError("image_size must be positive")
    dist = vector(intrinsics, "dist_coeffs", 5)
    direction = vector(extrinsics, "laser_dir_cam", 3)
    origin = vector(extrinsics, "p0_cam_m", 3)
    frames_used = int(extrinsics.get("frames_used", 0))
    rms_px = finite(extrinsics.get("rms_px", 0.0), "rms_px")

    lines = [
        "// SPDX-License-Identifier: MIT",
        "#pragma once",
        "",
        "// Generated from calibration_capture/camera_intrinsics.json and",
        "// calibration_capture/laser_camera_extrinsics.json.",
        "// Run tools/sync_laser_camera_calibration.py after PC re-calibration.",
        f"#define LASER_CAMERA_CALIBRATION_VALID 1",
        f"#define LASER_CAMERA_CALIBRATION_FRAMES_USED {frames_used}",
        f"#define LASER_CAMERA_CALIBRATION_RMS_PX {c_float(rms_px)}",
        "",
        "// Camera intrinsics are expressed at the calibration image size.",
        f"#define LASER_CAMERA_CALIBRATION_IMAGE_WIDTH_PX {c_float(width)}",
        f"#define LASER_CAMERA_CALIBRATION_IMAGE_HEIGHT_PX {c_float(height)}",
        f"#define LASER_CAMERA_CALIBRATION_FX_PX {c_float(finite(camera_matrix[0][0], 'camera_matrix[0][0]'))}",
        f"#define LASER_CAMERA_CALIBRATION_FY_PX {c_float(finite(camera_matrix[1][1], 'camera_matrix[1][1]'))}",
        f"#define LASER_CAMERA_CALIBRATION_CX_PX {c_float(finite(camera_matrix[0][2], 'camera_matrix[0][2]'))}",
        f"#define LASER_CAMERA_CALIBRATION_CY_PX {c_float(finite(camera_matrix[1][2], 'camera_matrix[1][2]'))}",
        f"#define LASER_CAMERA_CALIBRATION_DIST_K1 {c_float(dist[0])}",
        f"#define LASER_CAMERA_CALIBRATION_DIST_K2 {c_float(dist[1])}",
        f"#define LASER_CAMERA_CALIBRATION_DIST_P1 {c_float(dist[2])}",
        f"#define LASER_CAMERA_CALIBRATION_DIST_P2 {c_float(dist[3])}",
        f"#define LASER_CAMERA_CALIBRATION_DIST_K3 {c_float(dist[4])}",
        "",
        "// Laser ray in the camera frame: P_cam = P0 + dir * range_m.",
        f"#define LASER_CAMERA_DIR_CAM_X {c_float(direction[0])}",
        f"#define LASER_CAMERA_DIR_CAM_Y {c_float(direction[1])}",
        f"#define LASER_CAMERA_DIR_CAM_Z {c_float(direction[2])}",
        f"#define LASER_CAMERA_P0_CAM_X {c_float(origin[0])}",
        f"#define LASER_CAMERA_P0_CAM_Y {c_float(origin[1])}",
        f"#define LASER_CAMERA_P0_CAM_Z {c_float(origin[2])}",
        "",
    ]
    content = "\n".join(lines)
    if not OUTPUT.exists() or OUTPUT.read_text(encoding="utf-8") != content:
        OUTPUT.write_text(content, encoding="utf-8", newline="\n")
        print(f"Updated {OUTPUT}")
    else:
        print(f"Up to date: {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
