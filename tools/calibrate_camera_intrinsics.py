# SPDX-License-Identifier: MIT
"""Camera intrinsic calibration from checkerboard photos.

Board: 11x8 inner corners (10x7 squares). Square size must be measured on the
printed board; adjust SQUARE_MM below. Outputs calibration.json + a report.
"""
from __future__ import annotations

import glob
import json
import sys
from pathlib import Path

import cv2
import numpy as np

SQUARE_MM = 25.0  # TODO: measure the printed checkerboard square size!
PATTERN = (11, 8)  # inner corners (cols, rows)

HERE = Path(__file__).resolve().parent.parent
PHOTO_DIR = HERE / "calibration_capture" / "checkerboard_latest" / "ds_recent"
OUT = HERE / "calibration_capture" / "camera_intrinsics.json"


def main() -> int:
    files = sorted(glob.glob(str(PHOTO_DIR / "*.jpg")))
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    used: list[str] = []
    img_size = None

    objp = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : PATTERN[0], 0 : PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)

    for path in files:
        img = cv2.imread(str(path))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        img_size = (w, h)
        ret, corners = cv2.findChessboardCorners(gray, PATTERN, None)
        if not ret:
            print(f"  skip {Path(path).name}: no pattern")
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(objp)
        image_points.append(corners)
        used.append(Path(path).name)
        print(f"  use {Path(path).name}: {len(corners)} corners")

    if len(object_points) < 3:
        print(f"FAIL: only {len(object_points)} usable frames, need >= 3")
        return 1

    print(f"Calibrating with {len(object_points)} frames ...")
    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, img_size, None, None
    )

    print(f"\nRMS reprojection error: {rms:.3f} px")
    print("Camera matrix:")
    print(mtx)
    print("\nDistortion coefficients (k1 k2 p1 p2 k3):")
    print(dist.ravel())

    fx, fy = mtx[0, 0], mtx[1, 1]
    cx, cy = mtx[0, 2], mtx[1, 2]

    report = {
        "square_mm": SQUARE_MM,
        "pattern": {"cols": PATTERN[0], "rows": PATTERN[1]},
        "image_size": list(img_size),
        "frames_used": len(object_points),
        "frames": used,
        "rms_px": float(rms),
        "camera_matrix": mtx.tolist(),
        "dist_coeffs": dist.ravel().tolist(),
        "fov_deg": {
            "horizontal": float(2 * np.degrees(np.arctan2(img_size[0] / 2, fx))),
            "vertical": float(2 * np.degrees(np.arctan2(img_size[1] / 2, fy))),
        },
        "notes": "fx/fy in px; sensor crop 240x284 portrait view uses centre region",
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nSaved: {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
