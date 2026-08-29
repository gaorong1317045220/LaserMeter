# SPDX-License-Identifier: MIT
"""Laser-camera extrinsic calibration from spot-on-wall photos.

Geometry: the laser emitter and camera are rigidly fixed to the device body.
For every single-distance photo we know the RAW laser range (from
measurement_records_v2.csv, un-doing the reference offset). The laser spot in
body frame is P_i = LASER_ORIGIN + LASER_DIR * range_i, and the camera
projection is u = K * (R_cb * P_i + t_cb), where (R_cb, t_cb) is the camera
pose relative to the laser emitter. Only 4 DOF are observable from a single
ray family: the laser direction in camera frame (2 angles) and the emitter
offset perpendicular to it (2 components). The two unobservable DOF (rotation
about the laser axis, translation along it) are fixed to zero, which is
exactly what spot-on-photo annotation needs.

Usage:
  python tools/calibrate_camera_extrinsics.py [photo_dir] [--csv path] [--auto]

photo_dir defaults to calibration_capture/laser_spot. Each photo must be
matched to a measurement record by its image_path basename (single_xxx.jpg /
p2p_a_xxx.jpg). --csv overrides the records file (default pc_app/data/
measurement_records_v2.csv). With --auto the brightest small blob is used;
otherwise candidates are printed and you pick one per photo.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

# --- device constants, mirrored from firmware -------------------------------
LASER_ORIGIN_M = np.array([0.0145, 0.0486527, -0.016256])   # FUSION_LASER_OFFSET
LASER_DIR = np.array([0.0, 1.0, 0.0])                        # FUSION_LASER_DIR
REFERENCE_OFFSET_MM = {0: 126.5, 1: -2.0, 2: 18.0}         # REAR/FRONT/TRIPOD (stored mm)

HERE = Path(__file__).resolve().parent.parent
CALIB_DIR = HERE / "calibration_capture"
DEFAULT_PHOTO_DIR = CALIB_DIR / "laser_spot"
DEFAULT_CSV = HERE / "pc_app" / "data" / "measurement_records_v2.csv"
MARKS = CALIB_DIR / "laser_spot_marks.json"
OUT = CALIB_DIR / "laser_camera_extrinsics.json"


def load_camera_intrinsics() -> dict:
    path = CALIB_DIR / "camera_intrinsics.json"
    data = json.loads(path.read_text(encoding="utf-8"))
    K = np.array(data["camera_matrix"], dtype=np.float64)
    dist = np.array(data["dist_coeffs"], dtype=np.float64)
    return {"K": K, "dist": dist, "size": data["image_size"]}


def load_ranges(csv_path: Path) -> dict[str, float]:
    """Map photo basename -> RAW laser range in meters."""
    out: dict[str, float] = {}
    if not csv_path.exists():
        return out
    with open(csv_path, encoding="utf-8", errors="replace") as fh:
        header = None
        for line in fh:
            fields = [f.strip() for f in line.rstrip("\n").split(",")]
            if fields[0] == "op" or fields[0].startswith("#"):
                header = fields
                continue
            if not fields or fields[0] != "SAVE" or len(fields) < 13:
                continue
            image = fields[9]
            if image == "-" or not image:
                continue
            recorded_mm = float(fields[3])
            reference = int(fields[4])
            offset_mm = REFERENCE_OFFSET_MM.get(reference, 0.0)
            raw_mm = recorded_mm - offset_mm
            out[Path(image).name] = raw_mm / 1000.0
    return out


def detect_spot_candidates(img_bgr, max_candidates: int = 8):
    """Return list of (x, y, area, mean_intensity) of small bright blobs."""
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    n, _, stats, cent = cv2.connectedComponentsWithStats(bw, 8)
    cands = []
    for i in range(1, n):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if 2 <= area <= 120:
            x, y = float(cent[i, 0]), float(cent[i, 1])
            region = gray[max(0, int(y) - 4):int(y) + 5, max(0, int(x) - 4):int(x) + 5]
            mean = float(region.mean()) if region.size else 0.0
            cands.append((x, y, area, mean))
    cands.sort(key=lambda c: -c[3])
    return cands[:max_candidates]


def pick_spot(img_bgr, path: Path, auto: bool) -> tuple[float, float]:
    cands = detect_spot_candidates(img_bgr)
    if not cands:
        print(f"  ! {path.name}: no bright blob found")
        return None
    if auto:
        x, y, _, _ = cands[0]
        print(f"  {path.name}: auto picked brightest blob ({x:.1f},{y:.1f})")
        return x, y
    print(f"  {path.name}: candidates:")
    for i, (x, y, area, mean) in enumerate(cands):
        print(f"    [{i}] ({x:.1f},{y:.1f}) area={area} mean={mean:.0f}")
    while True:
        try:
            choice = input("    pick index (empty=first): ").strip()
            idx = int(choice) if choice else 0
            if 0 <= idx < len(cands):
                return cands[idx][0], cands[idx][1]
        except ValueError:
            pass


def project(K, dist, p_cam):
    """OpenCV-style projection of one 3D camera-frame point (with distortion)."""
    p = np.asarray(p_cam, dtype=np.float64).reshape(3)
    x, y, z = p
    xn, yn = x / z, y / z
    r2 = xn * xn + yn * yn
    k1, k2, p1, p2, k3 = dist
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    return np.array([K[0, 0] * xd + K[0, 2], K[1, 1] * yd + K[1, 2]])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("photo_dir", nargs="?", type=Path, default=DEFAULT_PHOTO_DIR)
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    ap.add_argument("--marks", type=Path, default=MARKS,
                    help="JSON {filename: [x, y]} of manually picked spots")
    ap.add_argument("--auto", action="store_true", help="use brightest blob without prompting")
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    if not args.photo_dir.is_dir():
        print(f"FAIL: photo dir not found: {args.photo_dir}")
        return 1
    intrinsics = load_camera_intrinsics()
    K, dist = intrinsics["K"], intrinsics["dist"]
    ranges = load_ranges(args.csv)
    print(f"Loaded {len(ranges)} measurement records with photos")
    hand_marks: dict = {}
    if args.marks.exists():
        hand_marks = json.loads(args.marks.read_text(encoding="utf-8"))
        print(f"Loaded {len(hand_marks)} hand-picked marks from {args.marks}")

    observations = []  # (range_m, u, v)
    for path in sorted(args.photo_dir.glob("*.jpg")):
        if path.name.endswith(".thumb.jpg"):
            continue
        if path.name not in ranges:
            print(f"  ! {path.name}: no matching measurement record, skipped")
            continue
        img = cv2.imread(str(path))
        if img is None:
            continue
        h, w = img.shape[:2]
        iw, ih = intrinsics["size"]
        if (w, h) != (iw, ih):
            sx, sy = w / iw, h / ih
            Kc = K.copy()
            Kc[0, 0] *= sx; Kc[1, 1] *= sy; Kc[0, 2] *= sx; Kc[1, 2] *= sy
            Kc, dist_c = Kc, dist
        else:
            Kc, dist_c = K, dist
        spot = None
        if path.name in hand_marks:
            spot = tuple(hand_marks[path.name])
            print(f"  {path.name}: using hand mark {spot}")
        elif hand_marks:
            # 有人工标记文件:未标记的照片视为用户明确跳过,不再自动检测/交互
            print(f"  {path.name}: no hand mark, skipped")
            continue
        if spot is None:
            spot = pick_spot(img, path, args.auto)
        if spot is None:
            continue
        observations.append((ranges[path.name], spot[0], spot[1], Kc, dist_c, path.name))
        print(f"  {path.name}: range={ranges[path.name]:.3f} m spot=({spot[0]:.1f},{spot[1]:.1f})")

    if len(observations) < 2:
        print(f"FAIL: need >= 2 usable photos, got {len(observations)}")
        return 1

    # --- initial guess ------------------------------------------------------
    # Far observations converge to the laser direction in camera frame.
    far = max(observations, key=lambda o: o[0])
    n_far = np.linalg.inv(far[3]) @ np.array([far[1], far[2], 1.0])
    dir_cam0 = n_far / np.linalg.norm(n_far)
    theta0 = float(np.arccos(np.clip(dir_cam0[2], -1.0, 1.0)))
    phi0 = float(np.arctan2(dir_cam0[1], dir_cam0[0]))
    # emitter offset guess from the nearest observation
    near = min(observations, key=lambda o: o[0])
    n_near = np.linalg.inv(near[3]) @ np.array([near[1], near[2], 1.0])
    n_near /= np.linalg.norm(n_near)
    p0_guess = near[0] * (n_near - dir_cam0)
    # perpendicular components relative to dir_cam0
    e1 = np.cross(dir_cam0, [0, 0, 1.0]); e1 /= np.linalg.norm(e1)
    e2 = np.cross(dir_cam0, e1); e2 /= np.linalg.norm(e2)
    a0 = float(np.dot(p0_guess, e1))
    b0 = float(np.dot(p0_guess, e2))
    x0 = np.array([theta0, phi0, a0, b0])

    def dir_from(theta, phi):
        return np.array([np.sin(theta) * np.cos(phi),
                         np.sin(theta) * np.sin(phi),
                         np.cos(theta)])

    def residual(x):
        theta, phi, a, b = x
        d = dir_from(theta, phi)
        e1 = np.cross(d, [0, 0, 1.0]); n1 = np.linalg.norm(e1)
        if n1 < 1e-9:
            e1 = np.array([1.0, 0.0, 0.0])
        else:
            e1 /= n1
        e2 = np.cross(d, e1); e2 /= np.linalg.norm(e2)
        p0 = a * e1 + b * e2
        res = []
        for (range_m, u, v, Kc, dist_c, _name) in observations:
            p_cam = p0 + d * range_m
            pu = project(Kc, dist_c, p_cam)
            res.append(pu[0] - u)
            res.append(pu[1] - v)
        return np.array(res)

    sol = least_squares(residual, x0, method="lm", max_nfev=4000)
    theta, phi, a, b = sol.x
    d_cam = dir_from(theta, phi)
    e1 = np.cross(d_cam, [0, 0, 1.0])
    if np.linalg.norm(e1) < 1e-9:
        e1 = np.array([1.0, 0.0, 0.0])
    else:
        e1 /= np.linalg.norm(e1)
    e2 = np.cross(d_cam, e1); e2 /= np.linalg.norm(e2)
    p0_cam = a * e1 + b * e2

    # Build full 6-DOF: choose the rotation that maps body +Y onto d_cam and
    # keeps zero roll about the laser axis (unobservable, fixed).
    y_body = np.array([0.0, 1.0, 0.0])
    axis = np.cross(y_body, d_cam)
    axis_n = np.linalg.norm(axis)
    if axis_n < 1e-12:
        R_cb = np.eye(3) if d_cam[1] > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        axis /= axis_n
        ang = np.arccos(np.clip(np.dot(y_body, d_cam), -1.0, 1.0))
        Kx = np.array([[0, -axis[2], axis[1]],
                       [axis[2], 0, -axis[0]],
                       [-axis[1], axis[0], 0]])
        R_cb = np.eye(3) + np.sin(ang) * Kx + (1 - np.cos(ang)) * (Kx @ Kx)
    # t_cb = p0_cam - R_cb @ LASER_ORIGIN_M (translation along ray fixed to 0)
    t_cb = p0_cam - R_cb @ LASER_ORIGIN_M

    # report residuals
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    print(f"\nOptimized: RMS reprojection = {rms:.3f} px over {len(observations)} photos")
    print(f"  laser dir in camera frame : {d_cam}")
    print(f"  camera origin rel. emitter : {t_cb} m")
    print(f"  R_cb =\n{R_cb}")

    out = {
        "ok": True,
        "frames_used": len(observations),
        "frames": [o[5] for o in observations],
        "rms_px": rms,
        "laser_dir_cam": d_cam.tolist(),
        "t_cam_from_emitter_m": t_cb.tolist(),
        "R_cam_body": R_cb.tolist(),
        "p0_cam_m": p0_cam.tolist(),
        "notes": ("Camera pose relative to laser emitter (body frame). "
                  "Rotation about laser axis and translation along it are "
                  "unobservable and fixed; spot annotation is unaffected."),
    }
    args.out.write_text(json.dumps(out, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\nSaved: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
