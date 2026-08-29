"""Interactive laser-spot picking for camera extrinsic calibration.

Shows each photo 2x magnified in an OpenCV window. Click the laser spot
(ignore the small crosshair dot baked into the centre of the frame). Keys:
  left click : mark / re-mark the spot
  n / space  : next photo
  b          : previous photo
  q / esc    : save and quit
Marks are saved to calibration_capture/laser_spot_marks.json as
{filename: [x, y]} in ORIGINAL image coordinates (the click is divided by
the zoom factor). Candidates found by bright-blob detection are drawn as
red circles for reference; the click wins.
"""
# SPDX-License-Identifier: MIT
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent.parent
DEFAULT_DIR = HERE / "calibration_capture" / "laser_spot"
MARKS_OUT = HERE / "calibration_capture" / "laser_spot_marks.json"
ZOOM = 2


def detect_candidates(gray, cx, cy):
    _, bw = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    n, _, stats, cent = cv2.connectedComponentsWithStats(bw, 8)
    out = []
    for i in range(1, n):
        x, y = float(cent[i, 0]), float(cent[i, 1])
        area = int(stats[i, cv2.CC_STAT_AREA])
        if 2 <= area <= 200 and (x - cx) ** 2 + (y - cy) ** 2 > 25 ** 2:
            out.append((int(x), int(y)))
    return out[:5]


def main() -> int:
    photo_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_DIR
    photos = sorted(photo_dir.glob("*.jpg"))
    photos = [p for p in photos if not p.name.endswith(".thumb.jpg")]
    if not photos:
        print(f"no photos in {photo_dir}")
        return 1

    marks: dict[str, list[int]] = {}
    if MARKS_OUT.exists():
        marks = json.loads(MARKS_OUT.read_text(encoding="utf-8"))

    idx = 0
    current: tuple[int, int] | None = None
    win = "laser spot picker"

    def mouse(event, x, y, _flags, _param):
        nonlocal current
        if event == cv2.EVENT_LBUTTONDOWN:
            current = (int(x / ZOOM), int(y / ZOOM))

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, mouse)

    while True:
        path = photos[idx]
        img = cv2.imread(str(path))
        if img is None:
            print(f"unreadable {path.name}, skipping")
            idx += 1
            continue
        h, w = img.shape[:2]
        current = None
        key = -1
        while key not in (ord("n"), ord(" "), ord("b"), ord("q"), 27):
            disp = cv2.resize(img, (w * ZOOM, h * ZOOM), interpolation=cv2.INTER_NEAREST)
            for cand_x, cand_y in detect_candidates(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), w / 2, h / 2):
                cv2.circle(disp, (cand_x * ZOOM, cand_y * ZOOM), 8 * ZOOM, (0, 0, 255), 2)
            if current:
                cv2.circle(disp, (current[0] * ZOOM, current[1] * ZOOM), 5 * ZOOM, (0, 255, 0), 3)
            title = f"{idx + 1}/{len(photos)} {path.name}  (click spot, n=next b=prev q=quit)"
            cv2.setWindowTitle(win, title)
            cv2.imshow(win, disp)
            key = cv2.waitKey(0) & 0xFF
            if key == ord("r"):
                current = None
        if current:
            marks[path.name] = list(current)
            print(f"  {path.name}: spot=({current[0]},{current[1]})")
        else:
            marks.pop(path.name, None)
            print(f"  {path.name}: no mark")
        if key in (ord("q"), 27):
            break
        idx += 1 if key in (ord("n"), ord(" ")) else -1
        idx = max(0, min(idx, len(photos) - 1))

    cv2.destroyAllWindows()
    MARKS_OUT.write_text(json.dumps(marks, indent=2), encoding="utf-8")
    print(f"\nSaved {len(marks)} marks to {MARKS_OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
