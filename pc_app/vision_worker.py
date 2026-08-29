"""视觉 Worker:设备照片 -> AI 检测(门/窗/插座)-> ROI 几何边缘提取 -> JSON。

运行环境:Python 3.12 虚拟环境(onnxruntime + cv2 + numpy)。
用法:
    python pc_app\\vision_worker.py <image.jpg> [--out result.json]
    python pc_app\\vision_worker.py <image.jpg> [--model <model.onnx>] [--conf 0.25] [--iou 0.45]

输出 JSON schema:
{
  "image": "...", "width": .., "height": .., "model": "model.onnx",
  "inference_ms": .., "geometry_ms": ..,
  "detections": [{
     "object_id", "class_id", "class_name", "confidence",
     "bbox_xyxy": [x1,y1,x2,y2], "mask_bbox_xyxy": [...],
     "corners_px": [[x,y]x4]  (LT,RT,RB,LB) | null,
     "corner_confidence": [4] | null,
     "sides": [...], "supported_sides", "quality", "method", "roi_xyxy"
  }]
}
"""

# SPDX-License-Identifier: MIT
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

if getattr(sys, "frozen", False):  # PyInstaller:数据文件在 _MEIPASS
    ROOT = Path(sys._MEIPASS)
else:
    ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "door_window_ai" / "src"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from door_window_ai.inference.onnx_backend import ONNXVisionBackend  # noqa: E402
from door_window_ai.refinement.quad_edges import (  # noqa: E402
    extract_quad_edges,
    mask_bbox,
    quad_to_json,
)

DEFAULT_MODEL = ROOT / "door_window_ai" / "models" / "door_window_outlet_v3" / "model.onnx"
V3_CLASS_NAMES = {0: "door", 1: "window", 2: "outlet"}


def analyze_image(
    image_bgr: np.ndarray,
    backend: ONNXVisionBackend,
    *,
    confidence: float,
) -> list[dict]:
    objects = backend.infer(image_bgr)
    results: list[dict] = []
    for obj in objects:
        if float(obj.confidence) < confidence:
            continue
        mask_bbox_xyxy = mask_bbox(obj.mask, obj.bbox_xyxy)
        quad = extract_quad_edges(image_bgr, mask_bbox_xyxy, mask=obj.mask)
        entry: dict = {
            "object_id": obj.object_id,
            "class_id": int(obj.class_id),
            "class_name": str(obj.class_name),
            "confidence": round(float(obj.confidence), 4),
            "bbox_xyxy": [float(v) for v in obj.bbox_xyxy],
            "mask_bbox_xyxy": [float(v) for v in mask_bbox_xyxy],
            **quad_to_json(quad),
        }
        results.append(entry)
    results.sort(key=lambda item: item["confidence"], reverse=True)
    return results


def main() -> int:
    parser = argparse.ArgumentParser(description="Laser meter vision worker")
    parser.add_argument("image", type=Path)
    parser.add_argument("--out", type=Path, default=None, help="JSON 输出文件(缺省打印到 stdout)")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--input-size", type=int, default=1024)
    args = parser.parse_args()

    if not args.image.is_file():
        print(json.dumps({"ok": False, "error": f"image not found: {args.image}"}))
        return 1
    if not args.model.is_file():
        print(json.dumps({"ok": False, "error": f"model not found: {args.model}"}))
        return 1

    image = cv2.imread(str(args.image))
    if image is None:
        print(json.dumps({"ok": False, "error": f"cannot decode image: {args.image}"}))
        return 1

    backend = ONNXVisionBackend(
        args.model,
        class_names=V3_CLASS_NAMES,
        input_size=args.input_size,
        confidence_threshold=args.conf,
        iou_threshold=args.iou,
    )

    t0 = time.perf_counter()
    detections = analyze_image(image, backend, confidence=args.conf)
    geometry_ms = (time.perf_counter() - t0) * 1000.0

    payload = {
        "ok": True,
        "image": str(args.image),
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "model": args.model.name,
        "confidence_threshold": args.conf,
        "geometry_ms": round(geometry_ms, 1),
        "detection_count": len(detections),
        "detections": detections,
    }

    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
