# SPDX-License-Identifier: MIT
"""Offline ONNX Runtime backend for Ultralytics segmentation exports.

The parser itself depends only on NumPy, so it can be fixture-tested without
ONNX Runtime, OpenCV, a GPU, or a model file.
"""

from __future__ import annotations

import ast
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import json
import logging
from pathlib import Path
from typing import Any, Literal

import numpy as np
from PIL import Image

from door_window_ai.inference.base import validate_image_bgr
from door_window_ai.inference.postprocess import normalize_class_names
from door_window_ai.schemas.vision_object import CLASS_NAMES, VisionObject, validate_class_contract

LOGGER = logging.getLogger(__name__)

OnnxOutputFormat = Literal[
    "auto",
    "ultralytics_raw",
    "ultralytics_legacy",
    "yolo26_e2e",
]


@dataclass(frozen=True, slots=True)
class LetterboxTransform:
    """Coordinates required to undo model-input letterboxing."""

    original_shape: tuple[int, int]
    input_shape: tuple[int, int]
    scale: float
    pad_left: int
    pad_top: int
    resized_shape: tuple[int, int]

    @classmethod
    def identity(cls, shape: tuple[int, int]) -> "LetterboxTransform":
        height, width = int(shape[0]), int(shape[1])
        return cls((height, width), (height, width), 1.0, 0, 0, (height, width))


def require_onnxruntime() -> Any:
    try:
        import onnxruntime
    except ImportError as exc:
        raise RuntimeError(
            "ONNX Runtime is required for ONNX inference. "
            "Install it with `pip install -e .[onnx]`."
        ) from exc
    return onnxruntime


def _letterbox(
    image_bgr: np.ndarray,
    input_shape: tuple[int, int],
    *,
    dtype: np.dtype[Any] | type = np.float32,
) -> tuple[np.ndarray, LetterboxTransform]:
    original_h, original_w = image_bgr.shape[:2]
    input_h, input_w = input_shape
    scale = min(input_w / original_w, input_h / original_h)
    resized_w = max(1, int(round(original_w * scale)))
    resized_h = max(1, int(round(original_h * scale)))
    pad_left = (input_w - resized_w) // 2
    pad_top = (input_h - resized_h) // 2
    # Match Ultralytics LetterBox exactly. PIL bilinear uses different sampling
    # coordinates than OpenCV INTER_LINEAR; the difference is large enough to
    # change low-confidence E2E detections and therefore invalidates a strict
    # PT/ONNX numerical parity check.
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - declared ONNX dependency
        raise RuntimeError("ONNX inference requires OpenCV for Ultralytics-compatible preprocessing") from exc
    resized = cv2.resize(
        image_bgr, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR
    )
    pad_right = input_w - resized_w - pad_left
    pad_bottom = input_h - resized_h - pad_top
    canvas = cv2.copyMakeBorder(
        resized,
        pad_top,
        pad_bottom,
        pad_left,
        pad_right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    rgb = canvas[..., ::-1]
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=dtype) / 255.0
    transform = LetterboxTransform(
        original_shape=(original_h, original_w),
        input_shape=(input_h, input_w),
        scale=float(scale),
        pad_left=pad_left,
        pad_top=pad_top,
        resized_shape=(resized_h, resized_w),
    )
    return tensor, transform


def _as_output_arrays(
    outputs: Mapping[str, Any] | Sequence[Any] | np.ndarray,
) -> list[np.ndarray]:
    values: Sequence[Any]
    if isinstance(outputs, Mapping):
        values = list(outputs.values())
    elif isinstance(outputs, np.ndarray):
        values = [outputs]
    else:
        values = outputs
    arrays = [np.asarray(value) for value in values]
    if not arrays:
        raise ValueError("The ONNX model returned no outputs")
    return arrays


def _find_prediction_and_prototypes(
    outputs: Mapping[str, Any] | Sequence[Any] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    arrays = _as_output_arrays(outputs)
    prototype_candidates = [array for array in arrays if array.ndim == 4]
    prediction_candidates = [array for array in arrays if array.ndim in {2, 3}]
    if not prototype_candidates:
        raise ValueError(
            "No 4-D mask prototype output found. Expected an Ultralytics "
            "segmentation export with predictions and prototypes."
        )
    if not prediction_candidates:
        raise ValueError("No 2-D/3-D detection prediction output found")
    # The prototype output normally has the largest spatial plane; the prediction
    # output normally has the largest number of scalar values.
    proto = max(prototype_candidates, key=lambda value: value.shape[-1] * value.shape[-2])
    prediction = max(prediction_candidates, key=lambda value: value.size)
    if proto.shape[0] != 1:
        raise ValueError(f"Only batch size 1 is supported, got prototypes {proto.shape}")
    proto = np.asarray(proto[0], dtype=np.float32)
    # Support uncommon NHWC prototype exports as well as standard NCHW.
    if proto.shape[0] > 128 and proto.shape[-1] <= 128:
        proto = proto.transpose(2, 0, 1)
    if proto.ndim != 3:
        raise ValueError(f"Mask prototypes must normalize to (C,H,W), got {proto.shape}")
    return np.asarray(prediction, dtype=np.float32), proto


def _prediction_rows(
    prediction: np.ndarray,
    *,
    expected_feature_counts: set[int],
) -> np.ndarray:
    value = np.asarray(prediction, dtype=np.float32)
    if value.ndim == 3:
        if value.shape[0] != 1:
            raise ValueError(f"Only batch size 1 is supported, got predictions {value.shape}")
        value = value[0]
    if value.ndim != 2:
        raise ValueError(f"Predictions must normalize to a 2-D array, got {value.shape}")
    row_features = value.shape[1] in expected_feature_counts
    column_features = value.shape[0] in expected_feature_counts
    if row_features and not column_features:
        return value
    if column_features and not row_features:
        return value.T
    if row_features and column_features:
        # This is only plausible for a tiny synthetic fixture; favor rows-first.
        return value
    raise ValueError(
        f"Cannot identify the prediction feature axis in shape {value.shape}; "
        f"expected one axis in {sorted(expected_feature_counts)}"
    )


def _looks_like_yolo26_e2e(rows: np.ndarray, class_count: int) -> bool:
    """Distinguish E2E rows from raw rows when two classes make widths equal."""
    if rows.shape[0] == 0:
        return True
    scores = rows[:, 4]
    active = np.isfinite(scores) & (scores > 1e-7)
    sample = rows[active] if np.any(active) else rows
    sample = sample[: min(256, len(sample))]
    class_column = sample[:, 5]
    class_like = np.mean(
        np.isfinite(class_column)
        & (class_column >= 0)
        & (class_column < class_count)
        & np.isclose(class_column, np.rint(class_column), atol=1e-4)
    )
    xyxy_like = np.mean((sample[:, 2] >= sample[:, 0]) & (sample[:, 3] >= sample[:, 1]))
    score_like = np.mean((sample[:, 4] >= 0) & (sample[:, 4] <= 1))
    return bool(class_like >= 0.9 and xyxy_like >= 0.8 and score_like >= 0.9)


def _resolve_output_format(
    rows: np.ndarray,
    *,
    requested: OnnxOutputFormat,
    class_count: int,
    mask_count: int,
) -> OnnxOutputFormat:
    valid = {"auto", "ultralytics_raw", "ultralytics_legacy", "yolo26_e2e"}
    if requested not in valid:
        raise ValueError(f"output_format must be one of {sorted(valid)}, got {requested!r}")
    feature_count = rows.shape[1]
    raw_count = 4 + class_count + mask_count
    legacy_count = 5 + class_count + mask_count
    e2e_count = 6 + mask_count
    if requested != "auto":
        expected = {
            "ultralytics_raw": raw_count,
            "ultralytics_legacy": legacy_count,
            "yolo26_e2e": e2e_count,
        }[requested]
        if feature_count != expected:
            raise ValueError(
                f"{requested} expects {expected} features per prediction, got {feature_count}"
            )
        return requested
    candidates: list[OnnxOutputFormat] = []
    if feature_count == raw_count:
        candidates.append("ultralytics_raw")
    if feature_count == legacy_count:
        candidates.append("ultralytics_legacy")
    if feature_count == e2e_count:
        candidates.append("yolo26_e2e")
    if len(candidates) == 1:
        return candidates[0]
    if "yolo26_e2e" in candidates:
        if _looks_like_yolo26_e2e(rows, class_count):
            return "yolo26_e2e"
        non_e2e = [candidate for candidate in candidates if candidate != "yolo26_e2e"]
        if len(non_e2e) == 1:
            return non_e2e[0]
    raise ValueError(
        f"Unsupported prediction width {feature_count}; expected raw={raw_count}, "
        f"legacy={legacy_count}, or YOLO26 E2E={e2e_count}. Set output_format "
        "explicitly if this model uses a custom export adapter."
    )


def _xywh_to_xyxy(boxes: np.ndarray) -> np.ndarray:
    result = np.empty_like(boxes, dtype=np.float32)
    result[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    result[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    result[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    result[:, 3] = boxes[:, 1] + boxes[:, 3] / 2
    return result


def _box_iou_one_to_many(box: np.ndarray, boxes: np.ndarray) -> np.ndarray:
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    intersection = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_a = max(0.0, float(box[2] - box[0])) * max(0.0, float(box[3] - box[1]))
    area_b = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(
        0.0, boxes[:, 3] - boxes[:, 1]
    )
    return intersection / np.maximum(area_a + area_b - intersection, 1e-9)


def _class_aware_nms(
    boxes: np.ndarray,
    scores: np.ndarray,
    class_ids: np.ndarray,
    *,
    iou_threshold: float,
    max_detections: int,
) -> np.ndarray:
    kept: list[int] = []
    for class_id in np.unique(class_ids):
        indices = np.flatnonzero(class_ids == class_id)
        order = indices[np.argsort(-scores[indices], kind="stable")]
        while order.size:
            current = int(order[0])
            kept.append(current)
            if order.size == 1:
                break
            remaining = order[1:]
            order = remaining[
                _box_iou_one_to_many(boxes[current], boxes[remaining]) <= iou_threshold
            ]
    kept.sort(key=lambda index: (-float(scores[index]), index))
    return np.asarray(kept[:max_detections], dtype=np.int64)


def _resize_bilinear_align_corners_false(
    values: np.ndarray,
    output_shape: tuple[int, int],
) -> np.ndarray:
    """Resize an ``(N,H,W)`` float stack like torch bilinear interpolation.

    Ultralytics ``scale_masks`` delegates to ``torch.nn.functional.interpolate``
    in bilinear mode with the default ``align_corners=False`` coordinate rule.
    Keeping this small NumPy implementation local makes the ONNX backend
    independently testable without importing torch or OpenCV.
    """
    source = np.asarray(values, dtype=np.float32)
    if source.ndim != 3 or source.shape[1] < 1 or source.shape[2] < 1:
        raise ValueError(f"values must have shape (N,H,W), got {source.shape}")
    target_h, target_w = int(output_shape[0]), int(output_shape[1])
    if target_h < 1 or target_w < 1:
        raise ValueError(f"Invalid output shape: {output_shape}")
    if source.shape[1:] == (target_h, target_w):
        return source.copy()

    def indices(source_size: int, target_size: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        coordinates = (
            (np.arange(target_size, dtype=np.float64) + 0.5)
            * source_size
            / target_size
            - 0.5
        )
        lower_raw = np.floor(coordinates).astype(np.int64)
        weights = (coordinates - lower_raw).astype(np.float32)
        lower = np.clip(lower_raw, 0, source_size - 1)
        upper = np.clip(lower_raw + 1, 0, source_size - 1)
        return lower, upper, weights

    y0, y1, wy = indices(source.shape[1], target_h)
    x0, x1, wx = indices(source.shape[2], target_w)
    vertical = (
        source[:, y0, :] * (1.0 - wy)[None, :, None]
        + source[:, y1, :] * wy[None, :, None]
    )
    return (
        vertical[:, :, x0] * (1.0 - wx)[None, None, :]
        + vertical[:, :, x1] * wx[None, None, :]
    ).astype(np.float32, copy=False)


def _scale_mask_logits_native(
    logits: np.ndarray,
    output_shape: tuple[int, int],
) -> np.ndarray:
    """Mirror Ultralytics ``scale_masks(..., padding=True)`` for logits."""
    values = np.asarray(logits, dtype=np.float32)
    if values.ndim != 3:
        raise ValueError(f"logits must have shape (N,H,W), got {values.shape}")
    mask_h, mask_w = values.shape[1:]
    output_h, output_w = int(output_shape[0]), int(output_shape[1])
    if (mask_h, mask_w) == (output_h, output_w):
        return values.copy()

    gain = min(mask_h / output_h, mask_w / output_w)
    pad_w = (mask_w - round(output_w * gain)) / 2
    pad_h = (mask_h - round(output_h * gain)) / 2
    top, left = round(pad_h - 0.1), round(pad_w - 0.1)
    bottom = mask_h - round(pad_h + 0.1)
    right = mask_w - round(pad_w + 0.1)
    if bottom <= top or right <= left:
        raise ValueError(
            "Letterbox removal produced an empty prototype crop: "
            f"prototype={(mask_h, mask_w)}, output={(output_h, output_w)}"
        )
    return _resize_bilinear_align_corners_false(
        values[:, top:bottom, left:right],
        (output_h, output_w),
    )


def _process_masks_native(
    coefficients: np.ndarray,
    prototypes: np.ndarray,
    boxes_original_xyxy: np.ndarray,
    *,
    output_shape: tuple[int, int],
    threshold: float,
) -> list[np.ndarray]:
    """Reconstruct masks in the same order as ``process_mask_native``.

    The native order is prototype logits -> remove letterbox/upsample ->
    threshold -> crop in original-image coordinates.  Applying sigmoid or
    cropping at prototype resolution before interpolation changes boundaries.
    """
    if coefficients.shape[1] != prototypes.shape[0]:
        raise ValueError(
            "Mask coefficient count does not match prototype channels: "
            f"{coefficients.shape[1]} vs {prototypes.shape[0]}"
        )
    proto_h, proto_w = prototypes.shape[1:]
    logits = coefficients @ prototypes.reshape(prototypes.shape[0], -1)
    logits = logits.reshape(-1, proto_h, proto_w)
    scaled_logits = _scale_mask_logits_native(logits, output_shape)

    if threshold <= 0.0:
        logit_threshold = -np.inf
    elif threshold >= 1.0:
        logit_threshold = np.inf
    else:
        logit_threshold = float(np.log(threshold / (1.0 - threshold)))
    masks = scaled_logits > logit_threshold

    height, width = int(output_shape[0]), int(output_shape[1])
    columns = np.arange(width, dtype=np.float32)[None, :]
    rows = np.arange(height, dtype=np.float32)[:, None]
    output: list[np.ndarray] = []
    for mask, box in zip(masks, boxes_original_xyxy, strict=True):
        crop = (
            (columns >= float(box[0]))
            & (columns < float(box[2]))
            & (rows >= float(box[1]))
            & (rows < float(box[3]))
        )
        output.append(np.asarray(mask & crop, dtype=np.bool_))
    return output


def _undo_letterbox_box(
    box: np.ndarray,
    transform: LetterboxTransform,
) -> tuple[float, float, float, float]:
    original_h, original_w = transform.original_shape
    x1 = (float(box[0]) - transform.pad_left) / transform.scale
    y1 = (float(box[1]) - transform.pad_top) / transform.scale
    x2 = (float(box[2]) - transform.pad_left) / transform.scale
    y2 = (float(box[3]) - transform.pad_top) / transform.scale
    return (
        float(np.clip(x1, 0, original_w)),
        float(np.clip(y1, 0, original_h)),
        float(np.clip(x2, 0, original_w)),
        float(np.clip(y2, 0, original_h)),
    )


def parse_onnx_segmentation_outputs(
    outputs: Mapping[str, Any] | Sequence[Any] | np.ndarray,
    *,
    class_names: Mapping[int | str, str] | Sequence[str] | None = None,
    input_shape: tuple[int, int],
    transform: LetterboxTransform | None = None,
    output_format: OnnxOutputFormat = "auto",
    confidence_threshold: float = 0.25,
    iou_threshold: float = 0.45,
    mask_threshold: float = 0.5,
    max_detections: int = 300,
    object_id_prefix: str = "onnx",
) -> list[VisionObject]:
    """Parse common Ultralytics segmentation ONNX output layouts.

    Supported layouts:

    * ``ultralytics_raw``: ``[xywh, class scores, mask coefficients]``;
    * ``ultralytics_legacy``: ``[xywh, objectness, class scores, coefficients]``;
    * ``yolo26_e2e``: ``[xyxy, confidence, class_id, coefficients]``.

    YOLO26 with exactly two classes has the same feature width as the raw
    layout. Auto mode recognizes the integer class-id column, but production
    metadata should still set the format explicitly for maximum auditability.
    """
    names = validate_class_contract(normalize_class_names(class_names))
    if not names:
        raise ValueError("At least one class name is required")
    if not 0.0 <= float(mask_threshold) <= 1.0:
        raise ValueError("mask_threshold must be between 0 and 1")
    class_count = len(names)
    prediction, prototypes = _find_prediction_and_prototypes(outputs)
    mask_count = int(prototypes.shape[0])
    expected_counts = {
        4 + class_count + mask_count,
        5 + class_count + mask_count,
        6 + mask_count,
    }
    rows = _prediction_rows(prediction, expected_feature_counts=expected_counts)
    resolved_format = _resolve_output_format(
        rows,
        requested=output_format,
        class_count=class_count,
        mask_count=mask_count,
    )

    if resolved_format == "yolo26_e2e":
        boxes = rows[:, :4].copy()
        scores = rows[:, 4].copy()
        class_ids = np.rint(rows[:, 5]).astype(np.int64)
        coefficients = rows[:, 6 : 6 + mask_count].copy()
        valid = (
            np.isfinite(rows).all(axis=1)
            & (scores >= confidence_threshold)
            & (class_ids >= 0)
            & (class_ids < class_count)
        )
        boxes, scores, class_ids, coefficients = (
            value[valid] for value in (boxes, scores, class_ids, coefficients)
        )
        order = np.argsort(-scores, kind="stable")[:max_detections]
        boxes, scores, class_ids, coefficients = (
            value[order] for value in (boxes, scores, class_ids, coefficients)
        )
    else:
        boxes = _xywh_to_xyxy(rows[:, :4])
        class_start = 5 if resolved_format == "ultralytics_legacy" else 4
        class_scores = rows[:, class_start : class_start + class_count]
        class_ids = np.argmax(class_scores, axis=1).astype(np.int64)
        scores = class_scores[np.arange(len(rows)), class_ids]
        if resolved_format == "ultralytics_legacy":
            scores = scores * rows[:, 4]
        coefficients = rows[:, class_start + class_count : class_start + class_count + mask_count]
        valid = np.isfinite(rows).all(axis=1) & (scores >= confidence_threshold)
        boxes, scores, class_ids, coefficients = (
            value[valid] for value in (boxes, scores, class_ids, coefficients)
        )
        keep = _class_aware_nms(
            boxes,
            scores,
            class_ids,
            iou_threshold=iou_threshold,
            max_detections=max_detections,
        )
        boxes, scores, class_ids, coefficients = (
            value[keep] for value in (boxes, scores, class_ids, coefficients)
        )

    if len(boxes) == 0:
        return []
    input_h, input_w = int(input_shape[0]), int(input_shape[1])
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, input_w)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, input_h)
    valid_geometry = (boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1])
    boxes, scores, class_ids, coefficients = (
        value[valid_geometry] for value in (boxes, scores, class_ids, coefficients)
    )
    if len(boxes) == 0:
        return []

    transform = transform or LetterboxTransform.identity((input_h, input_w))
    if transform.input_shape != (input_h, input_w):
        raise ValueError(
            f"transform input shape {transform.input_shape} does not match {(input_h, input_w)}"
        )
    original_boxes = np.asarray(
        [_undo_letterbox_box(box, transform) for box in boxes],
        dtype=np.float32,
    )
    # A box can have positive area in the letterboxed tensor but collapse after
    # undoing the letterbox and clipping to the original image (for example
    # when both x coordinates lie beyond the right edge).  Filter once more in
    # original-image coordinates before constructing VisionObject; the schema
    # intentionally rejects zero-area boxes.
    original_geometry = (
        np.isfinite(original_boxes).all(axis=1)
        & (original_boxes[:, 2] > original_boxes[:, 0])
        & (original_boxes[:, 3] > original_boxes[:, 1])
    )
    original_boxes, scores, class_ids, coefficients = (
        value[original_geometry]
        for value in (original_boxes, scores, class_ids, coefficients)
    )
    if len(original_boxes) == 0:
        return []
    native_masks = _process_masks_native(
        coefficients,
        prototypes,
        original_boxes,
        output_shape=transform.original_shape,
        threshold=float(mask_threshold),
    )
    objects: list[VisionObject] = []
    for index, (box, score, class_id, mask) in enumerate(
        zip(original_boxes, scores, class_ids, native_masks, strict=True)
    ):
        class_id_int = int(class_id)
        objects.append(
            VisionObject(
                object_id=f"{object_id_prefix}_{index:04d}",
                class_id=class_id_int,
                class_name=names.get(class_id_int, f"class_{class_id_int}"),
                confidence=float(score),
                bbox_xyxy=tuple(float(value) for value in box),
                mask=mask,
                corners_px=None,
                corner_confidence=None,
            )
        )
    return objects


# A concise alias for callers/tests that use the model-family name.
parse_yolo_seg_outputs = parse_onnx_segmentation_outputs


def _parse_names_metadata(raw: str | None) -> dict[int, str] | None:
    if not raw:
        return None
    for parser in (json.loads, ast.literal_eval):
        try:
            value = parser(raw)
        except (ValueError, SyntaxError, json.JSONDecodeError):
            continue
        if isinstance(value, (Mapping, list, tuple)):
            return normalize_class_names(value)
    return None


def _load_adjacent_class_names(model_path: Path) -> dict[int, str] | None:
    path = model_path.with_name("classes.json")
    if not path.is_file():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, Mapping) and "classes" in payload:
        payload = payload["classes"]
    if not isinstance(payload, (Mapping, list, tuple)):
        raise ValueError(f"Unsupported classes.json payload in {path}")
    return normalize_class_names(payload)


def _load_adjacent_model_metadata(model_path: Path) -> Mapping[str, Any]:
    path = model_path.with_name("model_metadata.json")
    if not path.is_file():
        return {}
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError(f"model_metadata.json must contain an object: {path}")
    return payload


class ONNXVisionBackend:
    """ONNX Runtime implementation intended for the future PC application."""

    def __init__(
        self,
        model_path: str | Path,
        *,
        class_names: Mapping[int | str, str] | Sequence[str] | None = None,
        input_size: int | tuple[int, int] = 1024,
        output_format: OnnxOutputFormat = "auto",
        confidence_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        mask_threshold: float = 0.5,
        max_detections: int = 300,
        providers: Sequence[str] | None = None,
    ) -> None:
        self.model_path = Path(model_path)
        self.class_names = None if class_names is None else normalize_class_names(class_names)
        self.input_size = input_size
        self.output_format = output_format
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.mask_threshold = float(mask_threshold)
        self.max_detections = int(max_detections)
        self.providers = None if providers is None else list(providers)
        self._session: Any | None = None
        self._input_name: str | None = None
        self._input_shape: tuple[int, int] | None = None
        self._input_dtype: np.dtype[Any] | type = np.float32

    def load(self) -> None:
        model = self.model_path.expanduser().resolve()
        if not model.is_file():
            raise FileNotFoundError(f"ONNX model does not exist: {model}")
        onnxruntime = require_onnxruntime()
        kwargs: dict[str, Any] = {}
        if self.providers is not None:
            kwargs["providers"] = self.providers
        LOGGER.info("Loading ONNX segmentation model from %s", model)
        self._session = onnxruntime.InferenceSession(str(model), **kwargs)
        inputs = self._session.get_inputs()
        if len(inputs) != 1:
            raise ValueError(f"Expected one ONNX image input, found {len(inputs)}")
        model_input = inputs[0]
        self._input_name = str(model_input.name)
        input_type = str(getattr(model_input, "type", "tensor(float)"))
        if input_type == "tensor(float16)":
            self._input_dtype = np.float16
        elif input_type == "tensor(float)":
            self._input_dtype = np.float32
        else:
            raise ValueError(
                f"Expected float32/float16 ONNX image input, got {input_type!r}"
            )
        shape = model_input.shape
        if len(shape) != 4:
            raise ValueError(f"Expected NCHW ONNX input, got shape {shape}")
        if isinstance(shape[1], int) and shape[1] != 3:
            raise ValueError(f"Expected a 3-channel NCHW ONNX input, got shape {shape}")
        if isinstance(shape[2], int) and isinstance(shape[3], int):
            self._input_shape = int(shape[2]), int(shape[3])
        elif isinstance(self.input_size, int):
            self._input_shape = self.input_size, self.input_size
        else:
            self._input_shape = int(self.input_size[0]), int(self.input_size[1])

        if self.class_names is None:
            metadata = self._session.get_modelmeta().custom_metadata_map or {}
            self.class_names = _parse_names_metadata(metadata.get("names"))
        adjacent_metadata = _load_adjacent_model_metadata(model)
        if self.class_names is None:
            self.class_names = _load_adjacent_class_names(model)
        if self.class_names is None and adjacent_metadata.get("classes") is not None:
            self.class_names = normalize_class_names(adjacent_metadata["classes"])
        if self.class_names is None:
            LOGGER.warning("No model class metadata found; using the phase AI-3 class contract")
            self.class_names = dict(CLASS_NAMES)
        self.class_names = validate_class_contract(self.class_names)
        if self.output_format == "auto" and adjacent_metadata.get("output_format"):
            metadata_format = str(adjacent_metadata["output_format"])
            if metadata_format not in {
                "ultralytics_raw",
                "ultralytics_legacy",
                "yolo26_e2e",
            }:
                raise ValueError(
                    f"Unsupported output_format {metadata_format!r} in model_metadata.json"
                )
            self.output_format = metadata_format  # type: ignore[assignment]

    def infer(self, image_bgr: np.ndarray) -> list[VisionObject]:
        validate_image_bgr(image_bgr)
        if self._session is None:
            self.load()
        assert self._session is not None
        assert self._input_name is not None
        assert self._input_shape is not None
        assert self.class_names is not None
        tensor, transform = _letterbox(
            image_bgr,
            self._input_shape,
            dtype=self._input_dtype,
        )
        values = self._session.run(None, {self._input_name: tensor})
        return parse_onnx_segmentation_outputs(
            values,
            class_names=self.class_names,
            input_shape=self._input_shape,
            transform=transform,
            output_format=self.output_format,
            confidence_threshold=self.confidence_threshold,
            iou_threshold=self.iou_threshold,
            mask_threshold=self.mask_threshold,
            max_detections=self.max_detections,
            object_id_prefix="onnx",
        )

    def close(self) -> None:
        self._session = None
        self._input_name = None
        self._input_shape = None
        self._input_dtype = np.float32
