# SPDX-License-Identifier: MIT
"""Conversion helpers that isolate model-framework result types."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np
from PIL import Image

from door_window_ai.schemas.vision_object import CLASS_NAMES, VisionObject


def to_numpy(value: Any, *, dtype: np.dtype[Any] | type | None = None) -> np.ndarray:
    """Convert numpy/torch-like values without importing torch."""
    if value is None:
        raise TypeError("Cannot convert None to an array")
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value, dtype=dtype)


def normalize_class_names(
    names: Mapping[int | str, str] | Sequence[str] | None,
) -> dict[int, str]:
    if names is None:
        return dict(CLASS_NAMES)
    if isinstance(names, Mapping):
        return {int(key): str(value) for key, value in names.items()}
    if isinstance(names, (str, bytes)):
        raise TypeError("class names must be a mapping or a sequence, not a string")
    return {index: str(value) for index, value in enumerate(names)}


def resize_mask_nearest(mask: np.ndarray, output_shape: tuple[int, int]) -> np.ndarray:
    """Resize a 2-D mask without requiring OpenCV in the base installation."""
    target_h, target_w = (int(output_shape[0]), int(output_shape[1]))
    if target_h < 1 or target_w < 1:
        raise ValueError(f"Invalid mask output shape: {output_shape}")
    source = np.asarray(mask)
    if source.ndim != 2 or source.shape[0] < 1 or source.shape[1] < 1:
        raise ValueError(f"mask must be a non-empty 2-D array, got {source.shape}")
    if source.shape == (target_h, target_w):
        return source.copy()
    rows = np.minimum(
        (np.arange(target_h, dtype=np.float64) * source.shape[0] / target_h).astype(int),
        source.shape[0] - 1,
    )
    cols = np.minimum(
        (np.arange(target_w, dtype=np.float64) * source.shape[1] / target_w).astype(int),
        source.shape[1] - 1,
    )
    return source[rows[:, None], cols[None, :]]


def resize_probability_bilinear(
    probability: np.ndarray, output_shape: tuple[int, int]
) -> np.ndarray:
    """Resize float mask probabilities before applying the binary threshold."""
    target_h, target_w = int(output_shape[0]), int(output_shape[1])
    source = np.asarray(probability, dtype=np.float32)
    if source.ndim != 2 or source.size == 0 or target_h < 1 or target_w < 1:
        raise ValueError("probability must be a non-empty 2-D array and output positive")
    if source.shape == (target_h, target_w):
        return source.copy()
    image = Image.fromarray(source, mode="F")
    return np.asarray(
        image.resize((target_w, target_h), resample=Image.Resampling.BILINEAR),
        dtype=np.float32,
    )


def _clip_bbox(
    bbox: np.ndarray, image_shape: tuple[int, int]
) -> tuple[float, float, float, float]:
    height, width = image_shape
    x1, y1, x2, y2 = np.asarray(bbox, dtype=np.float64).reshape(4)
    x1 = float(np.clip(x1, 0.0, float(width)))
    x2 = float(np.clip(x2, 0.0, float(width)))
    y1 = float(np.clip(y1, 0.0, float(height)))
    y2 = float(np.clip(y2, 0.0, float(height)))
    return x1, y1, x2, y2


def ultralytics_result_to_vision_objects(
    result: Any,
    *,
    class_names: Mapping[int | str, str] | Sequence[str] | None = None,
    mask_threshold: float = 0.5,
    object_id_prefix: str = "yolo",
) -> list[VisionObject]:
    """Convert one Ultralytics segmentation result to the stable schema.

    The conversion uses duck typing deliberately: importing this module never
    imports Ultralytics or torch.
    """
    boxes_result = getattr(result, "boxes", None)
    if boxes_result is None:
        return []
    xyxy_value = getattr(boxes_result, "xyxy", None)
    conf_value = getattr(boxes_result, "conf", None)
    cls_value = getattr(boxes_result, "cls", None)
    if xyxy_value is None or conf_value is None or cls_value is None:
        raise ValueError("Ultralytics result.boxes must expose xyxy, conf and cls")

    boxes = to_numpy(xyxy_value, dtype=np.float32).reshape(-1, 4)
    confidences = to_numpy(conf_value, dtype=np.float32).reshape(-1)
    class_ids = to_numpy(cls_value).reshape(-1).astype(np.int64)
    if not (len(boxes) == len(confidences) == len(class_ids)):
        raise ValueError("Ultralytics box, confidence and class counts do not match")
    # A valid segmentation result may contain no detections and therefore no
    # masks object.  Conversely, non-empty boxes without masks usually means a
    # detection checkpoint was supplied to the segmentation backend.  Failing
    # here prevents that configuration error from silently becoming
    # VisionObject(mask=None).
    if len(boxes) == 0:
        return []

    original_shape = getattr(result, "orig_shape", None)
    if original_shape is None:
        original_image = getattr(result, "orig_img", None)
        if original_image is None:
            raise ValueError("Ultralytics result must expose orig_shape or orig_img")
        original_shape = np.asarray(original_image).shape[:2]
    image_shape = int(original_shape[0]), int(original_shape[1])

    result_names = class_names if class_names is not None else getattr(result, "names", None)
    names = normalize_class_names(result_names)

    masks: np.ndarray | None = None
    masks_result = getattr(result, "masks", None)
    masks_value = None if masks_result is None else getattr(masks_result, "data", None)
    if masks_value is None:
        raise ValueError(
            "Ultralytics segmentation result contains boxes but no masks. "
            "Use a segmentation checkpoint (for example, *-seg.pt), not a "
            "detection-only checkpoint."
        )
    masks = to_numpy(masks_value, dtype=np.float32)
    if masks.ndim == 2:
        masks = masks[None, ...]
    if masks.ndim != 3:
        raise ValueError(f"Ultralytics masks must have shape (N,H,W), got {masks.shape}")
    if masks.shape[0] != len(boxes):
        raise ValueError("Ultralytics mask and box counts do not match")

    objects: list[VisionObject] = []
    for index, (bbox, confidence, class_id) in enumerate(
        zip(boxes, confidences, class_ids, strict=True)
    ):
        candidate = masks[index]
        if candidate.shape != image_shape:
            candidate = resize_mask_nearest(candidate, image_shape)
        mask = candidate >= float(mask_threshold)
        class_id_int = int(class_id)
        objects.append(
            VisionObject(
                object_id=f"{object_id_prefix}_{index:04d}",
                class_id=class_id_int,
                class_name=names.get(class_id_int, f"class_{class_id_int}"),
                confidence=float(confidence),
                bbox_xyxy=_clip_bbox(bbox, image_shape),
                mask=mask,
                corners_px=None,
                corner_confidence=None,
            )
        )
    return objects


def ultralytics_results_to_vision_objects(
    results: Any,
    **kwargs: Any,
) -> list[VisionObject]:
    """Convert the single-image result returned by ``YOLO.predict``."""
    if isinstance(results, (list, tuple)):
        if len(results) != 1:
            raise ValueError(f"Expected exactly one image result, got {len(results)}")
        results = results[0]
    return ultralytics_result_to_vision_objects(results, **kwargs)
