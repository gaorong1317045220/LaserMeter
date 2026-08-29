# SPDX-License-Identifier: MIT
"""Stable product-facing result type independent of model frameworks."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

"""Stable class contracts for the v2 and v3 model generations.

The original product API used ``door_opening``/``window_opening``.  v3 adds
the outlet class and intentionally uses the public visual-region names
``door``/``window``/``outlet``.  Keep the v2 default so existing manifests and
models remain loadable, while all callers validate the exact map they use.
"""

CLASS_NAMES: dict[int, str] = {0: "door_opening", 1: "window_opening"}
CLASS_NAMES_V2: dict[int, str] = dict(CLASS_NAMES)
CLASS_NAMES_V3: dict[int, str] = {0: "door", 1: "window", 2: "outlet"}
CLASS_CONTRACTS: tuple[dict[int, str], ...] = (CLASS_NAMES_V2, CLASS_NAMES_V3)


def class_contract_for_names(
    names: Mapping[int | str, str] | Sequence[str],
) -> dict[int, str]:
    """Normalize a class map and return the matching supported contract."""

    if isinstance(names, Mapping):
        normalized = {int(key): str(value) for key, value in names.items()}
    elif isinstance(names, (str, bytes)):
        raise TypeError("Class names must be a mapping or sequence")
    else:
        normalized = {index: str(value) for index, value in enumerate(names)}
    for contract in CLASS_CONTRACTS:
        if normalized == contract:
            return dict(contract)
    raise ValueError(
        f"Class contract must be exactly one of {list(CLASS_CONTRACTS)}, got {normalized}"
    )


def validate_class_contract(
    names: Mapping[int | str, str] | Sequence[str],
    *,
    expected: Mapping[int | str, str] | Sequence[str] | None = None,
) -> dict[int, str]:
    """Normalize and enforce an exact v2 or v3 class map.

    ``expected`` is useful at a training/export boundary where accepting a
    different supported generation would still be unsafe.
    """
    normalized = class_contract_for_names(names)
    if expected is not None:
        expected_normalized = class_contract_for_names(expected)
        if normalized != expected_normalized:
            raise ValueError(
                f"Class contract mismatch: expected {expected_normalized}, got {normalized}"
            )
    return normalized


@dataclass(slots=True)
class VisionObject:
    object_id: str
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: tuple[float, float, float, float]
    mask: np.ndarray | None = None
    corners_px: np.ndarray | None = None
    corner_confidence: np.ndarray | None = None

    def __post_init__(self) -> None:
        self.object_id = str(self.object_id)
        self.class_id = int(self.class_id)
        self.class_name = str(self.class_name)
        expected_names = {
            0: {CLASS_NAMES_V2[0], CLASS_NAMES_V3[0]},
            1: {CLASS_NAMES_V2[1], CLASS_NAMES_V3[1]},
            2: {CLASS_NAMES_V3[2]},
        }
        if self.class_id not in expected_names or self.class_name not in expected_names[self.class_id]:
            raise ValueError(
                f"VisionObject class must match a supported v2/v3 contract, got "
                f"{self.class_id}={self.class_name!r}"
            )
        self.confidence = float(self.confidence)
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
        if len(self.bbox_xyxy) != 4:
            raise ValueError("bbox_xyxy must contain x1, y1, x2, y2")
        self.bbox_xyxy = tuple(float(v) for v in self.bbox_xyxy)  # type: ignore[assignment]
        x1, y1, x2, y2 = self.bbox_xyxy
        if not np.isfinite(self.bbox_xyxy).all() or x2 <= x1 or y2 <= y1:
            raise ValueError("bbox_xyxy must be finite with x2>x1 and y2>y1")
        if self.mask is not None:
            self.mask = np.asarray(self.mask)
            if self.mask.ndim != 2:
                raise ValueError(f"mask must be two-dimensional, got {self.mask.shape}")
        if self.corners_px is not None:
            self.corners_px = np.asarray(self.corners_px, dtype=np.float32)
            if self.corners_px.shape != (4, 2):
                raise ValueError(
                    "corners_px must have shape (4, 2) in LT, RT, RB, LB order"
                )
        if self.corner_confidence is not None:
            self.corner_confidence = np.asarray(self.corner_confidence, dtype=np.float32)
            if self.corner_confidence.shape not in {(4,), (4, 1)}:
                raise ValueError("corner_confidence must provide one score for each corner")
            self.corner_confidence = self.corner_confidence.reshape(4)

    def to_dict(
        self,
        *,
        include_mask: bool = False,
        mask_path: str | None = None,
    ) -> dict[str, Any]:
        """Create a JSON-safe payload; large masks are opt-in or referenced by path."""
        result: dict[str, Any] = {
            "object_id": self.object_id,
            "class_id": self.class_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox_xyxy": list(self.bbox_xyxy),
            "corners_px": None if self.corners_px is None else self.corners_px.tolist(),
            "corner_confidence": (
                None if self.corner_confidence is None else self.corner_confidence.tolist()
            ),
            "mask": (
                self.mask.astype(np.uint8).tolist()
                if include_mask and self.mask is not None
                else None
            ),
        }
        if mask_path is not None:
            result["mask_path"] = mask_path
        return result

    @classmethod
    def from_dict(
        cls,
        value: Mapping[str, Any],
        *,
        base_dir: str | Path | None = None,
    ) -> "VisionObject":
        mask = value.get("mask")
        mask_path = value.get("mask_path")
        if mask is None and mask_path and base_dir is not None:
            from PIL import Image

            path = Path(mask_path)
            if not path.is_absolute():
                path = Path(base_dir) / path
            if not path.is_file():
                raise FileNotFoundError(f"Serialized VisionObject mask not found: {path}")
            mask = np.asarray(Image.open(path).convert("L")) > 0
        return cls(
            object_id=value.get("object_id", ""),
            class_id=value.get("class_id", -1),
            class_name=value.get("class_name", ""),
            confidence=value.get("confidence", 0.0),
            bbox_xyxy=value.get("bbox_xyxy", (0, 0, 0, 0)),
            mask=mask,
            corners_px=value.get("corners_px"),
            corner_confidence=value.get("corner_confidence"),
        )
