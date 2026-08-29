# SPDX-License-Identifier: MIT
"""Framework-independent inference backend contract."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

from door_window_ai.schemas.vision_object import VisionObject


class BackendNotLoadedError(RuntimeError):
    """Raised when a backend cannot infer before its resources are loaded."""


@runtime_checkable
class VisionBackend(Protocol):
    """Stable boundary used by future PC/device integrations.

    Implementations may use different model runtimes internally, but callers only
    receive :class:`VisionObject` values.
    """

    def load(self) -> None:
        """Load model/runtime resources."""

    def infer(self, image_bgr: np.ndarray) -> list[VisionObject]:
        """Run inference for one uint8 BGR image."""

    def close(self) -> None:
        """Release model/runtime resources."""


def validate_image_bgr(image_bgr: np.ndarray) -> np.ndarray:
    """Validate the common image contract without importing an image framework."""
    if not isinstance(image_bgr, np.ndarray):
        raise TypeError("image_bgr must be a numpy.ndarray")
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError(
            f"image_bgr must have shape (height, width, 3), got {image_bgr.shape}"
        )
    if image_bgr.shape[0] < 1 or image_bgr.shape[1] < 1:
        raise ValueError("image_bgr must not be empty")
    if image_bgr.dtype != np.uint8:
        raise TypeError(f"image_bgr must have dtype uint8, got {image_bgr.dtype}")
    return image_bgr
