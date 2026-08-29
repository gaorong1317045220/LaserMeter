# SPDX-License-Identifier: MIT
"""Runtime API for door/window/outlet ONNX inference."""

from .schemas.vision_object import (
    CLASS_CONTRACTS,
    CLASS_NAMES,
    CLASS_NAMES_V2,
    CLASS_NAMES_V3,
    VisionObject,
    class_contract_for_names,
    validate_class_contract,
)

__all__ = [
    "CLASS_NAMES",
    "CLASS_NAMES_V2",
    "CLASS_NAMES_V3",
    "CLASS_CONTRACTS",
    "class_contract_for_names",
    "VisionObject",
    "validate_class_contract",
]

__version__ = "0.1.0"
