# SPDX-License-Identifier: MIT
"""Product-facing visual inference schema."""

from .vision_object import (
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
