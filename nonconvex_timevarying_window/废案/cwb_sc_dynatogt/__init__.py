"""Continuous whole-body safety for SC-DynaTOGT.

This sibling method keeps the base planner's ``x=[K,D]`` decision vector and
adds attitude-aware cuboid/plane-section verification and active constraints.
"""

from .body_model import CuboidBody
from .config import WholeBodySafetyConfig
from .constraint_generation import (
    WholeBodyOptimizationResult,
    WholeBodyResultStatus,
    optimize_with_whole_body_safety,
)
from .whole_body_safety import (
    TrajectorySafetyReport,
    VerificationStatus,
    verify_whole_body_trajectory,
)

__all__ = [
    "CuboidBody",
    "TrajectorySafetyReport",
    "VerificationStatus",
    "WholeBodyOptimizationResult",
    "WholeBodyResultStatus",
    "WholeBodySafetyConfig",
    "optimize_with_whole_body_safety",
    "verify_whole_body_trajectory",
]
