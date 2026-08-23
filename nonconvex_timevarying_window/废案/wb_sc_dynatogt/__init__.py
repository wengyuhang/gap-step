"""Whole-body, attitude-aware extension of SC-DynaTOGT.

WBSC-DynaTOGT keeps the Schwarz--Christoffel/MINCO/TOGT backbone while
replacing the fixed spherical gate inset with an oriented cuboid and a jointly
optimized yaw flat output; roll/pitch remain dynamically consistent.
"""

from .collider import ColliderConfig, CuboidCollider
from .config import WBSCOptimizationConfig
from .optimizer import WBSCOptimizationResult, optimize_track
from .preprocessing import WBPreprocessedGate, WBPreprocessingConfig, preprocess_boundary
from .validation import AttitudeSafetyReport, SphereSafetyReport, validate_legacy_sphere, validate_whole_body
from .yaw import YawTrajectory, yaw_from_unconstrained, yaw_to_unconstrained

__all__ = [
    "AttitudeSafetyReport",
    "ColliderConfig",
    "CuboidCollider",
    "SphereSafetyReport",
    "WBPreprocessedGate",
    "WBPreprocessingConfig",
    "WBSCOptimizationConfig",
    "WBSCOptimizationResult",
    "YawTrajectory",
    "optimize_track",
    "preprocess_boundary",
    "validate_legacy_sphere",
    "validate_whole_body",
    "yaw_from_unconstrained",
    "yaw_to_unconstrained",
]
