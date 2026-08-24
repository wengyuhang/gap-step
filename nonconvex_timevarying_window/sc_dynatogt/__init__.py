"""SC-DynaTOGT for ordered traversal of dynamic non-convex windows.

The package follows the accompanying experiment plan in three deliberately
separate stages: Chang-style *boundary-only* resampling, an offline disk
Schwarz--Christoffel map, and TOGT's joint spatial/temporal trajectory
optimization.
"""

from .environment import MotionProfile, SCDynamicWindow, SCWindowTrack
from .collision import (
    CuboidBody,
    point_to_oriented_cuboid_distance_squared,
    whole_body_clearance_residual,
)
from .optimizer import OptimizationConfig, OptimizationResult, optimize_track

__all__ = [
    "MotionProfile",
    "OptimizationConfig",
    "OptimizationResult",
    "SCDynamicWindow",
    "CuboidBody",
    "point_to_oriented_cuboid_distance_squared",
    "whole_body_clearance_residual",
    "SCWindowTrack",
    "optimize_track",
]
