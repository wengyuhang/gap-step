"""SC-DynaTOGT for ordered traversal of dynamic non-convex windows.

The package follows the accompanying experiment plan in three deliberately
separate stages: Chang-style *boundary-only* resampling, an offline disk
Schwarz--Christoffel map, and TOGT's joint spatial/temporal trajectory
optimization.
"""

from .environment import MotionProfile, SCDynamicWindow, SCWindowTrack
from .optimizer import OptimizationConfig, OptimizationResult, optimize_track

__all__ = [
    "MotionProfile",
    "OptimizationConfig",
    "OptimizationResult",
    "SCDynamicWindow",
    "SCWindowTrack",
    "optimize_track",
]

