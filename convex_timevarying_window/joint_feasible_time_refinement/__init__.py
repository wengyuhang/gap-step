"""Joint feasibility-preserving time refinement for convex moving windows."""

from .refinement import (
    JointTimeRefinementConfig,
    JointTimeRefinementResult,
    refine_joint_feasible_time,
)

__all__ = [
    "JointTimeRefinementConfig",
    "JointTimeRefinementResult",
    "refine_joint_feasible_time",
]
