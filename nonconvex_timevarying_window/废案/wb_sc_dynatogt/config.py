"""Configuration objects for WBSC-DynaTOGT."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig

from .collider import ColliderConfig


@dataclass(frozen=True)
class WBSCOptimizationConfig(OptimizationConfig):
    """Original SC-DynaTOGT soft costs plus one planned pose constraint.

    The default optimizer uses ``x=[K,D,Y]``.  MINCO and differential
    flatness are evaluated inside every nonlinear-program iteration, so the
    cuboid constraint feeds back to time, crossing points, yaw, and the
    acceleration-derived roll/pitch rather than being a post-hoc check.
    """

    collider: ColliderConfig = field(default_factory=ColliderConfig)
    collision_weight: float = 0.0
    yaw_snap_weight: float = 0.0
    optimize_yaw: bool = True
    hard_collision_constraints: bool = True
    hard_constraint_edge_samples: int = 9
    hard_constraint_tolerance: float = 1.0e-6
    hard_solver_iteration_limit: int = 1_000

    def __post_init__(self) -> None:
        super().__post_init__()
        if not np.isfinite(self.collision_weight) or self.collision_weight < 0.0:
            raise ValueError("collision_weight must be finite and nonnegative")
        if not np.isfinite(self.yaw_snap_weight) or self.yaw_snap_weight < 0.0:
            raise ValueError("yaw_snap_weight must be finite and nonnegative")
        if self.hard_constraint_edge_samples < 2:
            raise ValueError("hard_constraint_edge_samples must be at least two")
        if not np.isfinite(self.hard_constraint_tolerance) or self.hard_constraint_tolerance < 0.0:
            raise ValueError("hard_constraint_tolerance must be finite and nonnegative")
        if self.hard_solver_iteration_limit < 1:
            raise ValueError("hard_solver_iteration_limit must be positive")


__all__ = ["WBSCOptimizationConfig"]
