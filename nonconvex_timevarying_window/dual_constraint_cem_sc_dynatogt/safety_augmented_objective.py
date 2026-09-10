"""SC-DynaTOGT objective augmented with the new method's safety integral."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCMappingError
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    add_traversal_time_gradients,
    backpropagate_to_k,
)

from .safety_penalty import (
    SafetyPenaltyConfig,
    boundary_point_cloud,
    safety_penalty_with_gradient,
)


@dataclass(frozen=True)
class SafetyAugmentedEvaluation:
    cost: float
    gradient: np.ndarray
    base_cost: float
    safety_integral: float


class SafetyAugmentedSCObjective:
    """Add safety to L-BFGS while preserving the original SC/MINCO model."""

    def __init__(self, base_objective, windows, safety_config=None):
        self.base = base_objective
        self.joint = base_objective.joint
        self.windows = tuple(windows)
        self.safety_config = (
            SafetyPenaltyConfig() if safety_config is None else safety_config
        )
        self.config = base_objective.config
        self.dimension = base_objective.dimension
        self.boundary_clouds = tuple(
            boundary_point_cloud(window.physical_polygon,
                                 self.safety_config.edge_spacing_m)
            for window in self.windows
        )
        self.last_evaluation = None
        self.invalid_trial_count = 0

    def initial_guess(self):
        return self.base.initial_guess()

    def forward(self, x):
        return self.base.forward(x)

    def evaluate(self, x):
        base = self.joint.evaluate(x)
        safety, waypoint_gradient, direct_duration_gradient = (
            safety_penalty_with_gradient(
                base.forward.trajectory, self.windows, self.safety_config,
                boundary_clouds=self.boundary_clouds,
            )
        )
        traversal_gradient = np.zeros(self.joint.window_count, dtype=float)
        spatial_gradient = np.empty_like(base.forward.d)
        for index in range(self.joint.window_count):
            spatial_gradient[index] = (
                base.forward.waypoint_jacobians[index].T @ waypoint_gradient[index]
            )
            if self.config.include_window_time_gradient:
                traversal_gradient[index] = float(
                    waypoint_gradient[index]
                    @ base.forward.waypoint_time_derivatives[index]
                )
        accumulated = add_traversal_time_gradients(
            direct_duration_gradient, traversal_gradient
        )
        temporal_gradient = backpropagate_to_k(base.forward.k, accumulated)
        safety_gradient = np.concatenate(
            (temporal_gradient, spatial_gradient.reshape(-1))
        )
        weight = self.safety_config.objective_weight
        evaluation = SafetyAugmentedEvaluation(
            cost=float(base.cost + weight * safety),
            gradient=base.gradient + weight * safety_gradient,
            base_cost=float(base.cost),
            safety_integral=float(safety),
        )
        self.last_evaluation = evaluation
        return evaluation

    def value_and_gradient(self, x):
        values = np.asarray(x, dtype=float)
        try:
            evaluation = self.evaluate(values)
            return evaluation.cost, evaluation.gradient
        except (SCMappingError, np.linalg.LinAlgError, FloatingPointError,
                OverflowError, ValueError, RuntimeError):
            self.invalid_trial_count += 1
            clipped = np.clip(values, -1.0e6, 1.0e6)
            cost = self.config.invalid_trial_cost * (
                1.0 + 1.0e-12 * float(clipped @ clipped)
            )
            return cost, 2.0e-12 * self.config.invalid_trial_cost * clipped


__all__ = ["SafetyAugmentedEvaluation", "SafetyAugmentedSCObjective"]
