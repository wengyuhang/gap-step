"""Conditionally safety-augmented TOGT objective with full analytic gradient."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    add_traversal_time_gradients,
    backpropagate_to_k,
)


@dataclass(frozen=True)
class ConditionalEvaluation:
    cost: float
    gradient: np.ndarray
    base_cost: float
    safety_integral: float


class SafetyAugmentedTOGTObjective:
    def __init__(self, base, safety_integral):
        self.base = base
        self.safety = safety_integral
        self.config = base.config
        self.dimension = base.dimension
        self.invalid_trial_count = 0
        self.last_evaluation = None

    def initial_guess(self):
        return self.base.initial_guess()

    def forward(self, x):
        return self.base.forward(x)

    def evaluate(self, x):
        values = np.asarray(x, dtype=float)
        base_cost, base_gradient = self.base.value_and_gradient(values)
        forward = self.base.forward(values)
        safety = self.safety.evaluate(forward.trajectory, with_gradient=True)
        k, _, _, _, _, _, jacobians, rates = self.base._geometry(values)
        spatial_gradient = tuple(
            jacobians[index].T @ safety.waypoint_gradient[index]
            for index in range(self.base.window_count)
        )
        traversal_gradient = np.einsum(
            "ij,ij->i", safety.waypoint_gradient, rates
        )
        duration_gradient = add_traversal_time_gradients(
            safety.duration_gradient, traversal_gradient
        )
        safety_gradient = np.concatenate((
            backpropagate_to_k(k, duration_gradient), *spatial_gradient,
        ))
        weight = self.safety.config.objective_weight
        result = ConditionalEvaluation(
            cost=float(base_cost + weight * safety.value),
            gradient=base_gradient + weight * safety_gradient,
            base_cost=float(base_cost),
            safety_integral=float(safety.value),
        )
        self.last_evaluation = result
        return result

    def value_and_gradient(self, x):
        values = np.asarray(x, dtype=float)
        try:
            result = self.evaluate(values)
            return result.cost, result.gradient
        except (ValueError, RuntimeError, FloatingPointError, OverflowError,
                np.linalg.LinAlgError):
            self.invalid_trial_count += 1
            clipped = np.clip(values, -1e6, 1e6)
            scale = self.config.invalid_trial_cost
            return scale * (1.0 + 1e-12 * float(clipped @ clipped)), 2e-12 * scale * clipped

    scipy_value_and_gradient = value_and_gradient


__all__ = ["ConditionalEvaluation", "SafetyAugmentedTOGTObjective"]
