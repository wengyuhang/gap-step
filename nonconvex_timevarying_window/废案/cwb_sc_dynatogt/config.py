"""Configuration for continuous whole-body SC-DynaTOGT safety."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WholeBodySafetyConfig:
    """Numerical settings for cuboid section verification and repair.

    V1 uses sampled conservative motion estimates, so a successful check is
    called ``NUMERICALLY_VERIFIED`` and never a formal certificate.
    """

    half_extents: tuple[float, float, float] = (0.26504, 0.12761, 0.05890)
    sc_safe_radius: float = 0.98
    plane_epsilon: float = 1.0e-9
    dedup_epsilon: float = 1.0e-8
    time_tolerance: float = 1.0e-4
    lambda_tolerance: float = 1.0e-4
    sc_inverse_tolerance: float = 1.0e-9
    max_interval_depth: int = 24
    max_outer_iterations: int = 12
    max_witnesses_per_round: int = 4
    safety_penalty_weight: float = 1.0e4
    velocity_inflation: float = 1.25
    certificate_epsilon: float = 1.0e-8
    interval_scan_steps: int = 64
    finite_difference_step: float = 2.0e-6
    tau_merge_tolerance: float = 2.0e-3
    lambda_merge_tolerance: float = 2.0e-3

    def __post_init__(self) -> None:
        extents = np.asarray(self.half_extents, dtype=float)
        if extents.shape != (3,) or not np.all(np.isfinite(extents)) or np.any(extents <= 0.0):
            raise ValueError("half_extents must contain three finite positive values")
        if not np.isfinite(self.sc_safe_radius) or not 0.0 < self.sc_safe_radius < 1.0:
            raise ValueError("sc_safe_radius must lie strictly between zero and one")
        positive = (
            self.plane_epsilon,
            self.dedup_epsilon,
            self.time_tolerance,
            self.lambda_tolerance,
            self.sc_inverse_tolerance,
            self.safety_penalty_weight,
            self.velocity_inflation,
            self.finite_difference_step,
            self.tau_merge_tolerance,
            self.lambda_merge_tolerance,
        )
        if any(not np.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("all tolerances, weights, and inflation factors must be positive")
        if not np.isfinite(self.certificate_epsilon) or self.certificate_epsilon < 0.0:
            raise ValueError("certificate_epsilon must be finite and nonnegative")
        if self.max_interval_depth < 1 or self.max_outer_iterations < 1:
            raise ValueError("iteration and subdivision limits must be positive")
        if self.max_witnesses_per_round < 1 or self.interval_scan_steps < 8:
            raise ValueError("witness limit must be positive and interval_scan_steps at least eight")
