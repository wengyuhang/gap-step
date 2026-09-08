"""TOGT-paper cubic positive-part penalty for composite trajectories.

Equation (12) of the TOGT paper integrates ``max(h_Psi, 0)^3``.  The
repository's reproduction objective intentionally uses the later C++
``smoothedL1`` implementation instead, so this module keeps the paper-form
kernel local to the interpolated-method comparison and does not change the
retained reproduction.
"""

from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicCheckSampling,
    DynamicLimits,
    PenaltyBreakdown,
    PenaltyWeights,
    QuadrotorParameters,
    constant_yaw_profile,
    cubic_positive_part,
    dynamic_check_interval_count,
    flatness_map,
)


def _centered_interval_cubic(value, lower: float, upper: float):
    mean = 0.5 * (upper + lower)
    radius = 0.5 * (upper - lower)
    return cubic_positive_part((value - mean) ** 2 - radius**2)


def _optional_centered_interval_cubic(value, lower: float, upper: float):
    if not (np.isfinite(lower) and np.isfinite(upper)):
        return np.zeros((), dtype=np.result_type(value, float))[()]
    return _centered_interval_cubic(value, lower, upper)


def instantaneous_paper_penalty(
    velocity,
    flatness,
    *,
    limits: DynamicLimits | None = None,
    weights: PenaltyWeights | None = None,
) -> PenaltyBreakdown:
    """Apply the paper's cubic positive-part kernel to retained residuals."""

    constraint_limits = DynamicLimits() if limits is None else limits
    penalty_weights = PenaltyWeights() if weights is None else weights
    velocity_array = np.asarray(velocity)
    if velocity_array.shape != (3,):
        raise ValueError("velocity must have shape (3,)")

    velocity_residual = (
        np.sum(velocity_array * velocity_array)
        - constraint_limits.max_velocity**2
    )
    velocity_cost = penalty_weights.velocity * cubic_positive_part(
        velocity_residual
    )

    collective_cost = penalty_weights.collective_thrust * (
        _optional_centered_interval_cubic(
            flatness.collective_thrust,
            constraint_limits.min_collective_thrust,
            constraint_limits.max_collective_thrust,
        )
    )

    body_rate = flatness.body_rate
    xy_residual = (
        np.sum(body_rate[:2] * body_rate[:2])
        - constraint_limits.max_body_rate_xy**2
    )
    z_residual = body_rate[2] ** 2 - constraint_limits.max_body_rate_z**2
    body_rate_cost = penalty_weights.body_rate * (
        cubic_positive_part(xy_residual) + cubic_positive_part(z_residual)
    )

    rotor_cost = np.zeros(
        (), dtype=np.result_type(flatness.rotor_thrusts, float)
    )
    for rotor_thrust in flatness.rotor_thrusts:
        rotor_cost = rotor_cost + _centered_interval_cubic(
            rotor_thrust,
            constraint_limits.min_rotor_thrust,
            constraint_limits.max_rotor_thrust,
        )
    rotor_cost = penalty_weights.rotor_thrust * rotor_cost

    return PenaltyBreakdown(
        velocity=velocity_cost,
        collective_thrust=collective_cost,
        body_rate=body_rate_cost,
        rotor_thrust=rotor_cost,
    )


def integrated_paper_dynamic_penalty(
    trajectory,
    *,
    parameters: QuadrotorParameters | None = None,
    limits: DynamicLimits | None = None,
    weights: PenaltyWeights | None = None,
    samples_per_segment: int | None = None,
    dynamic_sampling: DynamicCheckSampling | None = None,
    yaw_profile=None,
    return_breakdown: bool = False,
):
    """Evaluate the sampled sum in TOGT equation (12).

    Every node ``j = 0, ..., kappa_i`` receives the displayed paper weight
    ``Delta t_i``.  In particular, this deliberately does not reuse the
    trapezoidal endpoint weights of the later C++ reproduction.
    """

    sampling = (
        DynamicCheckSampling() if dynamic_sampling is None else dynamic_sampling
    )
    if samples_per_segment is not None and samples_per_segment < 2:
        raise ValueError("samples_per_segment must be at least two")
    params = QuadrotorParameters() if parameters is None else parameters
    constraint_limits = DynamicLimits() if limits is None else limits
    penalty_weights = PenaltyWeights() if weights is None else weights
    profile = constant_yaw_profile() if yaw_profile is None else yaw_profile

    durations = trajectory.durations
    dtype = np.result_type(
        durations.dtype, trajectory.coefficients.dtype, float
    )
    integrated = [np.zeros((), dtype=dtype) for _ in range(4)]
    elapsed = np.zeros((), dtype=dtype)
    for segment, duration in enumerate(durations):
        interval_count = (
            dynamic_check_interval_count(duration, sampling)
            if samples_per_segment is None
            else samples_per_segment - 1
        )
        fractions = np.linspace(0.0, 1.0, interval_count + 1)
        step = duration / interval_count
        for fraction in fractions:
            local_time = duration * fraction
            global_time = elapsed + local_time
            yaw, yaw_rate, yaw_acceleration = profile(global_time)
            flatness = flatness_map(
                trajectory.evaluate_segment(segment, local_time, 2),
                trajectory.evaluate_segment(segment, local_time, 3),
                trajectory.evaluate_segment(segment, local_time, 4),
                yaw=yaw,
                yaw_rate=yaw_rate,
                yaw_acceleration=yaw_acceleration,
                parameters=params,
            )
            point_penalty = instantaneous_paper_penalty(
                trajectory.evaluate_segment(segment, local_time, 1),
                flatness,
                limits=constraint_limits,
                weights=penalty_weights,
            )
            for component, value in enumerate(
                (
                    point_penalty.velocity,
                    point_penalty.collective_thrust,
                    point_penalty.body_rate,
                    point_penalty.rotor_thrust,
                )
            ):
                integrated[component] = (
                    integrated[component]
                    + step * value
                )
        elapsed = elapsed + duration

    breakdown = PenaltyBreakdown(*[value[()] for value in integrated])
    return breakdown if return_breakdown else breakdown.total


__all__ = [
    "instantaneous_paper_penalty",
    "integrated_paper_dynamic_penalty",
]
