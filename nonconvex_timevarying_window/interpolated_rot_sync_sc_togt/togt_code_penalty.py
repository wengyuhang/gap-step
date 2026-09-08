"""Penalty used by the released TOGT C++ implementation.

This transcribes the value-side behavior of ``TrajSolver::addPenaltyCost`` and
``QuadManifold::computeRobustPenalityCost`` from the bundled reproduction:
duration-adaptive 8--32 intervals, trapezoidal endpoint weights, smoothed-L1
constraint penalties, and the special inverted-attitude singularity branch.
Gradients remain the comparison adapter's existing finite differences because
the interpolated SC segment is not part of the released C++ implementation.
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
    dynamic_check_interval_count,
    smoothed_l1,
)


_CPP_SINGULARITY_THRESHOLD = 1.0e-3


def _centered_interval_penalty(value, lower: float, upper: float):
    mean = 0.5 * (upper + lower)
    radius = 0.5 * (upper - lower)
    return smoothed_l1((value - mean) ** 2 - radius**2)


def instantaneous_togt_code_penalty(
    velocity,
    acceleration,
    jerk,
    snap,
    *,
    yaw: float = 0.0,
    yaw_rate: float = 0.0,
    yaw_acceleration: float = 0.0,
    parameters: QuadrotorParameters | None = None,
    limits: DynamicLimits | None = None,
    weights: PenaltyWeights | None = None,
) -> PenaltyBreakdown:
    """Evaluate the released C++ robust penalty at one PVAJS state."""

    params = QuadrotorParameters() if parameters is None else parameters
    constraint_limits = DynamicLimits() if limits is None else limits
    penalty_weights = PenaltyWeights() if weights is None else weights
    velocity_array = np.asarray(velocity)
    acceleration_array = np.asarray(acceleration)
    jerk_array = np.asarray(jerk)
    snap_array = np.asarray(snap)
    for name, value in (
        ("velocity", velocity_array),
        ("acceleration", acceleration_array),
        ("jerk", jerk_array),
        ("snap", snap_array),
    ):
        if value.shape != (3,):
            raise ValueError(f"{name} must have shape (3,)")

    dtype = np.result_type(
        velocity_array,
        acceleration_array,
        jerk_array,
        snap_array,
        yaw,
        yaw_rate,
        yaw_acceleration,
        float,
    )
    alpha = acceleration_array.astype(dtype, copy=False) + np.asarray(
        (0.0, 0.0, params.gravity), dtype=dtype
    )
    alpha_norm = np.sqrt(np.sum(alpha * alpha))
    if not np.isfinite(float(np.real(alpha_norm))) or float(
        np.real(alpha_norm)
    ) <= params.singularity_epsilon:
        raise ValueError("flatness singularity: specific force has near-zero norm")
    body_z = alpha / alpha_norm
    collective_thrust = params.mass * alpha_norm

    velocity_residual = (
        np.sum(velocity_array * velocity_array)
        - constraint_limits.max_velocity**2
    )
    velocity_cost = penalty_weights.velocity * smoothed_l1(velocity_residual)

    # This is the exact branch condition in computeRobustPenalityCost.
    if abs(float(np.real(body_z[2] + 1.0))) <= _CPP_SINGULARITY_THRESHOLD:
        omega_xy_squared = (
            jerk_array[0] ** 2 + jerk_array[1] ** 2
        ) / (alpha_norm**2)
        body_cost = penalty_weights.body_rate * smoothed_l1(
            omega_xy_squared - constraint_limits.max_body_rate_xy**2
        )
        collective_cost = penalty_weights.rotor_thrust * (
            _centered_interval_penalty(
                collective_thrust,
                4.0 * constraint_limits.min_rotor_thrust,
                4.0 * constraint_limits.max_rotor_thrust,
            )
        )
        zero = np.zeros((), dtype=dtype)[()]
        return PenaltyBreakdown(
            velocity=velocity_cost,
            collective_thrust=collective_cost,
            body_rate=body_cost,
            rotor_thrust=zero,
        )

    alpha_dot_jerk = np.sum(alpha * jerk_array)
    jerk_norm_squared = np.sum(jerk_array * jerk_array)
    identity = np.eye(3, dtype=dtype)
    normal_jacobian = (
        identity / alpha_norm
        - np.outer(alpha, alpha) / alpha_norm**3
    )
    body_z_dot = normal_jacobian @ jerk_array
    body_z_ddot = (
        -2.0 * jerk_array * alpha_dot_jerk / alpha_norm**3
        - alpha * jerk_norm_squared / alpha_norm**3
        + alpha * 3.0 * alpha_dot_jerk**2 / alpha_norm**5
        + normal_jacobian @ snap_array
    )

    cosine, sine = np.cos(yaw), np.sin(yaw)
    denominator = body_z[2] + 1.0
    denominator_squared = denominator**2
    omega_term = body_z_dot[2] / denominator
    omega_term_1 = body_z[0] * sine - body_z[1] * cosine
    omega_term_2 = body_z[0] * cosine + body_z[1] * sine
    omega_term_3 = (
        body_z[1] * body_z_dot[0] - body_z[0] * body_z_dot[1]
    )
    body_rate = np.asarray(
        (
            body_z_dot[0] * sine
            - body_z_dot[1] * cosine
            - omega_term_1 * omega_term,
            body_z_dot[0] * cosine
            + body_z_dot[1] * sine
            - omega_term_2 * omega_term,
            omega_term_3 / denominator + yaw_rate,
        ),
        dtype=dtype,
    )

    omega_dot_term_1 = body_z_dot[0] * sine - body_z_dot[1] * cosine
    omega_dot_term_2 = body_z_dot[0] * cosine + body_z_dot[1] * sine
    omega_dot_term_3 = (
        body_z[1] * body_z_ddot[0] - body_z[0] * body_z_ddot[1]
    )
    body_rate_derivative = np.asarray(
        (
            body_z_ddot[0] * sine
            - body_z_ddot[1] * cosine
            - body_z_ddot[2] * omega_term_1 / denominator
            - body_z_dot[2] * omega_dot_term_1 / denominator
            + body_z_dot[2] ** 2 * omega_term_1 / denominator_squared,
            body_z_ddot[0] * cosine
            + body_z_ddot[1] * sine
            - body_z_ddot[2] * omega_term_2 / denominator
            - body_z_dot[2] * omega_dot_term_2 / denominator
            + body_z_dot[2] ** 2 * omega_term_2 / denominator_squared,
            omega_dot_term_3 / denominator
            - omega_term_3 * body_z_dot[2] / denominator_squared
            + yaw_acceleration,
        ),
        dtype=dtype,
    )

    inertia = params.inertia
    torque = inertia @ body_rate_derivative + np.cross(
        body_rate, inertia @ body_rate
    )
    wrench = np.concatenate((np.atleast_1d(collective_thrust), torque))
    rotor_thrusts = params.allocation_matrix @ wrench

    xy_residual = (
        np.sum(body_rate[:2] * body_rate[:2])
        - constraint_limits.max_body_rate_xy**2
    )
    z_residual = body_rate[2] ** 2 - constraint_limits.max_body_rate_z**2
    body_cost = penalty_weights.body_rate * (
        smoothed_l1(xy_residual) + smoothed_l1(z_residual)
    )
    rotor_cost = np.zeros((), dtype=dtype)
    for rotor_thrust in rotor_thrusts:
        rotor_cost = rotor_cost + _centered_interval_penalty(
            rotor_thrust,
            constraint_limits.min_rotor_thrust,
            constraint_limits.max_rotor_thrust,
        )
    rotor_cost = penalty_weights.rotor_thrust * rotor_cost
    zero = np.zeros((), dtype=dtype)[()]
    return PenaltyBreakdown(
        velocity=velocity_cost,
        collective_thrust=zero,
        body_rate=body_cost,
        rotor_thrust=rotor_cost,
    )


def integrated_togt_code_penalty(
    trajectory,
    *,
    parameters: QuadrotorParameters | None = None,
    limits: DynamicLimits | None = None,
    weights: PenaltyWeights | None = None,
    dynamic_sampling: DynamicCheckSampling | None = None,
    yaw_profile=None,
    return_breakdown: bool = False,
):
    """Integrate the released C++ penalty with its adaptive trapezoid grid."""

    params = QuadrotorParameters() if parameters is None else parameters
    constraint_limits = DynamicLimits() if limits is None else limits
    penalty_weights = PenaltyWeights() if weights is None else weights
    sampling = (
        DynamicCheckSampling() if dynamic_sampling is None else dynamic_sampling
    )
    profile = constant_yaw_profile() if yaw_profile is None else yaw_profile
    durations = trajectory.durations
    dtype = np.result_type(
        durations.dtype, trajectory.coefficients.dtype, float
    )
    integrated = [np.zeros((), dtype=dtype) for _ in range(4)]
    elapsed = np.zeros((), dtype=dtype)
    for segment, duration in enumerate(durations):
        interval_count = dynamic_check_interval_count(duration, sampling)
        fractions = np.linspace(0.0, 1.0, interval_count + 1)
        step = duration / interval_count
        for node, fraction in enumerate(fractions):
            local_time = duration * fraction
            yaw, yaw_rate, yaw_acceleration = profile(elapsed + local_time)
            point = instantaneous_togt_code_penalty(
                trajectory.evaluate_segment(segment, local_time, 1),
                trajectory.evaluate_segment(segment, local_time, 2),
                trajectory.evaluate_segment(segment, local_time, 3),
                trajectory.evaluate_segment(segment, local_time, 4),
                yaw=yaw,
                yaw_rate=yaw_rate,
                yaw_acceleration=yaw_acceleration,
                parameters=params,
                limits=constraint_limits,
                weights=penalty_weights,
            )
            node_weight = 0.5 if node in (0, interval_count) else 1.0
            for component, value in enumerate(
                (
                    point.velocity,
                    point.collective_thrust,
                    point.body_rate,
                    point.rotor_thrust,
                )
            ):
                integrated[component] = (
                    integrated[component] + node_weight * step * value
                )
        elapsed = elapsed + duration
    breakdown = PenaltyBreakdown(*[value[()] for value in integrated])
    return breakdown if return_breakdown else breakdown.total


__all__ = [
    "instantaneous_togt_code_penalty",
    "integrated_togt_code_penalty",
]
