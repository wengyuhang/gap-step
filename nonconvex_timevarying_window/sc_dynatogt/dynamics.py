"""Quadrotor differential flatness and TOGT dynamic-constraint costs.

The flat outputs are position and yaw.  Position derivatives through snap
produce attitude, body rate, angular acceleration, collective thrust and an
approximation of the four individual rotor thrusts.  Constraint violations
use the ``smoothedL1`` implementation and residuals in the bundled TOGT
reproduction.  The paper's cubic positive part remains available separately.

The optimization path differentiates the complete MINCO solve and objective
with one float64 reverse-mode pass.  The holomorphic NumPy implementation is
kept as an independent complex-step validation backend.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Mapping

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .minco import BoundaryState, MincoSnap


_TOGT_SMOOTHING_EPSILON = 1.0e-2


def _default_inertia() -> NDArray[np.float64]:
    # QuadA in the TOGT reproduction package.
    return np.array([0.005, 0.005, 0.01], dtype=float)


def _analytic_squared_norm(vector: NDArray[np.generic]) -> np.generic:
    """Non-Hermitian squared norm, equal to Euclidean norm for real input."""

    return np.sum(vector * vector)


def _analytic_norm(vector: NDArray[np.generic]) -> np.generic:
    return np.sqrt(_analytic_squared_norm(vector))


def _check_real_norm(norm: complex | float, name: str, epsilon: float) -> None:
    if not np.isfinite(np.real(norm)) or np.real(norm) <= epsilon:
        raise ValueError(f"flatness singularity: {name} has near-zero norm")


def _normalize_with_derivatives(
    vector: NDArray[np.generic],
    first: NDArray[np.generic],
    second: NDArray[np.generic],
    *,
    name: str,
    epsilon: float,
) -> tuple[NDArray[np.generic], NDArray[np.generic], NDArray[np.generic]]:
    """Normalize a vector and analytically differentiate it twice."""

    norm = _analytic_norm(vector)
    _check_real_norm(norm, name, epsilon)
    dot = np.sum(vector * first)
    norm_dot = dot / norm
    norm_ddot = (
        (np.sum(first * first) + np.sum(vector * second)) / norm
        - dot * dot / norm**3
    )
    unit = vector / norm
    unit_dot = first / norm - vector * norm_dot / norm**2
    unit_ddot = (
        second / norm
        - 2.0 * first * norm_dot / norm**2
        - vector * norm_ddot / norm**2
        + 2.0 * vector * norm_dot * norm_dot / norm**3
    )
    return unit, unit_dot, unit_ddot


@dataclass
class QuadrotorParameters:
    """Physical parameters and rotor wrench allocation.

    ``mixing_matrix`` maps rotor thrusts to ``[collective, tau_x, tau_y,
    tau_z]``.  Its default matches ``standard_quad.yaml`` in the TOGT
    reproduction rather than assuming an abstract equal-torque vehicle.
    """

    mass: float = 1.0
    gravity: float = 9.8066
    inertia: ArrayLike = field(default_factory=_default_inertia)
    arm_length: float = 0.15
    yaw_moment_coefficient: float = 0.01
    mixing_matrix: ArrayLike | None = None
    singularity_epsilon: float = 1.0e-8

    def __post_init__(self) -> None:
        if not np.isfinite(self.mass) or self.mass <= 0.0:
            raise ValueError("mass must be finite and positive")
        if not np.isfinite(self.gravity) or self.gravity <= 0.0:
            raise ValueError("gravity must be finite and positive")
        inertia = np.asarray(self.inertia, dtype=float)
        if inertia.shape == (3,):
            if np.any(inertia <= 0.0):
                raise ValueError("principal inertias must be positive")
            inertia_matrix = np.diag(inertia)
        elif inertia.shape == (3, 3):
            if not np.allclose(inertia, inertia.T, atol=1.0e-12):
                raise ValueError("inertia matrix must be symmetric")
            if np.min(np.linalg.eigvalsh(inertia)) <= 0.0:
                raise ValueError("inertia matrix must be positive definite")
            inertia_matrix = inertia
        else:
            raise ValueError("inertia must have shape (3,) or (3, 3)")
        self.inertia = inertia_matrix

        if self.mixing_matrix is None:
            arm = float(self.arm_length)
            yaw = float(self.yaw_moment_coefficient)
            # Same signs/order as the explicit T_bm in standard_quad.yaml.
            mixing = np.array(
                [
                    [1.0, 1.0, 1.0, 1.0],
                    [arm, -arm, -arm, arm],
                    [-arm, -arm, arm, arm],
                    [yaw, -yaw, yaw, -yaw],
                ],
                dtype=float,
            )
        else:
            mixing = np.asarray(self.mixing_matrix, dtype=float)
        if mixing.shape != (4, 4) or not np.all(np.isfinite(mixing)):
            raise ValueError("mixing_matrix must be a finite 4 x 4 matrix")
        if abs(np.linalg.det(mixing)) <= 1.0e-12:
            raise ValueError("mixing_matrix must be invertible")
        self.mixing_matrix = mixing
        self.allocation_matrix = np.linalg.inv(mixing)


@dataclass(frozen=True)
class FlatnessState:
    """Quadrotor state/input quantities recovered from flat outputs."""

    rotation: NDArray[np.generic]
    body_rate: NDArray[np.generic]
    body_rate_derivative: NDArray[np.generic]
    collective_thrust: np.generic
    torque: NDArray[np.generic]
    rotor_thrusts: NDArray[np.generic]
    body_z: NDArray[np.generic]

    @property
    def omega(self) -> NDArray[np.generic]:
        return self.body_rate

    @property
    def omega_dot(self) -> NDArray[np.generic]:
        return self.body_rate_derivative


@dataclass(frozen=True)
class FlatnessSamples:
    time: NDArray[np.float64]
    rotation: NDArray[np.generic]
    body_rate: NDArray[np.generic]
    body_rate_derivative: NDArray[np.generic]
    collective_thrust: NDArray[np.generic]
    torque: NDArray[np.generic]
    rotor_thrusts: NDArray[np.generic]


def flatness_map(
    acceleration: ArrayLike,
    jerk: ArrayLike,
    snap: ArrayLike,
    *,
    yaw: complex | float = 0.0,
    yaw_rate: complex | float = 0.0,
    yaw_acceleration: complex | float = 0.0,
    parameters: QuadrotorParameters | None = None,
) -> FlatnessState:
    """Recover attitude and actuation from position PVAJS and yaw.

    The construction differentiates the orthonormal body frame twice.  It is
    equivalent to standard quadrotor differential flatness, while avoiding a
    numerical time derivative for angular acceleration/single-rotor thrust.
    """

    params = QuadrotorParameters() if parameters is None else parameters
    acceleration_array = np.asarray(acceleration)
    jerk_array = np.asarray(jerk)
    snap_array = np.asarray(snap)
    for name, value in (
        ("acceleration", acceleration_array),
        ("jerk", jerk_array),
        ("snap", snap_array),
    ):
        if value.shape != (3,):
            raise ValueError(f"{name} must have shape (3,), got {value.shape}")

    dtype = np.result_type(
        acceleration_array.dtype,
        jerk_array.dtype,
        snap_array.dtype,
        yaw,
        yaw_rate,
        yaw_acceleration,
        float,
    )
    gravity_vector = np.array([0.0, 0.0, params.gravity], dtype=dtype)
    specific_force = acceleration_array.astype(dtype, copy=False) + gravity_vector
    force_norm = _analytic_norm(specific_force)
    _check_real_norm(force_norm, "specific force", params.singularity_epsilon)

    body_z, body_z_dot, body_z_ddot = _normalize_with_derivatives(
        specific_force,
        jerk_array.astype(dtype, copy=False),
        snap_array.astype(dtype, copy=False),
        name="specific force",
        epsilon=params.singularity_epsilon,
    )

    cosine = np.cos(yaw)
    sine = np.sin(yaw)
    heading_x = np.array([cosine, sine, 0.0], dtype=dtype)
    heading_y = np.array([-sine, cosine, 0.0], dtype=dtype)
    heading_y_dot = -yaw_rate * heading_x
    heading_y_ddot = -yaw_acceleration * heading_x - yaw_rate**2 * heading_y

    raw_x = np.cross(heading_y, body_z)
    raw_x_dot = np.cross(heading_y_dot, body_z) + np.cross(
        heading_y, body_z_dot
    )
    raw_x_ddot = (
        np.cross(heading_y_ddot, body_z)
        + 2.0 * np.cross(heading_y_dot, body_z_dot)
        + np.cross(heading_y, body_z_ddot)
    )
    body_x, body_x_dot, body_x_ddot = _normalize_with_derivatives(
        raw_x,
        raw_x_dot,
        raw_x_ddot,
        name="heading/body-z cross product",
        epsilon=params.singularity_epsilon,
    )
    body_y = np.cross(body_z, body_x)
    body_y_dot = np.cross(body_z_dot, body_x) + np.cross(body_z, body_x_dot)
    body_y_ddot = (
        np.cross(body_z_ddot, body_x)
        + 2.0 * np.cross(body_z_dot, body_x_dot)
        + np.cross(body_z, body_x_ddot)
    )

    rotation = np.column_stack((body_x, body_y, body_z))
    rotation_dot = np.column_stack((body_x_dot, body_y_dot, body_z_dot))
    rotation_ddot = np.column_stack((body_x_ddot, body_y_ddot, body_z_ddot))
    omega_hat = rotation.T @ rotation_dot
    omega_hat_dot = rotation_dot.T @ rotation_dot + rotation.T @ rotation_ddot

    # Extract the skew part explicitly.  For real arithmetic the symmetric
    # residue is roundoff; retaining this expression also works analytically
    # under complex-step perturbations.
    body_rate = 0.5 * np.array(
        [
            omega_hat[2, 1] - omega_hat[1, 2],
            omega_hat[0, 2] - omega_hat[2, 0],
            omega_hat[1, 0] - omega_hat[0, 1],
        ]
    )
    body_rate_derivative = 0.5 * np.array(
        [
            omega_hat_dot[2, 1] - omega_hat_dot[1, 2],
            omega_hat_dot[0, 2] - omega_hat_dot[2, 0],
            omega_hat_dot[1, 0] - omega_hat_dot[0, 1],
        ]
    )

    collective_thrust = params.mass * force_norm
    angular_momentum = params.inertia @ body_rate
    torque = params.inertia @ body_rate_derivative + np.cross(
        body_rate, angular_momentum
    )
    wrench = np.concatenate((np.atleast_1d(collective_thrust), torque))
    rotor_thrusts = params.allocation_matrix @ wrench

    return FlatnessState(
        rotation=rotation,
        body_rate=body_rate,
        body_rate_derivative=body_rate_derivative,
        collective_thrust=collective_thrust,
        torque=torque,
        rotor_thrusts=rotor_thrusts,
        body_z=body_z,
    )


# Common descriptive aliases.
quadrotor_flatness = flatness_map
differential_flatness = flatness_map


YawProfile = Callable[[complex | float], tuple[complex | float, complex | float, complex | float]]


def constant_yaw_profile(yaw: float = 0.0) -> YawProfile:
    """Return a yaw profile compatible with the quadrature routines."""

    def profile(_time: complex | float) -> tuple[float, float, float]:
        return yaw, 0.0, 0.0

    # Mark this closure so the reverse-mode implementation can prove that yaw
    # has no time dependence.  Arbitrary custom profiles retain the original
    # complex-step path, preserving their existing semantics.
    setattr(profile, "_sc_dynatogt_constant_yaw", float(yaw))
    return profile


def flatness_from_trajectory(
    trajectory: MincoSnap,
    time: complex | float,
    *,
    yaw: complex | float = 0.0,
    yaw_rate: complex | float = 0.0,
    yaw_acceleration: complex | float = 0.0,
    parameters: QuadrotorParameters | None = None,
) -> FlatnessState:
    """Evaluate a solved MINCO trajectory and apply :func:`flatness_map`."""

    return flatness_map(
        trajectory.evaluate(time, 2),
        trajectory.evaluate(time, 3),
        trajectory.evaluate(time, 4),
        yaw=yaw,
        yaw_rate=yaw_rate,
        yaw_acceleration=yaw_acceleration,
        parameters=parameters,
    )


def sample_flatness(
    trajectory: MincoSnap,
    times: ArrayLike,
    *,
    parameters: QuadrotorParameters | None = None,
    yaw_profile: YawProfile | None = None,
) -> FlatnessSamples:
    """Sample flatness-derived attitude and actuation at global times."""

    grid = np.asarray(times, dtype=float)
    if grid.ndim != 1:
        raise ValueError("times must be one-dimensional")
    params = QuadrotorParameters() if parameters is None else parameters
    profile = constant_yaw_profile() if yaw_profile is None else yaw_profile
    states = []
    for instant in grid:
        yaw, yaw_rate, yaw_acceleration = profile(float(instant))
        states.append(
            flatness_from_trajectory(
                trajectory,
                float(instant),
                yaw=yaw,
                yaw_rate=yaw_rate,
                yaw_acceleration=yaw_acceleration,
                parameters=params,
            )
        )
    return FlatnessSamples(
        time=grid,
        rotation=np.stack([state.rotation for state in states]),
        body_rate=np.stack([state.body_rate for state in states]),
        body_rate_derivative=np.stack(
            [state.body_rate_derivative for state in states]
        ),
        collective_thrust=np.asarray(
            [state.collective_thrust for state in states]
        ),
        torque=np.stack([state.torque for state in states]),
        rotor_thrusts=np.stack([state.rotor_thrusts for state in states]),
    )


@dataclass(frozen=True)
class DynamicLimits:
    """Velocity, thrust and body-rate limits used by TOGT penalties."""

    max_velocity: float = 60.0
    min_collective_thrust: float = 0.0
    max_collective_thrust: float = np.inf
    max_body_rate_xy: float = 10.0
    max_body_rate_z: float = 10.0
    # QuadA / standard_planning.yaml in the bundled TOGT reproduction.
    min_rotor_thrust: float = 0.25
    max_rotor_thrust: float = 5.0

    def __post_init__(self) -> None:
        positive = {
            "max_velocity": self.max_velocity,
            "max_body_rate_xy": self.max_body_rate_xy,
            "max_body_rate_z": self.max_body_rate_z,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if self.min_collective_thrust > self.max_collective_thrust:
            raise ValueError("collective-thrust bounds are reversed")
        if self.min_rotor_thrust > self.max_rotor_thrust:
            raise ValueError("rotor-thrust bounds are reversed")
        if not np.isfinite(self.min_rotor_thrust) or not np.isfinite(
            self.max_rotor_thrust
        ):
            raise ValueError("rotor-thrust bounds must be finite")


@dataclass(frozen=True)
class DynamicCheckSampling:
    """Duration-adaptive quadrature settings from ``standard_planning.yaml``.

    The C++ implementation calls the clamped integer ``numCheckPerPiece`` and
    then evaluates nodes ``j = 0, ..., numCheckPerPiece``.  Thus a value of
    eight means eight intervals and nine trapezoidal nodes.
    """

    check_time_sec: float = 0.05
    min_num_check: int = 8
    max_num_check: int = 32

    def __post_init__(self) -> None:
        if not np.isfinite(self.check_time_sec) or self.check_time_sec <= 0.0:
            raise ValueError("check_time_sec must be finite and positive")
        if self.min_num_check < 1:
            raise ValueError("min_num_check must be positive")
        if self.max_num_check < self.min_num_check:
            raise ValueError("max_num_check must not be smaller than min_num_check")


def dynamic_check_interval_count(
    duration: complex | float,
    sampling: DynamicCheckSampling | None = None,
) -> int:
    """Return the frozen dynamicConstCheck interval count for one piece.

    Complex-step perturbations deliberately select the active integer from the
    real duration.  Reverse mode calls this function with a detached real
    duration, so both gradient backends use exactly the same active set.
    """

    settings = DynamicCheckSampling() if sampling is None else sampling
    real_duration = float(np.real(duration))
    if not np.isfinite(real_duration) or real_duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    count = int(real_duration / settings.check_time_sec)
    return min(max(count, settings.min_num_check), settings.max_num_check)


def _quadrature_interval_count(
    duration: complex | float,
    samples_per_segment: int | None,
    sampling: DynamicCheckSampling,
) -> int:
    """Resolve explicit fixed nodes or the duration-adaptive default."""

    if samples_per_segment is None:
        return dynamic_check_interval_count(duration, sampling)
    if samples_per_segment < 2:
        raise ValueError("samples_per_segment must be at least two")
    return samples_per_segment - 1


@dataclass(frozen=True)
class PenaltyWeights:
    velocity: float = 1.0
    collective_thrust: float = 1.0
    body_rate: float = 1.0
    rotor_thrust: float = 1.0

    def __post_init__(self) -> None:
        for name in ("velocity", "collective_thrust", "body_rate", "rotor_thrust"):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} penalty weight must be nonnegative")


@dataclass(frozen=True)
class PenaltyBreakdown:
    velocity: np.generic
    collective_thrust: np.generic
    body_rate: np.generic
    rotor_thrust: np.generic

    @property
    def total(self) -> np.generic:
        return (
            self.velocity
            + self.collective_thrust
            + self.body_rate
            + self.rotor_thrust
        )


def cubic_positive_part(residual: complex | float | NDArray[np.generic]):
    """Return ``max(residual, 0)**3`` without breaking complex step.

    The active set is selected from the real part.  Away from the exact
    constraint boundary this is the analytic continuation of the cubic
    positive-part penalty and has the correct one-sided derivative at zero.
    """

    value = np.asarray(residual)
    result = np.where(np.real(value) > 0.0, value**3, np.zeros_like(value))
    if result.ndim == 0:
        return result[()]
    return result


def smoothed_l1(
    residual: complex | float | NDArray[np.generic],
    mu: float = _TOGT_SMOOTHING_EPSILON,
):
    """Evaluate the exact C++ ``smoothedL1`` reproduction penalty.

    Branches are selected from the real part so complex-step differentiation
    follows the same active branch as real arithmetic.  This function, rather
    than :func:`cubic_positive_part`, is used by the reproduction objective.
    """

    if not np.isfinite(mu) or mu <= 0.0:
        raise ValueError("mu must be finite and positive")
    value = np.asarray(residual)
    transition = (mu - 0.5 * value) * (value / mu) ** 3
    result = np.where(
        np.real(value) < 0.0,
        np.zeros_like(value),
        np.where(np.real(value) > mu, value - 0.5 * mu, transition),
    )
    if result.ndim == 0:
        return result[()]
    return result


def _centered_interval_penalty(
    value: np.generic, lower: float, upper: float
) -> np.generic:
    """C++ interval residual ``(value-mean)^2-radius^2`` and smoothed L1."""

    mean = 0.5 * (upper + lower)
    radius = 0.5 * (upper - lower)
    residual = (value - mean) ** 2 - radius**2
    return smoothed_l1(residual)


def _optional_centered_interval_penalty(
    value: np.generic, lower: float, upper: float
) -> np.generic:
    """Apply a centered interval only when both optional bounds are finite."""

    if not (np.isfinite(lower) and np.isfinite(upper)):
        return np.zeros((), dtype=np.result_type(value, float))[()]
    return _centered_interval_penalty(value, lower, upper)


def instantaneous_constraint_penalty(
    velocity: ArrayLike,
    flatness: FlatnessState,
    *,
    limits: DynamicLimits | None = None,
    weights: PenaltyWeights | None = None,
) -> PenaltyBreakdown:
    """Evaluate reproduction-compatible smoothed penalties at one state."""

    constraint_limits = DynamicLimits() if limits is None else limits
    penalty_weights = PenaltyWeights() if weights is None else weights
    velocity_array = np.asarray(velocity)
    if velocity_array.shape != (3,):
        raise ValueError("velocity must have shape (3,)")

    velocity_residual = (
        _analytic_squared_norm(velocity_array) - constraint_limits.max_velocity**2
    )
    velocity_cost = penalty_weights.velocity * smoothed_l1(velocity_residual)

    collective_cost = penalty_weights.collective_thrust * (
        _optional_centered_interval_penalty(
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
        smoothed_l1(xy_residual) + smoothed_l1(z_residual)
    )

    rotor_cost = np.zeros((), dtype=np.result_type(flatness.rotor_thrusts, float))
    for rotor_thrust in flatness.rotor_thrusts:
        rotor_cost = rotor_cost + _centered_interval_penalty(
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


def integrated_dynamic_penalty(
    trajectory: MincoSnap,
    *,
    parameters: QuadrotorParameters | None = None,
    limits: DynamicLimits | None = None,
    weights: PenaltyWeights | None = None,
    samples_per_segment: int | None = None,
    dynamic_sampling: DynamicCheckSampling | None = None,
    yaw_profile: YawProfile | None = None,
    return_breakdown: bool = False,
) -> np.generic | PenaltyBreakdown:
    """Integrate dynamic violations piecewise using trapezoidal quadrature.

    With the default ``samples_per_segment=None``, each piece follows TOGT's
    ``dynamicConstCheck`` rule.  An integer explicitly selects a fixed number
    of nodes and preserves the former API behavior.
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
    dtype = np.result_type(durations.dtype, trajectory.coefficients.dtype, float)
    integrated = [np.zeros((), dtype=dtype) for _ in range(4)]
    elapsed = np.zeros((), dtype=dtype)
    for segment, duration in enumerate(durations):
        interval_count = _quadrature_interval_count(
            duration, samples_per_segment, sampling
        )
        fractions = np.linspace(0.0, 1.0, interval_count + 1)
        step = duration / interval_count
        for node, fraction in enumerate(fractions):
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
            point_penalty = instantaneous_constraint_penalty(
                trajectory.evaluate_segment(segment, local_time, 1),
                flatness,
                limits=constraint_limits,
                weights=penalty_weights,
            )
            quadrature_weight = 0.5 if node in (0, interval_count) else 1.0
            values = (
                point_penalty.velocity,
                point_penalty.collective_thrust,
                point_penalty.body_rate,
                point_penalty.rotor_thrust,
            )
            for component, value in enumerate(values):
                integrated[component] = (
                    integrated[component] + quadrature_weight * step * value
                )
        elapsed = elapsed + duration

    breakdown = PenaltyBreakdown(*[value[()] for value in integrated])
    return breakdown if return_breakdown else breakdown.total


@dataclass(frozen=True)
class ObjectiveWeights:
    time: float = 1.0
    snap_energy: float = 0.0

    def __post_init__(self) -> None:
        if self.time < 0.0 or self.snap_energy < 0.0:
            raise ValueError("objective weights must be nonnegative")


def trajectory_objective(
    trajectory: MincoSnap,
    *,
    parameters: QuadrotorParameters | None = None,
    limits: DynamicLimits | None = None,
    penalty_weights: PenaltyWeights | None = None,
    objective_weights: ObjectiveWeights | None = None,
    samples_per_segment: int | None = None,
    dynamic_sampling: DynamicCheckSampling | None = None,
    yaw_profile: YawProfile | None = None,
) -> np.generic:
    """TOGT objective: total time, optional snap energy, and violations."""

    weights = ObjectiveWeights() if objective_weights is None else objective_weights
    return (
        weights.time * trajectory.total_time
        + weights.snap_energy * trajectory.snap_energy()
        + integrated_dynamic_penalty(
            trajectory,
            parameters=parameters,
            limits=limits,
            weights=penalty_weights,
            samples_per_segment=samples_per_segment,
            dynamic_sampling=dynamic_sampling,
            yaw_profile=yaw_profile,
        )
    )


def _constant_yaw_value(yaw_profile: YawProfile | None) -> float | None:
    """Return a constant yaw value, or ``None`` for a general profile."""

    if yaw_profile is None:
        return 0.0
    marker = getattr(yaw_profile, "_sc_dynatogt_constant_yaw", None)
    return None if marker is None else float(marker)


def _torch_polynomial_derivative(coefficients, time, derivative: int):
    """Evaluate one degree-seven piece without constructing a dense basis."""

    value = coefficients.new_zeros((3,))
    multiplier = 1
    for power in range(derivative, 8):
        if power == derivative:
            multiplier = 1
            for factor in range(power - derivative + 1, power + 1):
                multiplier *= factor
        elif derivative:
            # power!/(power-derivative)! from the preceding power's value.
            multiplier = multiplier * power // (power - derivative)
        value = value + multiplier * coefficients[power] * time ** (
            power - derivative
        )
    return value


def _torch_normalize_with_derivatives(
    vector, first, second, *, name: str, epsilon: float
):
    """Tensor equivalent of :func:`_normalize_with_derivatives`."""

    norm = (vector * vector).sum().sqrt()
    if not np.isfinite(float(norm.detach().cpu())) or float(norm.detach().cpu()) <= epsilon:
        raise ValueError(f"flatness singularity: {name} has near-zero norm")
    dot = (vector * first).sum()
    norm_dot = dot / norm
    norm_ddot = (
        ((first * first).sum() + (vector * second).sum()) / norm
        - dot * dot / norm**3
    )
    unit = vector / norm
    unit_dot = first / norm - vector * norm_dot / norm**2
    unit_ddot = (
        second / norm
        - 2.0 * first * norm_dot / norm**2
        - vector * norm_ddot / norm**2
        + 2.0 * vector * norm_dot * norm_dot / norm**3
    )
    return unit, unit_dot, unit_ddot


def _torch_flatness(acceleration, jerk, snap, yaw: float, parameters):
    """Constant-yaw flatness map used inside the reverse-mode objective."""

    # Torch is obtained from the tensor instead of imported at module load.
    torch = __import__("torch")
    gravity = acceleration.new_tensor([0.0, 0.0, parameters.gravity])
    specific_force = acceleration + gravity
    body_z, body_z_dot, body_z_ddot = _torch_normalize_with_derivatives(
        specific_force,
        jerk,
        snap,
        name="specific force",
        epsilon=parameters.singularity_epsilon,
    )

    yaw_tensor = acceleration.new_tensor(yaw)
    zero = acceleration.new_zeros(())
    heading_y = torch.stack(
        (-torch.sin(yaw_tensor), torch.cos(yaw_tensor), zero)
    )
    raw_x = torch.linalg.cross(heading_y, body_z, dim=0)
    raw_x_dot = torch.linalg.cross(heading_y, body_z_dot, dim=0)
    raw_x_ddot = torch.linalg.cross(heading_y, body_z_ddot, dim=0)
    body_x, body_x_dot, body_x_ddot = _torch_normalize_with_derivatives(
        raw_x,
        raw_x_dot,
        raw_x_ddot,
        name="heading/body-z cross product",
        epsilon=parameters.singularity_epsilon,
    )
    body_y = torch.linalg.cross(body_z, body_x, dim=0)
    body_y_dot = torch.linalg.cross(body_z_dot, body_x, dim=0) + torch.linalg.cross(
        body_z, body_x_dot, dim=0
    )
    body_y_ddot = (
        torch.linalg.cross(body_z_ddot, body_x, dim=0)
        + 2.0 * torch.linalg.cross(body_z_dot, body_x_dot, dim=0)
        + torch.linalg.cross(body_z, body_x_ddot, dim=0)
    )

    rotation = torch.stack((body_x, body_y, body_z), dim=1)
    rotation_dot = torch.stack((body_x_dot, body_y_dot, body_z_dot), dim=1)
    rotation_ddot = torch.stack((body_x_ddot, body_y_ddot, body_z_ddot), dim=1)
    omega_hat = rotation.T @ rotation_dot
    omega_hat_dot = rotation_dot.T @ rotation_dot + rotation.T @ rotation_ddot
    body_rate = 0.5 * torch.stack(
        (
            omega_hat[2, 1] - omega_hat[1, 2],
            omega_hat[0, 2] - omega_hat[2, 0],
            omega_hat[1, 0] - omega_hat[0, 1],
        )
    )
    body_rate_derivative = 0.5 * torch.stack(
        (
            omega_hat_dot[2, 1] - omega_hat_dot[1, 2],
            omega_hat_dot[0, 2] - omega_hat_dot[2, 0],
            omega_hat_dot[1, 0] - omega_hat_dot[0, 1],
        )
    )

    collective = parameters.mass * (specific_force * specific_force).sum().sqrt()
    inertia = torch.as_tensor(
        parameters.inertia,
        dtype=acceleration.dtype,
        device=acceleration.device,
    )
    angular_momentum = inertia @ body_rate
    torque = inertia @ body_rate_derivative + torch.linalg.cross(
        body_rate, angular_momentum, dim=0
    )
    allocation = torch.as_tensor(
        parameters.allocation_matrix,
        dtype=acceleration.dtype,
        device=acceleration.device,
    )
    rotor_thrusts = allocation @ torch.cat((collective.reshape(1), torque))
    return collective, body_rate, rotor_thrusts


def _torch_smoothed_l1(residual, mu: float = _TOGT_SMOOTHING_EPSILON):
    """Tensor equivalent of the reproduction's C++ ``smoothedL1``."""

    torch = __import__("torch")
    transition = (mu - 0.5 * residual) * (residual / mu) ** 3
    return torch.where(
        residual < 0.0,
        torch.zeros_like(residual),
        torch.where(residual > mu, residual - 0.5 * mu, transition),
    )


def _torch_centered_interval_penalty(value, lower: float, upper: float):
    mean = 0.5 * (upper + lower)
    radius = 0.5 * (upper - lower)
    return _torch_smoothed_l1((value - mean) ** 2 - radius**2)


def _torch_optional_centered_interval_penalty(
    value, lower: float, upper: float
):
    if not (np.isfinite(lower) and np.isfinite(upper)):
        return value.new_zeros(())
    return _torch_centered_interval_penalty(value, lower, upper)


def _torch_trajectory_objective(
    trajectory: MincoSnap,
    coefficients,
    durations,
    *,
    parameters: QuadrotorParameters | None,
    limits: DynamicLimits | None,
    penalty_weights: PenaltyWeights | None,
    objective_weights: ObjectiveWeights | None,
    samples_per_segment: int | None,
    dynamic_sampling: DynamicCheckSampling | None,
    yaw: float,
):
    """Complete real TOGT objective represented as one autograd graph."""

    if samples_per_segment is not None and samples_per_segment < 2:
        raise ValueError("samples_per_segment must be at least two")
    params = QuadrotorParameters() if parameters is None else parameters
    constraint_limits = DynamicLimits() if limits is None else limits
    penalties = PenaltyWeights() if penalty_weights is None else penalty_weights
    weights = ObjectiveWeights() if objective_weights is None else objective_weights
    sampling = (
        DynamicCheckSampling() if dynamic_sampling is None else dynamic_sampling
    )

    dynamic_cost = durations.new_zeros(())
    for segment, duration in enumerate(durations.unbind()):
        interval_count = _quadrature_interval_count(
            float(duration.detach().cpu()), samples_per_segment, sampling
        )
        fractions = np.linspace(0.0, 1.0, interval_count + 1)
        step = duration / interval_count
        for node, fraction in enumerate(fractions):
            local_time = duration * float(fraction)
            velocity = _torch_polynomial_derivative(
                coefficients[segment], local_time, 1
            )
            acceleration = _torch_polynomial_derivative(
                coefficients[segment], local_time, 2
            )
            jerk = _torch_polynomial_derivative(coefficients[segment], local_time, 3)
            snap = _torch_polynomial_derivative(coefficients[segment], local_time, 4)
            collective, body_rate, rotor_thrusts = _torch_flatness(
                acceleration, jerk, snap, yaw, params
            )

            velocity_residual = (
                (velocity * velocity).sum() - constraint_limits.max_velocity**2
            )
            point_cost = penalties.velocity * _torch_smoothed_l1(velocity_residual)
            point_cost = point_cost + penalties.collective_thrust * (
                _torch_optional_centered_interval_penalty(
                    collective,
                    constraint_limits.min_collective_thrust,
                    constraint_limits.max_collective_thrust,
                )
            )
            xy_residual = (
                (body_rate[:2] * body_rate[:2]).sum()
                - constraint_limits.max_body_rate_xy**2
            )
            z_residual = body_rate[2] ** 2 - constraint_limits.max_body_rate_z**2
            point_cost = point_cost + penalties.body_rate * (
                _torch_smoothed_l1(xy_residual)
                + _torch_smoothed_l1(z_residual)
            )
            rotor_cost = durations.new_zeros(())
            for rotor_thrust in rotor_thrusts.unbind():
                rotor_cost = rotor_cost + _torch_centered_interval_penalty(
                    rotor_thrust,
                    constraint_limits.min_rotor_thrust,
                    constraint_limits.max_rotor_thrust,
                )
            point_cost = point_cost + penalties.rotor_thrust * rotor_cost
            quadrature_weight = 0.5 if node in (0, interval_count) else 1.0
            dynamic_cost = dynamic_cost + quadrature_weight * step * point_cost

    return (
        weights.time * durations.sum()
        + weights.snap_energy
        * trajectory._torch_snap_energy(coefficients, durations)
        + dynamic_cost
    )


def objective_with_gradient(
    trajectory: MincoSnap,
    *,
    parameters: QuadrotorParameters | None = None,
    limits: DynamicLimits | None = None,
    penalty_weights: PenaltyWeights | None = None,
    objective_weights: ObjectiveWeights | None = None,
    samples_per_segment: int | None = None,
    dynamic_sampling: DynamicCheckSampling | None = None,
    yaw_profile: YawProfile | None = None,
    gradient_backend: str = "autodiff",
    complex_step: float = 1.0e-30,
) -> tuple[float, NDArray[np.float64], NDArray[np.float64]]:
    """Return objective and gradients w.r.t. waypoints and durations.

    The default ``gradient_backend='autodiff'`` uses one differentiable MINCO
    solve and one reverse pass, so the number of solves does not grow with the
    parameter dimension.  ``'complex_step'`` remains available as an
    independent numerical-gradient oracle.  General time-varying yaw callables
    automatically use that oracle; :func:`constant_yaw_profile` stays on the
    fast path.  ``samples_per_segment=None`` enables the original duration-
    adaptive dynamicConstCheck rule; an explicit integer fixes the node count.
    """

    def cost(candidate: MincoSnap):
        return trajectory_objective(
            candidate,
            parameters=parameters,
            limits=limits,
            penalty_weights=penalty_weights,
            objective_weights=objective_weights,
            samples_per_segment=samples_per_segment,
            dynamic_sampling=dynamic_sampling,
            yaw_profile=yaw_profile,
        )

    normalized_backend = gradient_backend.lower().replace("-", "_")
    if normalized_backend not in {
        "autodiff",
        "torch",
        "reverse_mode",
        "complex",
        "complex_step",
    }:
        raise ValueError(
            "gradient_backend must be 'autodiff' or 'complex_step', "
            f"got {gradient_backend!r}"
        )

    yaw = _constant_yaw_value(yaw_profile)
    use_complex_step = normalized_backend in {"complex", "complex_step"}
    if yaw is None:
        use_complex_step = True
    if use_complex_step:
        return trajectory.parameter_gradient(cost, step=complex_step)

    torch, points, durations, coefficients = trajectory._torch_parameterization()
    objective = _torch_trajectory_objective(
        trajectory,
        coefficients,
        durations,
        parameters=parameters,
        limits=limits,
        penalty_weights=penalty_weights,
        objective_weights=objective_weights,
        samples_per_segment=samples_per_segment,
        dynamic_sampling=dynamic_sampling,
        yaw=yaw,
    )
    point_gradient, time_gradient = torch.autograd.grad(
        objective, (points, durations), allow_unused=True
    )
    if point_gradient is None:
        point_array = np.zeros_like(trajectory.intermediate_points, dtype=float)
    else:
        point_array = point_gradient.detach().cpu().numpy()
    if time_gradient is None:  # pragma: no cover - time always participates
        time_array = np.zeros_like(trajectory.durations, dtype=float)
    else:
        time_array = time_gradient.detach().cpu().numpy()
    return float(objective.detach().cpu()), point_array, time_array


def build_objective_with_gradient(
    start_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike,
    end_state: BoundaryState | Mapping[str, ArrayLike] | ArrayLike,
    intermediate_points: ArrayLike,
    durations: ArrayLike,
    **kwargs,
) -> tuple[float, NDArray[np.float64], NDArray[np.float64]]:
    """Construct MINCO and evaluate an L-BFGS-ready value/gradient tuple."""

    trajectory = MincoSnap(
        start_state, end_state, intermediate_points, durations
    )
    return objective_with_gradient(trajectory, **kwargs)


def constraint_extrema(
    trajectory: MincoSnap,
    *,
    parameters: QuadrotorParameters | None = None,
    samples_per_segment: int = 33,
    yaw_profile: YawProfile | None = None,
) -> dict[str, float | NDArray[np.float64]]:
    """Report sampled extrema for diagnostics and visualizations."""

    samples = trajectory.sample(samples_per_segment=samples_per_segment)
    flat = sample_flatness(
        trajectory,
        samples.time,
        parameters=parameters,
        yaw_profile=yaw_profile,
    )
    speed = np.linalg.norm(np.real(samples.velocity), axis=1)
    body_rate = np.real(flat.body_rate)
    collective = np.real(flat.collective_thrust)
    rotors = np.real(flat.rotor_thrusts)
    return {
        "max_velocity": float(np.max(speed)),
        "max_body_rate_xy": float(np.max(np.linalg.norm(body_rate[:, :2], axis=1))),
        "max_abs_body_rate_z": float(np.max(np.abs(body_rate[:, 2]))),
        "min_collective_thrust": float(np.min(collective)),
        "max_collective_thrust": float(np.max(collective)),
        "min_rotor_thrust": np.min(rotors, axis=0),
        "max_rotor_thrust": np.max(rotors, axis=0),
    }


__all__ = [
    "DynamicCheckSampling",
    "DynamicLimits",
    "FlatnessSamples",
    "FlatnessState",
    "ObjectiveWeights",
    "PenaltyBreakdown",
    "PenaltyWeights",
    "QuadrotorParameters",
    "build_objective_with_gradient",
    "constant_yaw_profile",
    "constraint_extrema",
    "cubic_positive_part",
    "differential_flatness",
    "dynamic_check_interval_count",
    "flatness_from_trajectory",
    "flatness_map",
    "instantaneous_constraint_penalty",
    "integrated_dynamic_penalty",
    "objective_with_gradient",
    "quadrotor_flatness",
    "sample_flatness",
    "smoothed_l1",
    "trajectory_objective",
]
