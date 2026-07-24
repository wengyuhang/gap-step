"""Deterministic per-seed closed-loop scenarios for the FAPP-PPO curriculum."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .config import EnvironmentConfig, QuadrotorConfig
from .dynamics import QuadrotorDynamics, QuadrotorState
from .geometry import DeformableWindow, radial_boundary, rotation_from_normal


Array = np.ndarray
STAGES = ("static", "moving", "deforming", "full")


@dataclass(frozen=True)
class ClosedLoopScenario:
    name: str
    seed: int
    initial_state: QuadrotorState
    windows: tuple[DeformableWindow, ...]
    order: tuple[int, ...]
    horizon: float


def _stage_parameters(stage: str, full_windows: int) -> tuple[int, float, float, float]:
    if stage == "static":
        return min(2, full_windows), 0.0, 0.0, 0.0
    if stage == "moving":
        return min(3, full_windows), 0.18, 0.08, 0.0
    if stage == "deforming":
        return min(3, full_windows), 0.24, 0.12, 0.65
    if stage == "full":
        return full_windows, 0.32, 0.18, 1.0
    raise ValueError(f"unknown curriculum stage {stage!r}; expected one of {STAGES}")


def _smooth_random_walk(
    rng: np.random.Generator,
    count: int,
    dimensions: int,
    scale: float,
) -> Array:
    increments = rng.normal(0.0, scale, size=(count - 1, dimensions))
    values = np.zeros((count, dimensions), dtype=float)
    values[1:] = np.cumsum(increments, axis=0)
    values -= np.linspace(0.0, 1.0, count)[:, None] * values.mean(axis=0)[None, :]
    return values


def _independent_opportunity_intervals(
    rng: np.random.Generator,
    environment: EnvironmentConfig,
) -> tuple[tuple[float, float], ...]:
    """Sample one window's exogenous schedule without route or vehicle inputs."""

    if environment.opportunity_mode == "always_open":
        return ((0.0, environment.episode_seconds),)

    horizon = environment.episode_seconds
    boundary_guard = max(0.15, environment.opportunity_transition)

    def sample_width() -> float:
        half_jitter = 0.5 * environment.opportunity_schedule_jitter
        return float(
            max(
                3.0 * environment.dt,
                environment.opportunity_width
                + rng.uniform(-half_jitter, half_jitter),
            )
        )

    if environment.opportunity_mode == "single_shot":
        width = sample_width()
        latest_start = horizon - boundary_guard - width
        if latest_start <= boundary_guard:
            raise ValueError("episode is too short for an independent single opportunity")
        start = float(rng.uniform(boundary_guard, latest_start))
        return ((start, start + width),)

    # Independent renewal process. Each window gets its own phase, widths, and
    # non-periodic recurrence gaps. Nothing in this process queries the route,
    # the policy, the vehicle state, or another window.
    mean_recurrence = environment.opportunity_rescue_delay
    phase = float(rng.uniform(boundary_guard, boundary_guard + mean_recurrence))
    intervals: list[tuple[float, float]] = []
    start = phase
    while start < horizon - boundary_guard:
        width = sample_width()
        end = min(horizon - boundary_guard, start + width)
        if end - start > 2.0 * environment.dt:
            intervals.append((start, end))
        recurrence_jitter = max(
            2.0 * environment.opportunity_schedule_jitter,
            0.12 * mean_recurrence,
        )
        recurrence = float(
            mean_recurrence
            + rng.uniform(-recurrence_jitter, recurrence_jitter)
        )
        recurrence = max(
            recurrence,
            width + 2.0 * environment.opportunity_transition + 2.0 * environment.dt,
        )
        start += recurrence
    if not intervals:
        raise ValueError("episode is too short for an independent repeated opportunity")
    return tuple(intervals)


def _component_rng(seed: int, window_index: int, stream: int) -> np.random.Generator:
    """Stable independent random stream for one window and one component."""

    sequence = np.random.SeedSequence(
        [int(seed), 20_260_724, int(window_index), int(stream)]
    )
    return np.random.default_rng(sequence)


def _smoothstep(value: Array) -> Array:
    clipped = np.clip(value, 0.0, 1.0)
    return clipped * clipped * (3.0 - 2.0 * clipped)


def _opportunity_scale(
    times: Array,
    intervals: tuple[tuple[float, float], ...],
    environment: EnvironmentConfig,
) -> Array:
    if environment.opportunity_mode == "always_open":
        return np.ones_like(times, dtype=float)
    activation = np.zeros_like(times, dtype=float)
    transition = environment.opportunity_transition
    for start, end in intervals:
        rise = _smoothstep((times - (start - transition)) / transition)
        fall = _smoothstep(((end + transition) - times) / transition)
        activation = np.maximum(activation, np.minimum(rise, fall))
    return environment.opportunity_closed_scale + (
        environment.opportunity_open_scale - environment.opportunity_closed_scale
    ) * activation


def build_scenario(
    *,
    seed: int,
    stage: str,
    environment: EnvironmentConfig,
    quadrotor: QuadrotorConfig,
) -> ClosedLoopScenario:
    """Create a fixed, fully queryable, non-periodic future window instance."""

    count, translation_scale, rotation_scale, deformation_scale = _stage_parameters(
        stage, environment.full_windows
    )
    translation_scale *= environment.motion_amplitude_multiplier
    rotation_scale *= environment.motion_amplitude_multiplier
    deformation_scale *= environment.deformation_amplitude_multiplier
    start = np.array([0.0, -environment.route_radius, 1.6], dtype=float)
    angles = -0.5 * np.pi + 2.0 * np.pi * np.arange(1, count + 1) / (count + 1)
    centers0 = np.column_stack(
        (
            environment.route_radius * np.cos(angles),
            environment.route_radius * np.sin(angles),
            1.6 + 0.25 * np.sin(2.0 * angles),
        )
    )
    route_points = np.vstack((start, centers0, start))
    schedules = tuple(
        _independent_opportunity_intervals(
            _component_rng(seed, index, 0), environment
        )
        for index in range(count)
    )
    if environment.opportunity_mode == "always_open":
        keyframe_times = np.linspace(0.0, environment.episode_seconds, 7)
    else:
        keyframe_count = int(np.ceil(environment.episode_seconds / 0.30)) + 1
        keyframe_times = np.linspace(
            0.0, environment.episode_seconds, keyframe_count
        )
    windows: list[DeformableWindow] = []
    for index, base_center in enumerate(centers0):
        pose_rng = _component_rng(seed, index, 1)
        shape_rng = _component_rng(seed, index, 2)
        time_density_factor = np.sqrt(6.0 / max(len(keyframe_times) - 1, 1))
        approach = route_points[index + 1] - route_points[index]
        nominal_rotation = rotation_from_normal(approach)
        nominal_rotvec = Rotation.from_matrix(nominal_rotation).as_rotvec()

        center_walk = _smooth_random_walk(
            pose_rng,
            len(keyframe_times),
            3,
            max(translation_scale * 0.45 * time_density_factor, 1.0e-12),
        )
        center_walk[:, 2] *= 0.55
        center_norms = np.linalg.norm(center_walk, axis=1, keepdims=True)
        center_walk *= np.minimum(
            1.0, translation_scale / np.maximum(center_norms, 1.0e-12)
        )
        centers = base_center[None, :] + center_walk
        rotation_walk = _smooth_random_walk(
            pose_rng,
            len(keyframe_times),
            3,
            rotation_scale * time_density_factor,
        )
        rotation_norms = np.linalg.norm(rotation_walk, axis=1, keepdims=True)
        rotation_walk *= np.minimum(
            1.0,
            (2.5 * rotation_scale) / np.maximum(rotation_norms, 1.0e-12),
        )
        rotation_vectors = nominal_rotvec[None, :] + rotation_walk

        base_coefficients = np.array(
            [0.10 + 0.025 * (index % 2), 0.13, 0.08, 0.04, 0.07],
            dtype=float,
        )
        coefficient_walk = _smooth_random_walk(
            shape_rng,
            len(keyframe_times),
            5,
            0.025 * deformation_scale * time_density_factor,
        )
        coefficients = base_coefficients[None, :] + coefficient_walk
        coefficients = np.clip(coefficients, -0.22, 0.22)
        scale_walk = _smooth_random_walk(
            shape_rng,
            len(keyframe_times),
            2,
            0.035 * deformation_scale * time_density_factor,
        )
        scale_walk = np.clip(scale_walk, -0.20, 0.20)
        timing_scale = _opportunity_scale(
            keyframe_times, schedules[index], environment
        )
        radii = (
            np.array([1.05, 0.88])[None, :]
            * (1.0 + scale_walk)
            * timing_scale[:, None]
        )
        boundaries = np.stack(
            [
                radial_boundary(
                    vertices=64,
                    radius_x=float(radius[0]),
                    radius_y=float(radius[1]),
                    coefficients=coefficient,
                )
                for radius, coefficient in zip(radii, coefficients)
            ],
            axis=0,
        )
        windows.append(
            DeformableWindow(
                name=f"G{index + 1}",
                keyframe_times=keyframe_times,
                centers=centers,
                rotation_vectors=rotation_vectors,
                boundary_keyframes=boundaries,
                safe_margin=environment.safe_margin,
                frame_thickness=environment.frame_thickness
                + environment.drone_radius,
                planned_opportunities=schedules[index],
                minimum_safe_area=environment.minimum_safe_area,
            )
        )

    initial_state = QuadrotorDynamics(quadrotor).hover_state(start)
    scenario = ClosedLoopScenario(
        name=f"{stage}_closed_loop_{count}",
        seed=seed,
        initial_state=initial_state,
        windows=tuple(windows),
        order=tuple(range(count)),
        horizon=environment.episode_seconds,
    )
    validate_window_separation(scenario, environment.min_window_separation)
    return scenario


def validate_window_separation(
    scenario: ClosedLoopScenario,
    minimum_separation: float,
    samples: int = 31,
) -> None:
    """Conservative center/radius test for pairwise safe-envelope separation."""

    for time in np.linspace(0.0, scenario.horizon, samples):
        states = [window.state(float(time)) for window in scenario.windows]
        radii = [
            float(np.max(np.linalg.norm(state.boundary, axis=1)))
            for state in states
        ]
        for i in range(len(states)):
            for j in range(i + 1, len(states)):
                center_distance = float(np.linalg.norm(states[i].center - states[j].center))
                clearance = center_distance - radii[i] - radii[j]
                if clearance < minimum_separation:
                    raise ValueError(
                        f"window envelopes {i}/{j} violate separation at t={time:.3f}: "
                        f"{clearance:.3f} < {minimum_separation:.3f}"
                    )
