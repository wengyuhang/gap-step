"""Deterministic continuously deforming scenario generation."""

from __future__ import annotations

import math

import numpy as np

from .config import DEFAULT_DIFFICULTIES, MDGConfig
from .dynamic_gate import EndpointState, Scenario, SplineSeries
from .gate_shapes import GATE_TYPES


SHAPE_ORDER = ("l_shape", "u_shape", "star", "crescent", "wave")


def _control_times(horizon: float, step: float) -> np.ndarray:
    count = int(np.ceil(horizon / step))
    return np.linspace(0.0, horizon, count + 1)


def _nominal_layout(count: int, world_size: tuple[float, float, float]) -> np.ndarray:
    if count <= 12:
        angles = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
        radius = min(world_size[0], world_size[1]) * 0.34
        return np.column_stack(
            (
                radius * np.cos(angles),
                radius * np.sin(angles),
                1.9 + 0.35 * np.sin(2.0 * angles),
            )
        )
    columns = int(np.ceil(np.sqrt(count * world_size[0] / world_size[1])))
    rows = int(np.ceil(count / columns))
    xs = np.linspace(-0.42 * world_size[0], 0.42 * world_size[0], columns)
    ys = np.linspace(-0.42 * world_size[1], 0.42 * world_size[1], rows)
    route: list[tuple[float, float, float]] = []
    for row, y in enumerate(ys):
        row_x = xs if row % 2 == 0 else xs[::-1]
        for x in row_x:
            route.append((float(x), float(y), 1.8 + 0.5 * ((row + len(route)) % 3) / 2))
    return np.asarray(route[:count], dtype=float)


def _closed_scale_controls(
    rng: np.random.Generator,
    times: np.ndarray,
    scale_range: tuple[float, float],
    target: float,
    phase: int,
) -> np.ndarray:
    values = rng.uniform(scale_range[0], scale_range[1], len(times))
    values[0] = values[1]
    values[-1] = values[-2]
    if target <= 0.0:
        return values
    closed_count = max(1, int(round(target * (len(times) - 1))))
    start = 1 + phase % max(1, len(times) - closed_count - 1)
    stop = min(len(times) - 1, start + closed_count)
    values[start:stop] = 0.14
    if start > 1:
        values[start - 1] = min(values[start - 1], 0.48)
    if stop < len(times) - 1:
        values[stop] = min(values[stop], 0.48)
    return values


def generate_scenario(
    config: MDGConfig,
    *,
    seed: int = 0,
    gate_count: int = 8,
    difficulty: str = "medium",
    closed_ratio: float = 0.20,
    shape: str | None = None,
    horizon: float | None = None,
) -> Scenario:
    if difficulty not in DEFAULT_DIFFICULTIES:
        raise ValueError(f"unknown difficulty {difficulty!r}")
    if not 0.0 <= closed_ratio <= 0.95:
        raise ValueError("closed_ratio must lie in [0, 0.95]")
    settings = DEFAULT_DIFFICULTIES[difficulty]
    duration = config.scenario.planning_horizon if horizon is None else float(horizon)
    times = _control_times(duration, config.scenario.motion_control_point_dt)
    nominal = _nominal_layout(gate_count, config.scenario.world_size)
    root = np.random.SeedSequence(seed)
    streams = root.spawn(gate_count)
    gates = []
    for index in range(gate_count):
        rng = np.random.default_rng(streams[index])
        kind = SHAPE_ORDER[index % len(SHAPE_ORDER)] if shape is None else shape
        if kind not in GATE_TYPES:
            raise ValueError(f"unknown gate shape {kind!r}")
        center_noise = rng.normal(0.0, settings.translation_amplitude / 2.5, (len(times), 3))
        center_noise[:, 2] *= 0.35
        center_values = nominal[index][None, :] + np.clip(
            center_noise,
            -settings.translation_amplitude,
            settings.translation_amplitude,
        )
        next_center = nominal[(index + 1) % gate_count]
        previous_center = nominal[(index - 1) % gate_count]
        tangent = next_center[:2] - previous_center[:2]
        yaw = math.atan2(float(tangent[1]), float(tangent[0]))
        base_rpy = np.array((0.0, np.pi / 2.0, yaw))
        rotation_amplitude = np.deg2rad(settings.rotation_amplitude_deg)
        rpy_values = base_rpy[None, :] + rng.uniform(
            -rotation_amplitude, rotation_amplitude, (len(times), 3)
        )
        scale_values = _closed_scale_controls(
            rng,
            times,
            (settings.scale_min, settings.scale_max),
            closed_ratio,
            phase=3 * index + seed,
        )
        deformation = rng.uniform(
            -settings.shape_change_ratio,
            settings.shape_change_ratio,
            len(times),
        )
        gate_type = GATE_TYPES[kind]
        gate = gate_type(
            gate_id=index,
            name=f"G{index + 1}_{kind}",
            center_profile=SplineSeries(times, center_values),
            rpy_profile=SplineSeries(times, rpy_values),
            scale_profile=SplineSeries(times, scale_values),
            deformation_profile=SplineSeries(times, deformation),
            boundary_samples=config.scenario.curve_boundary_samples,
        )
        gate.validate_over(np.arange(0.0, duration + 1.0e-9, 0.10))
        gates.append(gate)
    start = EndpointState(
        position=np.array((0.0, 0.0, 1.8)),
        velocity=np.zeros(3),
        acceleration=np.zeros(3),
        jerk=np.zeros(3),
        yaw=0.0,
    )
    return Scenario(
        name=f"mdg_{difficulty}_{gate_count}g_close{int(round(100*closed_ratio)):02d}_seed{seed}",
        seed=seed,
        horizon=duration,
        gates=tuple(gates),
        order=tuple(range(gate_count)),
        start=start,
        difficulty=difficulty,
        closed_ratio=closed_ratio,
    )


def measured_closed_ratio(
    scenario: Scenario, config: MDGConfig, gate_index: int
) -> float:
    times = np.arange(
        0.0,
        scenario.horizon + 0.5 * config.tracking.gate_sample_dt,
        config.tracking.gate_sample_dt,
    )
    gate = scenario.gates[gate_index]
    closed = [
        gate.safe_polygon(float(time), config.safety.safety_radius).is_empty
        for time in times
    ]
    return float(np.mean(closed))


__all__ = ["SHAPE_ORDER", "generate_scenario", "measured_closed_ratio"]
