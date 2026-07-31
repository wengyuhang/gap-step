"""Hungarian matching and dense safety validation for disc tracks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import linear_sum_assignment

from .config import MDGConfig
from .dynamic_gate import DynamicGate, Scenario
from .geometry import signed_clearance
from .models import Disc, DiscTrack
from .safe_disks import candidates_for_method


@dataclass
class _TrackBuilder:
    track_id: int
    steps: list[int]
    times: list[float]
    centers: list[np.ndarray]
    radii: list[float]

    @property
    def last_step(self) -> int:
        return self.steps[-1]


def _match_cost(
    previous_center: np.ndarray,
    previous_radius: float,
    current: Disc,
    config: MDGConfig,
) -> float:
    tracking = config.tracking
    return float(
        np.linalg.norm(previous_center - current.center)
        / tracking.match_distance_scale
        + tracking.match_radius_weight
        * abs(previous_radius - current.radius)
        / tracking.match_radius_scale
    )


def _raw_tracks(
    times: np.ndarray,
    samples: list[list[Disc]],
    config: MDGConfig,
) -> list[_TrackBuilder]:
    builders: list[_TrackBuilder] = []
    next_id = 0
    for step, (time, discs) in enumerate(zip(times, samples)):
        eligible = [
            item
            for item in builders
            if step - item.last_step <= config.tracking.max_gap_steps + 1
        ]
        matched_discs: set[int] = set()
        matched_tracks: set[int] = set()
        if eligible and discs:
            costs = np.asarray(
                [
                    [
                        _match_cost(item.centers[-1], item.radii[-1], disc, config)
                        for disc in discs
                    ]
                    for item in eligible
                ]
            )
            rows, columns = linear_sum_assignment(costs)
            for row, column in zip(rows, columns):
                if costs[row, column] > config.tracking.match_cost_max:
                    continue
                item = eligible[int(row)]
                disc = discs[int(column)]
                item.steps.append(step)
                item.times.append(float(time))
                item.centers.append(disc.center.copy())
                item.radii.append(float(disc.radius))
                matched_discs.add(int(column))
                matched_tracks.add(item.track_id)
        for index, disc in enumerate(discs):
            if index in matched_discs:
                continue
            builders.append(
                _TrackBuilder(
                    next_id,
                    [step],
                    [float(time)],
                    [disc.center.copy()],
                    [float(disc.radius)],
                )
            )
            next_id += 1
    return [
        item
        for item in builders
        if len(item.times) >= config.tracking.min_length_steps
    ]


def _validated_tracks(
    gate: DynamicGate,
    gate_id: int,
    raw: _TrackBuilder,
    config: MDGConfig,
    *,
    point_mode: bool,
    safety_cache: dict[float, object],
) -> list[DiscTrack]:
    def safe_at(time: float):
        key = round(float(time), 9)
        if key not in safety_cache:
            safety_cache[key] = gate.safe_polygon(
                float(time), config.safety.safety_radius
            )
        return safety_cache[key]

    times = np.asarray(raw.times)
    centers = np.asarray(raw.centers)
    radii = np.asarray(raw.radii)
    minimum = 1.0e-10 if point_mode else config.disks.min_radius
    maximum_gap = (
        config.tracking.max_gap_steps + 1.5
    ) * config.tracking.gate_sample_dt
    sample_clearance = np.asarray(
        [
            signed_clearance(
                safe_at(float(time)),
                center,
            )
            for time, center in zip(times, centers)
        ]
    )
    adjusted = radii.copy()
    if not point_mode and config.tracking.enable_validation_shrink:
        adjusted = np.minimum(
            adjusted, sample_clearance - config.tracking.shrink_margin
        )
    good = sample_clearance >= 0.0
    if not point_mode:
        good &= adjusted >= minimum

    runs: list[np.ndarray] = []
    current: list[int] = []
    for index in range(len(times)):
        contiguous = not current or times[index] - times[current[-1]] <= maximum_gap
        if good[index] and contiguous:
            current.append(index)
        else:
            if len(current) >= config.tracking.min_length_steps:
                runs.append(np.asarray(current, dtype=int))
            current = [index] if good[index] else []
    if len(current) >= config.tracking.min_length_steps:
        runs.append(np.asarray(current, dtype=int))

    output: list[DiscTrack] = []
    pending = list(runs)
    split_index = 0
    while pending:
        indices = pending.pop(0)
        run_times = times[indices]
        run_centers = centers[indices]
        run_radii = adjusted[indices]
        center_curve = PchipInterpolator(run_times, run_centers, axis=0)
        radius_curve = PchipInterpolator(run_times, run_radii)
        dense_count = max(
            2,
            int(
                np.ceil(
                    (run_times[-1] - run_times[0])
                    / config.tracking.validation_dt
                )
            )
            + 1,
        )
        dense = np.linspace(run_times[0], run_times[-1], dense_count)
        correction = 0.0
        invalid_time: float | None = None
        for time in dense:
            center = np.asarray(center_curve(time), dtype=float)
            radius = float(radius_curve(time))
            safe = safe_at(float(time))
            clearance = signed_clearance(safe, center)
            if clearance < 0.0:
                invalid_time = float(time)
                break
            correction = max(correction, radius - clearance)
        candidate_radii = run_radii.copy()
        if (
            correction > 0.0
            and not point_mode
            and config.tracking.enable_validation_shrink
        ):
            candidate_radii -= correction + config.tracking.shrink_margin
        bad_position: int | None = None
        if invalid_time is not None:
            bad_position = int(np.argmin(np.abs(run_times - invalid_time)))
        elif np.min(candidate_radii) < minimum:
            bad_position = int(np.argmin(candidate_radii))
        if bad_position is not None:
            for part in (indices[:bad_position], indices[bad_position + 1 :]):
                if len(part) >= config.tracking.min_length_steps:
                    pending.append(part)
            continue
        output.append(
            DiscTrack(
                gate_id,
                raw.track_id * 1000 + split_index,
                run_times,
                run_centers,
                candidate_radii,
                [(float(run_times[0]), float(run_times[-1]))],
            )
        )
        split_index += 1
    return output


def build_gate_tracks(
    gate: DynamicGate,
    config: MDGConfig,
    *,
    method: str = "mdg_free",
    horizon: float | None = None,
) -> list[DiscTrack]:
    duration = config.scenario.planning_horizon if horizon is None else horizon
    dt = 0.01 if method == "dense_oracle" else config.tracking.gate_sample_dt
    times = np.arange(0.0, duration + 0.5 * dt, dt)
    samples = [
        candidates_for_method(gate, float(time), config, method) for time in times
    ]
    raw = _raw_tracks(times, samples, config)
    point_mode = method in {"center", "uniform_point"}
    safety_cache: dict[float, object] = {}
    tracks: list[DiscTrack] = []
    for item in raw:
        tracks.extend(
            _validated_tracks(
                gate,
                gate.gate_id,
                item,
                config,
                point_mode=point_mode,
                safety_cache=safety_cache,
            )
        )
    return tracks


def build_scenario_tracks(
    scenario: Scenario,
    config: MDGConfig,
    *,
    method: str = "mdg_free",
) -> dict[int, list[DiscTrack]]:
    return {
        gate.gate_id: build_gate_tracks(
            gate, config, method=method, horizon=scenario.horizon
        )
        for gate in scenario.gates
    }

__all__ = ["build_gate_tracks", "build_scenario_tracks"]
