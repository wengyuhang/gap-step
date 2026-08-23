"""Adaptive V1 continuous-time verification of every cuboid section edge."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    QuadrotorParameters,
    YawProfile,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import ForwardPass

from .body_model import CuboidBody
from .config import WholeBodySafetyConfig
from .gate_frame import frame_at
from .plane_section import (
    CrossingInterval,
    PlaneSection,
    find_planned_crossing_interval,
    plane_section_at,
    split_into_topology_stable_intervals,
)
from .sc_inverse import inverse_sc_map


class VerificationStatus(Enum):
    """Safety result; V1 never emits ``CERTIFIED``."""

    SAFE = "safe"
    UNSAFE = "unsafe"
    NUMERICALLY_VERIFIED = "numerically_verified"
    CERTIFIED = "certified"
    UNCERTIFIED = "uncertified"
    NUMERICAL_FAILURE = "numerical_failure"


@dataclass(frozen=True)
class SafetyWitness:
    """A reproducible section-edge sample stored in segment-relative time."""

    window_index: int
    minco_segment_index: int
    normalized_time: float
    body_edge_a: tuple[int, int]
    body_edge_b: tuple[int, int]
    section_lambda: float
    sc_radius: float | None
    margin: float | None
    outside_sc_domain: bool
    topology_key: tuple[tuple[int, int], ...]
    local_point: np.ndarray
    world_point: np.ndarray


@dataclass(frozen=True)
class WindowSafetyReport:
    """Continuous-section V1 evidence for one prescribed window crossing."""

    window_index: int
    crossing_interval: CrossingInterval
    status: VerificationStatus
    minimum_margin: float | None
    witnesses: tuple[SafetyWitness, ...]
    checked_cells: int
    maximum_depth: int


@dataclass(frozen=True)
class TrajectorySafetyReport:
    """Aggregate whole-body report in prescribed traversal order."""

    status: VerificationStatus
    windows: tuple[WindowSafetyReport, ...]


@dataclass
class _Accumulator:
    checked_cells: int = 0
    maximum_depth: int = 0
    minimum_margin: float = float("inf")
    witnesses: list[SafetyWitness] = None  # type: ignore[assignment]
    numerical_failure: bool = False
    uncertified: bool = False

    def __post_init__(self) -> None:
        self.witnesses = []


def _segment_coordinates(durations: np.ndarray, time: float) -> tuple[int, float]:
    cumulative = np.concatenate(([0.0], np.cumsum(durations)))
    segment = min(int(np.searchsorted(cumulative[1:], time, side="right")), len(durations) - 1)
    tau = (time - cumulative[segment]) / durations[segment]
    return segment, float(np.clip(tau, 0.0, 1.0))


def _section_vertex(section: PlaneSection, source: tuple[int, int]):
    return next((vertex for vertex in section.vertices if vertex.source_body_edge == source), None)


def _make_witness(
    *,
    window_index: int,
    time: float,
    durations: np.ndarray,
    edge_a: tuple[int, int],
    edge_b: tuple[int, int],
    section_lambda: float,
    sc_radius: float | None,
    margin: float | None,
    outside: bool,
    topology_key: tuple[tuple[int, int], ...],
    local_point: np.ndarray,
    track: SCWindowTrack,
) -> SafetyWitness:
    segment, tau = _segment_coordinates(durations, time)
    window = track.windows[window_index]
    gate = frame_at(window, time)
    world = gate.center + gate.basis @ (gate.scale * local_point)
    return SafetyWitness(
        window_index, segment, tau, edge_a, edge_b, float(section_lambda),
        sc_radius, margin, outside, topology_key, local_point.copy(), world,
    )


def _verify_edge_cell(
    *,
    track: SCWindowTrack,
    window_index: int,
    section_at,
    durations: np.ndarray,
    edge_a: tuple[int, int],
    edge_b: tuple[int, int],
    topology_key: tuple[tuple[int, int], ...],
    time_bounds: tuple[float, float],
    lambda_bounds: tuple[float, float],
    depth: int,
    config: WholeBodySafetyConfig,
    accumulator: _Accumulator,
) -> None:
    if accumulator.witnesses or accumulator.numerical_failure:
        return
    accumulator.checked_cells += 1
    accumulator.maximum_depth = max(accumulator.maximum_depth, depth)
    ta, tb = time_bounds
    la, lb = lambda_bounds
    tc, lc = 0.5 * (ta + tb), 0.5 * (la + lb)
    time_samples = np.linspace(ta, tb, 5)
    lambda_samples = (la, lc, lb)
    q_by_time: list[tuple[np.ndarray, np.ndarray]] = []
    sample_data: list[tuple[float, float, np.ndarray, np.ndarray]] = []
    window = track.windows[window_index]
    polygon = Polygon(window.safe_polygon)
    inverse_norm = 0.0
    for time in time_samples:
        section = section_at(float(time))
        first, second = _section_vertex(section, edge_a), _section_vertex(section, edge_b)
        if first is None or second is None or section.degenerate:
            accumulator.uncertified = True
            return
        q_by_time.append((first.local, second.local))
        for fraction in lambda_samples:
            point = (1.0 - fraction) * first.local + fraction * second.local
            if not polygon.covers(Point(float(point[0]), float(point[1]))):
                accumulator.minimum_margin = min(accumulator.minimum_margin, -float("inf"))
                accumulator.witnesses.append(_make_witness(
                    window_index=window_index, time=float(time), durations=durations,
                    edge_a=edge_a, edge_b=edge_b, section_lambda=float(fraction),
                    sc_radius=None, margin=None, outside=True,
                    topology_key=topology_key, local_point=point, track=track,
                ))
                return
            inverse = inverse_sc_map(
                window.sc_map, point, tolerance=config.sc_inverse_tolerance,
                max_iterations=40,
            )
            if not inverse.converged:
                accumulator.numerical_failure = True
                return
            radius = float(np.linalg.norm(inverse.z))
            margin = float(config.sc_safe_radius**2 - radius**2)
            accumulator.minimum_margin = min(accumulator.minimum_margin, margin)
            sample_data.append((float(time), float(fraction), point, inverse.z))
            try:
                inverse_norm = max(inverse_norm, float(np.linalg.norm(np.linalg.inv(window.sc_map.jacobian(inverse.z)), 2)))
            except np.linalg.LinAlgError:
                accumulator.numerical_failure = True
                return
            if margin < 0.0:
                accumulator.witnesses.append(_make_witness(
                    window_index=window_index, time=float(time), durations=durations,
                    edge_a=edge_a, edge_b=edge_b, section_lambda=float(fraction),
                    sc_radius=radius, margin=margin, outside=False,
                    topology_key=topology_key, local_point=point, track=track,
                ))
                return

    centers = np.stack([(1.0 - lc) * pair[0] + lc * pair[1] for pair in q_by_time])
    dt_step = max((tb - ta) / 4.0, np.finfo(float).eps)
    vt = config.velocity_inflation * float(np.max(np.linalg.norm(np.diff(centers, axis=0), axis=1)) / dt_step)
    vlambda = config.velocity_inflation * max(float(np.linalg.norm(pair[1] - pair[0])) for pair in q_by_time)
    center_z = next(item[3] for item in sample_data if abs(item[0] - tc) < 1e-12 and abs(item[1] - lc) < 1e-12)
    upper = float(np.linalg.norm(center_z)) + inverse_norm * (vt * (tb - ta) / 2.0 + vlambda * (lb - la) / 2.0)
    if upper <= config.sc_safe_radius - config.certificate_epsilon:
        return
    if (tb - ta <= config.time_tolerance and lb - la <= config.lambda_tolerance):
        return  # V1 dense numerical acceptance, never promoted to CERTIFIED.
    if depth >= config.max_interval_depth:
        accumulator.uncertified = True
        return
    time_contribution = vt * (tb - ta)
    lambda_contribution = vlambda * (lb - la)
    if time_contribution >= lambda_contribution and tb - ta > config.time_tolerance:
        _verify_edge_cell(
            track=track, window_index=window_index, section_at=section_at,
            durations=durations, edge_a=edge_a, edge_b=edge_b,
            topology_key=topology_key, time_bounds=(ta, tc),
            lambda_bounds=(la, lb), depth=depth + 1, config=config,
            accumulator=accumulator,
        )
        _verify_edge_cell(
            track=track, window_index=window_index, section_at=section_at,
            durations=durations, edge_a=edge_a, edge_b=edge_b,
            topology_key=topology_key, time_bounds=(tc, tb),
            lambda_bounds=(la, lb), depth=depth + 1, config=config,
            accumulator=accumulator,
        )
    else:
        _verify_edge_cell(
            track=track, window_index=window_index, section_at=section_at,
            durations=durations, edge_a=edge_a, edge_b=edge_b,
            topology_key=topology_key, time_bounds=(ta, tb),
            lambda_bounds=(la, lc), depth=depth + 1, config=config,
            accumulator=accumulator,
        )
        _verify_edge_cell(
            track=track, window_index=window_index, section_at=section_at,
            durations=durations, edge_a=edge_a, edge_b=edge_b,
            topology_key=topology_key, time_bounds=(ta, tb),
            lambda_bounds=(lc, lb), depth=depth + 1, config=config,
            accumulator=accumulator,
        )


def verify_whole_body_trajectory(
    *,
    forward: ForwardPass,
    track: SCWindowTrack,
    body: CuboidBody,
    config: WholeBodySafetyConfig,
    yaw_profile: YawProfile | None = None,
    parameters: QuadrotorParameters | None = None,
) -> TrajectorySafetyReport:
    """Verify all prescribed crossings with adaptive time/section-edge cells.

    This is the plan's V1 numerical verifier. It checks complete section
    boundaries throughout only the connected crossing component containing
    each planned traversal time.
    """

    if len(track.order) != len(forward.traversal_times):
        raise ValueError("forward pass and track have inconsistent window counts")
    reports: list[WindowSafetyReport] = []
    for crossing_index, window_index in enumerate(track.order):
        window = track.windows[window_index]
        crossing = find_planned_crossing_interval(
            window_index=window_index,
            traversal_time=float(forward.traversal_times[crossing_index]),
            trajectory=forward.trajectory,
            window=window,
            body=body,
            config=config,
            yaw_profile=yaw_profile,
            parameters=parameters,
        )
        section_at = lambda time, w=window: plane_section_at(
            forward.trajectory, time, w, body, config,
            yaw_profile=yaw_profile, parameters=parameters,
        )
        intervals = split_into_topology_stable_intervals(
            crossing, section_at=section_at,
            time_tolerance=config.time_tolerance,
            max_depth=config.max_interval_depth,
        )
        accumulator = _Accumulator()
        if not intervals:
            accumulator.uncertified = True
        for interval in intervals:
            middle = section_at(0.5 * (interval.start + interval.end))
            sources = [vertex.source_body_edge for vertex in middle.vertices]
            for index, edge_a in enumerate(sources):
                edge_b = sources[(index + 1) % len(sources)]
                _verify_edge_cell(
                    track=track, window_index=window_index, section_at=section_at,
                    durations=np.asarray(forward.durations, dtype=float),
                    edge_a=edge_a, edge_b=edge_b, topology_key=interval.topology_key,
                    time_bounds=(interval.start, interval.end), lambda_bounds=(0.0, 1.0),
                    depth=0, config=config, accumulator=accumulator,
                )
                if accumulator.witnesses or accumulator.numerical_failure:
                    break
            if accumulator.witnesses or accumulator.numerical_failure:
                break
        if accumulator.witnesses:
            status = VerificationStatus.UNSAFE
        elif accumulator.numerical_failure:
            status = VerificationStatus.NUMERICAL_FAILURE
        elif accumulator.uncertified:
            status = VerificationStatus.UNCERTIFIED
        else:
            status = VerificationStatus.NUMERICALLY_VERIFIED
        minimum = None if not np.isfinite(accumulator.minimum_margin) else accumulator.minimum_margin
        reports.append(WindowSafetyReport(
            window_index, crossing, status, minimum, tuple(accumulator.witnesses),
            accumulator.checked_cells, accumulator.maximum_depth,
        ))
    statuses = {report.status for report in reports}
    if VerificationStatus.UNSAFE in statuses:
        overall = VerificationStatus.UNSAFE
    elif VerificationStatus.NUMERICAL_FAILURE in statuses:
        overall = VerificationStatus.NUMERICAL_FAILURE
    elif VerificationStatus.UNCERTIFIED in statuses:
        overall = VerificationStatus.UNCERTIFIED
    else:
        overall = VerificationStatus.NUMERICALLY_VERIFIED
    return TrajectorySafetyReport(overall, tuple(reports))


__all__ = [
    "SafetyWitness", "TrajectorySafetyReport", "VerificationStatus",
    "WindowSafetyReport", "verify_whole_body_trajectory",
]
