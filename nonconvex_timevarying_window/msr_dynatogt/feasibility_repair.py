"""High-density sampled feasibility checks and time-dilation repair.

The checks in this module are deliberately described as sampled feasibility.
They do not constitute a continuous-time proof between sample nodes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    constraint_extrema,
    flatness_map,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    JointTOGTObjective,
    OptimizationResult,
    optimize_track,
)
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations

from .config import FeasibilityConfig, MSRConfig


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class CrossingCheck:
    index: int
    window_index: int
    time: float
    contains: bool
    plane_error: float
    boundary_margin: float
    waypoint_error: float


@dataclass(frozen=True)
class FeasibilityReport:
    sampled_dynamic_limits_satisfied: bool
    window_order_legal: bool
    window_internal_legal: bool
    sampled_feasible: bool
    samples_per_segment: int
    sample_count: int
    max_velocity: float
    max_body_rate_xy: float
    max_abs_body_rate_z: float
    min_collective_thrust: float
    max_collective_thrust: float
    min_rotor_thrust: FloatArray
    max_rotor_thrust: FloatArray
    min_boundary_margin: float
    violating_segments: tuple[int, ...]
    crossings: tuple[CrossingCheck, ...]
    failure_reasons: tuple[str, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        return {
            "sampled_dynamic_limits_satisfied": self.sampled_dynamic_limits_satisfied,
            "window_order_legal": self.window_order_legal,
            "window_internal_legal": self.window_internal_legal,
            "sampled_feasible": self.sampled_feasible,
            "feasibility_claim": "高密度采样可行" if self.sampled_feasible else "高密度采样未通过",
            "samples_per_segment": self.samples_per_segment,
            "sample_count": self.sample_count,
            "max_velocity": self.max_velocity,
            "max_body_rate_xy": self.max_body_rate_xy,
            "max_abs_body_rate_z": self.max_abs_body_rate_z,
            "min_collective_thrust": self.min_collective_thrust,
            "max_collective_thrust": self.max_collective_thrust,
            "min_rotor_thrust": self.min_rotor_thrust.tolist(),
            "max_rotor_thrust": self.max_rotor_thrust.tolist(),
            "min_boundary_margin": self.min_boundary_margin,
            "violating_segments": list(self.violating_segments),
            "crossings": [crossing.__dict__ for crossing in self.crossings],
            "failure_reasons": list(self.failure_reasons),
        }


@dataclass(frozen=True)
class RepairOutcome:
    result: OptimizationResult
    feasibility: FeasibilityReport
    triggered: bool
    succeeded: bool
    mode: str
    scale_factor: float
    before_total_time: float
    repaired_total_time_before_reoptimization: float
    after_total_time: float
    reoptimization_improvement: float
    reoptimization_seconds: float
    reoptimization_iterations: int
    reoptimization_evaluations: int
    reoptimization_optimizer_success: bool
    reoptimization_accepted: bool
    attempts: int
    before_max_rotor_thrust: float
    after_max_rotor_thrust: float
    message: str


def _within_upper(value: FloatArray | float, upper: float, tolerance: float) -> NDArray[np.bool_] | bool:
    if not np.isfinite(upper):
        return np.ones_like(value, dtype=bool) if isinstance(value, np.ndarray) else True
    return np.asarray(value) <= upper + tolerance * max(1.0, abs(upper))


def _within_lower(value: FloatArray | float, lower: float, tolerance: float) -> NDArray[np.bool_] | bool:
    if not np.isfinite(lower):
        return np.ones_like(value, dtype=bool) if isinstance(value, np.ndarray) else True
    return np.asarray(value) >= lower - tolerance * max(1.0, abs(lower))


def _point_to_boundary_distance(point: FloatArray, polygon: FloatArray) -> float:
    vertices = np.asarray(polygon, dtype=float)
    following = np.roll(vertices, -1, axis=0)
    edges = following - vertices
    denominator = np.einsum("ij,ij->i", edges, edges)
    fraction = np.divide(
        np.einsum("ij,ij->i", point - vertices, edges),
        denominator,
        out=np.zeros(len(vertices)),
        where=denominator > 0.0,
    )
    closest = vertices + np.clip(fraction, 0.0, 1.0)[:, None] * edges
    return float(np.min(np.linalg.norm(closest - point, axis=1)))


def check_feasibility(
    track: SCWindowTrack,
    result: OptimizationResult,
    config: MSRConfig | FeasibilityConfig,
    *,
    optimization_config=None,
) -> FeasibilityReport:
    """Check dynamics and prescribed window crossings on a dense grid."""

    if isinstance(config, MSRConfig):
        sampling = config.feasibility
        optimization = config.optimization if optimization_config is None else optimization_config
    else:
        sampling = config
        if optimization_config is None:
            raise ValueError("optimization_config is required with FeasibilityConfig")
        optimization = optimization_config
    limits = optimization.dynamic_limits
    params = optimization.quadrotor
    tolerance = sampling.limit_relative_tolerance

    maximum_velocity = 0.0
    maximum_body_xy = 0.0
    maximum_body_z = 0.0
    minimum_collective = float("inf")
    maximum_collective = -float("inf")
    minimum_rotor = np.full(4, np.inf)
    maximum_rotor = np.full(4, -np.inf)
    violating: set[int] = set()
    sample_count = 0
    numerical_failure = False

    for segment, duration in enumerate(np.asarray(result.durations, dtype=float)):
        local_grid = np.linspace(0.0, float(duration), sampling.samples_per_segment)
        try:
            velocity = np.real(result.trajectory.evaluate_segment(segment, local_grid, 1))
            acceleration = np.real(result.trajectory.evaluate_segment(segment, local_grid, 2))
            jerk = np.real(result.trajectory.evaluate_segment(segment, local_grid, 3))
            snap = np.real(result.trajectory.evaluate_segment(segment, local_grid, 4))
            speeds = np.linalg.norm(velocity, axis=1)
            collectives = []
            body_xy = []
            body_z = []
            rotors = []
            for node in range(len(local_grid)):
                state = flatness_map(
                    acceleration[node], jerk[node], snap[node], parameters=params
                )
                collectives.append(float(np.real(state.collective_thrust)))
                body = np.real(state.body_rate)
                body_xy.append(float(np.linalg.norm(body[:2])))
                body_z.append(float(abs(body[2])))
                rotors.append(np.real(state.rotor_thrusts))
            collective_array = np.asarray(collectives)
            body_xy_array = np.asarray(body_xy)
            body_z_array = np.asarray(body_z)
            rotor_array = np.asarray(rotors)
        except (ValueError, FloatingPointError, np.linalg.LinAlgError):
            numerical_failure = True
            violating.add(segment)
            continue

        sample_count += len(local_grid)
        maximum_velocity = max(maximum_velocity, float(np.max(speeds)))
        maximum_body_xy = max(maximum_body_xy, float(np.max(body_xy_array)))
        maximum_body_z = max(maximum_body_z, float(np.max(body_z_array)))
        minimum_collective = min(minimum_collective, float(np.min(collective_array)))
        maximum_collective = max(maximum_collective, float(np.max(collective_array)))
        minimum_rotor = np.minimum(minimum_rotor, np.min(rotor_array, axis=0))
        maximum_rotor = np.maximum(maximum_rotor, np.max(rotor_array, axis=0))

        segment_ok = bool(
            np.all(_within_upper(speeds, limits.max_velocity, tolerance))
            and np.all(_within_upper(body_xy_array, limits.max_body_rate_xy, tolerance))
            and np.all(_within_upper(body_z_array, limits.max_body_rate_z, tolerance))
            and np.all(_within_lower(collective_array, limits.min_collective_thrust, tolerance))
            and np.all(_within_upper(collective_array, limits.max_collective_thrust, tolerance))
            and np.all(_within_lower(rotor_array, limits.min_rotor_thrust, tolerance))
            and np.all(_within_upper(rotor_array, limits.max_rotor_thrust, tolerance))
        )
        if not segment_ok:
            violating.add(segment)

    dynamic_ok = bool(not numerical_failure and not violating)
    times = np.asarray(result.traversal_times, dtype=float)
    order_ok = bool(
        len(times) == len(track.order)
        and np.all(np.isfinite(times))
        and np.all(np.diff(times) > 0.0)
        and (len(times) == 0 or (times[0] > 0.0 and times[-1] < result.total_time))
    )
    crossings: list[CrossingCheck] = []
    internal_ok = order_ok
    margins: list[float] = []
    if order_ok:
        for crossing_index, window_index in enumerate(track.order):
            instant = float(times[crossing_index])
            trajectory_point = np.real(result.trajectory.evaluate(instant))
            waypoint_error = float(
                np.linalg.norm(trajectory_point - result.waypoints[crossing_index])
            )
            window = track.windows[window_index]
            local, plane_error = window.world_to_local(trajectory_point, instant)
            contains = bool(
                waypoint_error <= sampling.waypoint_tolerance
                and plane_error <= sampling.plane_tolerance
                and window.contains(
                    trajectory_point,
                    instant,
                    plane_tolerance=sampling.plane_tolerance,
                )
            )
            scale = float(window.state_at(instant)[2])
            margin = _point_to_boundary_distance(local, window.safe_polygon) * scale
            if not contains:
                margin = -margin
            margins.append(margin)
            crossings.append(
                CrossingCheck(
                    index=crossing_index,
                    window_index=window_index,
                    time=instant,
                    contains=contains,
                    plane_error=plane_error,
                    boundary_margin=margin,
                    waypoint_error=waypoint_error,
                )
            )
            internal_ok = internal_ok and contains

    reasons = []
    if numerical_failure:
        reasons.append("flatness_numerical_failure")
    if violating:
        reasons.append("sampled_dynamic_limit_violation")
    if sample_count:
        if not bool(np.all(_within_upper(maximum_velocity, limits.max_velocity, tolerance))):
            reasons.append("sampled_velocity_upper_limit")
        if not bool(np.all(_within_upper(maximum_body_xy, limits.max_body_rate_xy, tolerance))):
            reasons.append("sampled_body_rate_xy_limit")
        if not bool(np.all(_within_upper(maximum_body_z, limits.max_body_rate_z, tolerance))):
            reasons.append("sampled_body_rate_z_limit")
        if not bool(np.all(_within_lower(minimum_collective, limits.min_collective_thrust, tolerance))):
            reasons.append("sampled_collective_thrust_lower_limit")
        if not bool(np.all(_within_upper(maximum_collective, limits.max_collective_thrust, tolerance))):
            reasons.append("sampled_collective_thrust_upper_limit")
        if not bool(np.all(_within_lower(minimum_rotor, limits.min_rotor_thrust, tolerance))):
            reasons.append("sampled_single_rotor_thrust_lower_limit")
        if not bool(np.all(_within_upper(maximum_rotor, limits.max_rotor_thrust, tolerance))):
            reasons.append("sampled_single_rotor_thrust_upper_limit")
    if not order_ok:
        reasons.append("window_order_invalid")
    if not internal_ok:
        reasons.append("window_internal_legality_invalid")
    feasible = bool(dynamic_ok and order_ok and internal_ok)
    return FeasibilityReport(
        sampled_dynamic_limits_satisfied=dynamic_ok,
        window_order_legal=order_ok,
        window_internal_legal=internal_ok,
        sampled_feasible=feasible,
        samples_per_segment=sampling.samples_per_segment,
        sample_count=sample_count,
        max_velocity=maximum_velocity if sample_count else float("inf"),
        max_body_rate_xy=maximum_body_xy if sample_count else float("inf"),
        max_abs_body_rate_z=maximum_body_z if sample_count else float("inf"),
        min_collective_thrust=minimum_collective if sample_count else -float("inf"),
        max_collective_thrust=maximum_collective if sample_count else float("inf"),
        min_rotor_thrust=minimum_rotor,
        max_rotor_thrust=maximum_rotor,
        min_boundary_margin=min(margins) if margins else -float("inf"),
        violating_segments=tuple(sorted(violating)),
        crossings=tuple(crossings),
        failure_reasons=tuple(reasons),
    )


def result_from_x(
    track: SCWindowTrack,
    x: FloatArray,
    config: MSRConfig,
    *,
    message: str = "reconstructed repair incumbent",
) -> OptimizationResult:
    """Reconstruct a serializable SC result without running an optimizer."""

    objective = JointTOGTObjective(track, config.optimization)
    final = objective.evaluate(x)
    extrema = constraint_extrema(
        final.forward.trajectory,
        parameters=config.optimization.quadrotor,
        samples_per_segment=config.feasibility.samples_per_segment,
    )
    return OptimizationResult(
        success=True,
        status=0,
        message=message,
        objective=final.cost,
        iterations=0,
        evaluations=1,
        gradient_inf_norm=float(np.linalg.norm(final.gradient, ord=np.inf)),
        x=np.asarray(x, dtype=float),
        k=final.forward.k,
        d=final.forward.d,
        durations=final.forward.durations,
        traversal_times=final.forward.traversal_times,
        waypoints=final.forward.waypoints,
        local_points=final.forward.local_points,
        trajectory=final.forward.trajectory,
        constraint_extrema=extrema,
        full_time_gradient=config.optimization.include_window_time_gradient,
        invalid_trial_count=0,
    )


def _rank(report: FeasibilityReport, result: OptimizationResult) -> tuple[int, int, float]:
    legal = report.window_order_legal and report.window_internal_legal
    return (0 if legal else 1, 0 if report.sampled_dynamic_limits_satisfied else 1, result.total_time)


def repair_candidate(
    track: SCWindowTrack,
    original: OptimizationResult,
    config: MSRConfig,
    *,
    original_report: FeasibilityReport | None = None,
) -> RepairOutcome:
    """Dilate time, bisect to a sampled-feasible boundary, then re-optimize."""

    before = original_report or check_feasibility(track, original, config)
    before_peak = float(np.max(before.max_rotor_thrust))
    if not config.repair.enabled or before.sampled_feasible:
        return RepairOutcome(
            result=original,
            feasibility=before,
            triggered=False,
            succeeded=before.sampled_feasible,
            mode=config.repair.mode,
            scale_factor=1.0,
            before_total_time=original.total_time,
            repaired_total_time_before_reoptimization=original.total_time,
            after_total_time=original.total_time,
            reoptimization_improvement=0.0,
            reoptimization_seconds=0.0,
            reoptimization_iterations=0,
            reoptimization_evaluations=0,
            reoptimization_optimizer_success=original.success,
            reoptimization_accepted=False,
            attempts=0,
            before_max_rotor_thrust=before_peak,
            after_max_rotor_thrust=before_peak,
            message="repair not required" if before.sampled_feasible else "repair disabled",
        )
    if not (before.window_order_legal and before.window_internal_legal):
        return RepairOutcome(
            result=original,
            feasibility=before,
            triggered=True,
            succeeded=False,
            mode=config.repair.mode,
            scale_factor=1.0,
            before_total_time=original.total_time,
            repaired_total_time_before_reoptimization=original.total_time,
            after_total_time=original.total_time,
            reoptimization_improvement=0.0,
            reoptimization_seconds=0.0,
            reoptimization_iterations=0,
            reoptimization_evaluations=0,
            reoptimization_optimizer_success=False,
            reoptimization_accepted=False,
            attempts=0,
            before_max_rotor_thrust=before_peak,
            after_max_rotor_thrust=before_peak,
            message="time repair cannot correct an illegal window crossing",
        )

    base_durations = np.asarray(original.durations, dtype=float)
    spatial = np.asarray(original.d, dtype=float)

    def reconstruct(durations: FloatArray) -> tuple[OptimizationResult, FeasibilityReport]:
        x = np.concatenate((k_from_durations(durations), spatial.reshape(-1)))
        candidate = result_from_x(track, x, config)
        return candidate, check_feasibility(track, candidate, config)

    attempts = 0
    high_durations = base_durations.copy()
    high_result = original
    high_report = before
    while not high_report.sampled_feasible:
        attempts += 1
        if config.repair.mode == "uniform":
            high_durations = high_durations * config.repair.expansion_factor
        else:
            affected: set[int] = set()
            for segment in high_report.violating_segments:
                low = max(0, segment - config.repair.local_neighbor_radius)
                high = min(len(high_durations), segment + config.repair.local_neighbor_radius + 1)
                affected.update(range(low, high))
            if not affected:
                break
            indices = np.asarray(sorted(affected), dtype=int)
            high_durations[indices] *= config.repair.expansion_factor
        if float(np.max(high_durations / base_durations)) > config.repair.maximum_scale:
            break
        high_result, high_report = reconstruct(high_durations)

    if not high_report.sampled_feasible:
        return RepairOutcome(
            result=original,
            feasibility=before,
            triggered=True,
            succeeded=False,
            mode=config.repair.mode,
            scale_factor=float(np.max(high_durations / base_durations)),
            before_total_time=original.total_time,
            repaired_total_time_before_reoptimization=original.total_time,
            after_total_time=original.total_time,
            reoptimization_improvement=0.0,
            reoptimization_seconds=0.0,
            reoptimization_iterations=0,
            reoptimization_evaluations=0,
            reoptimization_optimizer_success=False,
            reoptimization_accepted=False,
            attempts=attempts,
            before_max_rotor_thrust=before_peak,
            after_max_rotor_thrust=before_peak,
            message="no sampled-feasible time scale found before maximum_scale",
        )

    low_alpha, high_alpha = 0.0, 1.0
    for _ in range(config.repair.binary_iterations):
        attempts += 1
        middle = 0.5 * (low_alpha + high_alpha)
        durations = base_durations + middle * (high_durations - base_durations)
        middle_result, middle_report = reconstruct(durations)
        if middle_report.sampled_feasible:
            high_alpha = middle
            high_result, high_report = middle_result, middle_report
        else:
            low_alpha = middle

    repaired_time = high_result.total_time
    reoptimization_config = config.repair_optimization()
    started = time.perf_counter()
    refined = optimize_track(
        track,
        config=reoptimization_config,
        initial_x=high_result.x,
    )
    reoptimization_seconds = time.perf_counter() - started
    refined_report = check_feasibility(
        track,
        refined,
        config.feasibility,
        optimization_config=reoptimization_config,
    )
    accepted = _rank(refined_report, refined) < _rank(high_report, high_result)
    if accepted:
        final_result, final_report = refined, refined_report
    else:
        final_result, final_report = high_result, high_report
    improvement = repaired_time - refined.total_time
    scale = float(np.max(high_result.durations / base_durations))
    return RepairOutcome(
        result=final_result,
        feasibility=final_report,
        triggered=True,
        succeeded=final_report.sampled_feasible,
        mode=config.repair.mode,
        scale_factor=scale,
        before_total_time=original.total_time,
        repaired_total_time_before_reoptimization=repaired_time,
        after_total_time=final_result.total_time,
        reoptimization_improvement=improvement,
        reoptimization_seconds=reoptimization_seconds,
        reoptimization_iterations=refined.iterations,
        reoptimization_evaluations=refined.evaluations,
        reoptimization_optimizer_success=refined.success,
        reoptimization_accepted=accepted,
        attempts=attempts,
        before_max_rotor_thrust=before_peak,
        after_max_rotor_thrust=float(np.max(final_report.max_rotor_thrust)),
        message=(
            "reoptimized candidate accepted"
            if accepted
            else "sampled-feasible repair incumbent retained after reoptimization check"
        ),
    )


__all__ = [
    "CrossingCheck",
    "FeasibilityReport",
    "RepairOutcome",
    "check_feasibility",
    "repair_candidate",
    "result_from_x",
]
