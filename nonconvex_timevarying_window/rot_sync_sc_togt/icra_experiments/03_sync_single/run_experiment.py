#!/usr/bin/env python3
"""ICRA experiment 03: focused speed sweep for analytic rotation Sync.

This adapter deliberately lives beside its outputs.  It reuses the repository's
SC preprocessing, degree-seven MINCO, RotSync segment, flatness dynamics, and
oriented-cuboid collision checker without changing the original method code.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, is_dataclass, replace
import json
import math
from pathlib import Path
import platform
import sys
import time
from types import SimpleNamespace
from typing import Any, Iterable

import numpy as np
from scipy.optimize import brentq
from shapely.geometry import Point, Polygon
from shapely.ops import polylabel

from nonconvex_timevarying_window.sc_dynatogt.boundary import (
    DenseBoundary,
    adaptive_chang_resample,
    validate_polygon,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    constraint_extrema,
    integrated_dynamic_penalty,
    sample_flatness,
    smoothed_l1,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.offset import OffsetResult, inward_offset
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    e1_boundaries,
)
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap, SCMappingError
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    durations_from_k,
    k_from_durations,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.collision import (
    _slab_cross_section,
    body_rotations,
    cuboid_vertices,
    cuboid_window_collision,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import (
    RotatingWindow,
    basis_from_normal,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import (
    RotSyncObjective,
    RotSyncOptimizationConfig,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import (
    DEFAULT_BODY,
    RotSyncScenario,
)


HERE = Path(__file__).resolve().parent
# The focused protocol fixes geometry and phase before looking at formal results.
# The axis lies inside the narrow vertical arm of the canonical L aperture.  Its
# local clearance is deliberately smaller than the aircraft radius, while the
# aperture's off-axis maximum incircle remains larger than the planning envelope.
SHAPES = ("L", "U")
SIZE_RATIOS = (1.15,)
OMEGAS = (0.0, 1.5, 3.0, 4.5, 6.0)
PHASES = (0.3,)
METHODS = ("Fixed-WP", "Optimized-MINCO", "SC+Sync")
FORMAL_SCENE_COUNT = len(SHAPES) * len(SIZE_RATIOS) * len(OMEGAS) * len(PHASES)
BODY = DEFAULT_BODY
PLANNING_RHO = BODY.circumscribed_radius + 0.015
THICKNESS = 0.14
MIN_SAFE_AREA = 1.0e-6
START = BoundaryState(np.asarray((-4.5, -0.8, 1.8)))
GOAL = BoundaryState(np.asarray((4.5, 0.8, 1.8)))
WINDOW_CENTER = np.asarray((0.0, 0.0, 1.8))
WINDOW_NORMAL = np.asarray((1.0, 0.0, 0.0))
L_AXIS_IN_CANONICAL_NARROW_ARM = np.asarray((0.4, 0.0))
CALIBRATION_U_AXIS_IN_CANONICAL_NOTCH = np.asarray((0.0, 1.5))


class TimeBudgetExceeded(RuntimeError):
    pass


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def shape_source(name: str) -> DenseBoundary:
    definitions = e1_boundaries()
    return definitions[{"L": "l_shape", "U": "u_shape", "star": "five_point_star"}[name]]


@dataclass(frozen=True)
class PreparedGeometry:
    shape: str
    ratio: float
    gate: PreprocessedGate
    canonical_axis: np.ndarray
    scale: float
    source_inradius: float
    physical_inradius: float
    axis_centered_incircle_radius: float
    safe_inradius: float
    sc_cyclic_shift: int


def _replace_safe_vertices(safe: OffsetResult, vertices: np.ndarray) -> OffsetResult:
    validation = validate_polygon(vertices, require_ccw=True, raise_on_error=True)
    return OffsetResult(
        vertices,
        safe.distance,
        float(validation.signed_area),
        safe.metadata,
        safe.diagnostics,
        validation,
    )


def prepare_geometry(
    shape: str,
    ratio: float,
    *,
    vertex_count: int,
    quadrature_order: int,
    canonical_axis: np.ndarray | None = None,
    source_boundary: DenseBoundary | None = None,
) -> PreparedGeometry:
    """Build the exact requested physical scale and a deterministic SC map.

    A cyclic reindex of the *same* safe polygon is tried when the SC parameter
    solver is ill-conditioned.  This changes neither its boundary nor its axis;
    the selected shift is persisted as numerical provenance.
    """
    source = shape_source(shape) if source_boundary is None else source_boundary
    source_polygon = Polygon(source.vertices)
    source_label = polylabel(source_polygon, tolerance=1.0e-10)
    source_inradius = float(source_label.distance(source_polygon.boundary))
    scale = float(ratio * PLANNING_RHO / source_inradius)
    axis = (
        np.asarray(canonical_axis, dtype=float).copy()
        if canonical_axis is not None
        else (
            L_AXIS_IN_CANONICAL_NARROW_ARM.copy() if shape == "L"
            else CALIBRATION_U_AXIS_IN_CANONICAL_NOTCH.copy() if shape == "U"
            else np.zeros(2)
        )
    )
    dense = DenseBoundary(
        (source.vertices - axis) * scale,
        (source.corners - axis) * scale,
        source.corner_indices,
    )
    config = PreprocessingConfig(
        vertex_counts=(int(vertex_count),),
        offset_distance=PLANNING_RHO,
        min_safe_area=MIN_SAFE_AREA,
        sc_fit_options={"quadrature_order": int(quadrature_order), "max_nfev": 1200},
    )
    sampled = adaptive_chang_resample(
        dense,
        vertex_counts=config.vertex_counts,
        boundary_tolerance=config.boundary_tolerance,
        concavity_tolerance=config.concavity_tolerance,
        concavity_angle_threshold_deg=config.concavity_angle_threshold_deg,
    )
    safe = inward_offset(
        sampled,
        distance=config.offset_distance,
        miter_limit=config.miter_limit,
        min_area=config.min_safe_area,
        arc_tolerance=config.arc_tolerance,
        scale_factor=config.clipper_scale,
    )
    failures = []
    mapping = None
    selected_safe = safe
    selected_shift = -1
    for shift in range(len(safe.vertices)):
        vertices = np.roll(safe.vertices, -shift, axis=0)
        try:
            mapping = SCDiskMap.fit(vertices, **dict(config.sc_fit_options))
            selected_safe = _replace_safe_vertices(safe, vertices)
            selected_shift = shift
            break
        except Exception as exc:  # every attempted failure is written by the caller
            failures.append({"shift": shift, "type": type(exc).__name__, "message": str(exc)})
    if mapping is None:
        error = RuntimeError("SC mapping failed for every cyclic indexing")
        setattr(error, "attempts", failures)
        raise error
    gate = PreprocessedGate(f"{shape}_ratio_{ratio:g}", dense, sampled, selected_safe, mapping, config)
    physical_polygon = Polygon(dense.vertices)
    safe_polygon = Polygon(selected_safe.vertices)
    physical_inradius = float(polylabel(physical_polygon, tolerance=1.0e-9).distance(physical_polygon.boundary))
    local_axis = Point(0.0, 0.0)
    axis_centered_incircle_radius = (
        float(local_axis.distance(physical_polygon.boundary))
        if physical_polygon.covers(local_axis) else 0.0
    )
    safe_inradius = float(polylabel(safe_polygon, tolerance=1.0e-9).distance(safe_polygon.boundary))
    return PreparedGeometry(
        shape, ratio, gate, axis, scale, source_inradius, physical_inradius,
        axis_centered_incircle_radius, safe_inradius, selected_shift,
    )


def build_scenario(geometry: PreparedGeometry, omega: float, phase: float, *, name: str) -> RotSyncScenario:
    basis, normal = basis_from_normal(WINDOW_NORMAL)
    window = RotatingWindow(
        name=geometry.gate.name,
        gate=geometry.gate,
        center=WINDOW_CENTER,
        plane_basis=basis,
        normal=normal,
        theta0=float(phase),
        omega=float(omega),
        thickness=THICKNESS,
        rho=PLANNING_RHO,
    )
    return RotSyncScenario(
        name=name,
        start_state=START,
        goal_state=GOAL,
        windows=(window,),
        description="ICRA experiment 03 deterministic single spinning non-convex window",
        body=BODY,
        difficulty="single-window-sync-value",
    )


def slug_number(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}".replace("-", "m").replace(".", "p")


def scenario_name(shape: str, ratio: float, omega: float, phase: float) -> str:
    return f"{shape}_r{slug_number(ratio,2)}_w{slug_number(omega,2)}_p{slug_number(phase,1)}"


def _segment_sample_times(trajectory, samples_per_segment: int) -> tuple[np.ndarray, np.ndarray]:
    times, weights = [], []
    elapsed = 0.0
    for index, duration in enumerate(np.asarray(trajectory.durations, dtype=float)):
        local = np.linspace(0.0, duration, samples_per_segment)
        global_times = elapsed + local
        trapezoid = np.ones(samples_per_segment)
        trapezoid[[0, -1]] = 0.5
        trapezoid *= duration / (samples_per_segment - 1)
        if index:
            global_times = global_times[1:]
            trapezoid = trapezoid[1:]
        times.extend(global_times)
        weights.extend(trapezoid)
        elapsed += duration
    return np.asarray(times), np.asarray(weights)


def _point_segment_distance_squared(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    a = polygon
    b = np.roll(polygon, -1, axis=0)
    edge = b - a
    denom = np.einsum("ij,ij->i", edge, edge)
    delta = points[:, None, :] - a[None, :, :]
    fraction = np.einsum("nmi,mi->nm", delta, edge) / denom[None, :]
    fraction = np.clip(fraction, 0.0, 1.0)
    closest = a[None, :, :] + fraction[..., None] * edge[None, :, :]
    distance2 = np.einsum("nmi,nmi->nm", points[:, None, :] - closest, points[:, None, :] - closest)
    return np.min(distance2, axis=1)


def sphere_frame_collision_penalty(scenario: RotSyncScenario, trajectory, samples_per_segment: int) -> float:
    """Sample the complete trajectory against the physical boundary curtain.

    The center-to-extruded-boundary distance is exact for a sphere: planar
    point-to-polyline distance combined with distance to the thickness slab.
    """
    times, weights = _segment_sample_times(trajectory, samples_per_segment)
    positions = np.asarray(trajectory.evaluate(times), dtype=float)
    total = 0.0
    for window in scenario.windows:
        delta = positions - window.center
        fixed = delta @ window.plane_basis
        theta = window.theta0 + window.omega * times
        cosine, sine = np.cos(theta), np.sin(theta)
        local = np.column_stack((
            fixed[:, 0] * cosine + fixed[:, 1] * sine,
            -fixed[:, 0] * sine + fixed[:, 1] * cosine,
        ))
        planar2 = _point_segment_distance_squared(local, window.physical_polygon)
        axial = np.maximum(np.abs(delta @ window.normal) - 0.5 * window.thickness, 0.0)
        residual = window.rho**2 - (planar2 + axial**2)
        total += float(np.sum(weights * np.real(smoothed_l1(residual))))
    return total


@dataclass(frozen=True)
class CostBreakdown:
    total_time: float
    smoothness: float
    dynamic_penalty: float
    collision_penalty: float
    weighted_total: float


class CollisionAwareObjective:
    """Common objective used without method-specific weight changes."""

    def __init__(self, scenario: RotSyncScenario, config: RotSyncOptimizationConfig, collision_weight: float):
        self.scenario = scenario
        self.config = config
        self.collision_weight = float(collision_weight)
        self.cost_evaluations = 0
        self.invalid_trial_count = 0

    def cost_breakdown(self, forward) -> CostBreakdown:
        trajectory = forward.trajectory
        dynamic = float(np.real(integrated_dynamic_penalty(
            trajectory,
            parameters=self.config.quadrotor,
            limits=self.config.dynamic_limits,
            weights=self.config.penalty_weights,
            samples_per_segment=self.config.samples_per_segment,
        )))
        collision = sphere_frame_collision_penalty(
            self.scenario, trajectory, self.config.samples_per_segment,
        )
        smoothness = float(trajectory.snap_energy())
        total = float(trajectory.total_time)
        weighted = (
            total + self.config.smoothness_weight * smoothness
            + self.config.dynamics_weight * dynamic + self.collision_weight * collision
        )
        return CostBreakdown(total, smoothness, dynamic, collision, float(weighted))

    def value(self, x: np.ndarray) -> float:
        self.cost_evaluations += 1
        value = self.cost_breakdown(self.forward(x)).weighted_total
        if not np.isfinite(value):
            raise FloatingPointError("objective became non-finite")
        return value

    def _safe_value(self, x: np.ndarray) -> float:
        try:
            return self.value(x)
        except (ValueError, FloatingPointError, OverflowError, np.linalg.LinAlgError, SCMappingError):
            self.invalid_trial_count += 1
            clipped = np.clip(x, -1.0e5, 1.0e5)
            return float(1.0e24 * (1.0 + 1.0e-12 * (clipped @ clipped)))

    def value_and_gradient(self, x: np.ndarray) -> tuple[float, np.ndarray]:
        values = np.asarray(x, dtype=float)
        self.split(values)
        base = self._safe_value(values)
        gradient = np.empty_like(values)
        for index in range(len(values)):
            step = self.config.finite_difference_step * max(1.0, abs(float(values[index])))
            plus, minus = values.copy(), values.copy()
            plus[index] += step
            minus[index] -= step
            gradient[index] = (self._safe_value(plus) - self._safe_value(minus)) / (2.0 * step)
        return base, gradient


class FixedWaypointObjective(CollisionAwareObjective):
    """Existing two-piece ordinary MINCO baseline with a fixed safe local point."""

    dimension = 2

    def __init__(self, scenario, config, collision_weight):
        super().__init__(scenario, config, collision_weight)
        point = polylabel(Polygon(scenario.windows[0].safe_polygon), tolerance=1.0e-7)
        self.fixed_q = np.asarray(point.coords[0], dtype=float)

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (2,) or not np.all(np.isfinite(values)):
            raise ValueError("Fixed-WP requires two finite duration parameters")
        return values

    def initial_guess(self):
        anchors = np.asarray((START.position, WINDOW_CENTER, GOAL.position))
        durations = np.maximum(
            np.linalg.norm(np.diff(anchors, axis=0), axis=1) / self.config.initial_speed,
            self.config.minimum_initial_free_duration,
        )
        return k_from_durations(durations)

    def forward(self, x):
        durations = durations_from_k(self.split(x))
        crossing = float(durations[0])
        window = self.scenario.windows[0]
        waypoint = window.world_point(self.fixed_q, crossing)
        trajectory = MincoSnap(START, GOAL, waypoint[None, :], durations)
        return SimpleNamespace(
            trajectory=trajectory,
            crossing_times=np.asarray((crossing,)),
            local_points=self.fixed_q[None, :],
            crossing_local_index=0,
            durations=durations,
            method="Fixed-WP",
        )


class OptimizedMincoObjective(CollisionAwareObjective):
    """Ordinary MINCO with optimized entry/crossing/exit path nodes and times."""

    dimension = 10  # K_0..K_3 and three two-dimensional SC latent nodes

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError("Optimized-MINCO requires ten finite variables")
        return values[:4], values[4:].reshape(3, 2)

    def initial_guess(self):
        window = self.scenario.windows[0]
        local = np.stack([window.local_point(np.zeros(2))] * 3)
        nominal_times = np.asarray((1.55, 1.75, 1.95))
        points = np.stack([
            window.world_point(local[i], nominal_times[i], z)
            for i, z in enumerate((-window.clearance_distance, 0.0, window.clearance_distance))
        ])
        anchors = np.vstack((START.position, points, GOAL.position))
        durations = np.maximum(
            np.linalg.norm(np.diff(anchors, axis=0), axis=1) / self.config.initial_speed,
            0.18,
        )
        return np.r_[k_from_durations(durations), np.zeros(6)]

    def forward(self, x):
        k, latent = self.split(x)
        durations = durations_from_k(k)
        knots = np.cumsum(durations)[:-1]
        window = self.scenario.windows[0]
        local = np.stack([window.local_point(d) for d in latent])
        points = np.stack([
            window.world_point(local[i], float(knots[i]), z)
            for i, z in enumerate((-window.clearance_distance, 0.0, window.clearance_distance))
        ])
        trajectory = MincoSnap(START, GOAL, points, durations)
        return SimpleNamespace(
            trajectory=trajectory,
            crossing_times=np.asarray((float(knots[1]),)),
            local_points=local,
            latent_points=latent,
            crossing_local_index=1,
            durations=durations,
            method="Optimized-MINCO",
        )


class CollisionAwareSyncObjective(RotSyncObjective, CollisionAwareObjective):
    """Original SC+Sync parameterization plus the common full-path collision cost."""

    def __init__(self, scenario, config, collision_weight):
        RotSyncObjective.__init__(self, scenario, config)
        self.collision_weight = float(collision_weight)

    def cost_breakdown(self, forward) -> CostBreakdown:
        return CollisionAwareObjective.cost_breakdown(self, forward)

    def value(self, x):
        return CollisionAwareObjective.value(self, x)

    def forward(self, x):
        result = RotSyncObjective.forward(self, x)
        return SimpleNamespace(
            **result.__dict__, crossing_local_index=0, method="SC+Sync",
        )

    def _safe_value(self, x):
        return CollisionAwareObjective._safe_value(self, x)

    def value_and_gradient(self, x):
        return CollisionAwareObjective.value_and_gradient(self, x)


OBJECTIVES = {
    "Fixed-WP": FixedWaypointObjective,
    "Optimized-MINCO": OptimizedMincoObjective,
    "SC+Sync": CollisionAwareSyncObjective,
}


@dataclass
class SolveRecord:
    optimizer_success: bool
    status: int
    message: str
    iterations: int
    evaluations: int
    solve_seconds: float
    budget_seconds: float
    timed_out: bool
    x: np.ndarray
    forward: Any
    cost: CostBreakdown
    invalid_trials: int


def solve_with_budget(objective, budget_seconds: float) -> SolveRecord:
    started = time.perf_counter()
    best_x = np.asarray(objective.initial_guess(), dtype=float)
    best_cost = float("inf")
    evaluations = 0

    def timed(values):
        nonlocal best_x, best_cost, evaluations
        if time.perf_counter() - started >= budget_seconds:
            raise TimeBudgetExceeded(f"total solve budget {budget_seconds:g}s exhausted")
        cost, gradient = objective.value_and_gradient(values)
        evaluations += 1
        if np.isfinite(cost) and cost < best_cost:
            best_cost = float(cost)
            best_x = np.asarray(values, dtype=float).copy()
        if time.perf_counter() - started >= budget_seconds:
            raise TimeBudgetExceeded(f"total solve budget {budget_seconds:g}s exhausted")
        return cost, gradient

    timed_out = False
    try:
        result = _minimize_togt_lbfgs(timed, best_x, objective.config.lbfgs_config())
        x = np.asarray(result.x, dtype=float)
        success, status, message = bool(result.success), int(result.status), str(result.message)
        iterations, evaluations = int(result.nit), int(result.nfev)
    except TimeBudgetExceeded as exc:
        timed_out = True
        x = best_x
        success, status, message = False, 124, str(exc)
        iterations = -1
    elapsed = time.perf_counter() - started
    forward = objective.forward(x)
    cost = objective.cost_breakdown(forward)
    return SolveRecord(
        success, status, message, iterations, evaluations, elapsed,
        float(budget_seconds), timed_out, x, forward, cost,
        int(objective.invalid_trial_count),
    )


def _knots(trajectory) -> np.ndarray:
    return np.cumsum(np.asarray(trajectory.durations, dtype=float))[:-1]


def _interface_residual(trajectory) -> float:
    if hasattr(trajectory, "interface_residuals"):
        values = trajectory.interface_residuals()
        return float(np.max(values)) if values.size else 0.0
    durations = np.asarray(trajectory.durations, dtype=float)
    residual = 0.0
    for index in range(len(durations) - 1):
        for derivative in range(4):
            left = trajectory.evaluate_segment(index, durations[index], derivative)
            right = trajectory.evaluate_segment(index + 1, 0.0, derivative)
            residual = max(residual, float(np.linalg.norm(left - right)))
    return residual


def _audit_grid(trajectory, crossing: float, dt: float) -> np.ndarray:
    count = int(np.ceil(trajectory.total_time / dt))
    base = np.linspace(0.0, trajectory.total_time, count + 1)
    knots = _knots(trajectory)
    probes = np.r_[knots, knots - 1.0e-8, knots + 1.0e-8, crossing]
    grid = np.unique(np.r_[base, probes])
    return grid[(grid >= 0.0) & (grid <= trajectory.total_time)]


def _evaluate_body_collision(scenario, trajectory, grid):
    positions = np.asarray(trajectory.evaluate(grid), dtype=float)
    rotations = body_rotations(trajectory, grid)
    hits = np.zeros(len(grid), dtype=bool)
    clearance = np.full(len(grid), np.inf)
    window = scenario.windows[0]
    for index, (instant, position, rotation) in enumerate(zip(grid, positions, rotations)):
        hits[index], clearance[index] = cuboid_window_collision(
            window, float(instant), position, rotation, scenario.body,
        )
    return positions, rotations, hits, clearance


def _plane_crossings(trajectory, window, grid, positions) -> list[float]:
    signed = (positions - window.center) @ window.normal
    roots = []
    for index in range(len(grid) - 1):
        left, right = float(signed[index]), float(signed[index + 1])
        if abs(left) <= 1.0e-10:
            roots.append(float(grid[index]))
        if left * right < 0.0:
            roots.append(float(brentq(
                lambda t: float((trajectory.evaluate(t) - window.center) @ window.normal),
                float(grid[index]), float(grid[index + 1]), xtol=1.0e-13,
            )))
    if abs(float(signed[-1])) <= 1.0e-10:
        roots.append(float(grid[-1]))
    roots.sort()
    deduplicated = []
    for value in roots:
        if not deduplicated or abs(value - deduplicated[-1]) > 1.0e-7:
            deduplicated.append(value)
    return deduplicated


def _valid_body_crossing(scenario, trajectory, instant: float) -> tuple[bool, float]:
    window = scenario.windows[0]
    position = np.asarray(trajectory.evaluate(instant), dtype=float)
    rotation = body_rotations(trajectory, np.asarray((instant,)))[0]
    section = _slab_cross_section(
        window, instant, position, rotation, scenario.body, tolerance=1.0e-9,
    )
    if section is None or section.is_empty:
        return False, 0.0
    polygon = Polygon(window.physical_polygon)
    clearance = float(section.distance(polygon.boundary))
    return bool(polygon.covers(section) and clearance > 1.0e-9), clearance


def _body_slab_duration(scenario, trajectory, grid, positions, rotations) -> tuple[float | None, float | None, float | None]:
    window = scenario.windows[0]
    signed = (positions - window.center) @ window.normal
    half = np.asarray(scenario.body.half_extents)
    projection = np.sum(np.abs(np.einsum("nij,j->ni", np.transpose(rotations, (0, 2, 1)), window.normal)) * half, axis=1)
    overlap = np.abs(signed) <= 0.5 * window.thickness + projection
    indices = np.flatnonzero(overlap)
    if not len(indices):
        return None, None, None
    entry, exit = float(grid[indices[0]]), float(grid[indices[-1]])
    return entry, exit, float(max(0.0, exit - entry))


def audit_solution(scenario: RotSyncScenario, forward, config: RotSyncOptimizationConfig, *, dt: float = 0.001):
    """Independent sampled audit of the real oriented cuboid and dynamics.

    The grid spacing is at most 1 ms.  Knots, both interface sides, prescribed
    crossing time, plane roots, and neighborhoods of the smallest clearances
    are added.  This remains a sampling check, not a continuous-time proof.
    """
    trajectory = forward.trajectory
    crossing = float(forward.crossing_times[0])
    grid = _audit_grid(trajectory, crossing, dt)
    positions, rotations, hits, clearance = _evaluate_body_collision(scenario, trajectory, grid)
    finite_indices = np.flatnonzero(np.isfinite(clearance))
    extra = []
    if len(finite_indices):
        critical = finite_indices[np.argsort(clearance[finite_indices])[: min(12, len(finite_indices))]]
        for index in critical:
            left = grid[max(0, index - 1)]
            right = grid[min(len(grid) - 1, index + 1)]
            extra.extend(np.linspace(left, right, 21))
        slab_indices = finite_indices[[0, -1]]
        for index in slab_indices:
            extra.extend(np.linspace(max(0.0, grid[index] - dt), min(trajectory.total_time, grid[index] + dt), 21))
    roots = _plane_crossings(trajectory, scenario.windows[0], grid, positions)
    for root in roots:
        extra.extend((root - 1.0e-8, root, root + 1.0e-8))
    if extra:
        grid = np.unique(np.r_[grid, np.asarray(extra)])
        grid = grid[(grid >= 0.0) & (grid <= trajectory.total_time)]
        positions, rotations, hits, clearance = _evaluate_body_collision(scenario, trajectory, grid)
        roots = _plane_crossings(trajectory, scenario.windows[0], grid, positions)

    class AuditTrajectory:
        def sample(self, **_kwargs):
            return trajectory.sample(times=grid)
        def evaluate(self, *args, **kwargs):
            return trajectory.evaluate(*args, **kwargs)

    extrema = constraint_extrema(AuditTrajectory(), parameters=config.quadrotor)
    limits = config.dynamic_limits
    tolerance = 1.0e-9
    dynamic_checks = {
        "velocity": bool(extrema["max_velocity"] <= limits.max_velocity + tolerance),
        "collective_thrust": bool(
            extrema["min_collective_thrust"] >= limits.min_collective_thrust - tolerance
            and extrema["max_collective_thrust"] <= limits.max_collective_thrust + tolerance
        ),
        "body_rate_xy": bool(extrema["max_body_rate_xy"] <= limits.max_body_rate_xy + tolerance),
        "body_rate_z": bool(extrema["max_abs_body_rate_z"] <= limits.max_body_rate_z + tolerance),
        "rotor_thrust": bool(
            np.min(extrema["min_rotor_thrust"]) >= limits.min_rotor_thrust - tolerance
            and np.max(extrema["max_rotor_thrust"]) <= limits.max_rotor_thrust + tolerance
        ),
    }
    dynamic_pass = bool(all(dynamic_checks.values()))
    q_index = int(forward.crossing_local_index)
    selected_q = np.asarray(forward.local_points[q_index], dtype=float)
    window = scenario.windows[0]
    expected = window.world_point(selected_q, crossing)
    crossing_error = float(np.linalg.norm(trajectory.evaluate(crossing) - expected))
    # SC images can land a few picometres outside the polygon representation
    # when the optimum is on an edge.  Use a 1 nm numerical membership tolerance;
    # physical cuboid containment is still checked independently without this.
    q_in_safe = bool(Polygon(window.safe_polygon).buffer(1.0e-9).covers(Point(selected_q)))
    endpoint_error = max(
        float(np.max(np.abs(np.stack([trajectory.evaluate(t, d) for d in range(4)]) - state.matrix)))
        for t, state in ((0.0, scenario.start_state), (trajectory.total_time, scenario.goal_state))
    )
    effective = []
    crossing_clearances = []
    for root in roots:
        valid, root_clearance = _valid_body_crossing(scenario, trajectory, root)
        effective.append(bool(valid))
        crossing_clearances.append(root_clearance)
    ordered_once = bool(len(roots) == 1 and len(effective) == 1 and effective[0])
    entry, exit, crossing_duration = _body_slab_duration(scenario, trajectory, grid, positions, rotations)
    finite = clearance[np.isfinite(clearance)]
    minimum_clearance = float(np.min(finite)) if finite.size else None
    interface = _interface_residual(trajectory)
    collision_free = not bool(np.any(hits))
    passed = bool(
        q_in_safe and crossing_error <= 1.0e-8 and endpoint_error <= 1.0e-8
        and ordered_once and collision_free and dynamic_pass and interface <= 1.0e-8
    )
    failures = []
    if not collision_free:
        failures.append("collision")
    if not dynamic_pass:
        failures.append("dynamics")
    if not ordered_once or not q_in_safe or crossing_error > 1.0e-8:
        failures.append("traversal")
    if endpoint_error > 1.0e-8 or interface > 1.0e-8:
        failures.append("boundary_or_interface")
    report = {
        "trajectory_validation_pass": passed,
        "evidence_level": "sampled_numerical_validation_not_continuous_time_proof",
        "audit_dt_max": float(np.max(np.diff(grid))),
        "audit_samples": int(len(grid)),
        "critical_time_refinement": True,
        "collision_free": collision_free,
        "colliding_samples": int(np.count_nonzero(hits)),
        "first_collision_time": float(grid[np.flatnonzero(hits)[0]]) if np.any(hits) else None,
        "minimum_frame_clearance": minimum_clearance,
        "sampled_dynamic_limits_satisfied": dynamic_pass,
        "dynamic_checks": dynamic_checks,
        "constraint_extrema": extrema,
        "endpoint_pvaj_error": endpoint_error,
        "maximum_c3_interface_jump": interface,
        "crossing_error": crossing_error,
        "q_in_safe_region": q_in_safe,
        "plane_crossing_times": roots,
        "effective_crossings": int(sum(effective)),
        "crossing_validity": effective,
        "crossing_clearances": crossing_clearances,
        "ordered_exactly_once": ordered_once,
        "body_slab_entry_time": entry,
        "body_slab_exit_time": exit,
        "crossing_duration": crossing_duration,
        "window_rotation_during_crossing": None if crossing_duration is None else abs(window.omega) * crossing_duration,
        "failure_reasons": failures,
    }
    samples = trajectory.sample(times=grid)
    flat = sample_flatness(trajectory, grid, parameters=config.quadrotor)
    data = {
        "time": grid,
        "position": np.asarray(samples.position, dtype=float),
        "velocity": np.asarray(samples.velocity, dtype=float),
        "acceleration": np.asarray(samples.acceleration, dtype=float),
        "jerk": np.asarray(samples.jerk, dtype=float),
        "snap": np.asarray(samples.snap, dtype=float),
        "speed": np.linalg.norm(samples.velocity, axis=1),
        "body_rotation": rotations,
        "body_rate": np.asarray(flat.body_rate, dtype=float),
        "collective_thrust": np.asarray(flat.collective_thrust, dtype=float),
        "rotor_thrusts": np.asarray(flat.rotor_thrusts, dtype=float),
        "frame_clearance": clearance,
        "collision": hits,
    }
    return report, data


def save_raw_trajectory(directory: Path, data: dict[str, np.ndarray]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(directory / "trajectory_raw.npz", **data)
    columns = np.column_stack((
        data["time"], data["position"], data["velocity"], data["acceleration"],
        data["jerk"], data["speed"], data["frame_clearance"], data["collision"].astype(int),
    ))
    np.savetxt(
        directory / "trajectory.csv", columns, delimiter=",",
        header="time,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,speed,frame_clearance,collision",
        comments="",
    )


SUMMARY_FIELDS = (
    "scenario", "shape", "size_ratio", "omega", "phase", "method", "status",
    "trajectory_pass", "optimizer_success", "timed_out", "failure_category", "failure_reasons",
    "flight_time", "solve_seconds", "budget_seconds", "objective", "time_cost", "smoothness_cost",
    "dynamic_penalty", "collision_penalty", "max_velocity", "max_body_rate_xy",
    "max_body_rate_z", "min_rotor_thrust", "max_rotor_thrust", "minimum_frame_clearance",
    "max_dynamic_relative_violation", "dynamic_margin_class",
    "crossing_time", "crossing_duration", "window_rotation_during_crossing", "plane_crossings",
    "effective_crossings", "colliding_samples", "audit_samples", "audit_dt_max", "iterations",
    "evaluations", "message", "physical_inradius", "axis_centered_incircle_radius",
    "safe_inradius", "safe_area", "sc_cyclic_shift",
)


def write_summary(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key) for key in SUMMARY_FIELDS})


def result_row(scenario, geometry, method, solve: SolveRecord, audit: dict[str, Any]) -> dict[str, Any]:
    extrema = audit["constraint_extrema"]
    failure_reasons = list(audit["failure_reasons"])
    if solve.timed_out:
        failure_reasons.append("solve_timeout")
    elif not solve.optimizer_success:
        failure_reasons.append("optimizer_nonconvergence")
    if audit["trajectory_validation_pass"]:
        category = "success"
    elif "collision" in failure_reasons:
        category = "collision"
    elif "dynamics" in failure_reasons:
        category = "dynamics"
    elif "traversal" in failure_reasons:
        category = "invalid_traversal"
    else:
        category = "solve_failure"
    return {
        "scenario": scenario.name,
        "shape": geometry.shape,
        "size_ratio": geometry.ratio,
        "omega": scenario.windows[0].omega,
        "phase": scenario.windows[0].theta0,
        "method": method,
        "status": "completed",
        "trajectory_pass": audit["trajectory_validation_pass"],
        "optimizer_success": solve.optimizer_success,
        "timed_out": solve.timed_out,
        "failure_category": category,
        "failure_reasons": ";".join(dict.fromkeys(failure_reasons)),
        "flight_time": solve.forward.trajectory.total_time,
        "solve_seconds": solve.solve_seconds,
        "budget_seconds": solve.budget_seconds,
        "objective": solve.cost.weighted_total,
        "time_cost": solve.cost.total_time,
        "smoothness_cost": solve.cost.smoothness,
        "dynamic_penalty": solve.cost.dynamic_penalty,
        "collision_penalty": solve.cost.collision_penalty,
        "max_velocity": extrema["max_velocity"],
        "max_body_rate_xy": extrema["max_body_rate_xy"],
        "max_body_rate_z": extrema["max_abs_body_rate_z"],
        "min_rotor_thrust": float(np.min(extrema["min_rotor_thrust"])),
        "max_rotor_thrust": float(np.max(extrema["max_rotor_thrust"])),
        "minimum_frame_clearance": audit["minimum_frame_clearance"],
        "crossing_time": float(solve.forward.crossing_times[0]),
        "crossing_duration": audit["crossing_duration"],
        "window_rotation_during_crossing": audit["window_rotation_during_crossing"],
        "plane_crossings": len(audit["plane_crossing_times"]),
        "effective_crossings": audit["effective_crossings"],
        "colliding_samples": audit["colliding_samples"],
        "audit_samples": audit["audit_samples"],
        "audit_dt_max": audit["audit_dt_max"],
        "iterations": solve.iterations,
        "evaluations": solve.evaluations,
        "message": solve.message,
        "physical_inradius": geometry.physical_inradius,
        "axis_centered_incircle_radius": geometry.axis_centered_incircle_radius,
        "safe_inradius": geometry.safe_inradius,
        "safe_area": geometry.gate.safe_region.area,
        "sc_cyclic_shift": geometry.sc_cyclic_shift,
    }


def geometry_failure_rows(shape, ratio, omega, phase, message):
    name = scenario_name(shape, ratio, omega, phase)
    return [{
        "scenario": name, "shape": shape, "size_ratio": ratio, "omega": omega,
        "phase": phase, "method": method, "status": "geometry_failure",
        "trajectory_pass": False, "optimizer_success": False, "timed_out": False,
        "failure_category": "geometry_mapping_failure", "failure_reasons": message,
        "budget_seconds": 180.0, "message": message,
    } for method in METHODS]


def make_config(weights: dict[str, float], max_iterations: int) -> RotSyncOptimizationConfig:
    return RotSyncOptimizationConfig(
        initial_speed=2.5,
        initial_sync_duration=0.48,
        minimum_initial_free_duration=0.55,
        smoothness_weight=float(weights["smoothness_weight"]),
        dynamics_weight=float(weights["dynamics_weight"]),
        samples_per_segment=int(weights["objective_samples_per_segment"]),
        max_iterations=int(max_iterations),
    )


def run_one(scenario, geometry, method, config, collision_weight, budget, directory):
    objective = OBJECTIVES[method](scenario, config, collision_weight)
    solve = solve_with_budget(objective, budget)
    audit, data = audit_solution(scenario, solve.forward, config, dt=0.001)
    row = result_row(scenario, geometry, method, solve, audit)
    violations = dynamic_violation_summary(audit, config.dynamic_limits)
    row["max_dynamic_relative_violation"] = violations["maximum_relative_violation"]
    row["dynamic_margin_class"] = violations["classification"]
    write_json(directory / "result.json", {
        "row": row,
        "decision_vector": solve.x,
        "selected_local_points": solve.forward.local_points,
        "crossing_times": solve.forward.crossing_times,
        "durations": solve.forward.trajectory.durations,
        "cost_breakdown": solve.cost,
        "optimizer": {
            "success": solve.optimizer_success, "status": solve.status,
            "message": solve.message, "iterations": solve.iterations,
            "evaluations": solve.evaluations, "solve_seconds": solve.solve_seconds,
            "budget_seconds": solve.budget_seconds, "timed_out": solve.timed_out,
            "invalid_trials": solve.invalid_trials,
        },
        "audit": audit,
        "dynamic_violation_summary": violations,
    })
    save_raw_trajectory(directory, data)
    return row, data


def dynamic_violation_summary(audit: dict[str, Any], limits) -> dict[str, Any]:
    """Quantify limit exceedance without weakening the formal pass criteria.

    A <=5% exceedance is labelled as a sensitivity/near-limit case.  It remains
    a formal dynamics failure; the band exists only to distinguish round-off or
    small engineering sensitivity from a material violation.
    """
    extrema = audit["constraint_extrema"]
    rotor_min = float(np.min(extrema["min_rotor_thrust"]))
    rotor_max = float(np.max(extrema["max_rotor_thrust"]))
    values = {
        "velocity_upper": max(0.0, float(extrema["max_velocity"]) / limits.max_velocity - 1.0),
        "collective_thrust_lower": max(0.0, (limits.min_collective_thrust - float(extrema["min_collective_thrust"])) / max(abs(limits.min_collective_thrust), 1.0e-12)),
        "collective_thrust_upper": max(0.0, float(extrema["max_collective_thrust"]) / limits.max_collective_thrust - 1.0),
        "body_rate_xy_upper": max(0.0, float(extrema["max_body_rate_xy"]) / limits.max_body_rate_xy - 1.0),
        "body_rate_z_upper": max(0.0, float(extrema["max_abs_body_rate_z"]) / limits.max_body_rate_z - 1.0),
        "rotor_thrust_lower": max(0.0, (limits.min_rotor_thrust - rotor_min) / max(abs(limits.min_rotor_thrust), 1.0e-12)),
        "rotor_thrust_upper": max(0.0, rotor_max / limits.max_rotor_thrust - 1.0),
    }
    maximum = float(max(values.values()))
    if maximum <= 1.0e-12:
        classification = "within_limits"
    elif maximum <= 0.05:
        classification = "near_limit_exceedance_le_5pct"
    else:
        classification = "material_exceedance_gt_5pct"
    return {
        "formal_limits_satisfied": bool(audit["sampled_dynamic_limits_satisfied"]),
        "relative_violations": values,
        "maximum_relative_violation": maximum,
        "classification": classification,
        "near_limit_band": 0.05,
        "near_limit_band_changes_formal_pass": False,
    }


def calibration_geometry(shape: str, *, vertex_count: int, quadrature_order: int):
    axis = CALIBRATION_U_AXIS_IN_CANONICAL_NOTCH if shape == "U" else np.zeros(2)
    return prepare_geometry(
        shape, 1.4, vertex_count=vertex_count, quadrature_order=quadrature_order,
        canonical_axis=axis,
    )


def calibrate(root: Path, *, vertex_count: int, quadrature_order: int) -> dict[str, float]:
    """Freeze one weight set using two tasks outside the formal 81-case grid."""
    calibration_root = root / "calibration"
    calibration_root.mkdir(parents=True, exist_ok=True)
    candidates = (
        {"smoothness_weight": 2.0e-4, "dynamics_weight": 0.05, "collision_weight": 80.0, "objective_samples_per_segment": 9},
        {"smoothness_weight": 2.0e-4, "dynamics_weight": 0.10, "collision_weight": 200.0, "objective_samples_per_segment": 11},
        {"smoothness_weight": 2.0e-4, "dynamics_weight": 0.20, "collision_weight": 500.0, "objective_samples_per_segment": 13},
    )
    tasks = (("U", 1.1, 0.6), ("star", 1.1, 0.6))
    rows = []
    for candidate_index, weights in enumerate(candidates):
        config = make_config(weights, max_iterations=28)
        for shape, omega, phase in tasks:
            geometry = calibration_geometry(shape, vertex_count=vertex_count, quadrature_order=quadrature_order)
            scenario = build_scenario(geometry, omega, phase, name=f"cal_{shape}_r1p4_w1p10_p0p6")
            for method in ("Optimized-MINCO", "SC+Sync"):
                target = calibration_root / f"candidate_{candidate_index}" / scenario.name / method
                target.mkdir(parents=True, exist_ok=True)
                row, _ = run_one(
                    scenario, geometry, method, config, weights["collision_weight"],
                    60.0, target,
                )
                row["candidate"] = candidate_index
                rows.append(row)
                print(f"CALIBRATION candidate={candidate_index} {scenario.name} {method} pass={row['trajectory_pass']}", flush=True)
    score = []
    for index, weights in enumerate(candidates):
        selected = [row for row in rows if row["candidate"] == index]
        passes = sum(bool(row["trajectory_pass"]) for row in selected)
        collisions = sum(row["failure_category"] == "collision" for row in selected)
        dynamics = sum(row["failure_category"] == "dynamics" for row in selected)
        median_time = float(np.median([row["flight_time"] for row in selected if row.get("flight_time") is not None]))
        score.append((passes, -collisions, -dynamics, -median_time, -index))
    chosen_index = int(max(range(len(candidates)), key=lambda index: score[index]))
    chosen = dict(candidates[chosen_index])
    frozen = {
        **chosen,
        "time_weight": 1.0,
        "selected_candidate": chosen_index,
        "selection_rule": "maximize sampled-valid trajectories, then fewer collision/dynamic failures, then lower median flight time",
        "calibration_tasks": [
            {"shape": shape, "size_ratio": 1.4, "omega": omega, "phase": phase}
            for shape, omega, phase in tasks
        ],
        "formal_grid_disjoint": True,
    }
    write_json(root / "frozen_weights.json", frozen)
    write_json(calibration_root / "calibration_results.json", {"candidates": candidates, "rows": rows, "scores": score, "chosen": frozen})
    with (calibration_root / "calibration_results.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = ("candidate",) + SUMMARY_FIELDS
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return frozen


def load_or_calibrate_weights(root: Path, args) -> dict[str, float]:
    frozen_path = root / "frozen_weights.json"
    if frozen_path.exists():
        return json.loads(frozen_path.read_text(encoding="utf-8"))
    return calibrate(root, vertex_count=args.vertex_count, quadrature_order=args.quadrature_order)


def _load_rows(summary_path: Path) -> list[dict[str, Any]]:
    if not summary_path.exists():
        return []
    with summary_path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _coerce_existing_row(row: dict[str, str]) -> dict[str, Any]:
    booleans = {"trajectory_pass", "optimizer_success", "timed_out"}
    integers = {"plane_crossings", "effective_crossings", "colliding_samples", "audit_samples", "iterations", "evaluations", "sc_cyclic_shift"}
    strings = {
        "scenario", "shape", "method", "status", "failure_category",
        "failure_reasons", "message", "dynamic_margin_class",
    }
    out = dict(row)
    for key, value in list(out.items()):
        if value == "":
            out[key] = None
        elif key in booleans:
            out[key] = value.lower() == "true"
        elif key in integers:
            out[key] = int(float(value))
        elif key not in strings:
            out[key] = float(value)
    return out


def backfill_dynamic_violation_metadata(root: Path, rows: list[dict[str, Any]], config) -> None:
    """Add the frozen 5% sensitivity label to results produced before it existed."""
    for row in rows:
        if row.get("status") != "completed" or row.get("dynamic_margin_class"):
            continue
        result_path = root / "scenarios" / row["scenario"] / row["method"] / "result.json"
        if not result_path.exists():
            continue
        result = json.loads(result_path.read_text(encoding="utf-8"))
        summary = dynamic_violation_summary(result["audit"], config.dynamic_limits)
        row["max_dynamic_relative_violation"] = summary["maximum_relative_violation"]
        row["dynamic_margin_class"] = summary["classification"]
        result["row"].update({
            "max_dynamic_relative_violation": summary["maximum_relative_violation"],
            "dynamic_margin_class": summary["classification"],
        })
        result["dynamic_violation_summary"] = summary
        write_json(result_path, result)


def plot_success_rates(root: Path, rows: list[dict[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(SHAPES), figsize=(7.2 * len(SHAPES), 4.8),
                             sharey=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    colours = ("#4C78A8", "#F58518", "#54A24B")
    markers = ("o", "s", "^")
    for ax, shape in zip(axes, SHAPES):
        for method, colour, marker in zip(METHODS, colours, markers):
            rates = []
            for omega in OMEGAS:
                selected = [
                    r for r in rows if r["shape"] == shape and r["method"] == method
                    and np.isclose(float(r["omega"]), omega)
                ]
                rates.append(
                    100.0 * sum(bool(r["trajectory_pass"]) for r in selected) / len(selected)
                    if selected else np.nan
                )
            ax.plot(OMEGAS, rates, color=colour, marker=marker, lw=2.1,
                    markerfacecolor="white", markeredgewidth=1.7, label=method)
        ax.set_xlabel("Window angular speed (rad/s)")
        ax.set_xticks(OMEGAS)
        ax.set_ylim(-3, 103)
        ax.grid(alpha=0.25)
        ax.set_title(f"{shape} aperture")
        ax.legend()
    axes[0].set_ylabel("Sampled-validation success (%)")
    fig.suptitle("Focused non-convex apertures: success versus rotation speed")
    fig.savefig(root / "success_rate.png", dpi=190)
    fig.savefig(root / "success_rate.pdf")
    plt.close(fig)


def plot_common_flight_times(root: Path, rows: list[dict[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    by_case = {}
    for row in rows:
        by_case.setdefault(row["scenario"], {})[row["method"]] = row
    common = [values for values in by_case.values() if all(m in values and bool(values[m]["trajectory_pass"]) for m in METHODS)]
    fig, axes = plt.subplots(1, len(SHAPES), figsize=(7.2 * len(SHAPES), 4.8),
                             sharey=True, constrained_layout=True)
    axes = np.atleast_1d(axes)
    colours = ("#4C78A8", "#F58518", "#54A24B")
    markers = ("o", "s", "^")
    for ax, shape in zip(axes, SHAPES):
        shape_common = [values for values in common if values[METHODS[0]]["shape"] == shape]
        for method, colour, marker in zip(METHODS, colours, markers):
            pairs = sorted(
                (float(values[method]["omega"]), float(values[method]["flight_time"]))
                for values in shape_common
            )
            if pairs:
                ax.plot([p[0] for p in pairs], [p[1] for p in pairs], color=colour,
                        marker=marker, lw=2.1, markerfacecolor="white",
                        markeredgewidth=1.7, label=method)
        ax.set_xticks(OMEGAS)
        ax.set_xlabel("Window angular speed (rad/s)")
        ax.set_title(f"{shape}: {len(shape_common)} common-success speeds")
        ax.grid(alpha=0.25)
        ax.legend()
    axes[0].set_ylabel("Flight time (s)")
    fig.suptitle("Flight time only where all three methods pass")
    fig.savefig(root / "common_success_flight_time.png", dpi=190)
    fig.savefig(root / "common_success_flight_time.pdf")
    plt.close(fig)


def _cuboid_edges(vertices: np.ndarray):
    signs = np.asarray([
        (-1,-1,-1),(-1,-1,1),(-1,1,-1),(-1,1,1),
        (1,-1,-1),(1,-1,1),(1,1,-1),(1,1,1),
    ])
    return [(i, j) for i in range(8) for j in range(i + 1, 8) if np.count_nonzero(signs[i] != signs[j]) == 1]


def plot_representatives(root: Path, rows: list[dict[str, Any]]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.colors import Normalize
    from matplotlib.cm import ScalarMappable

    representatives = tuple(
        scenario_name("U", SIZE_RATIOS[0], omega, PHASES[0])
        for omega in (OMEGAS[0], OMEGAS[len(OMEGAS) // 2], OMEGAS[-1])
    )
    available = []
    maximum_speed = 7.0
    for case in representatives:
        method_data = {}
        for method in METHODS:
            path = root / "scenarios" / case / method / "trajectory_raw.npz"
            result_path = root / "scenarios" / case / method / "result.json"
            if path.exists() and result_path.exists():
                with np.load(path, allow_pickle=False) as arrays:
                    method_data[method] = {key: np.asarray(arrays[key]) for key in arrays.files}
                method_data[method]["result"] = json.loads(result_path.read_text(encoding="utf-8"))
                maximum_speed = max(maximum_speed, float(np.max(method_data[method]["speed"])))
        available.append((case, method_data))
    norm = Normalize(0.0, maximum_speed)
    fig, axes = plt.subplots(3, 3, figsize=(14.2, 11.2), constrained_layout=True)
    for row_index, (case, method_data) in enumerate(available):
        # Use one y-z range for all three methods in a scenario.  Include the
        # complete physical apertures and cuboids so no geometry is clipped.
        row_yz = []
        for method, item in method_data.items():
            row_yz.append(item["position"][:, 1:3])
            result = item["result"]
            crossing = float(result["row"]["crossing_time"])
            case_config = json.loads((root / "scenarios" / case / "config.json").read_text(encoding="utf-8"))
            physical = np.asarray(case_config["window"]["physical_polygon"], dtype=float)
            theta = float(case_config["window"]["theta0"]) + float(case_config["window"]["omega"]) * crossing
            rotation = np.asarray(((math.cos(theta), -math.sin(theta)), (math.sin(theta), math.cos(theta))))
            row_yz.append(physical @ rotation.T + WINDOW_CENTER[1:3])
            index = int(np.argmin(np.abs(item["time"] - crossing)))
            body_vertices = cuboid_vertices(item["position"][index], item["body_rotation"][index], BODY)
            row_yz.append(body_vertices[:, 1:3])
        row_limits = None
        if row_yz:
            values = np.vstack(row_yz)
            lower = values.min(axis=0)
            upper = values.max(axis=0)
            span = np.maximum(upper - lower, 0.25)
            margin = 0.06 * span
            row_limits = ((lower[0] - margin[0], upper[0] + margin[0]),
                          (lower[1] - margin[1], upper[1] + margin[1]))
        for column_index, method in enumerate(METHODS):
            ax = axes[row_index, column_index]
            item = method_data.get(method)
            if item is None:
                ax.text(0.5, 0.5, "No trajectory", ha="center", va="center", transform=ax.transAxes)
                ax.set_title(f"{case}\n{method}")
                continue
            position, speed = item["position"], item["speed"]
            yz = position[:, 1:3]
            segments = np.stack((yz[:-1], yz[1:]), axis=1)
            collection = LineCollection(segments, cmap="viridis", norm=norm, linewidth=2.2)
            collection.set_array(0.5 * (speed[:-1] + speed[1:]))
            ax.add_collection(collection)
            result = item["result"]
            crossing = float(result["row"]["crossing_time"])
            case_config = json.loads((root / "scenarios" / case / "config.json").read_text(encoding="utf-8"))
            physical = np.asarray(case_config["window"]["physical_polygon"], dtype=float)
            theta = float(case_config["window"]["theta0"]) + float(case_config["window"]["omega"]) * crossing
            rotation = np.asarray(((math.cos(theta), -math.sin(theta)), (math.sin(theta), math.cos(theta))))
            boundary = physical @ rotation.T + WINDOW_CENTER[1:3]
            boundary = np.vstack((boundary, boundary[0]))
            ax.plot(boundary[:, 0], boundary[:, 1], color="black", lw=1.4)
            index = int(np.argmin(np.abs(item["time"] - crossing)))
            vertices = cuboid_vertices(item["position"][index], item["body_rotation"][index], BODY)
            for left, right in _cuboid_edges(vertices):
                ax.plot(vertices[[left, right], 1], vertices[[left, right], 2], color="#D62728", lw=0.8)
            if row_limits:
                ax.set_xlim(*row_limits[0]); ax.set_ylim(*row_limits[1])
            ax.set_aspect("equal", adjustable="box")
            ax.grid(alpha=0.2)
            ax.set_title(f"{case}\n{method}", fontsize=9)
            if row_index == 2:
                ax.set_xlabel("world y (m)")
            if column_index == 0:
                ax.set_ylabel("world z (m)")
    scalar = ScalarMappable(norm=norm, cmap="viridis")
    fig.colorbar(scalar, ax=axes.ravel().tolist(), label="Speed (m/s)", shrink=0.82)
    fig.suptitle("Representative speed-coloured trajectories; shared scale and true cuboid proportions")
    fig.savefig(root / "representative_speed_trajectories.png", dpi=190)
    fig.savefig(root / "representative_speed_trajectories.pdf")
    plt.close(fig)


def write_report(root: Path, rows: list[dict[str, Any]], weights: dict[str, Any]) -> None:
    complete = [row for row in rows if row.get("status") == "completed"]
    geometry_lines = []
    for shape in SHAPES:
        selected = [row for row in complete if row.get("shape") == shape]
        if not selected:
            geometry_lines.append(f"- {shape}：待几何生成。")
            continue
        sample = selected[0]
        axis_radius = float(sample["axis_centered_incircle_radius"])
        geometry_lines.append(
            f"- {shape}：轴心容纳圆半径 `{axis_radius:.9f} m` "
            f"（机体外接球的 `{axis_radius / BODY.circumscribed_radius:.2%}`）；"
            f"偏轴最大内切圆 `{float(sample['physical_inradius']):.9f} m`，"
            f"为规划包络半径的 `{float(sample['physical_inradius']) / PLANNING_RHO:.2f}` 倍。"
        )
    lines = [
        "# 实验三结果：单窗口 Sync 聚焦转速对比", "",
        f"本报告统计固定 L/U 形开口、固定初相位 {PHASES[0]:g} rad 下的 {FORMAL_SCENE_COUNT} 个预定转速场景。",
        "L 轴位于窄臂内，U 轴固定在凹槽中；两者的偏轴最大内切圆都大于规划包络球，因此不是全局几何不可行。",
        "三种方法的验收使用最大 1 ms 网格，并加密分段接口、穿窗根和最小净距邻域。",
        "这是名义模型的采样数值验收，不是连续时间安全证明。", "",
        "## 几何核对", "",
        f"- 真实机体外接球半径：`{BODY.circumscribed_radius:.9f} m`；规划包络半径：`{PLANNING_RHO:.9f} m`。",
        *geometry_lines, "",
        "## 冻结权重", "", "```json", json.dumps(_jsonable(weights), ensure_ascii=False, indent=2), "```", "",
        "## 汇总", "",
        "|方法|采样验收成功|优化器收敛|碰撞失败|动力学失败|穿窗失败|求解/其他失败|飞行时间中位数/s|求解时间中位数/s|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for method in METHODS:
        selected = [row for row in rows if row["method"] == method]
        counts = {
            "collision": sum("collision" in str(row.get("failure_reasons") or "").split(";") for row in selected),
            "dynamics": sum("dynamics" in str(row.get("failure_reasons") or "").split(";") for row in selected),
            "invalid_traversal": sum("traversal" in str(row.get("failure_reasons") or "").split(";") for row in selected),
            "solve_failure": sum(row.get("failure_category") == "solve_failure" for row in selected),
            "geometry_mapping_failure": sum(row.get("failure_category") == "geometry_mapping_failure" for row in selected),
        }
        solve_times = [float(row["solve_seconds"]) for row in selected if row.get("solve_seconds") is not None]
        flight_times = [float(row["flight_time"]) for row in selected if row.get("flight_time") is not None and bool(row.get("trajectory_pass"))]
        lines.append(
            f"|{method}|{sum(bool(row.get('trajectory_pass')) for row in selected)}/{len(selected)}|"
            f"{sum(bool(row.get('optimizer_success')) for row in selected)}/{len(selected)}|"
            f"{counts['collision']}|{counts['dynamics']}|{counts['invalid_traversal']}|"
            f"{counts['solve_failure'] + counts['geometry_mapping_failure']}|"
            f"{np.median(flight_times):.4f}|{np.median(solve_times):.3f}|" if solve_times and flight_times else
            f"|{method}|0/{len(selected)}|0/{len(selected)}|0|0|0|{len(selected)}|n/a|n/a|"
        )
    lines += ["", "## 逐转速结果", ""]
    for shape in SHAPES:
        lines += [f"### {shape} 形", "", "|rad/s|Fixed-WP|Optimized-MINCO|SC+Sync|", "|---:|---|---|---|"]
        for omega in OMEGAS:
            cells = []
            for method in METHODS:
                matches = [r for r in rows if r["shape"] == shape and r["method"] == method and np.isclose(float(r["omega"]), omega)]
                if not matches:
                    cells.append("未完成")
                    continue
                row = matches[0]
                state = "通过" if bool(row.get("trajectory_pass")) else f"失败：{row.get('failure_category')}"
                cells.append(f"{state}，{float(row['flight_time']):.3f} s" if row.get("flight_time") is not None else state)
            lines.append(f"|{omega:g}|{cells[0]}|{cells[1]}|{cells[2]}|")
        lines.append("")
    by_case = {}
    for row in rows:
        by_case.setdefault(row["scenario"], {})[row["method"]] = row
    common = [values for values in by_case.values() if all(m in values and bool(values[m].get("trajectory_pass")) for m in METHODS)]
    lines += ["", f"三方法共同成功场景：{len(common)}/{FORMAL_SCENE_COUNT}。共同成功飞行时间只在这组配对场景上比较。", ""]
    if common:
        sync_vs_ordinary = np.asarray([
            float(values["SC+Sync"]["flight_time"]) / float(values["Optimized-MINCO"]["flight_time"]) - 1.0
            for values in common
        ])
        sync_vs_fixed = np.asarray([
            float(values["SC+Sync"]["flight_time"]) / float(values["Fixed-WP"]["flight_time"]) - 1.0
            for values in common
        ])
        lines += [
            f"在共同成功场景中，SC+Sync 相对 Optimized-MINCO 的飞行时间中位变化为 "
            f"{np.median(sync_vs_ordinary) * 100:+.2f}%（均值 {np.mean(sync_vs_ordinary) * 100:+.2f}%），",
            f"相对 Fixed-WP 的中位变化为 {np.median(sync_vs_fixed) * 100:+.2f}%"
            f"（均值 {np.mean(sync_vs_fixed) * 100:+.2f}%）。", "",
        ]
    ordinary_rows = [row for row in rows if row.get("method") in ("Fixed-WP", "Optimized-MINCO")]
    ordinary_pass_count = sum(bool(row.get("trajectory_pass")) for row in ordinary_rows)
    sync_failures = [row for row in rows if row.get("method") == "SC+Sync" and not bool(row.get("trajectory_pass"))]
    dynamic_violations = [
        row for row in rows
        if float(row.get("max_dynamic_relative_violation") or 0.0) > 0.0
    ]
    if sync_failures:
        lines += [
            "## 结论边界", "",
            f"两条普通基线合计通过 {ordinary_pass_count}/{len(ordinary_rows)}；轴心局部窄于机体本身不会阻止它们选择偏轴路径。",
            "失败原因与动力学超限程度按下表保留；5% 只是敏感性分带，不改变正式成败。", "",
        ]
        limits = make_config(weights, 120).dynamic_limits
        lines += ["|形状|方法|转速|最大相对超限|程度|超限项|", "|---|---|---:|---:|---|---|"]
        for row in sorted(dynamic_violations, key=lambda item: (float(item["omega"]), str(item["shape"]), str(item["method"]))):
            result_path = root / "scenarios" / row["scenario"] / row["method"] / "result.json"
            result = json.loads(result_path.read_text(encoding="utf-8"))
            audit = result["audit"]
            extrema = audit["constraint_extrema"]
            exceeded = []
            if not audit["dynamic_checks"]["velocity"]:
                exceeded.append(f"speed {float(extrema['max_velocity']):.3f}>{limits.max_velocity:g} m/s")
            if not audit["dynamic_checks"]["body_rate_xy"]:
                exceeded.append(f"body-rate-xy {float(extrema['max_body_rate_xy']):.3f}>{limits.max_body_rate_xy:g} rad/s")
            if not audit["dynamic_checks"]["body_rate_z"]:
                exceeded.append(f"body-rate-z {float(extrema['max_abs_body_rate_z']):.3f}>{limits.max_body_rate_z:g} rad/s")
            if not audit["dynamic_checks"]["rotor_thrust"]:
                minimum = float(np.min(extrema["min_rotor_thrust"]))
                maximum = float(np.max(extrema["max_rotor_thrust"]))
                exceeded.append(f"rotor thrust [{minimum:.3f}, {maximum:.3f}] outside [{limits.min_rotor_thrust:g}, {limits.max_rotor_thrust:g}] N")
            relative = float(row.get("max_dynamic_relative_violation") or 0.0)
            margin_class = str(row.get("dynamic_margin_class") or "unknown")
            lines.append(f"|{row['shape']}|{row['method']}|{float(row['omega']):g}|{relative:.2%}|{margin_class}|{'<br>'.join(exceeded)}|")
        lines.append("")
    expected_rows = FORMAL_SCENE_COUNT * len(METHODS)
    if len(complete) != expected_rows:
        lines.append(f"当前完成求解与验收 {len(complete)}/{expected_rows} 项；未完成项不从分母移除。")
        lines.append("")
    lines += [
        "失败类别按碰撞、动力学、无效穿窗、求解/几何顺序归类；CSV 中保留全部同时发生的原因。",
        "净距是实际姿态长方体与物理门框边界在厚度带内的采样最小距离。",
        "穿越持续时间按实际姿态长方体与门厚带重叠的采样首末时刻测量，窗口转角为 |omega| × 持续时间。",
        "优化器收敛与轨迹采样验收分开：达到全局120次迭代或180 s预算的轨迹若完整验收通过，仍保留轨迹和优化器未收敛/超时标记。",
        "", "![成功率](success_rate.png)", "", "![共同成功飞行时间](common_success_flight_time.png)",
        "", "![代表性速度着色轨迹](representative_speed_trajectories.png)",
    ]
    (root / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=HERE / "focused_results")
    parser.add_argument("--budget-seconds", type=float, default=180.0)
    # Match the existing RotSync formal upper bound.  The 180 s wall budget is
    # an independent hard outer limit and includes all objective evaluations.
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--vertex-count", type=int, default=256)
    parser.add_argument("--quadrature-order", type=int, default=64)
    parser.add_argument("--shapes", nargs="+", choices=SHAPES, default=list(SHAPES),
                        help="optional deterministic shape shard; default is L and U")
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS),
                        help="optional deterministic method shard; default is all methods")
    parser.add_argument("--skip-calibration", action="store_true", help="require an existing frozen_weights.json")
    parser.add_argument("--only", nargs=4, metavar=("SHAPE", "RATIO", "OMEGA", "PHASE"), help="run one formal scenario for debugging")
    parser.add_argument("--rerun-existing", action="store_true", help="replace rows selected by --only after an audit or implementation fix")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)
    root = args.outdir.resolve()
    root.mkdir(parents=True, exist_ok=True)
    if args.skip_calibration:
        weights = json.loads((root / "frozen_weights.json").read_text(encoding="utf-8"))
    else:
        weights = load_or_calibrate_weights(root, args)
    config = make_config(weights, args.max_iterations)
    protocol = {
        "experiment": "ICRA experiment 03 - focused single-window Sync speed sweep",
        "formal_scene_count": FORMAL_SCENE_COUNT,
        "methods_per_scene": 3,
        "shapes": SHAPES,
        "size_ratios": SIZE_RATIOS,
        "omegas_rad_s": OMEGAS,
        "initial_phases_rad": PHASES,
        "body_half_extents_m": BODY.half_extents,
        "body_full_dimensions_m": 2.0 * np.asarray(BODY.half_extents),
        "body_circumscribed_radius_m": BODY.circumscribed_radius,
        "planning_envelope_radius_m": PLANNING_RHO,
        "planning_margin_m": 0.015,
        "window_thickness_m": THICKNESS,
        "minimum_safe_area_m2": MIN_SAFE_AREA,
        "rotation_axes_canonical": {
            "L": L_AXIS_IN_CANONICAL_NARROW_ARM,
            "U": CALIBRATION_U_AXIS_IN_CANONICAL_NOTCH,
        },
        "axis_rule": "L fixed at (0.4,0.0) inside its narrow arm; U fixed at (0,1.5) in its notch; both become local origin after transform",
        "geometry_hypothesis": "axis-centred contained-circle radius < body circumscribed radius, while global aperture inradius > planning-envelope radius",
        "start_state_pvaj": START.matrix,
        "goal_state_pvaj": GOAL.matrix,
        "solver_budget_seconds_per_method_scene": args.budget_seconds,
        "max_iterations": args.max_iterations,
        "vertex_count": args.vertex_count,
        "sc_quadrature_order": args.quadrature_order,
        "audit": "oriented cuboid, <=1 ms grid plus critical refinement; sampled validation only",
        "common_objective": "time + smoothness_weight*snap + dynamics_weight*P_dyn + collision_weight*P_full_trajectory_sphere",
        "weights": weights,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "formal_run_command": "bash nonconvex_timevarying_window/rot_sync_sc_togt/icra_experiments/03_sync_single/run.sh",
        "last_invocation_argv": sys.argv,
    }
    write_json(root / "experiment_config.json", protocol)

    selected_grid = [(shape, ratio, omega, phase) for shape in args.shapes for ratio in SIZE_RATIOS for omega in OMEGAS for phase in PHASES]
    if args.only:
        shape, ratio, omega, phase = args.only
        selected_grid = [(shape, float(ratio), float(omega), float(phase))]
    summary_path = root / "results.csv"
    rows = [_coerce_existing_row(row) for row in _load_rows(summary_path)]
    backfill_dynamic_violation_metadata(root, rows, config)
    write_summary(summary_path, rows)
    if args.rerun_existing:
        if not args.only:
            parser.error("--rerun-existing requires --only")
        rerun_name = scenario_name(args.only[0], float(args.only[1]), float(args.only[2]), float(args.only[3]))
        rows = [
            row for row in rows
            if not (row["scenario"] == rerun_name and row["method"] in args.methods)
        ]
        write_summary(summary_path, rows)
    existing = {(row["scenario"], row["method"]) for row in rows}
    geometry_cache = {}
    geometry_failures = {}
    for shape, ratio, omega, phase in selected_grid:
        key = (shape, ratio)
        if key not in geometry_cache and key not in geometry_failures:
            try:
                geometry_cache[key] = prepare_geometry(
                    shape, ratio, vertex_count=args.vertex_count,
                    quadrature_order=args.quadrature_order,
                )
                geometry_cache[key].gate.save(root / "geometry" / f"{shape}_r{slug_number(ratio,2)}")
                prepared = geometry_cache[key]
                write_json(root / "geometry" / f"{shape}_r{slug_number(ratio,2)}" / "experiment_geometry.json", {
                    "shape": prepared.shape,
                    "size_ratio": prepared.ratio,
                    "canonical_axis": prepared.canonical_axis,
                    "scale": prepared.scale,
                    "source_inradius": prepared.source_inradius,
                    "physical_inradius": prepared.physical_inradius,
                    "axis_centered_incircle_radius": prepared.axis_centered_incircle_radius,
                    "body_circumscribed_radius": BODY.circumscribed_radius,
                    "planning_envelope_radius": PLANNING_RHO,
                    "axis_centered_radius_less_than_body_radius": prepared.axis_centered_incircle_radius < BODY.circumscribed_radius,
                    "off_axis_inradius_greater_than_planning_envelope": prepared.physical_inradius > PLANNING_RHO,
                    "safe_inradius": prepared.safe_inradius,
                    "safe_area": prepared.gate.safe_region.area,
                    "physical_polygon": prepared.gate.dense_boundary.vertices,
                    "safe_polygon": prepared.gate.safe_polygon,
                    "sc_cyclic_shift": prepared.sc_cyclic_shift,
                    "sc_fit_diagnostics": prepared.gate.sc_map.diagnostics,
                    "offset_diagnostics": prepared.gate.safe_region.diagnostics,
                })
            except Exception as exc:
                attempts = getattr(exc, "attempts", None)
                geometry_failures[key] = f"{type(exc).__name__}: {exc}"
                write_json(root / "geometry" / f"{shape}_r{slug_number(ratio,2)}" / "failure.json", {
                    "shape": shape, "ratio": ratio, "type": type(exc).__name__,
                    "message": str(exc), "sc_cyclic_attempts": attempts,
                })
        name = scenario_name(shape, ratio, omega, phase)
        if key in geometry_failures:
            for row in geometry_failure_rows(shape, ratio, omega, phase, geometry_failures[key]):
                if row["method"] not in args.methods:
                    continue
                if (name, row["method"]) not in existing:
                    rows.append(row); existing.add((name, row["method"]))
            write_summary(summary_path, rows)
            continue
        geometry = geometry_cache[key]
        scenario = build_scenario(geometry, omega, phase, name=name)
        case_dir = root / "scenarios" / name
        case_dir.mkdir(parents=True, exist_ok=True)
        window = scenario.windows[0]
        write_json(case_dir / "config.json", {
            "scenario": name, "shape": shape, "size_ratio": ratio, "omega": omega, "phase": phase,
            "start_state_pvaj": scenario.start_state.matrix, "goal_state_pvaj": scenario.goal_state.matrix,
            "body_half_extents": BODY.half_extents, "planning_rho": PLANNING_RHO,
            "window": {
                "center": window.center, "normal": window.normal, "plane_basis": window.plane_basis,
                "theta0": window.theta0, "omega": window.omega, "thickness": window.thickness,
                "physical_polygon": window.physical_polygon, "safe_polygon": window.safe_polygon,
                "physical_inradius": geometry.physical_inradius, "safe_inradius": geometry.safe_inradius,
                "axis_centered_incircle_radius": geometry.axis_centered_incircle_radius,
                "axis_centered_radius_less_than_body_radius": geometry.axis_centered_incircle_radius < BODY.circumscribed_radius,
                "off_axis_inradius_greater_than_planning_envelope": geometry.physical_inradius > PLANNING_RHO,
                "physical_to_envelope_diameter_ratio": geometry.physical_inradius / PLANNING_RHO,
                "safe_area": geometry.gate.safe_region.area, "sc_cyclic_shift": geometry.sc_cyclic_shift,
                "canonical_axis_before_transform": geometry.canonical_axis,
            },
            "optimization": config, "collision_weight": weights["collision_weight"],
        })
        for method in args.methods:
            if (name, method) in existing:
                continue
            print(f"START {name} {method}", flush=True)
            target = case_dir / method
            target.mkdir(parents=True, exist_ok=True)
            try:
                row, _ = run_one(
                    scenario, geometry, method, config, float(weights["collision_weight"]),
                    float(args.budget_seconds), target,
                )
            except Exception as exc:
                row = {
                    "scenario": name, "shape": shape, "size_ratio": ratio, "omega": omega,
                    "phase": phase, "method": method, "status": "solve_failure",
                    "trajectory_pass": False, "optimizer_success": False, "timed_out": False,
                    "failure_category": "solve_failure", "failure_reasons": f"{type(exc).__name__}: {exc}",
                    "budget_seconds": args.budget_seconds, "message": str(exc),
                    "physical_inradius": geometry.physical_inradius, "safe_inradius": geometry.safe_inradius,
                    "safe_area": geometry.gate.safe_region.area, "sc_cyclic_shift": geometry.sc_cyclic_shift,
                }
                write_json(target / "failure.json", row)
            rows.append(row)
            existing.add((name, method))
            write_summary(summary_path, rows)
            write_json(root / "status.json", {"completed_rows": len(rows), "expected_rows": FORMAL_SCENE_COUNT * len(METHODS), "last": row})
            print(f"DONE {name} {method} pass={row.get('trajectory_pass')} category={row.get('failure_category')} solve={row.get('solve_seconds')}", flush=True)
    if not args.no_plots:
        plot_success_rates(root, rows)
        plot_common_flight_times(root, rows)
        plot_representatives(root, rows)
    write_report(root, rows, weights)
    write_json(root / "results.json", rows)
    print(f"OUTPUT {root}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
