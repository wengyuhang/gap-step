"""Real MINCO/L-BFGS solves for Experiment B.

Both methods share the unchanged SC-DynaTOGT objective and the same initial
``[K,D]`` vector.  A sampled metric center-clearance term is common to both;
Ours additionally integrates the exact cuboid-section/intersection penalty.
The safety contribution is differentiated by centered differences as an
independent, auditable prototype backend while the base TOGT gradient remains
analytic.  No trajectory is prescribed by the visualization code.
"""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np
from numpy.typing import ArrayLike
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.cwb_sc_dynatogt.body_model import CuboidBody
from nonconvex_timevarying_window.cwb_sc_dynatogt.config import WholeBodySafetyConfig
from nonconvex_timevarying_window.cwb_sc_dynatogt.gate_frame import frame_at
from nonconvex_timevarying_window.cwb_sc_dynatogt.plane_section import (
    find_planned_crossing_interval,
    plane_section_at,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
    constraint_extrema,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    ForwardPass,
    JointTOGTObjective,
    OptimizationConfig,
    _minimize_togt_lbfgs,
)
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.scenarios import build_boundary_scenario
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import five_point_star_boundary

from .geometry import IntersectionMetrics, PlaneSection, exact_intersection_metrics
from .stress_case import WORLD_CLEARANCE


@dataclass(frozen=True)
class SafetyObjectiveConfig:
    samples: int = 49
    center_weight: float = 2.0e6
    exact_area_weight: float = 2.0e8
    finite_difference_step: float = 2.0e-5
    # A 20 mm numerical guard keeps the independent dense validation above
    # the required 0.315 m between quadrature nodes.
    optimization_center_clearance: float = 0.335
    # Active dense-validation witnesses stored as (MINCO segment, tau).
    center_witnesses: tuple[tuple[int, float], ...] = ()
    adaptive_center_samples: int = 41


@dataclass(frozen=True)
class SolvedMethod:
    method: str
    x0: np.ndarray
    x: np.ndarray
    forward: ForwardPass
    optimizer_success: bool
    status: int
    message: str
    iterations: int
    evaluations: int
    solve_time: float
    objective: float
    base_objective: float
    center_penalty: float
    area_penalty: float
    constraint_extrema: dict[str, object]

    @property
    def total_time(self) -> float:
        return float(np.sum(self.forward.durations))


def build_solver_problem() -> tuple[SCWindowTrack, OptimizationConfig, CuboidBody]:
    """Build the single-star, closed, full-E4-motion optimization problem."""

    preprocessing = PreprocessingConfig()
    center = np.array([[0.0, 0.0, 1.8]])
    angles = np.array([[0.0, np.pi / 2.0, 0.0]])
    endpoint = np.array([-4.6, -2.0, 3.0])
    scenario = build_boundary_scenario(
        (("star", five_point_star_boundary()),),
        mode="full",
        preprocessing_config=preprocessing,
        centers=center,
        angles=angles,
        start=endpoint,
        goal=endpoint.copy(),
        name="experiment_b_closed",
        motion_scale=1.0,
    )
    optimization = OptimizationConfig(
        max_iterations=180,
        samples_per_segment=8,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        penalty_weights=PenaltyWeights(
            velocity=0.0,
            collective_thrust=0.0,
            body_rate=1.0,
            rotor_thrust=1.0,
        ),
        dynamic_limits=DynamicLimits(
            max_velocity=60.0,
            max_body_rate_xy=10.0,
            max_body_rate_z=10.0,
            min_rotor_thrust=0.25,
            max_rotor_thrust=5.0,
        ),
    )
    # Flat square rotor-plane footprint and short height, as requested.
    body = CuboidBody(
        0.5 * np.array([0.5300801927129876, 0.5300801927129876, 0.11779559838066389])
    )
    return scenario.track, optimization, body


class ExactAreaTOGTObjective:
    """Base TOGT plus shared world-clearance and optional exact-area terms."""

    def __init__(
        self,
        base: JointTOGTObjective,
        body: CuboidBody,
        *,
        use_exact_area: bool,
        config: SafetyObjectiveConfig | None = None,
    ) -> None:
        self.base = base
        self.body = body
        self.use_exact_area = bool(use_exact_area)
        self.config = SafetyObjectiveConfig() if config is None else config
        self.section_config = WholeBodySafetyConfig(
            half_extents=tuple(float(value) for value in body.half_extents),
            time_tolerance=5.0e-4,
            lambda_tolerance=5.0e-4,
            max_interval_depth=18,
        )
        self.last_terms = (0.0, 0.0)

    def _sample_metrics(
        self, forward: ForwardPass
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        total = float(np.sum(forward.durations))
        crossing = float(forward.traversal_times[0])
        # Resolve the complete plane-contact interval around the optimized
        # traversal time instead of diluting samples over a long closed lap.
        times = np.linspace(
            max(0.0, crossing - 1.25),
            min(total, crossing + 1.25),
            self.config.samples,
        )
        starts = np.concatenate(([0.0], np.cumsum(forward.durations[:-1])))
        witness_times = [
            float(starts[segment] + tau * forward.durations[segment])
            for segment, tau in self.config.center_witnesses
            if 0 <= segment < len(forward.durations) and 0.0 <= tau <= 1.0
        ]
        if witness_times:
            times = np.unique(np.concatenate((times, witness_times)))
        center_values = np.zeros_like(times)
        area_values = np.zeros_like(times)
        window = self.base.track.windows[self.base.track.order[0]]
        physical_local = np.asarray(window.physical_boundary, dtype=float)
        for index, instant in enumerate(times):
            section = plane_section_at(
                forward.trajectory,
                float(instant),
                window,
                self.body,
                self.section_config,
                parameters=self.base.config.quadrotor,
            )
            if len(section.vertices) < 3:
                continue
            gate = frame_at(window, float(instant))
            planar_world = section.local_polygon * gate.scale
            body_polygon = Polygon(planar_world)
            if body_polygon.area <= 1.0e-12:
                continue
            gate_polygon = Polygon(physical_local * gate.scale)
            center_world = np.asarray(
                forward.trajectory.evaluate(float(instant)), dtype=float
            )
            center_plane = (center_world - gate.center) @ gate.basis
            point = Point(float(center_plane[0]), float(center_plane[1]))
            distance = float(gate_polygon.boundary.distance(point))
            signed_distance = distance if gate_polygon.covers(point) else -distance
            violation = max(
                self.config.optimization_center_clearance - signed_distance, 0.0
            )
            # This is the common metric-center hard-constraint surrogate, not
            # the paper's exact-area functional.  A quadratic stays stiff at
            # millimetre-scale violations; the area term retains its cubic
            # ratio formula in exact_intersection_metrics.
            center_values[index] = violation**2

            if self.use_exact_area:
                exact_section = PlaneSection(
                    np.stack([vertex.world for vertex in section.vertices]),
                    planar_world,
                    float(body_polygon.area),
                    section.degenerate,
                )
                metrics = exact_intersection_metrics(
                    exact_section, physical_local * gate.scale, area_epsilon=1.0e-9
                )
                area_values[index] = metrics.penalty
        return times, center_values, area_values

    def safety_value(self, values: ArrayLike) -> tuple[float, float, float]:
        forward = self.base.forward(values)
        times, center_values, area_values = self._sample_metrics(forward)
        center = float(np.trapz(center_values, times))
        # Active verifier witnesses are point constraints.  They must not be
        # diluted by a quadrature time step, otherwise a narrow violation can
        # slide between integration nodes while L-BFGS reports convergence.
        window = self.base.track.windows[self.base.track.order[0]]
        starts = np.concatenate(([0.0], np.cumsum(forward.durations[:-1])))
        for segment, tau in self.config.center_witnesses:
            if not (0 <= segment < len(forward.durations) and 0.0 <= tau <= 1.0):
                continue
            instant = float(starts[segment] + tau * forward.durations[segment])
            section = plane_section_at(
                forward.trajectory,
                instant,
                window,
                self.body,
                self.section_config,
                parameters=self.base.config.quadrotor,
            )
            if len(section.vertices) < 3:
                continue
            gate = frame_at(window, instant)
            physical = Polygon(
                np.asarray(window.physical_boundary, dtype=float) * gate.scale
            )
            position = np.asarray(forward.trajectory.evaluate(instant), dtype=float)
            local = (position - gate.center) @ gate.basis
            point = Point(float(local[0]), float(local[1]))
            distance = float(physical.boundary.distance(point))
            signed = distance if physical.covers(point) else -distance
            violation = max(
                self.config.optimization_center_clearance - signed, 0.0
            )
            center += violation**2
        # Recompute the current cuboid/plane contact interval on every trial.
        # This prevents the optimizer from moving a violation away from a
        # fixed time grid or a stale active witness.
        try:
            contact = find_planned_crossing_interval(
                window_index=self.base.track.order[0],
                traversal_time=float(forward.traversal_times[0]),
                trajectory=forward.trajectory,
                window=window,
                body=self.body,
                config=self.section_config,
                parameters=self.base.config.quadrotor,
            )
        except ValueError:
            contact = None
        if contact is not None:
            contact_penalties = []
            for instant in np.linspace(
                contact.start, contact.end, self.config.adaptive_center_samples
            ):
                gate = frame_at(window, float(instant))
                physical = Polygon(
                    np.asarray(window.physical_boundary, dtype=float) * gate.scale
                )
                position = np.asarray(
                    forward.trajectory.evaluate(float(instant)), dtype=float
                )
                local = (position - gate.center) @ gate.basis
                point = Point(float(local[0]), float(local[1]))
                distance = float(physical.boundary.distance(point))
                signed = distance if physical.covers(point) else -distance
                violation = max(
                    self.config.optimization_center_clearance - signed, 0.0
                )
                contact_penalties.append(violation**2)
            center += float(np.mean(contact_penalties))
        area = float(np.trapz(area_values, times))
        total = self.config.center_weight * center
        if self.use_exact_area:
            total += self.config.exact_area_weight * area
        return total, center, area

    def value_and_gradient(self, values: ArrayLike) -> tuple[float, np.ndarray]:
        x = np.asarray(values, dtype=float)
        base = self.base.evaluate(x)
        safety, center, area = self.safety_value(x)
        gradient = np.zeros_like(x)
        for index in range(len(x)):
            step = self.config.finite_difference_step * max(1.0, abs(x[index]))
            plus, minus = x.copy(), x.copy()
            plus[index] += step
            minus[index] -= step
            plus_value = self.safety_value(plus)[0]
            minus_value = self.safety_value(minus)[0]
            gradient[index] = (plus_value - minus_value) / (2.0 * step)
        self.last_terms = (center, area)
        return float(base.cost + safety), base.gradient + gradient


def solve_method(
    method: str,
    base: JointTOGTObjective,
    body: CuboidBody,
    x0: ArrayLike,
    *,
    safety_config: SafetyObjectiveConfig | None = None,
) -> SolvedMethod:
    """Run one complete L-BFGS solve from the supplied common initialization."""

    if method not in {"Old-0.315", "Ours"}:
        raise ValueError("method must be Old-0.315 or Ours")
    objective = ExactAreaTOGTObjective(
        base, body, use_exact_area=method == "Ours", config=safety_config
    )
    initial = np.asarray(x0, dtype=float).copy()
    started = perf_counter()
    result = _minimize_togt_lbfgs(
        objective.value_and_gradient, initial, base.config
    )
    elapsed = perf_counter() - started
    base_final = base.evaluate(result.x)
    safety, center, area = objective.safety_value(result.x)
    return SolvedMethod(
        method=method,
        x0=initial,
        x=np.asarray(result.x, dtype=float),
        forward=base_final.forward,
        optimizer_success=bool(result.success),
        status=int(result.status),
        message=str(result.message),
        iterations=int(result.nit),
        evaluations=int(result.nfev),
        solve_time=float(elapsed),
        objective=float(base_final.cost + safety),
        base_objective=float(base_final.cost),
        center_penalty=float(center),
        area_penalty=float(area),
        constraint_extrema=constraint_extrema(
            base_final.forward.trajectory,
            parameters=base.config.quadrotor,
            samples_per_segment=33,
        ),
    )


def common_initial_guess(base: JointTOGTObjective, seed: int) -> np.ndarray:
    """Return the same reproducible E4-style perturbed initialization."""

    rng = np.random.default_rng(int(seed))
    values = base.initial_guess()
    values[: base.temporal_dimension] += rng.normal(
        0.0, 0.04, base.temporal_dimension
    )
    values[base.temporal_dimension :] += rng.normal(
        0.0, 0.12, base.spatial_dimension
    )
    return values


__all__ = [
    "ExactAreaTOGTObjective",
    "SafetyObjectiveConfig",
    "SolvedMethod",
    "build_solver_problem",
    "common_initial_guess",
    "solve_method",
]
