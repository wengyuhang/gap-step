"""Experiment-only baselines requested by groups E0 and E2.

None of these mappings is used by the SC-DynaTOGT main algorithm.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
from shapely.ops import polylabel

from .dynamics import constraint_extrema, objective_with_gradient
from .environment import _point_in_polygon, rotation_and_derivative
from .minco import BoundaryState, MincoSnap
from .optimizer import OptimizationConfig, _minimize_togt_lbfgs
from .time_mapping import backpropagate_to_k, durations_from_k, k_from_durations, traversal_times


def original_togt_convex_map(d: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """Reproduce the convex-combination map in the TOGT source code.

    The implementation follows ``Polyhedron::toP`` from the reproduction:
    normalize an ``n``-vector, square its first ``n-1`` entries, and use the
    last squared entry as the implicit weight of vertex zero.
    """

    polygon = np.asarray(vertices, dtype=float)
    values = np.asarray(d, dtype=float)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(values) != len(polygon):
        raise ValueError("an n-vertex convex polygon requires an n-vector d")
    norm = float(np.linalg.norm(values))
    if norm <= 1.0e-15:
        raise ValueError("the original TOGT convex map is undefined at d=0")
    unit = values / norm
    return polygon[0] + (polygon[1:] - polygon[0]).T @ (unit[:-1] ** 2)


def original_togt_convex_jacobian(d: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """Analytic real Jacobian of :func:`original_togt_convex_map`."""

    polygon = np.asarray(vertices, dtype=float)
    values = np.asarray(d, dtype=float)
    norm = float(np.linalg.norm(values))
    if len(values) != len(polygon) or norm <= 1.0e-15:
        raise ValueError("invalid original TOGT convex coordinates")
    unit = values / norm
    basis = polygon[1:] - polygon[0]
    grad_unit = np.zeros((len(values), 2), dtype=float)
    grad_unit[:-1] = 2.0 * unit[:-1, None] * basis
    projection = (np.eye(len(values)) - np.outer(unit, unit)) / norm
    return grad_unit.T @ projection


def fixed_safe_center(safe_polygon: np.ndarray) -> np.ndarray:
    """E2 fixed-point baseline: the pole of inaccessibility of the safe set."""

    polygon = np.asarray(safe_polygon, dtype=float)
    point = polylabel(Polygon(polygon), tolerance=1.0e-7)
    return np.array([point.x, point.y], dtype=float)


def convex_hull_polygon(vertices: np.ndarray) -> np.ndarray:
    """Return the CCW convex hull used only by the E2 ablation."""

    points = np.asarray(vertices, dtype=float)
    hull = ConvexHull(points)
    return points[hull.vertices]


class StaticBaselineWindow:
    """Original-TOGT convex mapping or E2's fixed-center ablation."""

    def __init__(self, name, polygon, center, rpy, *, kind: str) -> None:
        if kind not in {"fixed", "original_convex"}:
            raise ValueError("baseline kind must be 'fixed' or 'original_convex'")
        self.name = str(name)
        self.polygon = np.asarray(polygon, dtype=float)
        self.center = np.asarray(center, dtype=float)
        self.rpy = np.asarray(rpy, dtype=float)
        if self.polygon.ndim != 2 or self.polygon.shape[1] != 2 or len(self.polygon) < 3:
            raise ValueError("polygon must have shape (n, 2), n >= 3")
        if self.center.shape != (3,) or self.rpy.shape != (3,):
            raise ValueError("center and rpy must have shape (3,)")
        self.kind = kind
        self.rotation, _ = rotation_and_derivative(self.rpy, np.zeros(3))
        self.basis = self.rotation[:, :2]
        self.local_center = fixed_safe_center(self.polygon)
        if kind == "original_convex":
            hull = convex_hull_polygon(self.polygon)
            if len(hull) != len(self.polygon) or not np.isclose(
                abs(Polygon(hull).area), abs(Polygon(self.polygon).area), rtol=1e-10
            ):
                raise ValueError("original TOGT mapping requires a convex polygon")

    @property
    def dimension(self) -> int:
        return 0 if self.kind == "fixed" else len(self.polygon)

    def initial_parameters(self) -> np.ndarray:
        return np.empty(0, dtype=float) if self.kind == "fixed" else np.ones(self.dimension)

    def local_point_and_jacobian(self, parameters):
        values = np.asarray(parameters, dtype=float)
        if values.shape != (self.dimension,):
            raise ValueError(f"baseline parameters must have shape ({self.dimension},)")
        if self.kind == "fixed":
            return self.local_center.copy(), np.empty((2, 0), dtype=float)
        return original_togt_convex_map(values, self.polygon), original_togt_convex_jacobian(values, self.polygon)

    def point_and_jacobian(self, parameters):
        local, jacobian = self.local_point_and_jacobian(parameters)
        return self.center + self.basis @ local, self.basis @ jacobian

    def contains(self, point) -> bool:
        relative = np.asarray(point, dtype=float) - self.center
        return abs(float(relative @ self.rotation[:, 2])) <= 1e-7 and _point_in_polygon(
            self.basis.T @ relative, self.polygon
        )


class BaselineTrack:
    """Static ordered gates used only by experiments E0/E2."""

    def __init__(self, name, start, goal, windows) -> None:
        self.name = str(name)
        self.start = np.asarray(start, dtype=float)
        self.goal = np.asarray(goal, dtype=float)
        self.windows = tuple(windows)
        if self.start.shape != (3,) or self.goal.shape != (3,) or not self.windows:
            raise ValueError("baseline track requires 3D endpoints and at least one window")


class BaselineOptimizationResult:
    def __init__(self, scipy_result, trajectory, durations, crossing_times, waypoints, parameters, extrema):
        self.success = bool(scipy_result.success)
        self.status = int(scipy_result.status)
        self.message = str(scipy_result.message)
        self.objective = float(scipy_result.fun)
        self.iterations = int(scipy_result.nit)
        self.evaluations = int(scipy_result.nfev)
        self.trajectory = trajectory
        self.durations = np.asarray(durations, dtype=float)
        self.traversal_times = np.asarray(crossing_times, dtype=float)
        self.waypoints = np.asarray(waypoints, dtype=float)
        self.parameters = tuple(np.asarray(value, dtype=float) for value in parameters)
        self.constraint_extrema = extrema

    @property
    def total_time(self) -> float:
        return float(np.sum(self.durations))

    def to_dict(self) -> dict[str, object]:
        def serializable(value):
            return value.tolist() if isinstance(value, np.ndarray) else value

        return {
            "success": self.success,
            "status": self.status,
            "message": self.message,
            "objective": self.objective,
            "iterations": self.iterations,
            "evaluations": self.evaluations,
            "durations": self.durations.tolist(),
            "traversal_times": self.traversal_times.tolist(),
            "waypoints": self.waypoints.tolist(),
            "parameters": [value.tolist() for value in self.parameters],
            "total_time": self.total_time,
            "constraint_extrema": {
                key: serializable(value) for key, value in self.constraint_extrema.items()
            },
        }


class BaselineTOGTObjective:
    """Original static TOGT objective with variable per-gate dimensions."""

    def __init__(self, track: BaselineTrack, config: OptimizationConfig | None = None):
        self.track = track
        self.config = OptimizationConfig() if config is None else config
        self.temporal_dimension = len(track.windows) + 1
        self.offsets = np.cumsum([0] + [window.dimension for window in track.windows])
        self.dimension = self.temporal_dimension + int(self.offsets[-1])

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,):
            raise ValueError(f"x must have shape ({self.dimension},)")
        k = values[: self.temporal_dimension]
        flat = values[self.temporal_dimension :]
        parameters = tuple(
            flat[self.offsets[index] : self.offsets[index + 1]]
            for index in range(len(self.track.windows))
        )
        return k, parameters

    def forward(self, x):
        k, parameters = self.split(x)
        durations = durations_from_k(k)
        mapped = [window.point_and_jacobian(value) for window, value in zip(self.track.windows, parameters)]
        waypoints = np.asarray([item[0] for item in mapped])
        jacobians = [item[1] for item in mapped]
        trajectory = MincoSnap(BoundaryState(self.track.start), BoundaryState(self.track.goal), waypoints, durations)
        return k, parameters, durations, waypoints, jacobians, trajectory

    def value_and_gradient(self, x):
        k, parameters, durations, waypoints, jacobians, trajectory = self.forward(x)
        cost, gradient_points, gradient_times = objective_with_gradient(
            trajectory,
            parameters=self.config.quadrotor,
            limits=self.config.dynamic_limits,
            penalty_weights=self.config.penalty_weights,
            objective_weights=self.config.objective_weights,
            samples_per_segment=self.config.samples_per_segment,
        )
        spatial = [jacobian.T @ gradient for jacobian, gradient in zip(jacobians, gradient_points)]
        return float(cost), np.concatenate((backpropagate_to_k(k, gradient_times), *spatial))

    def initial_guess(self):
        parameters = tuple(window.initial_parameters() for window in self.track.windows)
        points = [self.track.start]
        points.extend(window.point_and_jacobian(value)[0] for window, value in zip(self.track.windows, parameters))
        points.append(self.track.goal)
        lengths = np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1)
        durations = np.maximum(lengths / self.config.initial_speed, self.config.minimum_initial_duration)
        return np.concatenate((k_from_durations(durations), *parameters))


def optimize_baseline_track(track: BaselineTrack, *, config=None, initial_x=None) -> BaselineOptimizationResult:
    objective = BaselineTOGTObjective(track, config)
    x0 = objective.initial_guess() if initial_x is None else np.asarray(initial_x, dtype=float)
    settings = objective.config
    result = _minimize_togt_lbfgs(objective.value_and_gradient, x0, settings)
    _, parameters, durations, waypoints, _, trajectory = objective.forward(result.x)
    crossing_times = traversal_times(durations, len(track.windows))
    extrema = constraint_extrema(
        trajectory, parameters=settings.quadrotor,
        samples_per_segment=max(
            33, 2 * (16 if settings.samples_per_segment is None else settings.samples_per_segment) + 1
        ),
    )
    return BaselineOptimizationResult(result, trajectory, durations, crossing_times, waypoints, parameters, extrema)


__all__ = [
    "BaselineOptimizationResult", "BaselineTOGTObjective", "BaselineTrack", "StaticBaselineWindow",
    "convex_hull_polygon", "fixed_safe_center", "optimize_baseline_track",
    "original_togt_convex_jacobian", "original_togt_convex_map",
]
