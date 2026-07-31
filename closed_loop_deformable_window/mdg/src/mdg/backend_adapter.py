"""Adapter from selected moving discs to the existing TOGT/MINCO backend."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.optimize import OptimizeResult

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
    constant_yaw_profile,
    constraint_extrema,
    objective_with_gradient,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    OptimizationConfig,
    _minimize_togt_lbfgs,
)
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    add_traversal_time_gradients,
    backpropagate_to_k,
    durations_from_k,
    k_from_durations,
    traversal_times,
)

from .config import MDGConfig
from .dynamic_gate import DynamicGate, Scenario
from .models import DiscTrack, GraphSolution


def free_point_map(xi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Map R² smoothly into the open unit disc and return its Jacobian."""

    value = np.asarray(xi, dtype=float)
    if value.shape != (2,):
        raise ValueError("xi must have shape (2,)")
    radius = float(np.linalg.norm(value))
    if radius < 1.0e-6:
        radius2 = radius * radius
        scale = 1.0 - radius2 / 3.0 + 2.0 * radius2 * radius2 / 15.0
        coefficient = -2.0 / 3.0 + 8.0 * radius2 / 15.0
    elif radius < 20.0:
        tanh = np.tanh(radius)
        scale = tanh / radius
        derivative = (radius / np.cosh(radius) ** 2 - tanh) / (radius * radius)
        coefficient = derivative / radius
    else:
        # tanh(r)->1 and sech²(r)->0; this form avoids overflowing cosh(r)
        # during coercive L-BFGS line-search trials.
        scale = 1.0 / radius
        coefficient = -1.0 / (radius**3)
    mapped = scale * value
    jacobian = scale * np.eye(2) + coefficient * np.outer(value, value)
    return mapped, jacobian


@dataclass
class SelectedDiscConstraint:
    gate: DynamicGate
    track: DiscTrack
    interval: tuple[float, float]
    selected_time: float
    free_point: bool

    def evaluate(
        self, xi: np.ndarray, time: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
        clamped = float(np.clip(time, self.interval[0], self.interval[1]))
        center_local, radius, center_dot, radius_dot = self.track.evaluate(clamped)
        if self.free_point:
            mapped, jacobian = free_point_map(xi)
        else:
            mapped, jacobian = np.zeros(2), np.zeros((2, 2))
        local = center_local + 0.95 * radius * mapped
        local_time_derivative = center_dot + 0.95 * radius_dot * mapped
        center, rotation, center_velocity, rotation_dot = self.gate.pose_with_derivative(
            clamped
        )
        basis = rotation[:, :2]
        basis_dot = rotation_dot[:, :2]
        world = center + basis @ local
        jac_xi = basis @ (0.95 * radius * jacobian)
        if time < self.interval[0] or time > self.interval[1]:
            time_derivative = np.zeros(3)
        else:
            time_derivative = (
                center_velocity + basis_dot @ local + basis @ local_time_derivative
            )
        rho = float(np.linalg.norm(local - center_local) / max(radius, 1.0e-12))
        return world, local, jac_xi, time_derivative, rho


@dataclass
class BackendResult:
    success: bool
    status: int
    message: str
    objective: float
    iterations: int
    evaluations: int
    x: np.ndarray
    k: np.ndarray
    xi: np.ndarray
    durations: np.ndarray
    traversal_times: np.ndarray
    waypoints: np.ndarray
    local_points: np.ndarray
    selected_point_offset_ratio: np.ndarray
    trajectory: MincoSnap
    constraint_extrema: dict[str, Any]
    interval_violation: float
    invalid_trial_count: int

    @property
    def total_time(self) -> float:
        return float(np.sum(self.durations))

    def to_dict(self) -> dict[str, Any]:
        def convert(value):
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, np.generic):
                return value.item()
            return value

        return {
            "success": self.success,
            "status": self.status,
            "message": self.message,
            "objective": self.objective,
            "iterations": self.iterations,
            "evaluations": self.evaluations,
            "x": self.x.tolist(),
            "k": self.k.tolist(),
            "xi": self.xi.tolist(),
            "durations": self.durations.tolist(),
            "traversal_times": self.traversal_times.tolist(),
            "waypoints": self.waypoints.tolist(),
            "local_points": self.local_points.tolist(),
            "selected_point_offset_ratio": self.selected_point_offset_ratio.tolist(),
            "total_time": self.total_time,
            "constraint_extrema": {
                key: convert(value) for key, value in self.constraint_extrema.items()
            },
            "interval_violation": self.interval_violation,
            "invalid_trial_count": self.invalid_trial_count,
        }


class MDGObjective:
    def __init__(
        self,
        scenario: Scenario,
        constraints: list[SelectedDiscConstraint],
        config: MDGConfig,
        *,
        free_points: bool,
    ) -> None:
        self.scenario = scenario
        self.constraints = constraints
        self.config = config
        self.free_points = free_points
        self.window_count = len(constraints)
        self.temporal_dimension = self.window_count + 1
        self.spatial_dimension = 2 * self.window_count if free_points else 0
        self.dimension = self.temporal_dimension + self.spatial_dimension
        endpoint = scenario.start
        self.start_state = BoundaryState(
            endpoint.position,
            endpoint.velocity,
            endpoint.acceleration,
            endpoint.jerk,
        )
        self.end_state = self.start_state
        self.yaw_profile = constant_yaw_profile(endpoint.yaw)
        self.optimization = OptimizationConfig(
            initial_speed=config.backend.initial_speed,
            max_iterations=config.backend.max_iterations,
            samples_per_segment=config.backend.samples_per_segment,
            include_window_time_gradient=True,
            objective_weights=ObjectiveWeights(
                time=config.backend.time_weight,
                snap_energy=config.backend.snap_weight,
            ),
            penalty_weights=PenaltyWeights(
                velocity=0.0,
                collective_thrust=0.0,
                body_rate=100.0,
                rotor_thrust=100.0,
            ),
            dynamic_limits=DynamicLimits(),
        )
        self.invalid_trials = 0

    def split(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError("invalid MDG optimizer vector")
        k = values[: self.temporal_dimension]
        if self.free_points:
            xi = values[self.temporal_dimension :].reshape(self.window_count, 2)
        else:
            xi = np.zeros((self.window_count, 2))
        return k, xi

    def evaluate(self, x: np.ndarray) -> tuple[float, np.ndarray, dict[str, Any]]:
        k, xi = self.split(x)
        durations = durations_from_k(k)
        times = traversal_times(durations, self.window_count)
        waypoints = np.empty((self.window_count, 3))
        local_points = np.empty((self.window_count, 2))
        jacobians: list[np.ndarray] = []
        time_derivatives = np.empty((self.window_count, 3))
        rho = np.empty(self.window_count)
        interval_gradient = np.zeros(self.window_count)
        interval_cost = 0.0
        maximum_interval_violation = 0.0
        for index, (constraint, time) in enumerate(zip(self.constraints, times)):
            (
                waypoints[index],
                local_points[index],
                jacobian,
                time_derivatives[index],
                rho[index],
            ) = constraint.evaluate(xi[index], float(time))
            jacobians.append(jacobian)
            low, high = constraint.interval
            if time < low:
                residual = float(time - low)
            elif time > high:
                residual = float(time - high)
            else:
                residual = 0.0
            interval_cost += self.config.backend.interval_penalty * residual * residual
            interval_gradient[index] = (
                2.0 * self.config.backend.interval_penalty * residual
            )
            maximum_interval_violation = max(
                maximum_interval_violation, abs(residual)
            )
        trajectory = MincoSnap(
            self.start_state, self.end_state, waypoints, durations
        )
        cost, waypoint_gradient, direct_duration_gradient = objective_with_gradient(
            trajectory,
            parameters=self.optimization.quadrotor,
            limits=self.optimization.dynamic_limits,
            penalty_weights=self.optimization.penalty_weights,
            objective_weights=self.optimization.objective_weights,
            samples_per_segment=self.optimization.samples_per_segment,
            yaw_profile=self.yaw_profile,
        )
        traversal_gradient = interval_gradient.copy()
        spatial_gradient = np.zeros_like(xi)
        for index in range(self.window_count):
            if self.free_points:
                spatial_gradient[index] = jacobians[index].T @ waypoint_gradient[index]
            traversal_gradient[index] += float(
                waypoint_gradient[index] @ time_derivatives[index]
            )
        duration_gradient = add_traversal_time_gradients(
            direct_duration_gradient, traversal_gradient
        )
        temporal_gradient = backpropagate_to_k(k, duration_gradient)
        gradient = temporal_gradient
        if self.free_points:
            gradient = np.concatenate((temporal_gradient, spatial_gradient.ravel()))
        return float(cost + interval_cost), gradient, {
            "k": k,
            "xi": xi,
            "durations": durations,
            "traversal_times": times,
            "waypoints": waypoints,
            "local_points": local_points,
            "rho": rho,
            "trajectory": trajectory,
            "interval_violation": maximum_interval_violation,
        }

    def scipy_value_and_gradient(self, x: np.ndarray) -> tuple[float, np.ndarray]:
        values = np.asarray(x, dtype=float)
        try:
            cost, gradient, _ = self.evaluate(values)
            return cost, gradient
        except (ValueError, FloatingPointError, OverflowError, np.linalg.LinAlgError):
            self.invalid_trials += 1
            clipped = np.clip(values, -1.0e4, 1.0e4)
            return 1.0e30 + float(clipped @ clipped), 2.0 * clipped


def selected_constraints(
    scenario: Scenario,
    tracks: dict[int, list[DiscTrack]],
    graph: GraphSolution,
    *,
    free_points: bool,
) -> list[SelectedDiscConstraint]:
    output: list[SelectedDiscConstraint] = []
    for node in graph.selected_nodes:
        if node.kind != "gate":
            continue
        candidates = {
            item.track_id: item for item in tracks[node.gate_index]
        }
        track = candidates[node.track_id]
        interval = track.interval_containing(node.time)
        if interval is None:
            raise ValueError("selected graph node is outside its disc track")
        output.append(
            SelectedDiscConstraint(
                scenario.gates[node.gate_index],
                track,
                interval,
                node.time,
                free_points,
            )
        )
    return output


def optimize_selected_path(
    scenario: Scenario,
    tracks: dict[int, list[DiscTrack]],
    graph: GraphSolution,
    config: MDGConfig,
    *,
    free_points: bool,
) -> BackendResult:
    constraints = selected_constraints(
        scenario, tracks, graph, free_points=free_points
    )
    objective = MDGObjective(
        scenario, constraints, config, free_points=free_points
    )
    selected_times = np.asarray(
        [node.time for node in graph.selected_nodes if node.kind != "start"]
    )
    durations = np.diff(np.concatenate(([0.0], selected_times)))
    durations = np.maximum(durations, 1.0e-3)
    x0 = k_from_durations(durations)
    if free_points:
        x0 = np.concatenate((x0, np.zeros(2 * len(constraints))))
    scipy_result: OptimizeResult = _minimize_togt_lbfgs(
        objective.scipy_value_and_gradient, x0, objective.optimization
    )
    cost, _, facts = objective.evaluate(np.asarray(scipy_result.x))
    extrema = constraint_extrema(
        facts["trajectory"],
        parameters=objective.optimization.quadrotor,
        samples_per_segment=config.backend.validation_samples_per_segment,
        yaw_profile=objective.yaw_profile,
    )
    return BackendResult(
        success=bool(np.isfinite(cost))
        and np.all(np.isfinite(scipy_result.x))
        and facts["interval_violation"]
        <= config.validation.interval_time_tolerance,
        status=int(scipy_result.status),
        message=str(scipy_result.message),
        objective=cost,
        iterations=int(scipy_result.nit),
        evaluations=int(scipy_result.nfev),
        x=np.asarray(scipy_result.x),
        k=facts["k"],
        xi=facts["xi"],
        durations=facts["durations"],
        traversal_times=facts["traversal_times"],
        waypoints=facts["waypoints"],
        local_points=facts["local_points"],
        selected_point_offset_ratio=facts["rho"],
        trajectory=facts["trajectory"],
        constraint_extrema=extrema,
        interval_violation=facts["interval_violation"],
        invalid_trial_count=objective.invalid_trials,
    )


__all__ = [
    "BackendResult",
    "MDGObjective",
    "SelectedDiscConstraint",
    "free_point_map",
    "optimize_selected_path",
    "selected_constraints",
]
