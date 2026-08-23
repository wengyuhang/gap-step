"""SC-DynaTOGT with one jointly planned whole-body crossing constraint."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

import numpy as np
from scipy.optimize import NonlinearConstraint, minimize

from nonconvex_timevarying_window.sc_dynatogt.dynamics import constraint_extrema, flatness_from_trajectory
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    JointTOGTObjective,
    _minimize_togt_lbfgs,
)
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCMappingError
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    add_traversal_time_gradients,
    backpropagate_to_k,
    durations_from_k,
)
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import traversal_times

from .collider import CuboidCollider
from .config import WBSCOptimizationConfig
from .objective import JointGradientResult, joint_objective_with_gradients, polygon_boundary_clearance
from .validation import AttitudeSafetyReport, validate_whole_body
from .yaw import YawTrajectory, yaw_from_unconstrained, yaw_to_unconstrained


@dataclass(frozen=True)
class WBSCForwardPass:
    """Decoded decision variables and fitted trajectories."""

    k: np.ndarray
    d: np.ndarray
    y: np.ndarray
    yaw_waypoints: np.ndarray
    durations: np.ndarray
    traversal_times: np.ndarray
    waypoints: np.ndarray
    local_points: np.ndarray
    point_jacobians: np.ndarray
    point_time_derivatives: np.ndarray
    trajectory: MincoSnap
    yaw_trajectory: YawTrajectory


@dataclass(frozen=True)
class WBSCObjectiveEvaluation:
    cost: float
    gradient: np.ndarray
    forward: WBSCForwardPass
    duration_gradient: np.ndarray
    point_gradient: np.ndarray
    yaw_gradient: np.ndarray
    cost_terms: dict[str, float]
    projected_min_clearance: float


@dataclass
class WBSCOptimizationResult:
    """Optimization output, including sampled whole-body safety evidence."""

    success: bool
    optimizer_success: bool
    status: int
    message: str
    objective: float
    iterations: int
    evaluations: int
    x: np.ndarray
    k: np.ndarray
    d: np.ndarray
    y: np.ndarray
    yaw_waypoints: np.ndarray
    durations: np.ndarray
    traversal_times: np.ndarray
    waypoints: np.ndarray
    local_points: np.ndarray
    trajectory: MincoSnap
    yaw_trajectory: YawTrajectory
    extrema: dict[str, float]
    config: WBSCOptimizationConfig
    collider: CuboidCollider
    body_scale: float = 1.0
    projected_min_clearance: float = float("nan")
    hard_constraint_minimum: float = float("nan")
    solver: str = "SLSQP"
    safety_report: AttitudeSafetyReport | None = None

    @property
    def total_time(self) -> float:
        return float(np.sum(self.durations))

    def to_dict(self) -> dict[str, Any]:
        def serializable(value: Any) -> Any:
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, np.generic):
                return value.item()
            if isinstance(value, dict):
                return {str(key): serializable(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [serializable(item) for item in value]
            return value

        report = None if self.safety_report is None else self.safety_report.to_dict()
        attitude_samples = []
        for time_value in np.linspace(0.0, self.total_time, 65):
            yaw_value = float(self.yaw_trajectory.evaluate(time_value))
            state = flatness_from_trajectory(
                self.trajectory,
                float(time_value),
                yaw=yaw_value,
                yaw_rate=float(self.yaw_trajectory.evaluate(time_value, 1)),
                yaw_acceleration=float(self.yaw_trajectory.evaluate(time_value, 2)),
                parameters=self.config.quadrotor,
            )
            rotation = np.asarray(np.real(state.rotation), dtype=float)
            pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
            roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
            actual_yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
            attitude_samples.append(
                {"time": float(time_value), "roll": roll, "pitch": pitch, "yaw": actual_yaw}
            )
        return {
            "success": self.success,
            "optimizer_success": self.optimizer_success,
            "status": self.status,
            "message": self.message,
            "objective": self.objective,
            "iterations": self.iterations,
            "evaluations": self.evaluations,
            "x": self.x.tolist(),
            "k": self.k.tolist(),
            "d": self.d.tolist(),
            "y": self.y.tolist(),
            "yaw_waypoints": self.yaw_waypoints.tolist(),
            "attitude_samples": attitude_samples,
            "durations": self.durations.tolist(),
            "traversal_times": self.traversal_times.tolist(),
            "waypoints": self.waypoints.tolist(),
            "local_points": self.local_points.tolist(),
            "total_time": self.total_time,
            "extrema": serializable(self.extrema),
            "projected_min_clearance": self.projected_min_clearance,
            "hard_constraint_minimum": self.hard_constraint_minimum,
            "solver": self.solver,
            "body_scale": self.body_scale,
            "safety_report": report,
            "collider": self.collider.manifest(),
            "config": _config_manifest(self.config),
        }


def _config_manifest(config: WBSCOptimizationConfig) -> dict[str, Any]:
    return asdict(config)


class WBSCObjective:
    """Chain SC, TOGT and whole-body terms into default ``x=[K,D,Y]``.

    Roll and pitch are not independent variables: they are recomputed from
    the current MINCO derivatives at every evaluation.  Their collision
    effect therefore changes the nonlinear constraints seen by the solver.
    """

    def __init__(
        self,
        track: SCWindowTrack,
        config: WBSCOptimizationConfig,
        collider: CuboidCollider,
        body_scale: float = 1.0,
    ) -> None:
        self.track = track
        self.config = config
        self.collider = collider
        self.body_scale = float(body_scale)
        if not 0.0 <= self.body_scale <= 1.0:
            raise ValueError("body_scale must lie in [0, 1]")
        self._position_objective = JointTOGTObjective(track, config)
        self.gate_count = len(track.order)
        self.invalid_trial_count = 0

    @property
    def dimension(self) -> int:
        base = (self.gate_count + 1) + 2 * self.gate_count
        return base + (self.gate_count if self.config.optimize_yaw else 0)

    def initial_guess(self, yaw_waypoints: np.ndarray | None = None) -> np.ndarray:
        position_guess = self._position_objective.initial_guess()
        yaw = (
            np.zeros(self.gate_count, dtype=float)
            if yaw_waypoints is None
            else np.asarray(yaw_waypoints, dtype=float)
        )
        if yaw.shape != (self.gate_count,):
            raise ValueError("yaw_waypoints must contain one yaw per window")
        if not self.config.optimize_yaw:
            if yaw_waypoints is not None and not np.allclose(yaw, 0.0):
                raise ValueError(
                    "nonzero yaw waypoints require optimize_yaw=True"
                )
            return position_guess
        return np.concatenate((position_guess, yaw_to_unconstrained(yaw)))

    def split(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        x = np.asarray(x, dtype=float)
        if x.shape != (self.dimension,):
            raise ValueError(f"expected decision vector of shape {(self.dimension,)}, got {x.shape}")
        nk = self.gate_count + 1
        nd = 2 * self.gate_count
        y = x[nk + nd :] if self.config.optimize_yaw else np.zeros(self.gate_count)
        return x[:nk], x[nk : nk + nd].reshape(self.gate_count, 2), y

    def forward(self, x: np.ndarray) -> WBSCForwardPass:
        k, d, y = self.split(x)
        durations = durations_from_k(k)
        times = traversal_times(durations, self.gate_count)

        waypoints = []
        local_points = []
        jacobians = []
        time_derivatives = []
        for index, window_index in enumerate(self.track.order):
            window = self.track.windows[window_index]
            point, local, jacobian, time_derivative = window.point_and_jacobians(d[index], times[index])
            waypoints.append(point)
            local_points.append(local)
            jacobians.append(jacobian)
            time_derivatives.append(time_derivative)
        waypoints_array = np.asarray(waypoints, dtype=float)
        trajectory = MincoSnap(
            BoundaryState(self.track.start),
            BoundaryState(self.track.goal),
            waypoints_array,
            durations,
        )
        yaw_waypoints = yaw_from_unconstrained(y)
        yaw_trajectory = YawTrajectory(yaw_waypoints, durations)
        return WBSCForwardPass(
            k=k,
            d=d,
            y=y,
            yaw_waypoints=yaw_waypoints,
            durations=durations,
            traversal_times=times,
            waypoints=waypoints_array,
            local_points=np.asarray(local_points, dtype=float),
            point_jacobians=np.asarray(jacobians, dtype=float),
            point_time_derivatives=np.asarray(time_derivatives, dtype=float),
            trajectory=trajectory,
            yaw_trajectory=yaw_trajectory,
        )

    def evaluate(self, x: np.ndarray) -> WBSCObjectiveEvaluation:
        forward = self.forward(x)
        joint: JointGradientResult = joint_objective_with_gradients(
            trajectory=forward.trajectory,
            yaw_trajectory=forward.yaw_trajectory,
            track=self.track,
            collider=self.collider,
            body_scale=self.body_scale,
            config=self.config,
        )
        spatial_d = np.einsum("lij,li->lj", forward.point_jacobians, joint.point_gradient)
        traversal_gradient = np.einsum(
            "li,li->l", joint.point_gradient, forward.point_time_derivatives
        )
        if not self.config.include_window_time_gradient:
            traversal_gradient = np.zeros_like(traversal_gradient)
        duration_gradient = add_traversal_time_gradients(
            joint.duration_gradient,
            traversal_gradient,
        )
        k_gradient = backpropagate_to_k(
            forward.k,
            duration_gradient,
        )
        y_gradient = joint.yaw_gradient * (2.0 / (1.0 + np.square(forward.y)))
        if not self.config.optimize_yaw:
            y_gradient = np.zeros_like(y_gradient)
        pieces = [k_gradient, spatial_d.reshape(-1)]
        if self.config.optimize_yaw:
            pieces.append(y_gradient)
        gradient = np.concatenate(pieces)
        if not np.isfinite(joint.cost) or not np.all(np.isfinite(gradient)):
            raise FloatingPointError("non-finite WBSC objective or gradient")
        return WBSCObjectiveEvaluation(
            cost=float(joint.cost),
            gradient=gradient,
            forward=forward,
            duration_gradient=duration_gradient,
            point_gradient=joint.point_gradient,
            yaw_gradient=joint.yaw_gradient,
            cost_terms={"collision": float(joint.collision_cost)},
            projected_min_clearance=joint.minimum_projected_clearance,
        )

    def value_and_gradient(self, x: np.ndarray) -> tuple[float, np.ndarray]:
        evaluation = self.evaluate(x)
        return evaluation.cost, evaluation.gradient

    def collision_constraints(self, x: np.ndarray) -> np.ndarray:
        """Return sampled cuboid clearances; feasible values are nonnegative."""

        forward = self.forward(x)
        body_points = self.body_scale * self.collider.edge_points(
            self.config.hard_constraint_edge_samples
        )
        residuals: list[np.ndarray] = []
        for crossing_index, window_index in enumerate(self.track.order):
            time = float(forward.traversal_times[crossing_index])
            position = np.asarray(forward.trajectory.evaluate(time), dtype=float)
            yaw = float(forward.yaw_trajectory.evaluate(time))
            state = flatness_from_trajectory(
                forward.trajectory,
                time,
                yaw=yaw,
                yaw_rate=float(forward.yaw_trajectory.evaluate(time, 1)),
                yaw_acceleration=float(forward.yaw_trajectory.evaluate(time, 2)),
                parameters=self.config.quadrotor,
            )
            rotation = np.asarray(np.real(state.rotation), dtype=float)
            window = self.track.windows[window_index]
            center, basis, scale, *_ = window.state_at(time)
            world_points = position[None, :] + body_points @ rotation.T
            local_points = (world_points - center[None, :]) @ basis / scale
            polygon = (
                window.physical_boundary
                if window.physical_boundary is not None
                else window.safe_polygon
            )
            clearance = float(scale) * polygon_boundary_clearance(local_points, polygon)
            residuals.append(clearance - self.collider.config.clearance)
        return np.concatenate(residuals)

    def scipy_collision_constraints(self, x: np.ndarray) -> np.ndarray:
        """Keep SLSQP trial points finite when SC or flatness is singular."""

        try:
            values = self.collision_constraints(np.asarray(x, dtype=float))
            if np.all(np.isfinite(values)):
                return values
        except (SCMappingError, np.linalg.LinAlgError, FloatingPointError, OverflowError, ValueError):
            pass
        count = self.gate_count * len(
            self.collider.edge_points(self.config.hard_constraint_edge_samples)
        )
        return np.full(count, -1.0e3, dtype=float)

    def scipy_value_and_gradient(self, x: np.ndarray) -> tuple[float, np.ndarray]:
        values = np.asarray(x, dtype=float)
        try:
            return self.value_and_gradient(values)
        except (SCMappingError, np.linalg.LinAlgError, FloatingPointError, OverflowError, ValueError):
            self.invalid_trial_count += 1
            clipped = np.clip(values, -1.0e6, 1.0e6)
            scale_squared = float(clipped @ clipped)
            cost = self.config.invalid_trial_cost * (1.0 + 1.0e-12 * scale_squared)
            gradient = 2.0e-12 * self.config.invalid_trial_cost * clipped
            return cost, gradient


def _yaw_profile(yaw_trajectory: YawTrajectory):
    def profile(time: float) -> tuple[float, float, float]:
        return (
            float(yaw_trajectory.evaluate(time)),
            float(yaw_trajectory.evaluate(time, derivative=1)),
            float(yaw_trajectory.evaluate(time, derivative=2)),
        )

    return profile


def _run_optimizer(
    track: SCWindowTrack,
    config: WBSCOptimizationConfig,
    collider: CuboidCollider,
    body_scale: float,
    x0: np.ndarray,
):
    objective = WBSCObjective(track, config, collider, body_scale)
    if config.hard_collision_constraints:
        constraint = NonlinearConstraint(
            objective.scipy_collision_constraints,
            0.0,
            np.inf,
            jac="2-point",
        )
        raw = minimize(
            objective.scipy_value_and_gradient,
            np.asarray(x0, dtype=float),
            method="SLSQP",
            jac=True,
            constraints=(constraint,),
            options={
                "maxiter": (
                    config.hard_solver_iteration_limit
                    if config.max_iterations == 0
                    else config.max_iterations
                ),
                "ftol": config.function_tolerance,
                "disp": False,
            },
        )
        minimum = float(np.min(objective.scipy_collision_constraints(np.asarray(raw.x, dtype=float))))
        raw["hard_constraint_minimum"] = minimum
        if bool(raw.success) and minimum < -config.hard_constraint_tolerance:
            raw.success = False
            raw.status = 4
            raw.message = (
                f"hard cuboid constraint violated by {-minimum:.3e} m"
            )
    else:
        raw = _minimize_togt_lbfgs(objective.scipy_value_and_gradient, x0, config)
    raw["invalid_trial_count"] = objective.invalid_trial_count
    evaluation = objective.evaluate(np.asarray(raw.x, dtype=float))
    return raw, evaluation


def _build_result(
    raw: Any,
    evaluation: WBSCObjectiveEvaluation,
    config: WBSCOptimizationConfig,
    collider: CuboidCollider,
    body_scale: float,
) -> WBSCOptimizationResult:
    forward = evaluation.forward
    extrema = constraint_extrema(
        forward.trajectory,
        parameters=config.quadrotor,
        samples_per_segment=max(
            33,
            2 * (16 if config.samples_per_segment is None else config.samples_per_segment) + 1,
        ),
        yaw_profile=_yaw_profile(forward.yaw_trajectory),
    )
    return WBSCOptimizationResult(
        success=False,
        optimizer_success=bool(raw.success),
        status=int(raw.status),
        message=str(raw.message),
        objective=float(evaluation.cost),
        iterations=int(getattr(raw, "nit", 0)),
        evaluations=int(getattr(raw, "nfev", 0)),
        x=np.asarray(raw.x, dtype=float),
        k=forward.k.copy(),
        d=forward.d.copy(),
        y=forward.y.copy(),
        yaw_waypoints=forward.yaw_waypoints.copy(),
        durations=forward.durations.copy(),
        traversal_times=forward.traversal_times.copy(),
        waypoints=forward.waypoints.copy(),
        local_points=forward.local_points.copy(),
        trajectory=forward.trajectory,
        yaw_trajectory=forward.yaw_trajectory,
        extrema=dict(extrema),
        config=config,
        collider=collider,
        body_scale=body_scale,
        projected_min_clearance=float(evaluation.projected_min_clearance),
        hard_constraint_minimum=float(raw.get("hard_constraint_minimum", np.nan)),
        solver="SLSQP" if config.hard_collision_constraints else "L-BFGS-B",
    )


def optimize_track(
    track: SCWindowTrack,
    config: WBSCOptimizationConfig | None = None,
    *,
    x0: np.ndarray | None = None,
    body_scale: float = 1.0,
) -> WBSCOptimizationResult:
    """Jointly optimize time, SC points and dynamically consistent attitude.

    ``body_scale=0`` is the point-model ablation.  Validation always checks the
    complete cuboid, so that ablation remains an unsafe optimistic upper bound.
    Yaw is explicit by default.  Roll/pitch remain tied to translational
    acceleration through differential flatness, while the whole-body
    constraint is evaluated inside every optimizer iteration.
    """

    config = WBSCOptimizationConfig() if config is None else config
    collider = CuboidCollider(config.collider)
    objective = WBSCObjective(track, config, collider, body_scale)
    initial = objective.initial_guess() if x0 is None else np.asarray(x0, dtype=float)
    raw, evaluation = _run_optimizer(track, config, collider, body_scale, initial)
    result = _build_result(raw, evaluation, config, collider, body_scale)
    report = validate_whole_body(track, result, collider=collider)
    result.safety_report = report
    result.success = bool(result.optimizer_success and report.safe)
    if not report.safe:
        result.message = (
            f"{result.message}; cuboid validation failed: {report.violations[0].reason}"
        )
    return result


__all__ = [
    "WBSCForwardPass",
    "WBSCObjective",
    "WBSCObjectiveEvaluation",
    "WBSCOptimizationResult",
    "optimize_track",
]
