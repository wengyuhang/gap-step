"""Joint L-BFGS optimization of SC points, free times and Sync times."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import time
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    PenaltyWeights,
    QuadrotorParameters,
    constraint_extrema,
    integrated_dynamic_penalty,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    OptimizationConfig as BaseLBFGSConfig,
    _minimize_togt_lbfgs,
)
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    durations_from_k,
    k_from_durations,
)

from .scenarios import RotSyncScenario
from .trajectory import (
    CompositeTrajectory,
    RotationSyncSegment,
)


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class RotSyncOptimizationConfig:
    """Small, explicit objective and numerical settings for this method."""

    initial_speed: float = 2.5
    initial_sync_duration: float = 0.9
    minimum_initial_free_duration: float = 0.55
    smoothness_weight: float = 2.0e-4
    dynamics_weight: float = 2.0e-3
    finite_difference_step: float = 2.0e-5
    samples_per_segment: int = 7
    audit_samples_per_segment: int = 81
    audit_max_step: float | None = None
    max_iterations: int = 40
    quadrotor: QuadrotorParameters = field(default_factory=QuadrotorParameters)
    dynamic_limits: DynamicLimits = field(
        default_factory=lambda: DynamicLimits(
            max_velocity=7.0,
            max_body_rate_xy=10.0,
            max_body_rate_z=10.0,
            min_rotor_thrust=0.25,
            max_rotor_thrust=5.0,
        )
    )
    penalty_weights: PenaltyWeights = field(
        default_factory=lambda: PenaltyWeights(
            velocity=1.0,
            collective_thrust=0.0,
            body_rate=1.0,
            rotor_thrust=1.0,
        )
    )

    def __post_init__(self) -> None:
        positive = (
            self.initial_speed,
            self.initial_sync_duration,
            self.minimum_initial_free_duration,
            self.finite_difference_step,
        )
        if not np.all(np.isfinite(positive)) or min(positive) <= 0.0:
            raise ValueError("initialization and finite-difference settings must be positive")
        if self.smoothness_weight < 0.0 or self.dynamics_weight < 0.0:
            raise ValueError("objective weights must be nonnegative")
        if self.samples_per_segment < 3 or self.audit_samples_per_segment < 3:
            raise ValueError("objective and audit samples_per_segment must be at least 3")
        if self.audit_max_step is not None and (
            not np.isfinite(self.audit_max_step) or self.audit_max_step <= 0.0
        ):
            raise ValueError("audit_max_step must be finite and positive when provided")
        if self.max_iterations < 1:
            raise ValueError("max_iterations must be positive")

    def lbfgs_config(self) -> BaseLBFGSConfig:
        return BaseLBFGSConfig(
            max_iterations=self.max_iterations,
            max_line_search_steps=32,
            memory_size=64,
            past_iterations=min(8, max(3, self.max_iterations // 2)),
            function_tolerance=1.0e-6,
            gradient_tolerance=1.0e-7,
            samples_per_segment=self.samples_per_segment,
        )


@dataclass(frozen=True)
class RotSyncForwardPass:
    free_durations: FloatArray
    sync_durations: FloatArray
    latent_points: FloatArray
    local_points: FloatArray
    entry_times: FloatArray
    crossing_times: FloatArray
    exit_times: FloatArray
    trajectory: CompositeTrajectory


@dataclass(frozen=True)
class ObjectiveBreakdown:
    total_time: float
    smoothness: float
    dynamic_penalty: float

    def weighted_total(self, config: RotSyncOptimizationConfig) -> float:
        return float(
            self.total_time
            + config.smoothness_weight * self.smoothness
            + config.dynamics_weight * self.dynamic_penalty
        )


@dataclass(frozen=True)
class RotSyncOptimizationResult:
    scenario_name: str
    success: bool
    status: int
    message: str
    objective: float
    iterations: int
    evaluations: int
    cost_evaluations: int
    solve_time: float
    x: FloatArray
    forward: Any
    breakdown: ObjectiveBreakdown
    extrema: dict[str, Any]
    max_acceleration: float
    max_c3_jump: float
    invalid_trial_count: int
    audit_samples_per_segment: int
    audit_max_step: float

    @property
    def total_time(self) -> float:
        return self.breakdown.total_time

    def to_dict(self) -> dict[str, Any]:
        def clean(value: Any) -> Any:
            if isinstance(value, np.ndarray):
                return value.tolist()
            if isinstance(value, np.generic):
                return value.item()
            if isinstance(value, dict):
                return {key: clean(item) for key, item in value.items()}
            return value

        values = {
            "scenario": self.scenario_name,
            "success": self.success,
            "status": self.status,
            "message": self.message,
            "objective": self.objective,
            "iterations": self.iterations,
            "optimizer_evaluations": self.evaluations,
            "cost_evaluations": self.cost_evaluations,
            "solve_time": self.solve_time,
            "decision_vector_x": self.x,
            "total_time": self.total_time,
            "free_durations": self.forward.free_durations,
            "sync_durations": self.forward.sync_durations,
            "entry_times": self.forward.entry_times,
            "crossing_times": self.forward.crossing_times,
            "exit_times": self.forward.exit_times,
            "latent_points": self.forward.latent_points,
            "selected_q": self.forward.local_points,
            "objective_breakdown": asdict(self.breakdown),
            "max_velocity": self.extrema["max_velocity"],
            "max_acceleration": self.max_acceleration,
            "constraint_extrema": self.extrema,
            "max_c3_jump": self.max_c3_jump,
            "invalid_trial_count": self.invalid_trial_count,
            "dynamic_audit_samples_per_segment": self.audit_samples_per_segment,
            "dynamic_audit_max_step": self.audit_max_step,
        }
        if hasattr(self.forward, "latent_entry_points"):
            values.update(
                {
                    "entry_latent_points": self.forward.latent_entry_points,
                    "exit_latent_points": self.forward.latent_exit_points,
                    "entry_local_points": self.forward.local_entry_points,
                    "exit_local_points": self.forward.local_exit_points,
                }
            )
        return clean(values)


class RotSyncObjective:
    """Decision vector ``[K_free, K_sync, d]`` with accumulated absolute phases."""

    def __init__(self, scenario: RotSyncScenario, config: RotSyncOptimizationConfig | None = None) -> None:
        self.scenario = scenario
        self.config = RotSyncOptimizationConfig() if config is None else config
        self.window_count = len(scenario.windows)
        self.dimension = (self.window_count + 1) + self.window_count + 2 * self.window_count
        self.cost_evaluations = 0
        self.invalid_trial_count = 0

    def split(self, x: ArrayLike) -> tuple[FloatArray, FloatArray, FloatArray]:
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError(f"x must be finite with shape ({self.dimension},)")
        n = self.window_count
        free_k = values[: n + 1]
        sync_k = values[n + 1 : 2 * n + 1]
        latent = values[2 * n + 1 :].reshape(n, 2)
        return free_k, sync_k, latent

    def initial_guess(self) -> FloatArray:
        anchors = [self.scenario.start_state.position]
        anchors.extend(window.center for window in self.scenario.windows)
        anchors.append(self.scenario.goal_state.position)
        lengths = np.linalg.norm(np.diff(np.asarray(anchors), axis=0), axis=1)
        free = np.maximum(
            lengths / self.config.initial_speed,
            self.config.minimum_initial_free_duration,
        )
        sync = np.full(self.window_count, self.config.initial_sync_duration)
        latent = np.zeros((self.window_count, 2))
        return np.concatenate((k_from_durations(free), k_from_durations(sync), latent.reshape(-1)))

    def forward(self, x: ArrayLike) -> RotSyncForwardPass:
        free_k, sync_k, latent = self.split(x)
        free_durations = durations_from_k(free_k)
        sync_durations = durations_from_k(sync_k)
        local_points = np.stack(
            [window.local_point(latent[index]) for index, window in enumerate(self.scenario.windows)]
        )
        free_segments: list[MincoSnap] = []
        sync_segments: list[RotationSyncSegment] = []
        entries, crossings, exits = [], [], []
        elapsed = 0.0
        current_state = self.scenario.start_state
        empty = np.empty((0, 3), dtype=float)
        for index, window in enumerate(self.scenario.windows):
            entry_time = elapsed + float(free_durations[index])
            sync = RotationSyncSegment(
                window,
                local_points[index],
                entry_time,
                float(sync_durations[index]),
            )
            free_segments.append(
                MincoSnap(current_state, sync.entry_state, empty, np.asarray((free_durations[index],)))
            )
            sync_segments.append(sync)
            entries.append(entry_time)
            crossings.append(entry_time + 0.5 * float(sync_durations[index]))
            elapsed = entry_time + float(sync_durations[index])
            exits.append(elapsed)
            current_state = sync.exit_state
        free_segments.append(
            MincoSnap(current_state, self.scenario.goal_state, empty, np.asarray((free_durations[-1],)))
        )
        return RotSyncForwardPass(
            free_durations=free_durations,
            sync_durations=sync_durations,
            latent_points=latent.copy(),
            local_points=local_points,
            entry_times=np.asarray(entries),
            crossing_times=np.asarray(crossings),
            exit_times=np.asarray(exits),
            trajectory=CompositeTrajectory(free_segments, sync_segments),
        )

    def breakdown(self, forward: RotSyncForwardPass) -> ObjectiveBreakdown:
        trajectory = forward.trajectory
        dynamic = integrated_dynamic_penalty(
            trajectory,
            parameters=self.config.quadrotor,
            limits=self.config.dynamic_limits,
            weights=self.config.penalty_weights,
            samples_per_segment=self.config.samples_per_segment,
        )
        return ObjectiveBreakdown(
            total_time=trajectory.total_time,
            smoothness=float(trajectory.snap_energy()),
            dynamic_penalty=float(np.real(dynamic)),
        )

    def value(self, x: ArrayLike) -> float:
        self.cost_evaluations += 1
        forward = self.forward(x)
        value = self.breakdown(forward).weighted_total(self.config)
        if not np.isfinite(value):
            raise FloatingPointError("objective became non-finite")
        return value

    def _safe_value(self, x: FloatArray) -> float:
        try:
            return self.value(x)
        except (ValueError, FloatingPointError, OverflowError, np.linalg.LinAlgError):
            self.invalid_trial_count += 1
            clipped = np.clip(x, -1.0e5, 1.0e5)
            return float(1.0e24 * (1.0 + 1.0e-12 * (clipped @ clipped)))

    def value_and_gradient(self, x: ArrayLike) -> tuple[float, FloatArray]:
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


def optimize_track(
    scenario: RotSyncScenario,
    *,
    config: RotSyncOptimizationConfig | None = None,
    initial_x: ArrayLike | None = None,
) -> RotSyncOptimizationResult:
    """Run the reused TOGT L-BFGS driver and reconstruct the exact composite path."""

    objective = RotSyncObjective(scenario, config)
    return _optimize_objective(objective, scenario, initial_x)


def _optimize_objective(
    objective: RotSyncObjective,
    scenario: RotSyncScenario,
    initial_x: ArrayLike | None,
) -> RotSyncOptimizationResult:
    x0 = objective.initial_guess() if initial_x is None else np.asarray(initial_x, dtype=float)
    objective.split(x0)
    started = time.perf_counter()
    scipy_result = _minimize_togt_lbfgs(
        objective.value_and_gradient,
        x0,
        objective.config.lbfgs_config(),
    )
    solve_time = time.perf_counter() - started
    forward = objective.forward(scipy_result.x)
    breakdown = objective.breakdown(forward)
    audit_samples = objective.config.audit_samples_per_segment
    if objective.config.audit_max_step is not None:
        audit_samples = max(
            audit_samples,
            int(np.ceil(np.max(forward.trajectory.durations) / objective.config.audit_max_step)) + 1,
        )
    extrema = constraint_extrema(
        forward.trajectory,
        parameters=objective.config.quadrotor,
        samples_per_segment=audit_samples,
    )
    samples = forward.trajectory.sample(samples_per_segment=audit_samples)
    max_acceleration = float(np.max(np.linalg.norm(samples.acceleration, axis=1)))
    residuals = forward.trajectory.interface_residuals()
    max_jump = float(np.max(residuals)) if residuals.size else 0.0
    return RotSyncOptimizationResult(
        scenario_name=scenario.name,
        success=bool(scipy_result.success),
        status=int(scipy_result.status),
        message=str(scipy_result.message),
        objective=breakdown.weighted_total(objective.config),
        iterations=int(scipy_result.nit),
        evaluations=int(scipy_result.nfev),
        cost_evaluations=objective.cost_evaluations,
        solve_time=solve_time,
        x=np.asarray(scipy_result.x, dtype=float),
        forward=forward,
        breakdown=breakdown,
        extrema=extrema,
        max_acceleration=max_acceleration,
        max_c3_jump=max_jump,
        invalid_trial_count=objective.invalid_trial_count,
        audit_samples_per_segment=audit_samples,
        audit_max_step=float(np.max(forward.trajectory.durations) / (audit_samples - 1)),
    )


__all__ = [
    "ObjectiveBreakdown",
    "RotSyncForwardPass",
    "RotSyncObjective",
    "RotSyncOptimizationConfig",
    "RotSyncOptimizationResult",
    "optimize_track",
]
