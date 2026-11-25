"""Joint SC-DynaTOGT optimization over traversal points and segment times.

For ``L`` ordered windows the unconstrained decision vector is exactly

``x = [K_0, ..., K_L, d_0x, d_0y, ..., d_(L-1)y]``.

``K`` is passed through the original TOGT positive-duration map.  Each
two-vector ``d_i`` is passed through ``B`` and the corresponding disk SC map.
The MINCO objective supplies partial derivatives with respect to waypoints
and durations; this module applies the spatial SC Jacobian, the dynamic-window
time derivative, prefix-sum adjoint, and finally the TOGT time-map adjoint.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.optimize import OptimizeResult, minimize

from .dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
    QuadrotorParameters,
    constraint_extrema,
    objective_with_gradient,
)
from .environment import SCWindowTrack
from .minco import BoundaryState, MincoSnap
from .sc_mapping import SCMappingError
from .time_mapping import (
    add_traversal_time_gradients,
    backpropagate_to_k,
    durations_from_k,
    k_from_durations,
    traversal_times,
)


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class OptimizationConfig:
    """Numerical and physical settings for the original TOGT objective."""

    initial_speed: float = 1.0
    minimum_initial_duration: float = 0.20
    # These defaults transcribe source/parameters/standard/standard_lbfgs.yaml.
    # As in the C++ solver, zero means that there is no iteration cap and the
    # past-cost test is the primary convergence test.
    max_iterations: int = 0
    max_line_search_steps: int = 64
    memory_size: int = 256
    past_iterations: int = 32
    function_tolerance: float = 1.0e-5
    gradient_tolerance: float = 0.0
    # None uses TOGT's duration-adaptive dynamicConstCheck (8..32 intervals).
    samples_per_segment: int | None = None
    include_window_time_gradient: bool = True
    objective_weights: ObjectiveWeights = field(default_factory=ObjectiveWeights)
    penalty_weights: PenaltyWeights = field(default_factory=PenaltyWeights)
    dynamic_limits: DynamicLimits = field(default_factory=DynamicLimits)
    quadrotor: QuadrotorParameters = field(default_factory=QuadrotorParameters)
    invalid_trial_cost: float = 1.0e30

    def __post_init__(self) -> None:
        if self.initial_speed <= 0.0 or self.minimum_initial_duration <= 0.0:
            raise ValueError("initial speed and duration must be positive")
        if self.max_iterations < 0 or self.max_line_search_steps < 1:
            raise ValueError("max_iterations must be nonnegative and line-search steps positive")
        if self.memory_size < 1 or self.past_iterations < 1:
            raise ValueError("memory_size and past_iterations must be positive")
        if (
            not np.isfinite(self.function_tolerance)
            or not np.isfinite(self.gradient_tolerance)
            or self.function_tolerance < 0.0
            or self.gradient_tolerance < 0.0
        ):
            raise ValueError("optimizer tolerances must be finite and nonnegative")
        if self.samples_per_segment is not None and self.samples_per_segment < 2:
            raise ValueError("samples_per_segment must be at least two")
        if not np.isfinite(self.invalid_trial_cost) or self.invalid_trial_cost <= 0.0:
            raise ValueError("invalid_trial_cost must be finite and positive")


@dataclass(frozen=True)
class ForwardPass:
    """Quantities constructed from one unconstrained optimizer vector."""

    k: FloatArray
    d: FloatArray
    durations: FloatArray
    traversal_times: FloatArray
    waypoints: FloatArray
    local_points: FloatArray
    waypoint_jacobians: tuple[FloatArray, ...]
    waypoint_time_derivatives: FloatArray
    trajectory: MincoSnap


@dataclass(frozen=True)
class ObjectiveEvaluation:
    cost: float
    gradient: FloatArray
    forward: ForwardPass
    waypoint_gradient: FloatArray
    direct_duration_gradient: FloatArray
    traversal_time_gradient: FloatArray
    accumulated_duration_gradient: FloatArray


@dataclass(frozen=True)
class OptimizationResult:
    """Serializable facts plus the solved continuous MINCO trajectory."""

    success: bool
    status: int
    message: str
    objective: float
    iterations: int
    evaluations: int
    gradient_inf_norm: float
    x: FloatArray
    k: FloatArray
    d: FloatArray
    durations: FloatArray
    traversal_times: FloatArray
    waypoints: FloatArray
    local_points: FloatArray
    trajectory: MincoSnap
    constraint_extrema: dict[str, Any]
    full_time_gradient: bool
    invalid_trial_count: int = 0

    @property
    def total_time(self) -> float:
        return float(np.sum(self.durations))

    def to_dict(self) -> dict[str, Any]:
        def serializable(value: Any) -> Any:
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
            "gradient_inf_norm": self.gradient_inf_norm,
            "x": self.x.tolist(),
            "k": self.k.tolist(),
            "d": self.d.tolist(),
            "durations": self.durations.tolist(),
            "traversal_times": self.traversal_times.tolist(),
            "waypoints": self.waypoints.tolist(),
            "local_points": self.local_points.tolist(),
            "total_time": self.total_time,
            "constraint_extrema": {
                key: serializable(value) for key, value in self.constraint_extrema.items()
            },
            "full_time_gradient": self.full_time_gradient,
            "invalid_trial_count": self.invalid_trial_count,
        }


class JointTOGTObjective:
    """Evaluate the exact joint objective and its full chain-rule gradient."""

    def __init__(self, track: SCWindowTrack, config: OptimizationConfig | None = None) -> None:
        self.track = track
        self.config = OptimizationConfig() if config is None else config
        self.window_count = len(track.order)
        self.temporal_dimension = self.window_count + 1
        self.spatial_dimension = 2 * self.window_count
        self.dimension = self.temporal_dimension + self.spatial_dimension
        self.start_state = BoundaryState(track.start)
        self.end_state = BoundaryState(track.goal)
        self.last_evaluation: ObjectiveEvaluation | None = None
        self.invalid_trial_count = 0

    def split(self, x: ArrayLike) -> tuple[FloatArray, FloatArray]:
        values = np.asarray(x, dtype=float)
        if values.shape != (self.dimension,) or not np.all(np.isfinite(values)):
            raise ValueError(f"x must be a finite vector with shape ({self.dimension},)")
        k = values[: self.temporal_dimension]
        d = values[self.temporal_dimension :].reshape(self.window_count, 2)
        return k, d

    def forward(self, x: ArrayLike) -> ForwardPass:
        k, d = self.split(x)
        durations = durations_from_k(k)
        crossing_times = traversal_times(durations, self.window_count)
        waypoints = np.empty((self.window_count, 3), dtype=float)
        local_points = np.empty((self.window_count, 2), dtype=float)
        jacobians: list[FloatArray] = []
        time_derivatives = np.empty((self.window_count, 3), dtype=float)

        for crossing_index, window_index in enumerate(self.track.order):
            window = self.track.windows[window_index]
            point, local, jacobian, time_derivative = window.point_and_jacobians(
                d[crossing_index], float(crossing_times[crossing_index])
            )
            waypoints[crossing_index] = point
            local_points[crossing_index] = local
            jacobians.append(jacobian)
            time_derivatives[crossing_index] = time_derivative

        trajectory = MincoSnap(
            self.start_state,
            self.end_state,
            waypoints,
            durations,
        )
        return ForwardPass(
            k=k.copy(),
            d=d.copy(),
            durations=durations,
            traversal_times=crossing_times,
            waypoints=waypoints,
            local_points=local_points,
            waypoint_jacobians=tuple(jacobians),
            waypoint_time_derivatives=time_derivatives,
            trajectory=trajectory,
        )

    def evaluate(self, x: ArrayLike) -> ObjectiveEvaluation:
        forward = self.forward(x)
        cost, waypoint_gradient, direct_duration_gradient = objective_with_gradient(
            forward.trajectory,
            parameters=self.config.quadrotor,
            limits=self.config.dynamic_limits,
            penalty_weights=self.config.penalty_weights,
            objective_weights=self.config.objective_weights,
            samples_per_segment=self.config.samples_per_segment,
        )

        spatial_gradient = np.empty_like(forward.d)
        traversal_gradient = np.zeros(self.window_count, dtype=float)
        for index in range(self.window_count):
            spatial_gradient[index] = forward.waypoint_jacobians[index].T @ waypoint_gradient[index]
            if self.config.include_window_time_gradient:
                traversal_gradient[index] = float(
                    waypoint_gradient[index] @ forward.waypoint_time_derivatives[index]
                )

        duration_gradient = add_traversal_time_gradients(
            direct_duration_gradient, traversal_gradient
        )
        temporal_gradient = backpropagate_to_k(forward.k, duration_gradient)
        gradient = np.concatenate((temporal_gradient, spatial_gradient.reshape(-1)))
        evaluation = ObjectiveEvaluation(
            cost=float(cost),
            gradient=gradient,
            forward=forward,
            waypoint_gradient=waypoint_gradient,
            direct_duration_gradient=direct_duration_gradient,
            traversal_time_gradient=traversal_gradient,
            accumulated_duration_gradient=duration_gradient,
        )
        self.last_evaluation = evaluation
        return evaluation

    def value_and_gradient(self, x: ArrayLike) -> tuple[float, FloatArray]:
        evaluation = self.evaluate(x)
        return evaluation.cost, evaluation.gradient

    def scipy_value_and_gradient(self, x: ArrayLike) -> tuple[float, FloatArray]:
        """Numerically safe wrapper for transient invalid L-BFGS line trials.

        The mathematical parameterization remains unconstrained.  This guard
        is used only when finite-precision SC quadrature, MINCO factorization,
        or flatness hits a singular trial; it returns a coercive value that
        directs the line search back toward its last valid region.
        """

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

    def initial_guess(self, d: ArrayLike | None = None) -> FloatArray:
        """Build the document's common ``d=0`` initialization and TOGT times."""

        if d is None:
            spatial = np.zeros((self.window_count, 2), dtype=float)
        else:
            spatial = np.asarray(d, dtype=float)
            if spatial.shape != (self.window_count, 2):
                raise ValueError(f"d must have shape ({self.window_count}, 2)")

        # Two deterministic fixed-point passes account for moving gate centers
        # while retaining the reproduction's distance/speed duration guess.
        durations = np.ones(self.temporal_dimension, dtype=float)
        for _ in range(2):
            crossing_times = traversal_times(durations, self.window_count)
            points = [self.track.start]
            for crossing_index, window_index in enumerate(self.track.order):
                points.append(
                    self.track.windows[window_index].to_point(
                        spatial[crossing_index], float(crossing_times[crossing_index])
                    )
                )
            points.append(self.track.goal)
            lengths = np.linalg.norm(np.diff(np.asarray(points), axis=0), axis=1)
            durations = np.maximum(
                lengths / self.config.initial_speed,
                self.config.minimum_initial_duration,
            )
        return np.concatenate((k_from_durations(durations), spatial.reshape(-1)))


class _PastCostStoppingCriterion:
    """Ring-buffer implementation of the bundled C++ LBFGS past test.

    SciPy's ``ftol`` compares consecutive iterates, whereas TOGT compares the
    current cost with the cost ``past`` accepted iterations ago.  Keeping the
    test here preserves the original semantics while still using SciPy's
    mature L-BFGS-B line search.
    """

    def __init__(self, past: int, tolerance: float) -> None:
        self.past = int(past)
        self.tolerance = float(tolerance)
        self._costs = np.empty(self.past, dtype=float)
        self._initialized = False
        self.iteration = 0
        self.triggered = False
        self.relative_reduction = float("inf")

    def record_initial(self, cost: float) -> None:
        value = float(cost)
        if not np.isfinite(value):
            raise ValueError("initial optimizer cost must be finite")
        self._costs[0] = value
        self._initialized = True

    def record_iteration(self, cost: float) -> bool:
        if not self._initialized:
            raise RuntimeError("record_initial must be called before optimizer iterations")
        value = float(cost)
        if not np.isfinite(value):
            return False
        self.iteration += 1
        slot = self.iteration % self.past
        if self.iteration >= self.past:
            self.relative_reduction = abs(self._costs[slot] - value) / max(1.0, abs(value))
            if self.relative_reduction < self.tolerance:
                self.triggered = True
                return True
        self._costs[slot] = value
        return False


def _minimize_togt_lbfgs(
    value_and_gradient: Callable[[ArrayLike], tuple[float, FloatArray]],
    x0: FloatArray,
    settings: OptimizationConfig,
) -> OptimizeResult:
    """Run L-BFGS-B with the original TOGT standard-LBFGS numerics.

    ``maxcor=256``, a 64-step line search, disabled gradient stopping, and the
    32-iteration relative-cost test come directly from ``standard_lbfgs.yaml``.
    SciPy has no literal unlimited-iteration sentinel, so TOGT's zero is
    represented by the largest signed 32-bit iteration/function budget; the
    past-cost convergence test normally terminates long before that budget.
    """

    start = np.asarray(x0, dtype=float)
    stopper = _PastCostStoppingCriterion(
        settings.past_iterations, settings.function_tolerance
    )
    cached_x: FloatArray | None = None
    cached_cost = float("nan")

    def evaluate(values: ArrayLike) -> tuple[float, FloatArray]:
        nonlocal cached_x, cached_cost
        array = np.asarray(values, dtype=float)
        cost, gradient = value_and_gradient(array)
        cached_x = array.copy()
        cached_cost = float(cost)
        if not stopper._initialized:
            stopper.record_initial(cached_cost)
        return cached_cost, np.asarray(gradient, dtype=float)

    def accepted_iteration(values: FloatArray) -> None:
        array = np.asarray(values, dtype=float)
        if cached_x is None or not np.array_equal(array, cached_x):
            cost, _ = evaluate(array)
        else:
            cost = cached_cost
        if stopper.record_iteration(cost):
            raise StopIteration

    unlimited = int(np.iinfo(np.int32).max)
    scipy_result: OptimizeResult = minimize(
        evaluate,
        start,
        method="L-BFGS-B",
        jac=True,
        callback=accepted_iteration,
        options={
            "maxiter": unlimited if settings.max_iterations == 0 else settings.max_iterations,
            "maxfun": unlimited,
            "maxls": settings.max_line_search_steps,
            "maxcor": settings.memory_size,
            # The original gradient tolerance is zero and its cost test uses
            # a past-iterate ring buffer, not SciPy's consecutive-step ftol.
            "ftol": 0.0,
            "gtol": settings.gradient_tolerance,
        },
    )
    if stopper.triggered:
        # SciPy faithfully returns the accepted iterate when StopIteration is
        # raised, but labels any callback stop as status 99.  This callback is
        # the original solver's positive LBFGS_STOP convergence condition.
        scipy_result.success = True
        scipy_result.status = 0
        scipy_result.message = (
            "CONVERGENCE: TOGT PAST-"
            f"{settings.past_iterations} RELATIVE COST REDUCTION "
            f"{stopper.relative_reduction:.3e} < {settings.function_tolerance:.3e}"
        )
    scipy_result["togt_past_stop"] = stopper.triggered
    scipy_result["togt_past_relative_reduction"] = stopper.relative_reduction
    return scipy_result


def optimize_track(
    track: SCWindowTrack,
    *,
    config: OptimizationConfig | None = None,
    initial_x: ArrayLike | None = None,
) -> OptimizationResult:
    """Run SciPy L-BFGS-B and return a fully reconstructed trajectory."""

    objective = JointTOGTObjective(track, config)
    x0 = objective.initial_guess() if initial_x is None else np.asarray(initial_x, dtype=float)
    objective.split(x0)  # validate before entering SciPy
    settings = objective.config
    scipy_result = _minimize_togt_lbfgs(
        objective.scipy_value_and_gradient, x0, settings
    )
    final = objective.evaluate(scipy_result.x)
    extrema = constraint_extrema(
        final.forward.trajectory,
        parameters=settings.quadrotor,
        samples_per_segment=max(
            33,
            2 * (16 if settings.samples_per_segment is None else settings.samples_per_segment) + 1,
        ),
    )
    return OptimizationResult(
        success=bool(scipy_result.success),
        status=int(scipy_result.status),
        message=str(scipy_result.message),
        objective=final.cost,
        iterations=int(scipy_result.nit),
        evaluations=int(scipy_result.nfev),
        gradient_inf_norm=float(np.linalg.norm(final.gradient, ord=np.inf)),
        x=np.asarray(scipy_result.x, dtype=float),
        k=final.forward.k,
        d=final.forward.d,
        durations=final.forward.durations,
        traversal_times=final.forward.traversal_times,
        waypoints=final.forward.waypoints,
        local_points=final.forward.local_points,
        trajectory=final.forward.trajectory,
        constraint_extrema=extrema,
        full_time_gradient=settings.include_window_time_gradient,
        invalid_trial_count=objective.invalid_trial_count,
    )


__all__ = [
    "ForwardPass",
    "JointTOGTObjective",
    "ObjectiveEvaluation",
    "OptimizationConfig",
    "OptimizationResult",
    "optimize_track",
]
