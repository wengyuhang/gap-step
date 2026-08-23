"""Active-witness safety penalties and warm-started outer optimization."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Sequence

import numpy as np
from numpy.typing import ArrayLike
from scipy.optimize import OptimizeResult
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    ForwardPass,
    JointTOGTObjective,
    ObjectiveEvaluation,
    _minimize_togt_lbfgs,
)

from .body_model import CuboidBody
from .config import WholeBodySafetyConfig
from .gate_frame import frame_at
from .plane_section import cuboid_world_vertices, plane_section_from_vertices
from .sc_inverse import inverse_sc_map
from .whole_body_safety import (
    SafetyWitness,
    TrajectorySafetyReport,
    VerificationStatus,
    verify_whole_body_trajectory,
)


@dataclass(frozen=True)
class ActiveSafetyConstraint:
    """One verifier witness and its current penalty weight."""

    witness: SafetyWitness
    weight: float

    def __post_init__(self) -> None:
        if not np.isfinite(self.weight) or self.weight <= 0.0:
            raise ValueError("constraint weight must be finite and positive")


@dataclass(frozen=True)
class ConstrainedObjectiveEvaluation:
    """Base TOGT evaluation plus finite active-witness safety terms."""

    cost: float
    gradient: np.ndarray
    base: ObjectiveEvaluation
    safety_cost: float
    stale_constraints: int


class WholeBodyResultStatus(Enum):
    """Optimization status kept separate from the inner L-BFGS status."""

    SAFE_NUMERICAL = "safe_numerical"
    SAFE_CERTIFIED = "safe_certified"
    UNSAFE = "unsafe"
    UNCERTIFIED = "uncertified"
    NUMERICAL_FAILURE = "numerical_failure"


@dataclass(frozen=True)
class OuterIterationRecord:
    """Compact progress record for one verify/repair round."""

    iteration: int
    status: VerificationStatus
    witness_count: int
    minimum_margin: float | None
    total_time: float
    objective: float


@dataclass(frozen=True)
class WholeBodyOptimizationResult:
    """Final ``[K,D]`` solution, report, active set, and outer-loop history."""

    status: WholeBodyResultStatus
    optimizer_success: bool
    x: np.ndarray
    forward: ForwardPass
    objective: float
    safety_report: TrajectorySafetyReport
    active_constraints: tuple[ActiveSafetyConstraint, ...]
    history: tuple[OuterIterationRecord, ...]
    optimizer_message: str

    @property
    def success(self) -> bool:
        """Return true only for a verified whole-body result."""

        return self.status in {WholeBodyResultStatus.SAFE_NUMERICAL, WholeBodyResultStatus.SAFE_CERTIFIED}


class WholeBodyConstrainedObjective:
    """Add differentiable finite active-witness terms to a base ``[K,D]`` objective.

    Only the finite active set is differentiated. The adaptive verifier is
    intentionally outside the objective. V1 uses centered finite differences
    for the small safety term while retaining the base analytic TOGT gradient.
    """

    def __init__(
        self,
        base: JointTOGTObjective,
        body: CuboidBody,
        safety_config: WholeBodySafetyConfig,
        active_constraints: Sequence[ActiveSafetyConstraint],
    ) -> None:
        if not isinstance(base, JointTOGTObjective):
            raise TypeError("base must be a JointTOGTObjective")
        self.base = base
        self.body = body
        self.config = safety_config
        self.active_constraints = tuple(active_constraints)

    def _constraint_value(self, forward: ForwardPass, active: ActiveSafetyConstraint) -> tuple[float, bool]:
        witness = active.witness
        durations = np.asarray(forward.durations, dtype=float)
        if not 0 <= witness.minco_segment_index < len(durations):
            return 0.0, True
        starts = np.concatenate(([0.0], np.cumsum(durations[:-1])))
        time = float(starts[witness.minco_segment_index] + witness.normalized_time * durations[witness.minco_segment_index])
        window = self.base.track.windows[witness.window_index]
        vertices, _ = cuboid_world_vertices(
            forward.trajectory, time, self.body,
            parameters=self.base.config.quadrotor,
        )
        section = plane_section_from_vertices(
            vertices, frame_at(window, time), self.body, time=time,
            plane_epsilon=self.config.plane_epsilon,
            dedup_epsilon=self.config.dedup_epsilon,
        )
        lookup = {vertex.source_body_edge: vertex for vertex in section.vertices}
        if witness.body_edge_a not in lookup or witness.body_edge_b not in lookup:
            return 0.0, True
        first, second = lookup[witness.body_edge_a], lookup[witness.body_edge_b]
        point = (1.0 - witness.section_lambda) * first.local + witness.section_lambda * second.local
        polygon = Polygon(window.safe_polygon)
        if polygon.covers(Point(float(point[0]), float(point[1]))):
            inverse = inverse_sc_map(
                window.sc_map, point, tolerance=self.config.sc_inverse_tolerance,
                max_iterations=40,
            )
            if not inverse.converged:
                return 0.0, True
            violation = max(float(inverse.z @ inverse.z - self.config.sc_safe_radius**2), 0.0)
            return active.weight * violation * violation, False

        # A fixed auxiliary preimage pool gives outside witnesses a recovery
        # direction instead of the base objective's invalid-trial wall. It is
        # the V1 discrete counterpart of the plan's auxiliary SC preimage U.
        angles = np.linspace(0.0, 2.0 * np.pi, 257)[:-1]
        disk = self.config.sc_safe_radius * np.column_stack((np.cos(angles), np.sin(angles)))
        safe_boundary = np.asarray(window.sc_map.evaluate_many(disk), dtype=float)
        residuals = safe_boundary - point[None, :]
        squared = np.einsum("ij,ij->i", residuals, residuals)
        return active.weight * float(np.min(squared)), False

    def _safety_value(self, x: np.ndarray) -> tuple[float, int]:
        try:
            forward = self.base.forward(x)
        except Exception:
            return self.base.config.invalid_trial_cost, len(self.active_constraints)
        total = 0.0
        stale = 0
        for active in self.active_constraints:
            value, is_stale = self._constraint_value(forward, active)
            total += value
            stale += int(is_stale)
        return float(total), stale

    def evaluate(self, x: ArrayLike) -> ConstrainedObjectiveEvaluation:
        """Return base cost/gradient plus the finite active safety terms."""

        values = np.asarray(x, dtype=float)
        self.base.split(values)
        base_evaluation = self.base.evaluate(values)
        safety_cost, stale = self._safety_value(values)
        gradient = np.zeros_like(values)
        step = self.config.finite_difference_step
        for index in range(len(values)):
            h = step * max(1.0, abs(values[index]))
            plus, minus = values.copy(), values.copy()
            plus[index] += h
            minus[index] -= h
            plus_value, _ = self._safety_value(plus)
            minus_value, _ = self._safety_value(minus)
            gradient[index] = (plus_value - minus_value) / (2.0 * h)
        return ConstrainedObjectiveEvaluation(
            base_evaluation.cost + safety_cost,
            base_evaluation.gradient + gradient,
            base_evaluation,
            safety_cost,
            stale,
        )

    def value_and_gradient(self, x: ArrayLike) -> tuple[float, np.ndarray]:
        """SciPy-compatible objective callback."""

        evaluation = self.evaluate(x)
        return evaluation.cost, evaluation.gradient


def _minimum_margin(report: TrajectorySafetyReport) -> float | None:
    values = [window.minimum_margin for window in report.windows if window.minimum_margin is not None]
    return min(values) if values else None


def _select_witnesses(
    report: TrajectorySafetyReport,
    existing: Sequence[ActiveSafetyConstraint],
    config: WholeBodySafetyConfig,
) -> list[SafetyWitness]:
    candidates = [witness for window in report.windows for witness in window.witnesses]
    candidates.sort(key=lambda item: float("-inf") if item.margin is None else item.margin)
    selected: list[SafetyWitness] = []
    prior = [item.witness for item in existing]
    for candidate in candidates:
        duplicate = any(
            candidate.window_index == other.window_index
            and candidate.minco_segment_index == other.minco_segment_index
            and abs(candidate.normalized_time - other.normalized_time) < config.tau_merge_tolerance
            and {candidate.body_edge_a, candidate.body_edge_b} == {other.body_edge_a, other.body_edge_b}
            and abs(candidate.section_lambda - other.section_lambda) < config.lambda_merge_tolerance
            for other in (*prior, *selected)
        )
        if not duplicate:
            selected.append(candidate)
        if len(selected) >= config.max_witnesses_per_round:
            break
    return selected


def _result_status(status: VerificationStatus) -> WholeBodyResultStatus:
    return {
        VerificationStatus.NUMERICALLY_VERIFIED: WholeBodyResultStatus.SAFE_NUMERICAL,
        VerificationStatus.CERTIFIED: WholeBodyResultStatus.SAFE_CERTIFIED,
        VerificationStatus.UNSAFE: WholeBodyResultStatus.UNSAFE,
        VerificationStatus.UNCERTIFIED: WholeBodyResultStatus.UNCERTIFIED,
        VerificationStatus.NUMERICAL_FAILURE: WholeBodyResultStatus.NUMERICAL_FAILURE,
        VerificationStatus.SAFE: WholeBodyResultStatus.SAFE_NUMERICAL,
    }[status]


def optimize_with_whole_body_safety(
    objective: JointTOGTObjective,
    x0: ArrayLike,
    body: CuboidBody,
    safety_config: WholeBodySafetyConfig,
) -> WholeBodyOptimizationResult:
    """Optimize, verify, add worst witnesses, and warm-start until terminal."""

    start = np.asarray(x0, dtype=float)
    objective.split(start)
    active: list[ActiveSafetyConstraint] = []
    history: list[OuterIterationRecord] = []
    scipy_result: OptimizeResult = _minimize_togt_lbfgs(
        objective.scipy_value_and_gradient, start, objective.config
    )
    final_evaluation = objective.evaluate(scipy_result.x)
    last_report: TrajectorySafetyReport | None = None
    for outer in range(safety_config.max_outer_iterations):
        report = verify_whole_body_trajectory(
            forward=final_evaluation.forward,
            track=objective.track,
            body=body,
            config=safety_config,
            parameters=objective.config.quadrotor,
        )
        last_report = report
        history.append(OuterIterationRecord(
            outer, report.status, sum(len(window.witnesses) for window in report.windows),
            _minimum_margin(report), float(np.sum(final_evaluation.forward.durations)),
            float(final_evaluation.cost),
        ))
        if report.status in {VerificationStatus.NUMERICALLY_VERIFIED, VerificationStatus.CERTIFIED}:
            break
        if report.status in {VerificationStatus.NUMERICAL_FAILURE, VerificationStatus.UNCERTIFIED}:
            break
        new_witnesses = _select_witnesses(report, active, safety_config)
        if new_witnesses:
            active.extend(ActiveSafetyConstraint(item, safety_config.safety_penalty_weight) for item in new_witnesses)
        elif active:
            # The same physical violation can remain after moving from outside
            # the SC domain to a radial violation. Escalate its multiplier
            # instead of falsely treating witness de-duplication as an impasse.
            active = [
                ActiveSafetyConstraint(item.witness, min(item.weight * 10.0, 1.0e12))
                for item in active
            ]
        else:
            break
        constrained = WholeBodyConstrainedObjective(objective, body, safety_config, active)
        scipy_result = _minimize_togt_lbfgs(
            constrained.value_and_gradient, np.asarray(scipy_result.x, dtype=float), objective.config
        )
        final_evaluation = objective.evaluate(scipy_result.x)
    assert last_report is not None
    return WholeBodyOptimizationResult(
        _result_status(last_report.status), bool(scipy_result.success),
        np.asarray(scipy_result.x, dtype=float), final_evaluation.forward,
        float(final_evaluation.cost), last_report, tuple(active), tuple(history),
        str(scipy_result.message),
    )


__all__ = [
    "ActiveSafetyConstraint", "ConstrainedObjectiveEvaluation",
    "OuterIterationRecord", "WholeBodyConstrainedObjective",
    "WholeBodyOptimizationResult", "WholeBodyResultStatus",
    "optimize_with_whole_body_safety",
]
