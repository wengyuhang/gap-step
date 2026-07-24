"""Feasibility-first candidate retention for MSR-DynaTOGT."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationResult

from .config import CandidatePoolConfig
from .feasibility_repair import FeasibilityReport, RepairOutcome
from .initializations import InitialGuess


@dataclass(frozen=True)
class Candidate:
    initialization: InitialGuess
    raw_result: OptimizationResult
    raw_feasibility: FeasibilityReport
    result: OptimizationResult
    feasibility: FeasibilityReport
    optimization_seconds: float
    repair_seconds: float = 0.0
    repair: RepairOutcome | None = None

    @property
    def legal(self) -> bool:
        return bool(
            self.feasibility.window_order_legal
            and self.feasibility.window_internal_legal
        )

    @property
    def dynamically_feasible(self) -> bool:
        return self.feasibility.sampled_dynamic_limits_satisfied

    @property
    def rank_key(self) -> tuple[int, int, float, float]:
        """Required ordering: legality, sampled dynamics, then flight time."""

        return (
            0 if self.legal else 1,
            0 if self.dynamically_feasible else 1,
            self.result.total_time,
            self.result.objective,
        )

    @property
    def wall_clock_seconds(self) -> float:
        return self.optimization_seconds + self.repair_seconds

    @property
    def iterations(self) -> int:
        extra = 0 if self.repair is None else self.repair.reoptimization_iterations
        return self.raw_result.iterations + extra

    @property
    def evaluations(self) -> int:
        extra = 0 if self.repair is None else self.repair.reoptimization_evaluations
        return self.raw_result.evaluations + extra

    def to_run_row(
        self,
        *,
        method: str,
        scene: str,
        seed: int,
        candidate_count: int,
        comparison_protocol: str,
    ) -> dict[str, Any]:
        repair = self.repair
        if repair is None or not repair.triggered:
            optimizer_success = self.raw_result.success
            final_source = "raw_optimizer_result"
        else:
            optimizer_success = repair.reoptimization_optimizer_success
            final_source = (
                "reoptimized_result"
                if repair.reoptimization_accepted
                else "sampled_feasible_repair_incumbent"
            )
        minimum_rotor = float(np.min(self.feasibility.min_rotor_thrust))
        maximum_rotor = float(np.max(self.feasibility.max_rotor_thrust))
        return {
            "method": method,
            "scene": scene,
            "seed": seed,
            "comparison_protocol": comparison_protocol,
            # Overall success is sampled feasibility, never optimizer.success.
            "success": self.feasibility.sampled_feasible,
            "optimizer_success": optimizer_success,
            "raw_optimizer_success": self.raw_result.success,
            "final_result_source": final_source,
            "total_time": self.result.total_time,
            "wall_clock_seconds": self.wall_clock_seconds,
            "iterations": self.iterations,
            "evaluations": self.evaluations,
            "window_order_legal": self.feasibility.window_order_legal,
            "window_internal_legal": self.feasibility.window_internal_legal,
            "minimum_boundary_margin": self.feasibility.min_boundary_margin,
            "maximum_body_rate": max(
                self.feasibility.max_body_rate_xy,
                self.feasibility.max_abs_body_rate_z,
            ),
            "maximum_body_rate_xy": self.feasibility.max_body_rate_xy,
            "maximum_abs_body_rate_z": self.feasibility.max_abs_body_rate_z,
            "minimum_collective_thrust": self.feasibility.min_collective_thrust,
            "maximum_collective_thrust": self.feasibility.max_collective_thrust,
            "minimum_single_rotor_thrust": minimum_rotor,
            "maximum_single_rotor_thrust": maximum_rotor,
            "sampled_max_velocity": self.feasibility.max_velocity,
            "sampled_dynamic_limits_satisfied": self.dynamically_feasible,
            "feasibility_claim": (
                "高密度采样可行"
                if self.feasibility.sampled_feasible
                else "高密度采样未通过"
            ),
            "initialization_type": self.initialization.kind,
            "initialization_label": self.initialization.label,
            "candidate_count": candidate_count,
            "repair_triggered": False if repair is None else repair.triggered,
            "repair_succeeded": False if repair is None else repair.succeeded,
            "repair_mode": "none" if repair is None else repair.mode,
            "repair_scale_factor": 1.0 if repair is None else repair.scale_factor,
            "repair_before_total_time": (
                self.raw_result.total_time if repair is None else repair.before_total_time
            ),
            "repair_after_total_time": (
                self.result.total_time if repair is None else repair.after_total_time
            ),
            "repair_reoptimization_improvement": (
                0.0 if repair is None else repair.reoptimization_improvement
            ),
            "repair_reoptimization_accepted": (
                False if repair is None else repair.reoptimization_accepted
            ),
            "repair_before_max_rotor_thrust": (
                float(np.max(self.raw_feasibility.max_rotor_thrust))
                if repair is None
                else repair.before_max_rotor_thrust
            ),
            "repair_after_max_rotor_thrust": maximum_rotor,
            "failure_reasons": " | ".join(self.feasibility.failure_reasons),
        }


class CandidatePool:
    """Deduplicate and retain candidates using the required lexicographic rank."""

    def __init__(self, config: CandidatePoolConfig | None = None) -> None:
        self.config = CandidatePoolConfig() if config is None else config
        self._candidates: list[Candidate] = []
        self.duplicates_removed = 0

    def _near_duplicate(self, left: Candidate, right: Candidate) -> bool:
        scale = max(left.result.total_time, right.result.total_time, 1.0)
        close_time = (
            abs(left.result.total_time - right.result.total_time)
            <= self.config.time_relative_tolerance * scale
        )
        close_waypoints = bool(
            np.max(np.abs(left.result.waypoints - right.result.waypoints), initial=0.0)
            <= self.config.waypoint_absolute_tolerance
        )
        close_durations = bool(
            np.max(np.abs(left.result.durations - right.result.durations), initial=0.0)
            <= self.config.duration_absolute_tolerance
        )
        return close_time and close_waypoints and close_durations

    def add(self, candidate: Candidate) -> bool:
        for index, retained in enumerate(self._candidates):
            if self._near_duplicate(retained, candidate):
                self.duplicates_removed += 1
                if candidate.rank_key < retained.rank_key:
                    self._candidates[index] = candidate
                    self._sort_and_trim()
                    return True
                return False
        self._candidates.append(candidate)
        self._sort_and_trim()
        return True

    def _sort_and_trim(self) -> None:
        self._candidates.sort(key=lambda item: item.rank_key)
        del self._candidates[self.config.max_candidates :]

    @property
    def candidates(self) -> tuple[Candidate, ...]:
        return tuple(self._candidates)

    @property
    def best(self) -> Candidate:
        if not self._candidates:
            raise RuntimeError("candidate pool is empty")
        return self._candidates[0]

    def __len__(self) -> int:
        return len(self._candidates)


__all__ = ["Candidate", "CandidatePool"]
