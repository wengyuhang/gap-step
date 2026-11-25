"""Feasibility-preserving homotopy refinement of flight time.

The aggressive local solve optimizes flight time, native TOGT dynamics, and
whole-body frame safety together.  Its endpoint is allowed to be infeasible.
The homotopy filter then searches the complete ``[K, D]`` segment between the
last accepted point and that endpoint, and accepts only candidates that pass
both the soft screens and the independent dense audits.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import time
from typing import Callable

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs

from convex_timevarying_window.conditional_dual_constraint_cem.objective import (
    SafetyAugmentedTOGTObjective,
)


@dataclass(frozen=True)
class JointTimeRefinementConfig:
    maximum_rounds: int = 3
    local_max_iterations: int = 300
    scan_points: int = 41
    hard_bisection_steps: int = 3
    soft_zero_tolerance: float = 1e-12
    minimum_time_improvement: float = 1e-4

    def __post_init__(self):
        if self.maximum_rounds < 1 or self.local_max_iterations < 1:
            raise ValueError("round and iteration counts must be positive")
        if self.scan_points < 3 or self.hard_bisection_steps < 0:
            raise ValueError("invalid homotopy resolution")
        if self.soft_zero_tolerance < 0 or self.minimum_time_improvement < 0:
            raise ValueError("tolerances must be nonnegative")


@dataclass(frozen=True)
class JointTimeRefinementResult:
    x: np.ndarray
    flight_time: float
    dynamic_audit: dict
    safety_audit: dict
    rounds: tuple[dict, ...]
    evaluations: tuple[dict, ...]


def canonicalize(x, objective):
    """Apply the same polygon/circle latent conventions as the parent CEM."""
    values = np.asarray(x, dtype=float).copy()
    offset = objective.temporal_dimension
    for window_index, size in enumerate(objective.spatial_dimensions):
        block = values[offset:offset + size]
        if objective.track.windows[window_index].aperture.kind == "polygon":
            norm = float(np.linalg.norm(block))
            if not np.isfinite(norm) or norm < 1e-8:
                block[:] = objective.track.windows[window_index].aperture.initial_d()
            else:
                block /= norm
        else:
            block[:] = np.clip(block, -12.0, 12.0)
        offset += size
    return values


def soft_metrics(base, safety, x):
    forward = base.forward(x)
    base_cost, _ = base.value_and_gradient(x)
    safety_value = safety.evaluate(forward.trajectory, with_gradient=False).value
    return forward, {
        "flight_time": float(forward.trajectory.total_time),
        # Subtraction of two equal floating-point values can leave a few ulps.
        "dynamic_integral": max(float(base_cost - forward.trajectory.total_time), 0.0),
        "safety_integral": max(float(safety_value), 0.0),
    }


def _soft_feasible(metrics, tolerance):
    return (
        metrics["dynamic_integral"] <= tolerance
        and metrics["safety_integral"] <= tolerance
    )


def _hard_evaluate(base, x, dynamic_audit, safety_audit):
    forward = base.forward(x)
    dynamic = dynamic_audit(forward.trajectory)
    safety = None
    if dynamic["passed"]:
        safety = safety_audit(forward.trajectory)
    passed = bool(dynamic["passed"] and safety is not None and safety["passed"])
    return passed, forward, dynamic, safety


def refine_joint_feasible_time(
    base,
    safety,
    seed_x,
    optimizer_config,
    dynamic_audit: Callable,
    safety_audit: Callable,
    config: JointTimeRefinementConfig = JointTimeRefinementConfig(),
    *,
    on_evaluation: Callable[[dict], None] | None = None,
):
    """Jointly reduce time while retaining only independently audited iterates."""
    incumbent = canonicalize(seed_x, base)
    passed, forward, incumbent_dynamic, incumbent_safety = _hard_evaluate(
        base, incumbent, dynamic_audit, safety_audit
    )
    if not passed:
        raise ValueError("seed_x must pass both independent hard audits")

    incumbent_time = float(forward.trajectory.total_time)
    objective = SafetyAugmentedTOGTObjective(base, safety)
    evaluations = []
    rounds = []

    def record(row):
        evaluations.append(row)
        if on_evaluation is not None:
            on_evaluation(row)

    for round_index in range(config.maximum_rounds):
        started = time.perf_counter()
        local_config = replace(optimizer_config, max_iterations=config.local_max_iterations)
        local_result = _minimize_togt_lbfgs(
            objective.scipy_value_and_gradient, incumbent, local_config
        )
        target = canonicalize(local_result.x, base)
        _, target_metrics = soft_metrics(base, safety, target)

        candidates = []
        for alpha in np.linspace(0.0, 1.0, config.scan_points)[1:]:
            x = canonicalize((1.0 - alpha) * incumbent + alpha * target, base)
            try:
                _, metrics = soft_metrics(base, safety, x)
                row = {
                    "round": round_index,
                    "stage": "homotopy_scan",
                    "alpha": float(alpha),
                    "x": x.copy(),
                    **metrics,
                    "soft_feasible": _soft_feasible(
                        metrics, config.soft_zero_tolerance
                    ),
                }
            except (ValueError, RuntimeError, FloatingPointError, OverflowError,
                    np.linalg.LinAlgError) as exc:
                row = {
                    "round": round_index,
                    "stage": "homotopy_scan",
                    "alpha": float(alpha),
                    "x": x.copy(),
                    "flight_time": 1e300,
                    "dynamic_integral": 1e300,
                    "safety_integral": 1e300,
                    "soft_feasible": False,
                    "error": str(exc),
                }
            record(row)
            if row["soft_feasible"] and row["flight_time"] < incumbent_time:
                candidates.append(row)

        accepted = None
        for row in sorted(candidates, key=lambda item: item["flight_time"]):
            hard_passed, candidate_forward, dynamic, safety_result = _hard_evaluate(
                base, row["x"], dynamic_audit, safety_audit
            )
            row["hard_dynamic_passed"] = bool(dynamic["passed"])
            row["hard_safety_passed"] = (
                None if safety_result is None else bool(safety_result["passed"])
            )
            if hard_passed:
                accepted = (row, candidate_forward, dynamic, safety_result)
                break

        # Refine the accepted/failing boundary in alpha.  Every accepted point
        # is still subjected to the full independent audits.
        if accepted is not None and config.hard_bisection_steps:
            accepted_row = accepted[0]
            lo = float(accepted_row["alpha"])
            larger = [row["alpha"] for row in evaluations
                      if row["round"] == round_index and row["alpha"] > lo]
            hi = min(larger) if larger else 1.0
            for _ in range(config.hard_bisection_steps):
                alpha = 0.5 * (lo + hi)
                x = canonicalize((1.0 - alpha) * incumbent + alpha * target, base)
                _, metrics = soft_metrics(base, safety, x)
                row = {
                    "round": round_index,
                    "stage": "hard_boundary_bisection",
                    "alpha": alpha,
                    "x": x.copy(),
                    **metrics,
                    "soft_feasible": _soft_feasible(
                        metrics, config.soft_zero_tolerance
                    ),
                }
                if row["soft_feasible"]:
                    hard_passed, candidate_forward, dynamic, safety_result = _hard_evaluate(
                        base, x, dynamic_audit, safety_audit
                    )
                    row["hard_dynamic_passed"] = bool(dynamic["passed"])
                    row["hard_safety_passed"] = (
                        None if safety_result is None else bool(safety_result["passed"])
                    )
                else:
                    hard_passed = False
                record(row)
                if hard_passed:
                    lo = alpha
                    accepted = (row, candidate_forward, dynamic, safety_result)
                else:
                    hi = alpha

        previous_time = incumbent_time
        if accepted is not None:
            row, forward, incumbent_dynamic, incumbent_safety = accepted
            if previous_time - row["flight_time"] >= config.minimum_time_improvement:
                incumbent = row["x"].copy()
                incumbent_time = float(row["flight_time"])
            else:
                accepted = None

        rounds.append({
            "round": round_index,
            "local_optimizer_success": bool(local_result.success),
            "local_optimizer_status": int(local_result.status),
            "local_optimizer_message": str(local_result.message),
            "local_optimizer_iterations": int(local_result.nit),
            "target_flight_time": target_metrics["flight_time"],
            "target_dynamic_integral": target_metrics["dynamic_integral"],
            "target_safety_integral": target_metrics["safety_integral"],
            "accepted": accepted is not None,
            "accepted_flight_time": incumbent_time,
            "time_improvement": previous_time - incumbent_time,
            "seconds": time.perf_counter() - started,
        })
        if accepted is None:
            break

    return JointTimeRefinementResult(
        x=incumbent,
        flight_time=incumbent_time,
        dynamic_audit=incumbent_dynamic,
        safety_audit=incumbent_safety,
        rounds=tuple(rounds),
        evaluations=tuple(evaluations),
    )


__all__ = [
    "JointTimeRefinementConfig",
    "JointTimeRefinementResult",
    "canonicalize",
    "refine_joint_feasible_time",
    "soft_metrics",
]
