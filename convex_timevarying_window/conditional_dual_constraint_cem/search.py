"""Full-covariance CEM using the established dual-constraint ranking."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.search import (
    rank_dual_constraints,
)
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    duration_jacobian_diagonal,
)


@dataclass(frozen=True)
class ConvexDualCEMConfig:
    seed: int = 20260910
    population: int = 64
    elite: int = 16
    memory: int = 8
    maximum_rounds: int = 50
    post_zero_rounds: int = 5
    dynamic_epsilon: float = 1e-6
    safety_epsilon: float = 1e-6
    old_distribution_weight: float = 0.35
    independent_time_std: float = 0.20
    common_time_std: float = 0.15
    spatial_std: float = 0.30

    def __post_init__(self):
        if not 0 < self.memory <= self.elite < self.population:
            raise ValueError("require 0 < memory <= elite < population")
        if self.maximum_rounds < 1 or self.post_zero_rounds < 0:
            raise ValueError("invalid round count")


def _canonicalize(x, objective):
    values = np.asarray(x, dtype=float).copy()
    offset = objective.temporal_dimension
    for window_index, size in enumerate(objective.spatial_dimensions):
        block = values[offset:offset + size]
        if objective.track.windows[window_index].aperture.kind == "polygon":
            norm = np.linalg.norm(block)
            if not np.isfinite(norm) or norm < 1e-8:
                block[:] = objective.track.windows[window_index].aperture.initial_d()
            else:
                block /= norm
        else:
            block[:] = np.clip(block, -12.0, 12.0)
        offset += size
    return values


def _initial_distribution(seed_x, objective, config):
    mean = _canonicalize(seed_x, objective)
    jacobian = duration_jacobian_diagonal(mean[:objective.temporal_dimension])
    time_std = config.independent_time_std / np.maximum(jacobian, 1e-12)
    covariance = np.diag(np.r_[
        time_std**2,
        np.full(objective.dimension-objective.temporal_dimension, config.spatial_std**2),
    ])
    common = config.common_time_std / np.maximum(jacobian, 1e-12)
    covariance[:objective.temporal_dimension, :objective.temporal_dimension] += np.outer(common, common)
    floor = np.diag(np.r_[
        np.full(objective.temporal_dimension, 2e-5),
        np.full(objective.dimension-objective.temporal_dimension, 2e-5),
    ])
    return mean, covariance, floor


def _evaluate(base, safety, x, row_id, round_index, mode):
    started = time.perf_counter()
    row = {"id": row_id, "round": round_index, "x": np.asarray(x, dtype=float)}
    try:
        forward = base.forward(row["x"])
        base_cost, _ = base.value_and_gradient(row["x"])
        safety_evaluation = safety.evaluate(forward.trajectory, with_gradient=False)
        dynamic_integral = max(float(base_cost-forward.trajectory.total_time), 0.0)
        safety_integral = max(float(safety_evaluation.value), 0.0)
        row.update(
            flight_time=float(forward.trajectory.total_time),
            dynamic_integral=dynamic_integral,
            safety_integral=safety_integral,
            safety_breakdown=list(safety_evaluation.per_window),
            numerical_failure=False,
        )
    except (ValueError, RuntimeError, FloatingPointError, OverflowError,
            np.linalg.LinAlgError) as exc:
        row.update(
            flight_time=1e300, dynamic_integral=1e300, safety_integral=1e300,
            numerical_failure=True, error=str(exc),
        )
    row["both_soft_zero"] = bool(
        row["dynamic_integral"] == 0.0 and row["safety_integral"] == 0.0
    )
    row["mode"] = mode
    row["evaluation_seconds"] = time.perf_counter()-started
    return row


def _rank(rows, config, mode, safety_weight):
    for row in rows:
        dynamic_excess = max(row["dynamic_integral"]-config.dynamic_epsilon, 0.0)
        safety_excess = max(row["safety_integral"]-config.safety_epsilon, 0.0)
        row["augmented_objective"] = row["flight_time"] + dynamic_excess
        if mode == "joint":
            row["augmented_objective"] += safety_weight*safety_excess
    if mode == "joint":
        return rank_dual_constraints(rows, config.dynamic_epsilon, config.safety_epsilon)
    ranked = sorted(rows, key=lambda row: (
        max(row["dynamic_integral"]-config.dynamic_epsilon, 0.0),
        row["augmented_objective"], row["id"],
    ))
    for level, row in enumerate(ranked):
        row["pareto_front"] = level
    return ranked


def conditional_dual_cem(base, safety, seed_x, mode,
                         config=ConvexDualCEMConfig(), *, on_record=None):
    """Search native variable-dimension ``[K,D]`` with full covariance."""
    if mode not in {"dynamic_only", "joint"}:
        raise ValueError("mode must be dynamic_only or joint")
    mean, covariance, floor = _initial_distribution(seed_x, base, config)
    rng = np.random.default_rng(config.seed)
    seed = _evaluate(base, safety, _canonicalize(seed_x, base), 0, -1, mode)
    seed["stage"] = "input_seed"
    rows, memory, summaries = [seed], [seed], []
    first_zero = None
    next_id = 1
    for round_index in range(config.maximum_rounds):
        samples = rng.multivariate_normal(mean, covariance, size=config.population)
        current = []
        for sample in samples:
            x = _canonicalize(sample, base)
            row = _evaluate(base, safety, x, next_id, round_index, mode)
            row["latent"] = x.copy()
            next_id += 1
            current.append(row)
            rows.append(row)
            if on_record is not None:
                on_record(row)
        ranked = _rank(memory+current, config, mode, safety.config.objective_weight)
        elite = ranked[:config.elite]
        elite_samples = np.asarray([row.get("latent", row["x"]) for row in elite])
        fresh_mean = elite_samples.mean(axis=0)
        fresh_covariance = np.cov(elite_samples, rowvar=False)+floor
        old = config.old_distribution_weight
        mean = old*mean+(1.0-old)*fresh_mean
        covariance = old*covariance+(1.0-old)*fresh_covariance
        memory = elite[:config.memory]
        zero_count = sum(row["both_soft_zero"] for row in current)
        summaries.append({
            "round": round_index,
            "both_soft_zero": zero_count,
            "best_dynamic_integral": min(row["dynamic_integral"] for row in current),
            "best_safety_integral": min(row["safety_integral"] for row in current),
            "best_augmented_objective": min(row["augmented_objective"] for row in current),
        })
        if zero_count and first_zero is None:
            first_zero = round_index
        if first_zero is not None and round_index-first_zero >= config.post_zero_rounds:
            break
    return rows, _rank(rows, config, mode, safety.config.objective_weight), summaries


__all__ = ["ConvexDualCEMConfig", "conditional_dual_cem"]
