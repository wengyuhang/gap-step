"""Frontend-free full-covariance CEM with parallel dynamic/safety scores."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.search import (
    polar_decode,
    polar_encode,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import integrated_dynamic_penalty
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import duration_jacobian_diagonal

from .safety_penalty import SafetyPenaltyConfig, integrated_safety_penalty


@dataclass(frozen=True)
class DualCEMConfig:
    seed: int = 20260909
    population: int = 128
    elite: int = 24
    memory: int = 12
    maximum_rounds: int = 12
    post_zero_rounds: int = 5
    dynamic_epsilon: float = 1.0e-6
    safety_epsilon: float = 1.0e-6
    old_distribution_weight: float = 0.35
    independent_time_std: float = 0.20
    common_time_std: float = 0.15
    angle_std: float = 0.60
    log_radius_std: float = 0.70

    def __post_init__(self):
        if not 0 < self.memory <= self.elite < self.population:
            raise ValueError("require 0 < memory <= elite < population")
        if self.maximum_rounds < 1 or self.post_zero_rounds < 0:
            raise ValueError("invalid round count")
        if self.dynamic_epsilon < 0 or self.safety_epsilon < 0:
            raise ValueError("constraint epsilons must be nonnegative")


def _pareto_fronts(rows, dynamic_epsilon=0.0, safety_epsilon=0.0):
    """Return nondominated fronts for minimising both soft constraint integrals."""
    remaining = list(range(len(rows)))
    fronts = []
    while remaining:
        front = []
        for i in remaining:
            a = rows[i]
            dominated = False
            for j in remaining:
                if i == j:
                    continue
                b = rows[j]
                if (b["dynamic_integral"] <= a["dynamic_integral"] + dynamic_epsilon and
                    b["safety_integral"] <= a["safety_integral"] + safety_epsilon and
                    (b["dynamic_integral"] < a["dynamic_integral"] - dynamic_epsilon or
                     b["safety_integral"] < a["safety_integral"] - safety_epsilon)):
                    dominated = True
                    break
            if not dominated:
                front.append(i)
        front.sort(key=lambda i: (rows[i]["augmented_objective"], rows[i]["flight_time"], rows[i]["id"]))
        fronts.append(front)
        chosen = set(front)
        remaining = [i for i in remaining if i not in chosen]
    return fronts


def rank_dual_constraints(rows, dynamic_epsilon=0.0, safety_epsilon=0.0):
    ranked = []
    for level, front in enumerate(_pareto_fronts(rows, dynamic_epsilon, safety_epsilon)):
        for i in front:
            rows[i]["pareto_front"] = level
            ranked.append(rows[i])
    return ranked


def _evaluate(objective, scenario, dynamic_config, safety_config, search_config,
              x, index, round_index, latent):
    started = time.perf_counter()
    row = {"id": index, "round": round_index, "x": np.asarray(x), "latent": np.asarray(latent)}
    try:
        forward = objective.forward(x)
        dynamic = integrated_dynamic_penalty(
            forward.trajectory, parameters=dynamic_config.quadrotor,
            limits=dynamic_config.dynamic_limits, weights=dynamic_config.penalty_weights,
            samples_per_segment=None, return_breakdown=True)
        safety = integrated_safety_penalty(
            forward.trajectory, scenario.windows, safety_config,
            return_breakdown=True)
        row.update(flight_time=float(forward.trajectory.total_time),
                   dynamic_integral=float(dynamic.total),
                   dynamic_breakdown={k: float(getattr(dynamic, k)) for k in
                                      ("velocity", "collective_thrust", "body_rate", "rotor_thrust")},
                   safety_integral=float(safety["total"]), safety_breakdown=safety["per_window"],
                   numerical_failure=False)
        # Fixed epsilon creates a ranking plateau only.  The unmodified raw
        # integrals below still decide exact-zero membership.
        dynamic_excess = max(row["dynamic_integral"] - search_config.dynamic_epsilon, 0.0)
        safety_excess = max(row["safety_integral"] - search_config.safety_epsilon, 0.0)
        row["augmented_objective"] = (row["flight_time"] + dynamic_excess +
                                        safety_config.objective_weight * safety_excess)
    except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
        row.update(flight_time=1e300, dynamic_integral=1e300, safety_integral=1e300,
                   augmented_objective=1e300, numerical_failure=True, error=str(exc))
    row["both_soft_zero"] = bool(
        row["dynamic_integral"] == 0.0 and row["safety_integral"] == 0.0)
    row["evaluation_seconds"] = time.perf_counter() - started
    return row


def _initial_distribution(seed_x, temporal_dimension, cem_config):
    mean = polar_encode(seed_x, temporal_dimension)
    jacobian = duration_jacobian_diagonal(np.asarray(seed_x)[:temporal_dimension])
    independent = cem_config.independent_time_std / jacobian
    common = cem_config.common_time_std / jacobian
    covariance = np.diag(np.r_[independent ** 2,
                               np.full(temporal_dimension - 1, cem_config.angle_std ** 2),
                               np.full(temporal_dimension - 1, cem_config.log_radius_std ** 2)])
    covariance[:temporal_dimension, :temporal_dimension] += np.outer(common, common)
    floor = np.diag(np.r_[np.full(temporal_dimension, 2e-5),
                          np.full(temporal_dimension - 1, 2e-5),
                          np.full(temporal_dimension - 1, 2e-4)])
    return mean, covariance, floor


def _update_distribution(memory, current, mean, covariance, floor, cem_config):
    ranked = rank_dual_constraints(memory + current, cem_config.dynamic_epsilon,
                                   cem_config.safety_epsilon)
    elite = ranked[:cem_config.elite]
    samples = np.asarray([row["latent"] for row in elite])
    fresh_mean = samples.mean(axis=0)
    fresh_covariance = np.cov(samples, rowvar=False) + floor
    old = cem_config.old_distribution_weight
    mean = old * mean + (1.0 - old) * fresh_mean
    covariance = old * covariance + (1.0 - old) * fresh_covariance
    return mean, covariance, elite[:cem_config.memory]


def dual_constraint_cem(objective, seed_x, scenario, dynamic_config,
                        cem_config=DualCEMConfig(), safety_config=SafetyPenaltyConfig(), *, on_record=None):
    """Search native ``[K,D]`` directly; there is no scene-specific frontend."""
    temporal_dimension = len(scenario.windows) + 1
    mean, covariance, floor = _initial_distribution(seed_x, temporal_dimension, cem_config)
    rng = np.random.default_rng(cem_config.seed)
    seed_latent = polar_encode(seed_x, temporal_dimension)
    seed_row = _evaluate(objective, scenario, dynamic_config, safety_config, cem_config,
                         seed_x, 0, -1, seed_latent)
    seed_row["stage"] = "input_seed"
    rows, memory, summaries = [seed_row], [seed_row], []
    first_zero = None
    next_id = 1
    for round_index in range(cem_config.maximum_rounds):
        latent_population = rng.multivariate_normal(mean, covariance, size=cem_config.population)
        current = []
        for latent in latent_population:
            row = _evaluate(objective, scenario, dynamic_config, safety_config, cem_config,
                            polar_decode(latent, temporal_dimension), next_id, round_index, latent)
            next_id += 1
            current.append(row)
            rows.append(row)
            if on_record is not None:
                on_record(row)
        mean, covariance, memory = _update_distribution(
            memory, current, mean, covariance, floor, cem_config)
        zero_count = sum(row["both_soft_zero"] for row in current)
        summaries.append({"round": round_index, "both_soft_zero": zero_count,
                          "best_dynamic_integral": min(r["dynamic_integral"] for r in current),
                          "best_safety_integral": min(r["safety_integral"] for r in current),
                          "best_augmented_objective": min(r["augmented_objective"] for r in current)})
        if zero_count and first_zero is None:
            first_zero = round_index
        if first_zero is not None and round_index - first_zero >= cem_config.post_zero_rounds:
            break
    return rows, rank_dual_constraints(rows, cem_config.dynamic_epsilon,
                                       cem_config.safety_epsilon), summaries


def resume_dual_constraint_cem(objective, seed_x, scenario, dynamic_config, prior_rows,
                               cem_config=DualCEMConfig(), safety_config=SafetyPenaltyConfig(), *,
                               on_record=None):
    """Replay a complete frozen prefix and add only missing post-zero rounds.

    The stored latent populations are checked against the seeded RNG before
    their scores update the distribution.  This makes the continuation an
    exact continuation of the earlier CEM run, rather than a new warm start.
    """
    temporal_dimension = len(scenario.windows) + 1
    mean, covariance, floor = _initial_distribution(seed_x, temporal_dimension, cem_config)
    rng = np.random.default_rng(cem_config.seed)
    seed_latent = polar_encode(seed_x, temporal_dimension)
    seed_row = _evaluate(objective, scenario, dynamic_config, safety_config, cem_config,
                         seed_x, 0, -1, seed_latent)
    memory = [seed_row]
    rows = list(prior_rows)
    summaries = []
    first_zero = None
    grouped = {}
    for row in rows:
        grouped.setdefault(int(row["round"]), []).append(row)
    if grouped and sorted(grouped) != list(range(max(grouped) + 1)):
        raise ValueError("frozen CEM prefix has missing rounds")
    for round_index in sorted(grouped):
        current = sorted(grouped[round_index], key=lambda row: int(row["id"]))
        if len(current) != cem_config.population:
            raise ValueError(f"frozen CEM round {round_index} is incomplete")
        generated = rng.multivariate_normal(mean, covariance, size=cem_config.population)
        stored = np.asarray([row["latent"] for row in current], dtype=float)
        error = float(np.max(np.abs(generated - stored)))
        if error > 1e-10:
            raise ValueError(f"frozen CEM RNG/distribution mismatch at round {round_index}: {error}")
        mean, covariance, memory = _update_distribution(
            memory, current, mean, covariance, floor, cem_config)
        zero_count = sum(bool(row["both_soft_zero"]) for row in current)
        summaries.append({"round": round_index, "both_soft_zero": zero_count,
                          "best_dynamic_integral": min(r["dynamic_integral"] for r in current),
                          "best_safety_integral": min(r["safety_integral"] for r in current),
                          "best_augmented_objective": min(r["augmented_objective"] for r in current),
                          "replayed": True})
        if zero_count and first_zero is None:
            first_zero = round_index

    next_round = 0 if not grouped else max(grouped) + 1
    next_id = 1 if not rows else max(int(row["id"]) for row in rows) + 1
    while next_round < cem_config.maximum_rounds:
        if first_zero is not None and next_round - first_zero > cem_config.post_zero_rounds:
            break
        latent_population = rng.multivariate_normal(mean, covariance, size=cem_config.population)
        current = []
        for latent in latent_population:
            row = _evaluate(objective, scenario, dynamic_config, safety_config, cem_config,
                            polar_decode(latent, temporal_dimension), next_id, next_round, latent)
            next_id += 1
            current.append(row)
            rows.append(row)
            if on_record is not None:
                on_record(row)
        mean, covariance, memory = _update_distribution(
            memory, current, mean, covariance, floor, cem_config)
        zero_count = sum(row["both_soft_zero"] for row in current)
        summaries.append({"round": next_round, "both_soft_zero": zero_count,
                          "best_dynamic_integral": min(r["dynamic_integral"] for r in current),
                          "best_safety_integral": min(r["safety_integral"] for r in current),
                          "best_augmented_objective": min(r["augmented_objective"] for r in current),
                          "replayed": False})
        if zero_count and first_zero is None:
            first_zero = next_round
        next_round += 1
    return rows, rank_dual_constraints(rows, cem_config.dynamic_epsilon,
                                       cem_config.safety_epsilon), summaries


__all__ = ["DualCEMConfig", "dual_constraint_cem", "resume_dual_constraint_cem",
           "rank_dual_constraints"]
