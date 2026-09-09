"""Phase-seeded, feasibility-guided CEM over native K and polar D coordinates."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import time

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    duration_jacobian_diagonal,
    k_from_durations,
)
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import screen_candidate


@dataclass(frozen=True)
class PhaseFrontEndConfig:
    first_arrival_bounds: tuple[float, float] = (1.0, 3.0)
    inter_window_bounds: tuple[float, float] = (1.1, 2.8)
    final_durations: tuple[float, ...] = (1.37, 1.8, 2.2)
    minimum_cycle: int = 2
    maximum_cycle: int = 39
    maximum_arrival: float = 14.0


@dataclass(frozen=True)
class CEMConfig:
    seed: int = 11
    population: int = 256
    elite: int = 32
    memory: int = 16
    maximum_rounds: int = 20
    post_feasible_rounds: int = 1
    old_distribution_weight: float = 0.3
    independent_time_std: float = 0.035
    common_time_std: float = 0.025
    angle_std: float = 0.035
    log_radius_std: float = 0.18

    def __post_init__(self):
        if not (0 < self.memory <= self.elite < self.population):
            raise ValueError("require 0 < memory <= elite < population")
        if self.maximum_rounds <= 0 or self.post_feasible_rounds < 0:
            raise ValueError("invalid round counts")
        if not 0 < self.old_distribution_weight < 1:
            raise ValueError("old_distribution_weight must lie in (0,1)")


def polar_encode(x, temporal_dimension):
    values = np.asarray(x, dtype=float)
    d = values[temporal_dimension:].reshape(-1, 2)
    radii = np.linalg.norm(d, axis=1)
    if np.any(radii <= 0) or not np.all(np.isfinite(values)):
        raise ValueError("polar encoding requires finite nonzero D blocks")
    return np.r_[values[:temporal_dimension], np.arctan2(d[:, 1], d[:, 0]), np.log(radii)]


def polar_decode(y, temporal_dimension):
    values = np.asarray(y, dtype=float)
    count = (len(values) - temporal_dimension) // 2
    angles = values[temporal_dimension:temporal_dimension + count]
    radii = np.exp(np.clip(values[temporal_dimension + count:], -3.0, 12.0))
    d = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
    return np.r_[values[:temporal_dimension], d.ravel()]


def feasible_rank(rows):
    """The reported solution set contains hard-screen passes only."""
    return sorted((row for row in rows if row["screen"]["passed"]),
                  key=lambda row: (row["flight_time"], row["id"]))


def proposal_key(screen, flight_time):
    """Order proposal evidence; this key never defines the returned solution."""
    if screen.get("passed"):
        return 4.0, -float(flight_time)
    spheres = screen.get("spheres", [])
    passed = sum(bool(item["passed"]) for item in spheres)
    if screen.get("reason", "").startswith("dynamics_"):
        return 3.0, -float(screen["dynamics"].get("max_velocity", np.inf))
    if screen.get("reason") == "crossing_order_or_count":
        return 2.0, 0.0
    if screen.get("reason", "").startswith("sphere_") and spheres:
        return 1.0, float(passed), float(spheres[-1].get("minimum_margin", -np.inf))
    return 0.0, -float(flight_time)


def _evaluate(objective, scenario, dynamic_config, x, metadata, index, on_record):
    start = time.perf_counter()
    row = dict(metadata, id=index, x=np.asarray(x, dtype=float))
    try:
        forward = objective.forward(row["x"])
        row["flight_time"] = float(forward.trajectory.total_time)
        row["screen"] = screen_candidate(forward, scenario, dynamic_config)
    except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
        row["flight_time"] = 1.0e300
        row["screen"] = dict(passed=False, reason="numerical_failure", error=str(exc))
    row["proposal_key"] = proposal_key(row["screen"], row["flight_time"])
    row["screen_seconds"] = time.perf_counter() - start
    if on_record:
        on_record(row)
    return row


def _arrival_options(window, templates, config):
    options = []
    omega = float(window.omega)
    if omega == 0:
        raise ValueError("phase front end requires rotating windows")
    for template_index, template in enumerate(templates):
        for cycle in range(config.minimum_cycle, config.maximum_cycle + 1):
            instant = (template["phase"] - window.theta0 + 2 * np.pi * cycle) / omega
            if 0 < instant < config.maximum_arrival:
                options.append((float(instant), template_index, np.asarray(template["d"], dtype=float)))
    return options


def phase_front_end(objective, center, scenario, dynamic_config, templates, config=PhaseFrontEndConfig(),
                    *, start_id=0, on_record=None):
    """Enumerate periodic aliases of independently validated single-window seeds."""
    temporal_dimension = len(scenario.windows) + 1
    options = [_arrival_options(window, templates, config) for window in scenario.windows]
    rows = []
    next_id = start_id
    for chosen in itertools.product(*options):
        arrivals = np.asarray([item[0] for item in chosen])
        gaps = np.diff(arrivals)
        if not config.first_arrival_bounds[0] <= arrivals[0] <= config.first_arrival_bounds[1]:
            continue
        if len(gaps) and not np.all((gaps >= config.inter_window_bounds[0]) &
                                    (gaps <= config.inter_window_bounds[1])):
            continue
        for final_duration in config.final_durations:
            durations = np.r_[arrivals[0], gaps, final_duration]
            x = np.r_[k_from_durations(durations), *(item[2] for item in chosen)]
            metadata = dict(stage="phase_front_end", template_indices=[item[1] for item in chosen],
                            durations=durations)
            rows.append(_evaluate(objective, scenario, dynamic_config, x, metadata, next_id, on_record))
            next_id += 1
    return rows, next_id


def geometry_complete(row, window_count):
    spheres = row["screen"].get("spheres", [])
    return len(spheres) == window_count and all(item["passed"] for item in spheres) and \
        row["screen"].get("reason") != "crossing_order_or_count"


def local_cem_search(objective, seed_x, scenario, dynamic_config, config=CEMConfig(), *,
                     start_id=0, on_record=None):
    """Adapt a full covariance; failed rows guide proposals but cannot be returned."""
    temporal_dimension = len(scenario.windows) + 1
    mean = polar_encode(seed_x, temporal_dimension)
    jacobian = duration_jacobian_diagonal(np.asarray(seed_x)[:temporal_dimension])
    independent = config.independent_time_std / jacobian
    covariance = np.diag(np.r_[independent**2,
                               np.full(len(scenario.windows), config.angle_std**2),
                               np.full(len(scenario.windows), config.log_radius_std**2)])
    common = config.common_time_std / jacobian
    covariance[:temporal_dimension, :temporal_dimension] += np.outer(common, common)
    floor = np.diag(np.r_[np.full(temporal_dimension, 2e-6),
                          np.full(len(scenario.windows), 2e-6),
                          np.full(len(scenario.windows), 2e-5)])
    rng = np.random.default_rng(config.seed)
    rows, memory = [], []
    first_feasible_round = None
    next_id = start_id
    round_summaries = []
    for round_index in range(config.maximum_rounds):
        latent = rng.multivariate_normal(mean, covariance, size=config.population)
        current = []
        for y in latent:
            x = polar_decode(y, temporal_dimension)
            row = _evaluate(objective, scenario, dynamic_config, x,
                            dict(stage="local_cem", round=round_index, latent=y), next_id, on_record)
            next_id += 1
            current.append(row)
            rows.append(row)
        pool = memory + current
        pool.sort(key=lambda row: tuple(row["proposal_key"]), reverse=True)
        elite = pool[:config.elite]
        elite_latent = np.asarray([row["latent"] for row in elite])
        fresh_mean = elite_latent.mean(axis=0)
        fresh_covariance = np.cov(elite_latent, rowvar=False) + floor
        old = config.old_distribution_weight
        mean = old * mean + (1 - old) * fresh_mean
        covariance = old * covariance + (1 - old) * fresh_covariance
        memory = elite[:config.memory]
        feasible_count = sum(row["screen"]["passed"] for row in current)
        round_summaries.append(dict(round=round_index, feasible=feasible_count,
                                    best_proposal_key=elite[0]["proposal_key"],
                                    best_flight_time=elite[0]["flight_time"]))
        if feasible_count and first_feasible_round is None:
            first_feasible_round = round_index
        if first_feasible_round is not None and round_index - first_feasible_round >= config.post_feasible_rounds:
            break
    return rows, feasible_rank(rows), round_summaries, next_id
