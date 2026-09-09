"""Direct bounded perturbations of native [K,D]; strict feasible-only ranking."""

from dataclasses import dataclass
import time

import numpy as np

from .safety import screen_candidate


@dataclass(frozen=True)
class SearchConfig:
    seed: int = 20260909
    per_scale: int = 100
    d_scales: tuple = (0.02, 0.05, 0.10)
    k_scales: tuple = (0.01, 0.03, 0.05)

    def __post_init__(self):
        if self.per_scale <= 0 or self.per_scale % 4 or len(self.d_scales) != len(self.k_scales) or not self.d_scales:
            raise ValueError("positive per_scale multiple of four and matching nonempty scales required")
        if not np.all(np.isfinite(self.d_scales + self.k_scales)) or min(self.d_scales + self.k_scales) <= 0:
            raise ValueError("scales must be finite and positive")


def generate_candidates(center, temporal_dimension, config):
    center = np.asarray(center, dtype=float)
    if center.ndim != 1 or not np.all(np.isfinite(center)) or temporal_dimension < 1 or (len(center) - temporal_dimension) % 2:
        raise ValueError("invalid [K,D] center")
    rng = np.random.default_rng(config.seed)
    k = center[:temporal_dimension]
    d = center[temporal_dimension:].reshape(-1, 2)
    for level, (ds, ks) in enumerate(zip(config.d_scales, config.k_scales)):
        for j in range(config.per_scale):
            mode = "D" if j < config.per_scale // 4 else "K" if j < config.per_scale // 2 else "DK"
            delta = np.zeros_like(center)
            if "K" in mode:
                delta[:temporal_dimension] = ks * np.maximum(1, np.abs(k)) * rng.uniform(-1, 1, len(k))
            if "D" in mode:
                angles = rng.uniform(0, 2 * np.pi, len(d))
                radii = np.sqrt(rng.uniform(0, 1, len(d))) * ds * np.maximum(1, np.linalg.norm(d, axis=1))
                delta[temporal_dimension:] = (radii[:, None] * np.column_stack((np.cos(angles), np.sin(angles)))).ravel()
            yield dict(level=level, mode=mode, delta=delta, x=center + delta)


def rank_feasible(rows):
    return sorted((r for r in rows if r["screen"]["passed"]), key=lambda r: (r["flight_time"], r["id"]))


def search(objective, center, scenario, config, search_config, *, on_record=None, screen=screen_candidate):
    rows = []
    temporal_dimension = len(scenario.windows) + 1

    def evaluate(candidate, index):
        start = time.perf_counter()
        row = dict(candidate, id=index)
        try:
            forward = objective.forward(candidate["x"])
            row["flight_time"] = float(forward.trajectory.total_time)
            row["screen"] = screen(forward, scenario, config)
        except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
            row["screen"] = dict(passed=False, reason="numerical_failure", error=str(exc))
        row["screen_seconds"] = time.perf_counter() - start
        rows.append(row)
        if on_record:
            on_record(row)

    evaluate(dict(level=-1, mode="nominal", x=np.asarray(center), delta=np.zeros_like(center)), 0)
    if not rows[0]["screen"]["passed"]:
        for i, candidate in enumerate(generate_candidates(center, temporal_dimension, search_config), 1):
            evaluate(candidate, i)
    return rows, rank_feasible(rows)
