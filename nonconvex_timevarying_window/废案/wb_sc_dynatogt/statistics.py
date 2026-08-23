"""Small dependency-free confidence-interval helpers used by experiments."""

from __future__ import annotations

import numpy as np


def wilson_interval(successes: int, total: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if total < 1 or successes < 0 or successes > total:
        raise ValueError("Wilson inputs require 0 <= successes <= total and total > 0")
    p = successes / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    half = z * np.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total)) / denominator
    low = 0.0 if successes == 0 else max(0.0, center - half)
    high = 1.0 if successes == total else min(1.0, center + half)
    return float(low), float(high)


def paired_bootstrap_interval(
    values: np.ndarray,
    *,
    resamples: int = 10_000,
    seed: int = 0,
) -> tuple[float, float]:
    samples = np.asarray(values, dtype=float)
    if samples.ndim != 1 or not len(samples) or not np.all(np.isfinite(samples)):
        raise ValueError("bootstrap values must be a nonempty finite vector")
    if resamples < 1:
        raise ValueError("resamples must be positive")
    rng = np.random.default_rng(seed)
    means = np.empty(resamples, dtype=float)
    # Chunk the index matrix so formal 10,000-resample reports remain light.
    cursor = 0
    while cursor < resamples:
        count = min(1_000, resamples - cursor)
        indices = rng.integers(0, len(samples), size=(count, len(samples)))
        means[cursor : cursor + count] = samples[indices].mean(axis=1)
        cursor += count
    low, high = np.quantile(means, [0.025, 0.975])
    return float(low), float(high)


__all__ = ["paired_bootstrap_interval", "wilson_interval"]
