"""The positive-duration change of variables used by the TOGT codebase."""

from __future__ import annotations

import numpy as np


def durations_from_k(k: np.ndarray) -> np.ndarray:
    """Map unconstrained temporal variables to strictly positive durations.

    This is a direct Python transcription of ``TrajSolver::forwardT`` in the
    source-level TOGT reproduction, not a newly selected softplus mapping.
    """

    values = np.asarray(k, dtype=float)
    limit = np.finfo(float).max
    tiny = np.nextafter(0.0, 1.0)
    with np.errstate(over="ignore", under="ignore", invalid="ignore"):
        positive = (0.5 * values + 1.0) * values + 1.0
        denominator = (0.5 * values - 1.0) * values + 1.0
        negative = 1.0 / denominator
    positive = np.minimum(positive, limit)
    negative = np.maximum(negative, tiny)
    return np.where(values > 0.0, positive, negative)


def k_from_durations(durations: np.ndarray) -> np.ndarray:
    """Inverse of :func:`durations_from_k` for positive durations."""

    values = np.asarray(durations, dtype=float)
    if np.any(values <= 0.0):
        raise ValueError("segment durations must be strictly positive")
    out = np.empty_like(values)
    large = values > 1.0
    out[large] = np.sqrt(2.0 * values[large] - 1.0) - 1.0
    out[~large] = 1.0 - np.sqrt(2.0 / values[~large] - 1.0)
    return out


def duration_jacobian_diagonal(k: np.ndarray) -> np.ndarray:
    """Return the diagonal of ``d tau(K) / dK``."""

    values = np.asarray(k, dtype=float)
    with np.errstate(over="ignore", under="ignore", invalid="ignore"):
        denominator = (0.5 * values - 1.0) * values + 1.0
        negative = (1.0 - values) / denominator**2
    negative = np.nan_to_num(negative, nan=0.0, posinf=np.finfo(float).max, neginf=0.0)
    return np.where(values > 0.0, values + 1.0, negative)


def traversal_times(durations: np.ndarray, window_count: int | None = None) -> np.ndarray:
    """Prefix sums ``t_i = sum_{j=0}^i T_j`` for the window crossings."""

    values = np.asarray(durations, dtype=float)
    count = len(values) - 1 if window_count is None else int(window_count)
    if count < 0 or count > len(values):
        raise ValueError("invalid number of windows for the duration vector")
    return np.cumsum(values)[:count]


def add_traversal_time_gradients(
    direct_duration_gradient: np.ndarray, traversal_gradient: np.ndarray
) -> np.ndarray:
    """Accumulate every ``g_t[i]`` into duration entries ``j <= i``."""

    out = np.asarray(direct_duration_gradient, dtype=float).copy()
    crossing = np.asarray(traversal_gradient, dtype=float)
    if len(out) != len(crossing) + 1:
        raise ValueError("L windows require L+1 segment-duration gradients")
    # Reverse cumulative sum is the exact transpose of the prefix-sum map.
    out[: len(crossing)] += np.cumsum(crossing[::-1])[::-1]
    return out


def backpropagate_to_k(k: np.ndarray, duration_gradient: np.ndarray) -> np.ndarray:
    """Apply the final TOGT time-map chain rule."""

    return duration_jacobian_diagonal(k) * np.asarray(duration_gradient, dtype=float)
