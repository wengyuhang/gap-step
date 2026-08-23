"""Exact-area safety functional and its stable-topology chain rule."""

from __future__ import annotations

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]


def instantaneous_penalty(section_area: float, intersection_area: float) -> float:
    """Return ``A [r(1-r)]^3`` with the documented A=0 extension."""

    area_a, area_c = float(section_area), float(intersection_area)
    if area_a < 0.0 or area_c < -1.0e-12 or area_c > area_a + 1.0e-10:
        raise ValueError("areas must satisfy 0 <= C <= A")
    if area_a == 0.0:
        return 0.0
    ratio = float(np.clip(area_c / area_a, 0.0, 1.0))
    return float(area_a * (ratio * (1.0 - ratio)) ** 3)


def instantaneous_penalty_gradient(
    section_area: float,
    intersection_area: float,
    section_area_gradient: ArrayLike,
    intersection_area_gradient: ArrayLike,
) -> FloatArray:
    """Apply the analytic chain rule inside one jointly stable topology cell.

    Derivatives of the active section/intersection vertices are supplied by
    the caller.  The formula intentionally rejects A=0: the theory only gives
    a value extension there, not a global C1 extension.
    """

    area_a, area_c = float(section_area), float(intersection_area)
    grad_a = np.asarray(section_area_gradient, dtype=float)
    grad_c = np.asarray(intersection_area_gradient, dtype=float)
    if grad_a.shape != grad_c.shape:
        raise ValueError("A and C gradients must have the same shape")
    if area_a <= 0.0 or area_c < 0.0 or area_c > area_a:
        raise ValueError("stable-topology gradient requires 0 <= C <= A and A > 0")
    ratio = area_c / area_a
    psi = (ratio * (1.0 - ratio)) ** 3
    psi_prime = 3.0 * ratio**2 * (1.0 - ratio) ** 2 * (1.0 - 2.0 * ratio)
    grad_ratio = (area_a * grad_c - area_c * grad_a) / area_a**2
    return psi * grad_a + area_a * psi_prime * grad_ratio


def integrated_penalty(times: ArrayLike, penalties: ArrayLike) -> float:
    grid = np.asarray(times, dtype=float)
    values = np.asarray(penalties, dtype=float)
    if grid.ndim != 1 or values.shape != grid.shape or len(grid) < 2:
        raise ValueError("times and penalties must be equal one-dimensional arrays")
    if np.any(np.diff(grid) <= 0.0) or np.any(values < 0.0):
        raise ValueError("times must increase and penalties must be nonnegative")
    return float(np.trapz(values, grid))


def integrated_penalty_gradient(
    times: ArrayLike,
    instantaneous_gradients: ArrayLike,
    *,
    terminal_penalty: float,
    total_time_gradient: ArrayLike,
) -> FloatArray:
    """Piecewise-Leibniz gradient including the non-cancelling terminal term."""

    grid = np.asarray(times, dtype=float)
    gradients = np.asarray(instantaneous_gradients, dtype=float)
    terminal_gradient = np.asarray(total_time_gradient, dtype=float)
    if grid.ndim != 1 or gradients.ndim != 2 or gradients.shape[0] != len(grid):
        raise ValueError("instantaneous_gradients must have shape (len(times), n)")
    if gradients.shape[1:] != terminal_gradient.shape:
        raise ValueError("terminal gradient shape does not match")
    if np.any(np.diff(grid) <= 0.0):
        raise ValueError("times must increase")
    return np.trapz(gradients, grid, axis=0) + float(terminal_penalty) * terminal_gradient


__all__ = [
    "instantaneous_penalty",
    "instantaneous_penalty_gradient",
    "integrated_penalty",
    "integrated_penalty_gradient",
]
