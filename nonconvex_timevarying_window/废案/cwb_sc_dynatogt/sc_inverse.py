"""Damped Newton inverse for the fixed normalized disk SC map."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class SCInverseResult:
    """Result and diagnostics from a two-real-dimensional Newton solve."""

    z: FloatArray
    residual_norm: float
    iterations: int
    converged: bool


def inverse_sc_map(
    mapping: SCDiskMap,
    point: ArrayLike,
    *,
    initial: ArrayLike | None = None,
    tolerance: float = 1.0e-10,
    max_iterations: int = 40,
) -> SCInverseResult:
    """Invert ``mapping`` by damped Newton while staying inside the unit disk.

    A point outside the target polygon raises ``ValueError``; failure to meet
    the tolerance is represented by ``converged=False`` and is not collision
    evidence by itself.
    """

    target = np.asarray(point, dtype=float)
    if target.shape != (2,) or not np.all(np.isfinite(target)):
        raise ValueError("point must be finite with shape (2,)")
    if not np.isfinite(tolerance) or tolerance <= 0.0 or max_iterations < 1:
        raise ValueError("tolerance and max_iterations must be positive")
    polygon = Polygon(mapping.vertices)
    if not polygon.covers(Point(float(target[0]), float(target[1]))):
        raise ValueError("inverse target is outside the SC polygon")
    if initial is None:
        z = np.zeros(2, dtype=float)
    else:
        z = np.asarray(initial, dtype=float).copy()
        if z.shape != (2,) or not np.all(np.isfinite(z)) or np.linalg.norm(z) >= 1.0:
            raise ValueError("initial must be a finite point strictly inside the unit disk")
    scale = max(float(np.linalg.norm(np.ptp(mapping.vertices, axis=0))), 1.0)
    best_z = z.copy()
    best_residual = float("inf")
    for iteration in range(max_iterations + 1):
        residual = np.asarray(mapping.evaluate(z), dtype=float) - target
        residual_norm = float(np.linalg.norm(residual))
        if residual_norm < best_residual:
            best_z, best_residual = z.copy(), residual_norm
        if residual_norm <= tolerance * scale:
            return SCInverseResult(z.copy(), residual_norm, iteration, True)
        if iteration == max_iterations:
            break
        jacobian = np.asarray(mapping.jacobian(z), dtype=float)
        try:
            step = np.linalg.solve(jacobian, residual)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(jacobian, residual, rcond=None)[0]
        accepted = False
        damping = 1.0
        for _ in range(30):
            candidate = z - damping * step
            radius = float(np.linalg.norm(candidate))
            if radius >= 1.0:
                candidate *= (1.0 - 1.0e-12) / radius
            candidate_residual = np.asarray(mapping.evaluate(candidate), dtype=float) - target
            if np.linalg.norm(candidate_residual) < residual_norm:
                z = candidate
                accepted = True
                break
            damping *= 0.5
        if not accepted:
            break
    return SCInverseResult(best_z, best_residual, max_iterations, False)


def sc_margin(mapping: SCDiskMap, point: ArrayLike, safe_radius: float, **inverse_kwargs: object) -> tuple[float, SCInverseResult]:
    """Return ``r^2-||Psi^-1(q)||^2`` and the inverse diagnostics."""

    radius = float(safe_radius)
    if not np.isfinite(radius) or not 0.0 < radius < 1.0:
        raise ValueError("safe_radius must lie strictly between zero and one")
    result = inverse_sc_map(mapping, point, **inverse_kwargs)
    return float(radius * radius - result.z @ result.z), result


__all__ = ["SCInverseResult", "inverse_sc_map", "sc_margin"]
