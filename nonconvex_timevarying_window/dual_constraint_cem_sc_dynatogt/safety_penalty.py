"""Smooth boundary-frame distance penalty on the crossing intervals.

The physical curve is represented by the preprocessing polyline, whose curved
pieces have at most 1 mm chord error and 1 cm chords.  Straight polygon edges
are additionally sampled at the same 1 cm spacing.  A Gibbs-weighted smooth
minimum of squared point distances is nonnegative and differentiable.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import (
    evaluate_fast,
    plane_intervals,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import smoothed_l1


@dataclass(frozen=True)
class SafetyPenaltyConfig:
    softmin_temperature_m2: float = 1.0e-4
    edge_spacing_m: float = 0.01
    check_time_sec: float = 0.05
    min_num_check: int = 8
    max_num_check: int = 32
    smoothing_epsilon: float = 0.01
    # Safety residuals have units of m^2 and are several orders smaller than
    # TOGT's squared thrust/rate residuals on this model.  This fixed scaling
    # lets one Pareto front meaningfully approach both axes; it is not a
    # feasibility epsilon and does not change the exact-zero eligibility rule.
    objective_weight: float = 100000.0

    def __post_init__(self):
        if self.softmin_temperature_m2 <= 0 or self.edge_spacing_m <= 0:
            raise ValueError("softmin temperature and edge spacing must be positive")
        if self.check_time_sec <= 0 or self.min_num_check < 1:
            raise ValueError("invalid quadrature configuration")
        if self.max_num_check < self.min_num_check or self.objective_weight < 0:
            raise ValueError("invalid maximum quadrature count or objective weight")


def boundary_point_cloud(vertices, spacing):
    """Densify every closed-polyline edge, including exact straight edges."""
    p = np.asarray(vertices, dtype=float)
    chunks = []
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        count = max(1, int(np.ceil(np.linalg.norm(b - a) / spacing)))
        chunks.append(a[None, :] + np.arange(count)[:, None] / count * (b - a)[None, :])
    return np.concatenate(chunks, axis=0)


def smooth_squared_distance(points, boundary_points, temperature):
    """Return the nonnegative Gibbs-weighted smooth squared distance."""
    q = np.asarray(points)
    b = np.asarray(boundary_points)
    squared = np.sum((q[:, None, :] - b[None, :, :]) ** 2, axis=2)
    minimum = np.min(np.real(squared), axis=1)
    shifted = squared - minimum[:, None]
    weights = np.exp(-shifted / temperature)
    return np.sum(weights * squared, axis=1) / np.sum(weights, axis=1)


def _local_coordinates(trajectory, window, grid):
    positions = evaluate_fast(trajectory, grid) - window.center
    base = positions @ window.plane_basis
    theta = window.theta0 + window.omega * grid
    c, s = np.cos(theta), np.sin(theta)
    q = np.column_stack((c * base[:, 0] + s * base[:, 1],
                         -s * base[:, 0] + c * base[:, 1]))
    return q, positions @ window.normal


def integrated_window_safety_penalty(trajectory, window, config=SafetyPenaltyConfig(), *, radius=None):
    """Integrate ``smoothedL1(rho^2-z^2-d_soft^2)`` on contact intervals."""
    sphere_radius = window.rho if radius is None else float(radius)
    intervals, _, _ = plane_intervals(trajectory, window, sphere_radius)
    cloud = boundary_point_cloud(window.physical_polygon, config.edge_spacing_m)
    total = 0.0
    nodes = 0
    for a, b in intervals:
        duration = b - a
        count = int(duration / config.check_time_sec)
        count = min(max(count, config.min_num_check), config.max_num_check)
        grid = np.linspace(a, b, count + 1)
        q, z = _local_coordinates(trajectory, window, grid)
        distance_squared = smooth_squared_distance(
            q, cloud, config.softmin_temperature_m2)
        residual = sphere_radius ** 2 - z * z - distance_squared
        values = np.asarray(smoothed_l1(residual, config.smoothing_epsilon), dtype=float)
        weights = np.ones(count + 1)
        weights[[0, -1]] = 0.5
        total += float((duration / count) * np.dot(weights, values))
        nodes += count + 1
    return {"integral": total, "interval_count": len(intervals), "nodes": nodes,
            "boundary_points": len(cloud)}


def integrated_safety_penalty(trajectory, windows, config=SafetyPenaltyConfig(), *, radius=None,
                              return_breakdown=False):
    per_window = [integrated_window_safety_penalty(trajectory, w, config, radius=radius)
                  for w in windows]
    total = float(sum(row["integral"] for row in per_window))
    if return_breakdown:
        return {"total": total, "per_window": per_window}
    return total


__all__ = ["SafetyPenaltyConfig", "boundary_point_cloud", "smooth_squared_distance",
           "integrated_window_safety_penalty", "integrated_safety_penalty"]
