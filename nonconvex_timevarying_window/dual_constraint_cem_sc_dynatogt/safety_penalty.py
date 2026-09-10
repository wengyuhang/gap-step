"""Smooth boundary-frame distance penalty integrated over trajectory time.

The physical curve is represented by the preprocessing polyline, whose curved
pieces have at most 1 mm chord error and 1 cm chords.  Straight polygon edges
are additionally sampled at the same 1 cm spacing.  A Gibbs-weighted smooth
minimum of squared point distances is nonnegative and differentiable.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

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


def _local_coordinates(positions, window, grid):
    positions = np.asarray(positions) - window.center
    base = positions @ window.plane_basis
    theta = window.theta0 + window.omega * grid
    c, s = np.cos(theta), np.sin(theta)
    q = np.column_stack((c * base[:, 0] + s * base[:, 1],
                         -s * base[:, 0] + c * base[:, 1]))
    return q, positions @ window.normal


def integrated_window_safety_penalty(trajectory, window, config=SafetyPenaltyConfig(), *, radius=None):
    """Directly time-integrate ``smoothedL1(rho^2-z^2-d_soft^2)``."""
    sphere_radius = window.rho if radius is None else float(radius)
    cloud = boundary_point_cloud(window.physical_polygon, config.edge_spacing_m)
    total = 0.0
    nodes = 0
    elapsed = 0.0
    for segment, duration in enumerate(np.asarray(trajectory.durations, dtype=float)):
        count = int(duration / config.check_time_sec)
        count = min(max(count, config.min_num_check), config.max_num_check)
        local_grid = np.linspace(0.0, duration, count + 1)
        grid = elapsed + local_grid
        positions = trajectory.evaluate_segment(segment, local_grid, 0)
        q, z = _local_coordinates(positions, window, grid)
        distance_squared = smooth_squared_distance(
            q, cloud, config.softmin_temperature_m2)
        residual = sphere_radius ** 2 - z * z - distance_squared
        values = np.asarray(smoothed_l1(residual, config.smoothing_epsilon), dtype=float)
        weights = np.ones(count + 1)
        weights[[0, -1]] = 0.5
        total += float((duration / count) * np.dot(weights, values))
        nodes += count + 1
        elapsed += duration
    return {"integral": total, "segment_count": trajectory.num_segments, "nodes": nodes,
            "boundary_points": len(cloud)}


def integrated_safety_penalty(trajectory, windows, config=SafetyPenaltyConfig(), *, radius=None,
                              return_breakdown=False):
    per_window = [integrated_window_safety_penalty(trajectory, w, config, radius=radius)
                  for w in windows]
    total = float(sum(row["integral"] for row in per_window))
    if return_breakdown:
        return {"total": total, "per_window": per_window}
    return total


def safety_penalty_with_gradient(
    trajectory,
    windows,
    config=SafetyPenaltyConfig(),
    *,
    radius=None,
    boundary_clouds=None,
):
    """Differentiate a direct time integral of the safety soft constraint.

    Every window is evaluated at every trapezoidal node of every MINCO
    segment, exactly like TOGT's dynamic soft-constraint integral.  This
    objective does not find contact intervals or run a collision detector.

    Returns ``(value, d_waypoints, d_durations)``.  Window motion through
    absolute time is included in ``d_durations``.
    """
    torch, points, durations, coefficients = trajectory._torch_parameterization()
    dtype = durations.dtype
    device = durations.device
    total = durations.new_zeros(())
    elapsed = durations.new_zeros(())

    if boundary_clouds is None:
        boundary_clouds = tuple(
            boundary_point_cloud(window.physical_polygon, config.edge_spacing_m)
            for window in windows
        )
    if len(boundary_clouds) != len(windows):
        raise ValueError("boundary_clouds must contain one cloud per window")

    prepared = []
    for window, cloud in zip(windows, boundary_clouds):
        prepared.append((
            window,
            torch.as_tensor(np.asarray(window.center), dtype=dtype, device=device),
            torch.as_tensor(np.asarray(window.plane_basis), dtype=dtype, device=device),
            torch.as_tensor(np.asarray(window.normal), dtype=dtype, device=device),
            torch.as_tensor(cloud, dtype=dtype, device=device),
            float(window.rho if radius is None else radius),
        ))

    for segment, real_duration in enumerate(np.asarray(trajectory.durations, dtype=float)):
        count = int(real_duration / config.check_time_sec)
        count = min(max(count, config.min_num_check), config.max_num_check)
        fractions = torch.linspace(0.0, 1.0, count + 1, dtype=dtype, device=device)
        local_time = durations[segment] * fractions
        powers = torch.stack(tuple(local_time ** power for power in range(8)), dim=1)
        positions = powers @ coefficients[segment]
        absolute_time = elapsed + local_time
        trapezoid = torch.ones(count + 1, dtype=dtype, device=device)
        trapezoid[0] = trapezoid[-1] = 0.5
        step = durations[segment] / count

        for window, center, basis, normal, cloud, sphere_radius in prepared:
                relative = positions - center
                base = relative @ basis
                theta = float(window.theta0) + float(window.omega) * absolute_time
                cosine, sine = torch.cos(theta), torch.sin(theta)
                local = torch.stack((
                    cosine * base[:, 0] + sine * base[:, 1],
                    -sine * base[:, 0] + cosine * base[:, 1],
                ), dim=1)
                squared = ((local[:, None, :] - cloud[None, :, :]) ** 2).sum(dim=2)
                # The common stabilizing exponential factor cancels from the
                # Gibbs numerator and denominator.
                minimum = squared.min(dim=1).values.detach()
                weights = torch.exp(-(squared - minimum[:, None]) /
                                    config.softmin_temperature_m2)
                distance_squared = ((weights * squared).sum(dim=1) /
                                    weights.sum(dim=1))
                plane_distance = relative @ normal
                residual = sphere_radius ** 2 - plane_distance ** 2 - distance_squared
                mu = config.smoothing_epsilon
                transition = (mu - 0.5 * residual) * (residual / mu) ** 3
                penalty = torch.where(
                    residual < 0.0,
                    torch.zeros_like(residual),
                    torch.where(residual > mu, residual - 0.5 * mu, transition),
                )
                total = total + step * (trapezoid * penalty).sum()
        elapsed = elapsed + durations[segment]

    point_gradient, duration_gradient = torch.autograd.grad(
        total, (points, durations), allow_unused=True
    )
    if point_gradient is None:
        point_array = np.zeros_like(trajectory.intermediate_points, dtype=float)
    else:
        point_array = point_gradient.detach().cpu().numpy()
    if duration_gradient is None:
        duration_array = np.zeros_like(trajectory.durations, dtype=float)
    else:
        duration_array = duration_gradient.detach().cpu().numpy()
    return float(total.detach().cpu()), point_array, duration_array


__all__ = ["SafetyPenaltyConfig", "boundary_point_cloud", "smooth_squared_distance",
           "integrated_window_safety_penalty", "integrated_safety_penalty",
           "safety_penalty_with_gradient"]
