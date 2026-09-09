"""Sampled sphere-to-solid-plane-aperture checks, restricted to |z| <= r_s.

The obstacle is the closed complement of the physical aperture. This is not
the legacy zero-width boundary curtain, and outside-aperture distance is zero.
Polynomial roots and all subsequent checks are numerical, not certificates.
"""

from __future__ import annotations

import numpy as np
from numpy.polynomial import Polynomial
from scipy.optimize import brentq
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.dynamics import flatness_map


def unit_roots(coefficients):
    """Find real roots on [0,1] using derivative partitions, including tangencies."""
    coefficients = np.asarray(coefficients, dtype=float)
    if not np.all(np.isfinite(coefficients)):
        raise ValueError("nonfinite polynomial")
    scale = np.max(np.abs(coefficients))
    if scale < 1e-14:
        raise ValueError("unresolved polynomial identically on a contact plane")
    poly = Polynomial(coefficients / scale).trim(tol=1e-14)
    if poly.degree() == 0:
        return []
    critical = unit_roots(poly.deriv().coef) if poly.degree() > 1 else []
    cuts = np.unique([0.0, *critical, 1.0])
    roots = [float(t) for t in cuts if abs(poly(t)) <= 1e-11]
    for a, b in zip(cuts[:-1], cuts[1:]):
        if poly(a) * poly(b) < 0:
            roots.append(float(brentq(poly, a, b, xtol=1e-14)))
    roots.sort()
    return [v for i, v in enumerate(roots) if i == 0 or v - roots[i - 1] > 1e-9]


def plane_intervals(trajectory, window, radius):
    if window.thickness != 0:
        raise ValueError("sphere formula requires zero thickness")
    intervals, crossings, contacts = [], [], []
    offset = 0.0
    for coefficients, duration in zip(trajectory.coefficients, trajectory.durations):
        z = coefficients @ window.normal
        z[0] -= window.center @ window.normal
        z = z * float(duration) ** np.arange(len(z))
        crossings.extend(offset + duration * t for t in unit_roots(z))
        cuts = [0.0, 1.0]
        for height in (-radius, radius):
            shifted = z.copy()
            shifted[0] -= height
            cuts.extend(unit_roots(shifted))
        cuts = np.unique(cuts)
        contacts.extend(offset + duration * cuts)
        for a, b in zip(cuts[:-1], cuts[1:]):
            if abs(Polynomial(z)((a + b) / 2)) <= radius + 1e-10:
                intervals.append((float(offset + duration * a), float(offset + duration * b)))
        offset += duration
    crossings.sort()
    crossings = [float(v) for i, v in enumerate(crossings)
                 if i == 0 or v - crossings[i - 1] > 1e-7]
    return intervals, crossings, contacts


def evaluate_fast(trajectory, times, derivative=0):
    times = np.asarray(times, dtype=float)
    knots = np.r_[0.0, np.cumsum(trajectory.durations)]
    result = np.empty((len(times), 3))
    indices = np.minimum(np.searchsorted(knots[1:], times, side="right"), len(knots) - 2)
    for segment in np.unique(indices):
        mask = indices == segment
        result[mask] = trajectory.evaluate_segment(int(segment), times[mask] - knots[segment], derivative)
    return result


def obstacle_distances(polygon, points):
    return np.asarray([Point(q).distance(polygon.boundary) if polygon.contains(Point(q)) else 0.0
                       for q in points])


def sphere_margins(trajectory, window, polygon, grid, radius):
    positions = evaluate_fast(trajectory, grid) - window.center
    base = positions @ window.plane_basis
    theta = window.theta0 + window.omega * grid
    c, s = np.cos(theta), np.sin(theta)
    q = np.column_stack((c * base[:, 0] + s * base[:, 1], -s * base[:, 0] + c * base[:, 1]))
    z = positions @ window.normal
    distance = obstacle_distances(polygon, q)
    return np.sqrt(z * z + distance * distance) - radius


def sphere_check(trajectory, window, radius, *, dt=0.0002, fine_dt=0.00005):
    intervals, crossings, contacts = plane_intervals(trajectory, window, radius)
    polygon = Polygon(window.physical_polygon)
    count, minimum, actual_dt = 0, float("inf"), 0.0
    stage = "coarse"
    for step in (min(0.002, np.deg2rad(1) / max(abs(window.omega), 1e-12)), dt):
        for a, b in intervals:
            grid = np.unique(np.r_[np.linspace(a, b, max(2, int(np.ceil((b - a) / step)) + 1)),
                                   [t for t in crossings + list(contacts) if a <= t <= b]])
            margins = sphere_margins(trajectory, window, polygon, grid, radius)
            if not np.all(np.isfinite(margins)):
                raise ValueError("nonfinite sphere distances")
            count += len(grid)
            minimum = min(minimum, float(np.min(margins)))
            if minimum <= 1e-9:
                return dict(passed=False, status="VIOLATED" if minimum < -1e-9 else "UNRESOLVED",
                            minimum_margin=minimum, samples=count, stage=stage,
                            intervals=intervals, crossings=crossings)
            if stage == "dense":
                actual_dt = max(actual_dt, float(np.max(np.diff(grid))))
                critical = margins < 0.005
                critical[0] = critical[-1] = True
                critical[1:-1] |= (margins[1:-1] <= margins[:-2]) & (margins[1:-1] <= margins[2:])
                for i in np.flatnonzero(critical[:-1] | critical[1:]):
                    extra = np.linspace(grid[i], grid[i + 1], max(2, int(np.ceil((grid[i + 1] - grid[i]) / fine_dt)) + 1))
                    values = sphere_margins(trajectory, window, polygon, extra, radius)
                    if not np.all(np.isfinite(values)):
                        raise ValueError("nonfinite refined distances")
                    count += len(extra)
                    minimum = min(minimum, float(np.min(values)))
                    if minimum <= 1e-9:
                        return dict(passed=False, status="VIOLATED" if minimum < -1e-9 else "UNRESOLVED",
                                    minimum_margin=minimum, samples=count, stage="refined",
                                    intervals=intervals, crossings=crossings)
        stage = "dense"
    return dict(passed=True, status="SAMPLED_PASS", minimum_margin=minimum, samples=count,
                maximum_dense_step=actual_dt, intervals=intervals, crossings=crossings)


def dynamics_check(trajectory, config, *, dt=0.001):
    """Full-flight native SC dynamics; short-circuit at the first violated sample."""
    grid = np.unique(np.r_[np.linspace(0, trajectory.total_time, int(np.ceil(trajectory.total_time / dt)) + 1),
                           np.cumsum(trajectory.durations)])
    derivatives = [evaluate_fast(trajectory, grid, d) for d in range(1, 5)]
    if not all(np.all(np.isfinite(v)) for v in derivatives):
        raise ValueError("nonfinite trajectory derivatives")
    speed = np.linalg.norm(derivatives[0], axis=1)
    limits = config.dynamic_limits
    if np.max(speed) > limits.max_velocity + 1e-9:
        return dict(passed=False, reason="velocity", max_velocity=float(np.max(speed)), samples=len(grid))
    for i, (acceleration, jerk, snap) in enumerate(zip(*derivatives[1:])):
        flat = flatness_map(acceleration, jerk, snap, parameters=config.quadrotor)
        checks = {
            "collective_thrust": limits.min_collective_thrust - 1e-9 <= flat.collective_thrust <= limits.max_collective_thrust + 1e-9,
            "body_rate_xy": np.linalg.norm(flat.body_rate[:2]) <= limits.max_body_rate_xy + 1e-9,
            "body_rate_z": abs(flat.body_rate[2]) <= limits.max_body_rate_z + 1e-9,
            "rotor_thrust": np.min(flat.rotor_thrusts) >= limits.min_rotor_thrust - 1e-9 and np.max(flat.rotor_thrusts) <= limits.max_rotor_thrust + 1e-9,
        }
        if not np.all(np.isfinite(np.r_[flat.collective_thrust, flat.body_rate, flat.rotor_thrusts])):
            raise ValueError("nonfinite flatness state")
        if not all(checks.values()):
            return dict(passed=False, reason=next(k for k, v in checks.items() if not v),
                        first_violation_time=float(grid[i]), samples=i + 1,
                        max_velocity=float(np.max(speed)))
    return dict(passed=True, max_velocity=float(np.max(speed)), samples=len(grid),
                maximum_step=float(np.max(np.diff(grid))))


def screen_candidate(forward, scenario, config):
    trajectory = forward.trajectory
    count = len(scenario.windows)
    if np.shape(forward.local_points) != (count, 2) or np.shape(forward.crossing_times) != (count,):
        return dict(passed=False, reason="window_dimension_mismatch")
    errors = []
    for t, state in ((0.0, scenario.start_state), (trajectory.total_time, scenario.goal_state)):
        errors.append(float(np.max(np.abs(np.stack([trajectory.evaluate(t, d) for d in range(4)]) - state.matrix))))
    for i, duration in enumerate(trajectory.durations[:-1]):
        errors.append(max(float(np.max(np.abs(trajectory.evaluate_segment(i, duration, d)
                                             - trajectory.evaluate_segment(i + 1, 0, d)))) for d in range(4)))
    if not np.all(np.isfinite(errors)) or max(errors) > 1e-8:
        return dict(passed=False, reason="boundary_or_interface", maximum_error=max(errors))
    spheres = []
    previous = -np.inf
    for i, window in enumerate(scenario.windows):
        if not Polygon(window.safe_polygon).buffer(1e-9).covers(Point(forward.local_points[i])):
            return dict(passed=False, reason="sc_membership",
                        window_index=i, spheres=spheres,
                        outside_distance=float(Point(forward.local_points[i]).distance(Polygon(window.safe_polygon))))
        crossing = forward.crossing_times[i]
        if np.linalg.norm(trajectory.evaluate(crossing) - window.world_point(forward.local_points[i], crossing)) > 1e-8:
            return dict(passed=False, reason="crossing_position", window_index=i, spheres=spheres)
        result = sphere_check(trajectory, window, window.rho)
        spheres.append(result)
        if not result["passed"]:
            return dict(passed=False, reason="sphere_" + result["status"].lower(), window_index=i, spheres=spheres)
        roots = result["crossings"]
        if len(roots) != 1 or abs(roots[0] - crossing) > 1e-7 or roots[0] <= previous:
            return dict(passed=False, reason="crossing_order_or_count", window_index=i, spheres=spheres)
        previous = roots[0]
    dynamics = dynamics_check(trajectory, config)
    return dict(passed=dynamics["passed"], reason="pass" if dynamics["passed"] else "dynamics_" + dynamics["reason"],
                spheres=spheres, dynamics=dynamics, maximum_boundary_error=max(errors))
