from types import SimpleNamespace

import numpy as np
import pytest
from numpy.polynomial import Polynomial
from shapely.geometry import Polygon

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.dynamics import sample_flatness
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import (
    unit_roots, plane_intervals, obstacle_distances, sphere_check, evaluate_fast, dynamics_check,
)
from nonconvex_timevarying_window.random_dk_sc_dynatogt.search import SearchConfig, generate_candidates, rank_feasible, search


def test_direct_dk_bounds_modes_and_reproducibility():
    center = np.array([0.3, -2.0, 11.0, 1.5])
    config = SearchConfig()
    rows = list(generate_candidates(center, 2, config))
    assert len(rows) == 300
    again = list(generate_candidates(center, 2, config))
    for row, other in zip(rows, again):
        np.testing.assert_array_equal(row["x"], other["x"])
        ds, ks = config.d_scales[row["level"]], config.k_scales[row["level"]]
        np.testing.assert_allclose(row["x"] - center, row["delta"], atol=1e-15)
        assert np.all(abs(row["delta"][:2]) <= ks * np.maximum(1, abs(center[:2])))
        assert np.linalg.norm(row["delta"][2:]) <= ds * np.linalg.norm(center[2:])
        if row["mode"] == "D":
            assert not np.any(row["delta"][:2])
        if row["mode"] == "K":
            assert not np.any(row["delta"][2:])
    assert sum(r["mode"] == "DK" for r in rows) == 150


def test_expanded_scales_keep_center_and_direct_k_parameterization():
    center = np.array([0.3, -2., 743., 100.])
    config = SearchConfig(d_scales=(0.25, 0.5, 1., 2.), k_scales=(0.1, 0.25, 0.5, 1.))
    rows = list(generate_candidates(center, 2, config))
    assert len(rows) == 400
    for row in rows:
        level = row["level"]
        assert np.all(abs(row["delta"][:2]) <= config.k_scales[level] * np.maximum(1, abs(center[:2])))
        assert np.linalg.norm(row["delta"][2:]) <= config.d_scales[level] * np.linalg.norm(center[2:])
        np.testing.assert_allclose(row["x"], center + row["delta"])
    assert any(abs(row["delta"][0]) > 0.5 for row in rows)
    assert any(np.linalg.norm(row["delta"][2:]) > np.linalg.norm(center[2:]) for row in rows)


def test_rejected_faster_candidate_never_wins_and_empty_has_no_fallback():
    rows = [dict(id=0, flight_time=1, screen=dict(passed=False)),
            dict(id=1, flight_time=3, screen=dict(passed=True)),
            dict(id=2, flight_time=2, screen=dict(passed=True))]
    assert [r["id"] for r in rank_feasible(rows)] == [2, 1]
    assert rank_feasible(rows[:1]) == []


def test_search_only_triggers_after_failure_and_evaluates_every_candidate():
    calls = []
    def forward(x):
        calls.append(x.copy())
        return SimpleNamespace(trajectory=SimpleNamespace(total_time=float(2 + x[0])))
    objective = SimpleNamespace(forward=forward)
    scene = SimpleNamespace(windows=(object(),))
    center = np.zeros(4)
    config = SearchConfig(per_scale=4)
    rows, ranked = search(objective, center, scene, None, config,
                          screen=lambda *_: dict(passed=True))
    assert len(calls) == len(rows) == len(ranked) == 1
    calls.clear()
    rows, ranked = search(objective, center, scene, None, config,
                          screen=lambda *_: dict(passed=False))
    assert len(calls) == len(rows) == 13
    assert ranked == []


def test_polynomial_roots_include_tangency_and_endpoints():
    poly = Polynomial.fromroots([0, 0.25, 0.25, 0.8, 1])
    np.testing.assert_allclose(unit_roots(poly.coef), [0, 0.25, 0.8, 1], atol=1e-8)
    with pytest.raises(ValueError):
        unit_roots(np.zeros(8))


class PolynomialTrajectory:
    def __init__(self, z, y=0):
        self.coefficients = np.zeros((1, 8, 3))
        self.coefficients[0, :len(z), 0] = z
        self.coefficients[0, 0, 1] = y
        self.durations = np.array([1.0])
        self.total_time = 1.0

    def evaluate_segment(self, segment, t, derivative=0):
        return np.stack([Polynomial(self.coefficients[segment, :, k]).deriv(derivative)(t) for k in range(3)], axis=-1)


def window():
    return SimpleNamespace(thickness=0, normal=np.array([1., 0, 0]), center=np.zeros(3),
                           plane_basis=np.array([[0., 0], [1, 0], [0, 1]]), theta0=0, omega=0,
                           physical_polygon=np.array([[-1., -1], [1, -1], [1, 1], [-1, 1]]))


def test_all_disconnected_contact_intervals_are_found():
    z = Polynomial.fromroots([0.2, 0.5, 0.8]) * 30
    trajectory = PolynomialTrajectory(z.coef)
    intervals, roots, _ = plane_intervals(trajectory, window(), 0.05)
    assert len(intervals) == 3
    np.testing.assert_allclose(roots, [0.2, 0.5, 0.8], atol=1e-9)
    for t in np.linspace(0, 1, 10001):
        if abs(z(t)) < 0.05:
            assert any(a <= t <= b for a, b in intervals)


def test_solid_exterior_distance_is_zero_in_u_notch():
    polygon = Polygon([[-2, -2], [2, -2], [2, 2], [1, 2], [1, -1], [-1, -1], [-1, 2], [-2, 2]])
    distances = obstacle_distances(polygon, [[0, 0], [-1.5, 0], [5, 0], [1, 0]])
    np.testing.assert_allclose(distances, [0, 0.5, 0, 0])


def test_sphere_rejects_outside_aperture_and_accepts_center():
    passing = sphere_check(PolynomialTrajectory([-2, 4]), window(), 0.2)
    failing = sphere_check(PolynomialTrajectory([-2, 4], y=2), window(), 0.2)
    assert passing["passed"] and passing["maximum_dense_step"] <= 0.0002 + 1e-12
    assert not failing["passed"]
    with pytest.raises(ValueError):
        bad_window = window()
        bad_window.thickness = 0.1
        sphere_check(PolynomialTrajectory([-2, 4]), bad_window, 0.2)


def test_vectorized_polynomial_evaluation_matches_native_and_dynamics():
    trajectory = MincoSnap(BoundaryState([-1., 0, 1]), BoundaryState([1., 0, 1]),
                          np.array([[0., 0.1, 1]]), np.array([2., 2.]))
    grid = np.linspace(0, 4, 51)
    for derivative in range(5):
        np.testing.assert_allclose(evaluate_fast(trajectory, grid, derivative), trajectory.evaluate(grid, derivative), atol=1e-11)
    config = OptimizationConfig()
    result = dynamics_check(trajectory, config, dt=0.02)
    flat = sample_flatness(trajectory, np.linspace(0, 4, 201), parameters=config.quadrotor)
    limits = config.dynamic_limits
    expected = bool(np.max(np.linalg.norm(flat.body_rate[:, :2], axis=1)) <= limits.max_body_rate_xy
                    and np.max(abs(flat.body_rate[:, 2])) <= limits.max_body_rate_z
                    and np.min(flat.rotor_thrusts) >= limits.min_rotor_thrust
                    and np.max(flat.rotor_thrusts) <= limits.max_rotor_thrust)
    assert result["passed"] == expected


def test_rotating_coordinates_use_absolute_window_time():
    from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import sphere_margins
    w = window()
    w.omega = np.pi / 2
    polygon = Polygon([[-2, -0.5], [2, -0.5], [2, 0.5], [-2, 0.5]])
    # The world point y=1 is inside the wide aperture at t=0 and outside at t=1.
    trajectory = PolynomialTrajectory([0.0], y=1)
    margins = sphere_margins(trajectory, w, polygon, np.array([0., 1.]), 0.2)
    np.testing.assert_allclose(margins, [0.3, -0.2], atol=1e-12)


def test_full_screen_and_final_body_audit_on_feasible_nominal_trajectory():
    from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import screen_candidate
    from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import final_audit
    from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
    start, goal = BoundaryState([-1., 0, 0]), BoundaryState([1., 0, 0])
    trajectory = MincoSnap(start, goal, np.array([[0., 0, 0]]), np.array([2., 2.]))
    w = window()
    w.name = "square"
    w.rho = 0.2
    w.safe_polygon = w.physical_polygon * 0.8
    w.rotated_basis = lambda _t: w.plane_basis
    w.world_point = lambda q, _t: w.center + w.plane_basis @ q
    forward = SimpleNamespace(trajectory=trajectory, crossing_times=np.array([2.]),
                              local_points=np.zeros((1, 2)), crossing_local_index=0)
    scenario = SimpleNamespace(windows=(w,), start_state=start, goal_state=goal,
                               body=CuboidBody(half_extents=(0.08, 0.08, 0.04)))
    config = OptimizationConfig()
    screen = screen_candidate(forward, scenario, config)
    assert screen["passed"]
    audit, _ = final_audit(scenario, forward, config)
    assert audit["trajectory_validation_pass"]
    assert audit["solid_exterior_violating_samples"] == 0
