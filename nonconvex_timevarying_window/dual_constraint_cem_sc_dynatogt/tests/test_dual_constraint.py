import numpy as np
from types import SimpleNamespace

from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.safety_penalty import (
    SafetyPenaltyConfig,
    boundary_point_cloud,
    integrated_safety_penalty,
    safety_penalty_with_gradient,
    smooth_squared_distance,
)
from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.search import rank_dual_constraints
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap


def test_boundary_cloud_contains_straight_edge_interior():
    cloud = boundary_point_cloud(np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.]]), .1)
    assert np.min(np.linalg.norm(cloud - np.array([.5, 0.]), axis=1)) < 1e-12


def test_soft_min_is_nonnegative_and_close_to_true_minimum():
    boundary = np.array([[0., 0.], [2., 0.]])
    point = np.array([[1., 1.]])
    value = smooth_squared_distance(point, boundary, 1e-4)[0]
    assert value >= 0.0
    assert value == 2.0


def test_parallel_constraints_form_pareto_layers_without_epsilon():
    rows = [
        {"id": 0, "dynamic_integral": 0., "safety_integral": 2., "augmented_objective": 3., "flight_time": 1.},
        {"id": 1, "dynamic_integral": 2., "safety_integral": 0., "augmented_objective": 3., "flight_time": 1.},
        {"id": 2, "dynamic_integral": 2., "safety_integral": 2., "augmented_objective": 5., "flight_time": 1.},
    ]
    ranked = rank_dual_constraints(rows)
    assert {ranked[0]["id"], ranked[1]["id"]} == {0, 1}
    assert rows[2]["pareto_front"] == 1


def test_safety_integral_autodiff_matches_finite_difference():
    start = BoundaryState(np.array([-1.0, 0.88, 0.0]))
    finish = BoundaryState(np.array([1.0, 0.88, 0.0]))
    waypoint = np.array([[0.0, 0.88, 0.0]])
    durations = np.array([0.7, 0.8])
    window = SimpleNamespace(
        center=np.zeros(3),
        plane_basis=np.array([[0., 0.], [1., 0.], [0., 1.]]),
        normal=np.array([1., 0., 0.]),
        theta0=0.0,
        omega=0.4,
        rho=0.4,
        physical_polygon=np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]),
    )
    config = SafetyPenaltyConfig(edge_spacing_m=0.1, min_num_check=8,
                                 max_num_check=8)
    trajectory = MincoSnap(start, finish, waypoint, durations)
    value, point_gradient, duration_gradient = safety_penalty_with_gradient(
        trajectory, (window,), config
    )
    np.testing.assert_allclose(
        value, integrated_safety_penalty(trajectory, (window,), config),
        rtol=1e-12, atol=1e-12,
    )
    assert value > 0.0
    assert np.all(np.isfinite(point_gradient))
    assert np.all(np.isfinite(duration_gradient))

    def cost(points, times):
        candidate = MincoSnap(start, finish, points, times)
        return safety_penalty_with_gradient(candidate, (window,), config)[0]

    step = 1.0e-6
    plus, minus = waypoint.copy(), waypoint.copy()
    plus[0, 1] += step
    minus[0, 1] -= step
    finite_point = (cost(plus, durations) - cost(minus, durations)) / (2 * step)
    np.testing.assert_allclose(point_gradient[0, 1], finite_point, rtol=2e-4, atol=2e-6)

    plus_time, minus_time = durations.copy(), durations.copy()
    plus_time[0] += step
    minus_time[0] -= step
    finite_time = (cost(waypoint, plus_time) - cost(waypoint, minus_time)) / (2 * step)
    np.testing.assert_allclose(duration_gradient[0], finite_time, rtol=2e-4, atol=2e-6)
