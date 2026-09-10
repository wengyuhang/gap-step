import numpy as np

from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.safety_penalty import (
    boundary_point_cloud,
    smooth_squared_distance,
)
from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.search import rank_dual_constraints


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
