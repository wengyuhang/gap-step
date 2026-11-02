import numpy as np
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.point_migration.run import point_cost, solve_point
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.scenario import build_case, VERTICES
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap


def test_physical_cost_gradient_and_analytic_optimum():
    track = build_case()
    q = np.array([-.8, .1])
    _, gradient = point_cost(q, track, 1.2)
    finite = np.array([(point_cost(q+np.eye(2)[k]*1e-6, track, 1.2)[0]
                        -point_cost(q-np.eye(2)[k]*1e-6, track, 1.2)[0])/2e-6
                       for k in range(2)])
    np.testing.assert_allclose(gradient, finite, atol=2e-8)
    optimum = np.array([-track.windows[0].state_at(1.2)[0][0], 0.])
    assert Polygon(VERTICES).contains(Point(optimum))
    assert optimum[0] > 0
    value, grad = point_cost(optimum, track, 1.2)
    assert abs(value-12.) < 1e-12
    np.testing.assert_allclose(grad, 0, atol=1e-12)


def test_single_left_start_can_migrate_without_body_safety_claim():
    result = solve_point(SCDiskMap.fit(VERTICES), build_case(), 1.2)
    np.testing.assert_allclose(result['physical_initial_q'], [-1., 0.], atol=1e-7)
    assert result['optimizer_success']
    assert result['final_room'] == 'right'
    assert abs(result['cost_gap_m']) < 1e-8
    assert result['physical_evaluations_outside'] == 0
    assert result['sampled_parameter_path_outside'] == 0
    assert result['sampled_parameter_path_has_body_infeasible_points']
    assert not result['full_planner_safety_verified']
    assert not result['flight_executed']
