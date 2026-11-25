import numpy as np
import pytest

from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.minco_penalty import ConfiguredMINCOCore
from convex_timevarying_window.comparisons.论文实验.实验2.static_calibration.planner import Objective
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState


def test_configured_minco_integral_and_adjoint_preserve_the_native_objective():
    head = BoundaryState(np.array([0., 0., 1.])).matrix.T
    tail = BoundaryState(np.array([9., 1., 2.])).matrix.T
    points = np.array([[4., -2., 1.]])
    times = np.array([1.813, 1.937])
    original = AnalyticTOGTCore().value_and_gradient(head, tail, points, times)
    configured = ConfiguredMINCOCore(1.).value_and_gradient(head, tail, points, times)
    for actual, expected in zip(configured, original):
        np.testing.assert_allclose(actual, expected, rtol=2e-8, atol=.005)


@pytest.mark.parametrize('center', [False, True])
def test_dynamic_togt_margin_has_the_full_time_and_point_gradient(center):
    track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
    original = Objective(track, center)
    margin = Objective(track, center, dynamic_limit_fraction=.9)
    initial = margin.initial_guess()
    np.testing.assert_array_equal(initial, original.initial_guess())
    assert margin.dimension == original.dimension
    assert margin.config.samples_per_segment == 32
    assert margin.config.dynamic_limits.max_rotor_thrust == .9*8.54858
    np.testing.assert_allclose(margin.config.dynamic_limits.max_body_rate_xy, np.deg2rad(198.))
    value, gradient = margin.value_and_gradient(initial)
    assert np.isfinite(value)
    for column in ([0, 3, 7] if center else [0, 3, 7, 8, 12]):
        step = np.eye(margin.dimension)[column]*1e-7
        numerical = (margin.value_and_gradient(initial+step)[0]-margin.value_and_gradient(initial-step)[0])/(2e-7)
        np.testing.assert_allclose(numerical, gradient[column], rtol=2e-3, atol=.1)
