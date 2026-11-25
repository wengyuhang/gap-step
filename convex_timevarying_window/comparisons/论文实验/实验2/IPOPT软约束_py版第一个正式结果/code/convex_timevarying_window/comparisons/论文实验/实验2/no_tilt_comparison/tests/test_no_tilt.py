import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.native import Penalty
from convex_timevarying_window.comparisons.论文实验.实验2.no_tilt_comparison.report import cell


def test_fifty_degree_tilt_with_feasible_motors_has_zero_penalty():
    acceleration = 9.8066*np.tan(np.deg2rad(50.))
    states = np.zeros((1, 5, 3))
    states[0, 2, 0] = acceleration
    sample = AnalyticTOGTCore().sample_dynamics(states)
    assert sample['tilt'][0] > .78
    assert sample['rotor_thrusts'].max() < 8.54858
    assert sample['instantaneous_penalty'][0] == 0.
    coefficients = np.zeros((1, 8, 3))
    coefficients[0, 2, 0] = .5*acceleration*2.**2
    assert Penalty().evaluate(coefficients, np.array([2.]))[0] == 0.


def test_missing_measured_time_and_collision_are_not_reference_times():
    assert cell(dict(passed=True, outcome='clear', time_s=None), 'time_s') == '缺测†'
    assert cell(dict(passed=False, outcome='collision', time_s=12.), 'time_s') == '碰撞'
