from types import SimpleNamespace

import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation

from convex_timevarying_window.togt_style_mpc import model
from convex_timevarying_window.togt_style_mpc.flatness import maps
from convex_timevarying_window.tilt_heading_minco import dynamics
from convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix.reference import MPC
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.x500_dynamics import (
    instantaneous, Penalty, RESIDUAL_SCALE)
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.symbolic_penalty import positive_penalty
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario


def test_shared_flatness_reproduces_the_existing_execution_reference():
    rng = np.random.default_rng(72)
    aero = model.functions()[2]
    for values in rng.normal(size=(12, 4, 3))*np.array([8., 4., 10., 20.])[None, :, None]:
        rotation, force, omega, alpha, rotors = maps()(*values)
        old_rotation, old_force, _ = dynamics.orientation(*values[:2])
        old_omega, old_alpha = dynamics.rates(*values)
        np.testing.assert_allclose(rotation, old_rotation, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(force, old_force, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(omega, old_omega, rtol=1e-10, atol=1e-10)
        rate, acceleration = np.asarray(old_omega).ravel(), np.asarray(old_alpha).ravel()
        quaternion = Rotation.from_matrix(np.asarray(old_rotation)).as_quat()[[3, 0, 1, 2]]
        state = np.r_[np.zeros(3), values[0], quaternion, rate]
        torque = model.X500_INERTIA*acceleration+np.cross(rate, model.X500_INERTIA*rate)
        torque -= np.asarray(aero(state, np.full(4, float(old_force)/4.))).ravel()
        expected = np.linalg.inv(model.MIXER)@np.r_[float(old_force), torque]
        np.testing.assert_allclose(np.asarray(rotors).ravel(), expected, rtol=1e-10, atol=1e-10)


def test_planning_and_mpc_use_identical_x500_motor_demands_and_physical_limits():
    rng = np.random.default_rng(89)
    values = rng.normal(size=(5, 15))
    values[:, 3:6] *= 12.
    values[:, 6:9] *= 4.
    tracker = MPC.__new__(MPC)
    tracker.cfg = SimpleNamespace(steps=4, dt=.04)
    tracker.flatness_map = maps().map(5)
    states, demands = tracker.references(lambda times: values, .4, np.array([1., 0., 0., 0.]))
    for item, demand, state in zip(values, demands, states):
        cost, omega, rotors, residuals = instantaneous()(item[3:6], item[6:9], item[9:12], item[12:15])
        np.testing.assert_allclose(np.asarray(rotors).ravel(), demand, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(np.asarray(omega).ravel(), state[10:13], rtol=1e-12, atol=1e-12)
        expected = 1000.*sum(float(positive_penalty(float(value))) for value in np.asarray(residuals).ravel())
        np.testing.assert_allclose(float(cost), expected, rtol=1e-12)
    assert model.FMAX == 8.54858
    np.testing.assert_allclose(model.FMIN, model.KF*150.**2)
    np.testing.assert_allclose(RESIDUAL_SCALE[:2], [3.8**2, 3.49**2])


def test_x500_full_trajectory_cost_and_gradient_match_the_lifted_integral(monkeypatch):
    monkeypatch.setattr(kernels, 'local_options', lambda name: {'cse': True, 'never_inline': True})
    for function in (kernels.node_residuals, kernels.dynamic_residuals, kernels.node_penalty, kernels.slack_integral):
        function.cache_clear()
    track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
    planner = Planner(track, solver='ipopt', dynamics_model='x500', pose_function=scenario.pose)
    initial = planner.seeds()[1]
    value, gradient, penalty = planner.objective(initial)
    variable = ca.MX.sym('x500_objective', planner.nvar)
    from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.symbolic_penalty import objective
    expression = objective(planner, variable)
    evaluate = ca.Function('x500_objective', [variable], [expression, ca.gradient(expression, variable)])
    expected, derivative = evaluate(initial)
    np.testing.assert_allclose(value, expected, rtol=1e-11, atol=1e-7)
    np.testing.assert_allclose(gradient, np.asarray(derivative).ravel(), rtol=1e-9, atol=1e-5)
    assert penalty > 0.
    direction = np.random.default_rng(41).normal(size=planner.nvar)
    step = 1e-6
    numerical = (planner.objective(initial+step*direction)[0]-planner.objective(initial-step*direction)[0])/(2*step)
    np.testing.assert_allclose(numerical, gradient@direction, rtol=1e-4, atol=.1)
