import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import rotor_targets, dynamics
from convex_timevarying_window.dynamic_passage_velocity_minco.repaired_planner import Planner, Config
from convex_timevarying_window.checked_soft_mpc_racing.mpc import KF, MASS, G, UPPER
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case


def test_allocation_couples_turning_and_max_collective():
    torque = ca.MX.sym('torque', 3)
    f = ca.Function('targets_test', [torque], [rotor_targets(UPPER[0], torque)])
    straight = np.array(f([0., 0., 0.])).ravel()
    turning = np.array(f([.6, .6, .1])).ravel()
    np.testing.assert_allclose(straight, 1000., atol=1e-6)
    assert turning.min() >= 150. and turning.max() <= 1000.+1e-6
    assert KF*np.sum(turning**2) < KF*np.sum(straight**2)


def test_coupled_dynamics_hover_equilibrium():
    derivative, step, _ = dynamics()
    x = np.r_[np.zeros(6), 1., np.zeros(6), np.full(4, np.sqrt(MASS*G/(4*KF)))]
    u = np.r_[MASS*G, np.zeros(3)]
    np.testing.assert_allclose(np.array(derivative(x, u)).ravel(), 0., atol=1e-10)
    np.testing.assert_allclose(np.array(step(x, u, .04)).ravel(), x, atol=1e-10)


def test_goal_recovery_and_no_reverse_entry():
    track, _ = build_case(Case(3., 4.5, 0.))
    p = Planner(track, Config(compile_objective=False, max_solve_ms=2000.))
    head = np.r_[track.goal-np.array([2., 0., 0.]), [2., 0., 0.], np.zeros(6)]
    plan = p.solve(head, 5., 1)
    assert plan.gate_count == 0 and plan.first_gate == 1
    np.testing.assert_allclose(plan.sample(5.), head[:3], atol=1e-8)
    np.testing.assert_allclose(plan.sample(5., 1), head[3:6], atol=1e-8)
    np.testing.assert_allclose(plan.sample(plan.end_time), track.goal, atol=1e-7)
    with pytest.raises(RuntimeError, match='no_reverse_reentry'):
        p.solve(head, 5., 0)


def test_repaired_cost_and_constraints_have_correct_gradients():
    track, _ = build_case(Case(3., 4.5, 0.))
    p = Planner(track, Config(compile_objective=False))
    x = np.r_[np.log([2.9, 1.6]), .2, -.6, 10.8, -3., -.3]
    par = np.r_[.166, track.start, np.zeros(9), np.tile(np.linspace(0., .2, 16), 2)]
    out = p.function(x, par)
    for i in range(7):
        dx = np.eye(7)[i]*2e-6
        a, b = p.function(x+dx, par), p.function(x-dx, par)
        np.testing.assert_allclose(float(out[1][i]), float(a[0]-b[0])/4e-6,
                                   rtol=3e-4, atol=1e-5)
        np.testing.assert_allclose(np.array(out[3])[:, i], np.array(a[2]-b[2]).ravel()/4e-6,
                                   rtol=3e-4, atol=1e-5)
