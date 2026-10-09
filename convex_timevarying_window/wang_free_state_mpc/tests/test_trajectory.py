import casadi as ca
import numpy as np
from 复现.polynomial_dynamic_racing_2023.minco import coefficients, basis
from convex_timevarying_window.wang_free_state_mpc.trajectory import from_arrival_jerk, arrival_jerk_for_middle, Plan


def test_reparameterization_exactly_recovers_wang_curve():
    rng = np.random.default_rng(710)
    for T in (.7, 2.3, 5.):
        head, tail = rng.normal(size=(2, 3, 3)); middle = rng.normal(size=3)
        expected = coefficients(head, tail, [middle], T)
        j = arrival_jerk_for_middle(head, tail, middle, T)
        actual = np.array(from_arrival_jerk(ca.DM(head.T), ca.DM(tail.T), ca.DM(j), T)).reshape(2, 6, 3)
        np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=1e-11)
        for d in range(5):
            np.testing.assert_allclose(basis(1., d)@actual[0], basis(0., d)@actual[1], atol=2e-9)


def test_arrival_jerk_is_independent_of_endpoint_pva():
    head = np.arange(9.).reshape(3, 3)*.1; tail = head+1.; T = 2.7
    for j in (np.zeros(3), np.array([2., -3., 5.])):
        C = np.array(from_arrival_jerk(ca.DM(head.T), ca.DM(tail.T), ca.DM(j), T)).reshape(2, 6, 3)
        for d in range(3):
            np.testing.assert_allclose(basis(0., d)@C[0]/(T/2)**d, head[d], atol=1e-10)
            np.testing.assert_allclose(basis(1., d)@C[1]/(T/2)**d, tail[d], atol=1e-10)
        np.testing.assert_allclose(basis(1., 3)@C[1]/(T/2)**3, j, atol=1e-10)


def test_sampler_keeps_six_coefficients_and_gate_clock():
    C = coefficients(np.zeros((3, 3)), np.array([[3., 1., 2.], [0., 0., 0.], [0., 0., 0.]]), [[1., 0., 1.]], 4.)
    plan = Plan(2., 0, np.array([2., 2.]), C, np.zeros(1), 0., 0., 'test', 1)
    ts = np.array([2., 3., 4., 5., 6.])
    for d in range(4): np.testing.assert_allclose(plan.sample_many(ts, d), [plan.sample(t, d) for t in ts], atol=1e-12)
    np.testing.assert_equal(plan.crossing_times, [6.])
