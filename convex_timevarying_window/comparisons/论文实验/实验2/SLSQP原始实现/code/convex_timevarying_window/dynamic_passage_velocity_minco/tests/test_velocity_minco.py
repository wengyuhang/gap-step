import casadi as ca
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from convex_timevarying_window.dynamic_passage_velocity_minco.planner import Config, Planner
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import minco_coefficients
from convex_timevarying_window.togt_minco_mpc.trajectory import minco_coefficients as position_minco, basis
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein


def state_at(C, T, piece, u, derivative):
    return np.array(C[8*piece:8*piece+8]).T@np.array(basis(u, derivative)).ravel()/T[piece]**derivative


def test_position_only_solution_is_in_the_free_velocity_family():
    head = np.zeros((3, 4)); head[:, 0] = [-16., 4., 3.2]
    tail = np.zeros((3, 4)); tail[:, 0] = [3.58, -3.52, 6.48]
    point = np.array([-2.42, -.165, 5.913]); times = np.array([2.92, 8.05])
    original = position_minco(ca.DM(head), ca.DM(tail), ca.DM(point), ca.DM(times))
    original = np.array(ca.Function('original', [], [original])()['o0'])
    velocity = state_at(original, times, 0, 1., 1)
    current = minco_coefficients(ca.DM(head), ca.DM(tail), ca.DM(point), ca.DM(velocity), ca.DM(times))
    current = np.array(ca.Function('current', [], [current])()['o0'])
    np.testing.assert_allclose(current, original, rtol=3e-8, atol=3e-7)


def test_boundary_conditions_and_fifth_derivative_continuity():
    rng = np.random.default_rng(10)
    head, tail = rng.normal(size=(3, 4)), rng.normal(size=(3, 4))
    point, velocity = rng.normal(size=3), rng.normal(size=3)
    times = np.array([2.3, 1.7])
    C = minco_coefficients(ca.DM(head), ca.DM(tail), ca.DM(point), ca.DM(velocity), ca.DM(times))
    C = np.array(ca.Function('boundary', [], [C])()['o0'])
    for d in range(4):
        np.testing.assert_allclose(state_at(C, times, 0, 0., d), head[:, d], atol=2e-9)
        np.testing.assert_allclose(state_at(C, times, 1, 1., d), tail[:, d], atol=2e-8)
    np.testing.assert_allclose(state_at(C, times, 0, 1., 0), point, atol=2e-9)
    np.testing.assert_allclose(state_at(C, times, 0, 1., 1), velocity, atol=2e-9)
    for d in range(6):
        np.testing.assert_allclose(state_at(C, times, 0, 1., d), state_at(C, times, 1, 0., d), atol=3e-8)


def test_bernstein_positive_velocity_implies_no_tail_reversal():
    controls = np.array([11., 8., 4., 2., .5, .1, 0.])
    transform = power_to_bernstein(np.eye(7))
    power = np.linalg.solve(transform, controls)
    values = np.polynomial.polynomial.polyval(np.linspace(0., 1., 1001), power)
    assert values.min() >= -1e-10


def test_objective_and_constraints_gradients_include_velocity_and_forward_tail():
    track, _ = build_case(Case(3., 4., 0.))
    cfg = Config(compile_objective=False, forward_tail=True, log_cost_scaling=True,
                 initial_tail_scale=1.)
    planner = Planner(track, cfg)
    x = np.r_[np.log([2.9, 1.4]), .2, -.6, 10.8, -3., -.3]
    par = np.r_[.166, track.start, np.zeros(9), np.tile(np.linspace(0., .2, 16), 2)]
    out = planner.function(x, par); gradient, jacobian = np.array(out[1]).ravel(), np.array(out[3])
    for i in range(7):
        delta = np.eye(7)[i]*2e-6
        a, b = planner.function(x+delta, par), planner.function(x-delta, par)
        np.testing.assert_allclose(gradient[i], float(a[0]-b[0])/4e-6, rtol=3e-4, atol=1e-5)
        np.testing.assert_allclose(jacobian[:, i], np.array(a[2]-b[2]).ravel()/4e-6,
                                   rtol=3e-4, atol=1e-5)


def test_forward_tail_fixed_input_regression():
    track, _ = build_case(Case(3., 4., 0.))
    cfg = Config(compile_objective=False, max_solve_ms=2000., forward_tail=True,
                 log_cost_scaling=True, initial_tail_scale=1.)
    planner = Planner(track, cfg)
    plan = planner.solve(np.r_[track.start, np.zeros(9)], .166, 0)
    assert plan.optimizer_stats['converged'] and plan.optimizer_stats['passage']['passed']
    times = np.linspace(plan.crossing_times[0], plan.end_time, 2001)
    positions, velocity = plan.sample_many(times), plan.sample_many(times, 1)
    normal = (track.goal-track.windows[0].center0)/np.linalg.norm(track.goal-track.windows[0].center0)
    assert np.max((positions-track.goal)@normal) < 1e-6
    assert np.min(velocity@normal) >= -1e-6
    assert plan.durations[1] < 2.
    np.testing.assert_allclose(plan.sample(plan.end_time), track.goal, atol=1e-7)
    for d in (1, 2, 3):
        np.testing.assert_allclose(plan.sample(plan.end_time, d), 0., atol=1e-6)
