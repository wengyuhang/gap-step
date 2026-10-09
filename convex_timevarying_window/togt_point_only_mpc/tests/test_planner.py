"""The experimental trajectory really has only the requested knots."""
import numpy as np

from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.togt_point_only_mpc.planner import Config, Planner
from convex_timevarying_window.togt_point_only_mpc.trajectory import Plan
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import (
    build_case as build_seven,
)


def test_single_gate_two_septic_pieces_and_no_gate_state_variables():
    track, _ = build_case(Case(2.5, 3.5, 0.))
    planner = Planner(track, Config(compile_objective=False), build_indices=[0])
    solver, evaluate, lo, hi, _, _ = planner.solvers[0]
    assert lo.size == hi.size == 4  # Two times, two gate coordinates.
    x = planner.initial(np.r_[track.start, np.zeros(9)], .17, 0)
    durations = np.exp(x[:2])
    par = np.r_[.17, track.start, np.zeros(9), np.tile([.15, .35, .65, .85], 2)]
    _, coefficients, _ = evaluate(x, par)
    C = np.asarray(coefficients).reshape(2, 8, 3)
    plan = Plan(.17, 0, durations, C, x, 0., 0., 'test', 1)
    crossing = float(plan.crossing_times[0])
    center, R, *_ = track.windows[0].state_at(crossing)
    np.testing.assert_allclose(plan.sample(.17), track.start, atol=1e-6)
    np.testing.assert_allclose(plan.sample(crossing), center+R[:, :2]@x[2:4], atol=1e-6)
    np.testing.assert_allclose(plan.sample(plan.end_time), track.goal, atol=1e-5)
    for derivative in range(7):
        left = np.asarray(basis(1., derivative)).ravel()@C[0]/durations[0]**derivative
        right = np.asarray(basis(0., derivative)).ravel()@C[1]/durations[1]**derivative
        np.testing.assert_allclose(left, right, atol=5e-3, rtol=1e-5)


def test_center_variant_locks_only_the_gate_coordinates():
    track, _ = build_case(Case(2.5, 3.5, 0.))
    free = Planner(track, Config(compile_objective=False), build_indices=[0])
    center = Planner(track, Config(compile_objective=False, free_points=False),
                     build_indices=[0])
    flo, fhi = free.solvers[0][2:4]
    clo, chi = center.solvers[0][2:4]
    np.testing.assert_array_equal(flo[:2], clo[:2])
    np.testing.assert_array_equal(fhi[:2], chi[:2])
    np.testing.assert_array_equal(clo[2:], np.zeros(2))
    np.testing.assert_array_equal(chi[2:], np.zeros(2))


def test_free_crossing_point_uses_physical_aperture_without_inset():
    track, _ = build_case(Case(3., 4., 0.))
    planner = Planner(track, Config(compile_objective=False), build_indices=[0])
    _, evaluate, _, _, lbg, ubg = planner.solvers[0]
    x = planner.initial(np.r_[track.start, np.zeros(9)], .17, 0)
    par = np.r_[.17, track.start, np.zeros(9), np.tile([.15, .35, .65, .85], 2)]
    x[2:4] = [0., 1.05]  # Inside physical opening, outside the former ±0.394 m band.
    g, _, _ = evaluate(x, par)
    values = np.asarray(g).ravel()[:4]
    assert np.all(values >= lbg[:4] - 1e-10)
    assert np.all(values <= ubg[:4] + 1e-10)
    x[3] = 1.20  # Outside the 1.15 m physical half-opening.
    g, _, _ = evaluate(x, par)
    assert np.any(np.asarray(g).ravel()[:4] > ubg[:4] + 1e-10)


def test_two_gate_horizon_has_two_free_gate_points_and_fixed_third_center():
    track, _ = build_seven(Case(1., 2., 0.))
    planner = Planner(track, Config(compile_objective=False), build_indices=[0])
    _, evaluate, lo, _, _, _ = planner.solvers[0]
    assert lo.size == 7  # Three times, two 2-D gate points.
    x = planner.initial(np.r_[track.start, np.zeros(9)], .17, 0)
    par = np.r_[.17, track.start, np.zeros(9), np.tile([.15, .35, .65, .85], 3)]
    _, coefficients, _ = evaluate(x, par)
    plan = Plan(.17, 0, np.exp(x[:3]), np.asarray(coefficients).reshape(3, 8, 3),
                x, 0., 0., 'test', 2)
    assert len(plan.crossing_times) == 2
    for k, crossing in enumerate(plan.crossing_times):
        center, R, *_ = track.windows[k].state_at(float(crossing))
        np.testing.assert_allclose(plan.sample(float(crossing)),
                                   center+R[:, :2]@x[3+2*k:5+2*k], atol=1e-5)
    np.testing.assert_allclose(plan.sample(plan.end_time),
                               track.windows[2].state_at(plan.end_time)[0], atol=1e-5)
