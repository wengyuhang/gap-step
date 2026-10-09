from types import SimpleNamespace
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case, build_case
from convex_timevarying_window.togt_point_only_mpc.trajectory import Plan
from convex_timevarying_window.dynamic_passage_minco.safety import (
    passage, power_to_bernstein, restrict, sine_range, rectangle_distance_squared,
)


def test_bernstein_restriction_encloses_polynomial():
    rng = np.random.default_rng(4); power = rng.normal(size=(8, 3))
    control = restrict(power_to_bernstein(power), .27, .61)
    values = np.polynomial.polynomial.polyval(np.linspace(.27, .61, 1001), power).T
    assert np.all(values >= control.min(axis=0)-1e-12)
    assert np.all(values <= control.max(axis=0)+1e-12)


def test_sine_bounds_include_internal_extrema():
    assert sine_range(0., 2*np.pi) == (-1., 1.)
    lo, hi = sine_range(.1, .2)
    assert lo <= np.sin(.1) and hi >= np.sin(.2)


def straight_plan(track, normal_speed):
    w = track.windows[0]; R = w.state_at(0.)[1]
    coefficients = np.zeros((1, 8, 3))
    coefficients[0, 0] = w.center0+normal_speed*.5*R[:, 2]
    coefficients[0, 1] = -normal_speed*R[:, 2]
    return Plan(0., 0, np.array([1.]), coefficients, np.zeros(3), 0., 0., 'test', 1)


def test_local_oracle_finds_fast_sweeping_frame_collision():
    # The gate passes its anchor at the same t=.5 as the body centre.
    track, _ = build_case(Case(3.5, 2., -3.5*np.pi))
    result = passage(straight_plan(track, 20.), track.windows[0], .57)
    assert not result['passed']
    assert result['outcome'] == 'violated'
    assert result['minimum_sample_distance_m'] < .57


def test_local_oracle_bounds_a_safe_crossing_without_full_flight_scan():
    track, _ = build_case(Case(2., 2., 0.))
    result = passage(straight_plan(track, 20.), track.windows[0], .57)
    assert result['passed']
    assert result['distance_lower_bound_m'] >= .57
    assert sum(v['end']-v['start'] for v in result['overlap_intervals']) < .06


def test_outside_point_cannot_pass_by_being_far_from_unsigned_frame():
    assert rectangle_distance_squared(np.array([3., 0., 0.]), np.array([1., 1.])) == 0.


def test_optimizer_derivatives_match_finite_differences():
    from convex_timevarying_window.dynamic_passage_minco.planner import Config, Planner
    track, _ = build_case(Case(3.5, 2., 0.))
    cfg = Config(compile_objective=False)
    planner = Planner(track, cfg)
    x = np.r_[np.log([2.9, 4.5]), .2, .4]
    offsets = np.tile(np.linspace(0., .2, cfg.witness_slots), 2)
    par = np.r_[.162, track.start, np.zeros(9), offsets]
    out = planner.function(x, par)
    gradient, jacobian = np.asarray(out[1]).ravel(), np.asarray(out[3])
    epsilon = 2e-6
    for i in range(4):
        delta = np.eye(4)[i]*epsilon
        a, b = planner.function(x+delta, par), planner.function(x-delta, par)
        np.testing.assert_allclose(gradient[i], float(a[0]-b[0])/(2*epsilon), rtol=2e-4, atol=2e-6)
        np.testing.assert_allclose(jacobian[:, i], np.asarray(a[2]-b[2]).ravel()/(2*epsilon),
                                   rtol=2e-4, atol=2e-6)
