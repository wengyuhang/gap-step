import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.phase_initialization import Planner
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap


def test_independent_gate_time_samples_are_reproducible_ordered_and_phase_diverse():
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    planner = Planner(track, True, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    times = np.array(planner.initial_times())
    np.testing.assert_array_equal(times, planner.initial_times())
    assert times.shape == (8, 8)
    assert np.all(times >= np.exp(planner.lower[:planner.n])-1e-12)
    assert np.all(times <= np.exp(planner.upper[:planner.n])+1e-12)
    normalized = times/times.sum(axis=1, keepdims=True)
    assert np.linalg.matrix_rank(normalized, tol=1e-8) > 1
    arrivals = np.cumsum(times, axis=1)[:, :-1]
    for gate, window in enumerate(track.windows):
        phases = (window.motion.frequency*arrivals[:, gate]+window.motion.phase/(2*np.pi)) % 1.
        assert len(set(np.floor(phases*8).astype(int))) == 8
        assert np.any(np.minimum(abs(phases-.25), abs(phases-.75)) > .1)


def test_free_and_center_have_same_center_curves_and_analytic_minco_states():
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    free = Planner(track, True, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    center = Planner(track, False, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    assert free.velocity_start-free.n == 25 and center.velocity_start-center.n == 0
    for a, b in zip(free.seeds(), center.seeds()):
        np.testing.assert_array_equal(a[:free.n], b[:center.n])
        np.testing.assert_array_equal(a[free.velocity_start:], b[center.velocity_start:])
        durations = np.exp(a[:free.n])
        arrivals = np.cumsum(durations)[:-1]
        points = np.array([window.state_at(at)[0] for window, at in zip(track.windows, arrivals)])
        np.testing.assert_allclose(np.asarray(free.point_function(a)).T, points, atol=1e-12)
        np.testing.assert_allclose(np.asarray(center.point_function(b)).T, points, atol=1e-12)
        np.testing.assert_allclose(free.curve_values(a), center.curve_values(b), atol=1e-9)
        curve = MincoSnap(BoundaryState(track.start), BoundaryState(track.goal), points, durations)
        expected = np.concatenate([curve.evaluate(arrivals, order).ravel() for order in (1, 2, 3)])
        np.testing.assert_allclose(a[free.velocity_start:], expected, rtol=1e-8, atol=1e-7)


def test_static_gates_use_independent_travel_time_samples():
    track, _ = scenario.build_case(scenario.Case(level='static'))
    planner = Planner(track, False, solver='ipopt', dynamic_limit_fraction=.95,
        pose_function=scenario.pose)
    times = np.array(planner.initial_times())
    assert np.isfinite(times).all() and np.all(times > 0.)
    assert np.linalg.matrix_rank(times/times.sum(axis=1, keepdims=True), tol=1e-8) > 1
