from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.wang_free_state_mpc.planner import Planner, Config
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan


@pytest.fixture(scope='module')
def planner():
    track, _ = build_track()
    cfg = replace(Config(), workspace_lower=(-28., -30., .65), workspace_upper=(28., 22., 12.))
    return Planner(track, cfg, build_indices=[0])


def fixture_values(planner):
    state = np.r_[planner.track.start, np.zeros(9)]
    x = planner.initial(state, .15, 0)
    return x, np.r_[.15, state, np.tile([.15, .35, .65, .85], 6)]


def test_moving_gate_gradient_for_time_p_v_a_j(planner):
    x, p = fixture_values(planner); fg = planner.solvers[0][0].fg
    _, gradient, _ = fg(x, p)
    for index in (0, 3, 9, 15, 24, 27):
        e = np.eye(len(x))[index]*1e-6
        fd = float(fg(x+e, p)[0]-fg(x-e, p)[0])/2e-6
        np.testing.assert_allclose(float(gradient[index]), fd, rtol=4e-4, atol=.1)


def test_gate_position_velocity_acceleration_and_incoming_jerk_are_free(planner):
    x, p = fixture_values(planner); vi, ai, ji, qi = planner.offsets(2)
    x[qi:qi+4] = [.15, -.12, -.08, .1]
    x[vi:vi+3] = [.25, -.1, .08]; x[ai:ai+3] = [.1, -.2, .05]; x[ji:ji+3] = [.12, -.08, .2]
    _, C, _ = planner.solvers[0][1](x, p)
    plan = Plan(.15, 0, np.repeat(np.exp(x[:3])/2, 2), np.array(C).reshape(6, 6, 3), x, 0., 0., 'test', 2)
    at = plan.crossing_times[0]; center, R = planner.windows[0].state_at(at)[:2]
    np.testing.assert_allclose(plan.sample(at), center+R[:, :2]@x[qi:qi+2], atol=1e-9)
    np.testing.assert_allclose(plan.sample(at, 1), planner.cfg.speed*x[vi:vi+3], atol=1e-9)
    np.testing.assert_allclose(plan.sample(at, 2), planner.cfg.acceleration*x[ai:ai+3], atol=1e-9)
    np.testing.assert_allclose(plan.sample(at-1e-9, 3), planner.cfg.jerk*x[ji:ji+3], atol=1e-6)


def test_unexecuted_search_candidate_is_not_time_trimmed(planner):
    x, _ = fixture_values(planner)
    planner.search[0] = SimpleNamespace(first_gate=0, made_at=.15, x=x)
    try: np.testing.assert_equal(planner.initial(np.zeros(12), 1.15, 0), x)
    finally: planner.search.clear()


def test_milliseconds_remaining_are_not_reset_to_a_new_leg(planner):
    x, _ = fixture_values(planner)
    planner.admitted = SimpleNamespace(first_gate=0, crossing_times=[.16,3.], x=x)
    try:
        with pytest.raises(ValueError, match='passage_already_committed'):
            planner.initial(np.r_[planner.track.start,np.zeros(9)], .15, 0)
    finally: planner.admitted = None


def test_center_ablation_changes_only_gate_coordinate_bounds():
    track, _ = build_track()
    center = Planner(track, Config(free_points=False), build_indices=[6])
    free = Planner(track, Config(free_points=True), build_indices=[6])
    qi = center.offsets(1)[3]
    np.testing.assert_equal(center.solvers[6][2][:qi], free.solvers[6][2][:qi])
    np.testing.assert_equal(center.solvers[6][3][:qi], free.solvers[6][3][:qi])
    np.testing.assert_equal(center.solvers[6][2][qi:], 0.)
    np.testing.assert_equal(center.solvers[6][3][qi:], 0.)
    state = np.r_[track.start, np.zeros(9)]; x = free.initial(state, .15, 6)
    p = np.r_[.15, state, np.tile([.15, .35, .65, .85], 4)]
    for a, b in zip(center.solvers[6][1](x, p), free.solvers[6][1](x, p)):
        np.testing.assert_allclose(a, b)
