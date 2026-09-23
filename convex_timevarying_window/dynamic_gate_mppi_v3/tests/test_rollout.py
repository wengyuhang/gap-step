import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import Rollout, hover_state, MASS, KF
from convex_timevarying_window.dynamic_gate_mppi_v3.controller import Controller
from convex_timevarying_window.dynamic_gate_mppi_v3.native_backend import Core


def test_vectorized_poses_match_original_queries():
    track, _ = build_track(); r = Rollout(track)
    times = np.array([0., .37, 1.28, 10.])
    centers, rotations = r.geometry.poses(times)
    for ti, t in enumerate(times):
        for wi, w in enumerate(track.windows):
            c, R, *_ = w.state_at(t)
            np.testing.assert_allclose(centers[ti, wi], c, atol=1e-13)
            np.testing.assert_allclose(rotations[ti, wi], R, atol=1e-13)


def test_hover_and_native_geometry_oracle():
    track, _ = build_track(); r = Rollout(track)
    state = hover_state(track.start)
    u = np.full((2, 50, 4), MASS*9.8066/4)
    b = r(state, u, r.prepare(0))
    assert b.valid.all()
    np.testing.assert_allclose(b.states, np.broadcast_to(state, b.states.shape), atol=1e-11)
    expected = r.geometry.interval_clearances(b.states[..., :3], b.states[..., 3:6], np.arange(101)*.01)
    np.testing.assert_allclose(b.clearances, expected, atol=1e-11)


def test_motor_speed_lag_not_thrust_derivative():
    track, _ = build_track(); r = Rollout(track)
    state = hover_state(track.start)
    for command, tau in ((8., .0125), (2., .025)):
        actual = r.integrate(state, np.full(4, command), .02)
        target = np.sqrt(command/KF)
        expected = target+(state[13:]-target)*np.exp(-.02/tau)
        np.testing.assert_allclose(actual[13:], expected, rtol=3e-5)


def test_illegal_inputs_are_hard_rejected():
    track, _ = build_track(); r = Rollout(track)
    u = np.full((1, 50, 4), 100.)
    assert not r(hover_state(track.start), u, r.prepare(0)).valid[0]


def test_x500_free_tail_gradient_finite_difference():
    c = Core(); head = np.zeros((3, 4)); tail = head.copy()
    tail[0, 0] = 5; tail[0, 1] = 1.
    points = np.empty((0, 3)); times = np.array([1.])
    _, _, _, gradient = c.value_and_gradient_with_tail(head, tail, points, times)
    for axis, derivative in ((0, 0), (0, 1), (1, 2), (2, 3)):
        plus, minus = tail.copy(), tail.copy(); h = 1e-5
        plus[axis, derivative] += h; minus[axis, derivative] -= h
        finite = (c.value_and_gradient_with_tail(head, plus, points, times)[0]
                  - c.value_and_gradient_with_tail(head, minus, points, times)[0])/(2*h)
        np.testing.assert_allclose(gradient[axis, derivative], finite, rtol=3e-3, atol=3e-3)


def test_selected_weighted_control_is_rescreened_and_cache_expires():
    track, _ = build_track(); c = Controller(track)
    state = hover_state(track.start)
    selection = c.select(state, 0.)
    assert selection is not None and selection.safe_samples > 0
    assert np.min(selection.clearances) > 0
    assert c.cache is None
    assert c.publish(selection, state, 0.)
    assert c.cached_action(state, 0.) is not None
    assert c.cached_action(state, 1.) is None
    displaced = state.copy(); displaced[0] += 1.
    assert c.cached_action(displaced, 0.) is None


def test_timing_or_tracking_mismatch_alone_is_not_danger():
    from types import SimpleNamespace
    class Trajectory:
        total_time = 10.
        def evaluate(self, t, derivative):
            v=np.array([0., 0., 3.]) if derivative == 0 else np.zeros(3)
            return np.broadcast_to(v,np.asarray(t).shape+(3,))
    track, _ = build_track(); c = Controller(track)
    reference = SimpleNamespace(made_at=0., first=0, crossing_times=np.array([4., 8.]),
                                trajectory=Trajectory())
    state = hover_state([0., 0., 3.])
    assert not c.reference_risk(state, 1., 0, reference)
    state[0] = .501
    assert not c.reference_risk(state, 1., 0, reference)
    state[0] = 0.; state[3] = 2.001
    assert not c.reference_risk(state, 1., 0, reference)
    state[3] = 0.
    assert not c.reference_risk(state, 5., 0, reference)
    assert c.reference_risk(state, 11., 0, reference)
