from dataclasses import replace
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import Geometry, GateProgress, frame_aperture
from convex_timevarying_window.dynamic_gate_mppi_v3.scheduler import Scheduler
from convex_timevarying_window.geometry import ConvexAperture


def test_polygon_frame_offset():
    v = np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])
    for vertices in (v, v[::-1]):
        f = frame_aperture(ConvexAperture("polygon", physical_vertices=vertices,
                                         safe_vertices=vertices), .065)
        np.testing.assert_allclose(np.abs(f.physical_vertices), 1.065)


def test_circle_not_replaced_by_chords():
    a = frame_aperture(ConvexAperture("circle", radius=1.), .065)
    np.testing.assert_allclose(a.boundary_distance(np.array([[0., 0.], [1.065, 0.]])), [1.065, 0.])


def test_all_frames_are_obstacles():
    track, _ = build_track()
    geo = Geometry(track)
    times = np.array([0., .01])
    p = []
    for t in times:
        c, r, *_ = track.windows[-1].state_at(t)
        p.append(c+r[:, :2]@geo.frames[-1].boundary_points()[0])
    assert geo.interval_clearances(p, np.zeros((2, 3)), times).min() < 0


def test_motion_bound_covers_full_rotating_frame():
    track, _ = build_track()
    geo = Geometry(track)
    for w, a, bound in zip(track.windows, geo.frames, geo.frame_speeds):
        for t in np.linspace(0, 5, 20):
            _, _, dc, dr = w.state_at(t)
            speeds = np.linalg.norm(dc+np.einsum("ij,nj->ni", dr[:, :2], a.boundary_points()), axis=1)
            assert speeds.max() <= bound+1e-10


def test_intersample_margin_is_not_sample_speed_only():
    track, _ = build_track()
    geo = Geometry(track)
    times = np.array([0., .01])
    p = np.tile(track.start, (2, 1))
    loose = geo.interval_clearances(p, np.zeros_like(p), times, acceleration_bound=0)
    strict = geo.interval_clearances(p, np.zeros_like(p), times, acceleration_bound=20)
    np.testing.assert_allclose(loose-strict, .001)


def test_replan_on_first_safe_prediction_without_confirmation_wait():
    s = Scheduler(); s.flag_risk()
    request = s.observe(.04, .04+np.arange(101)*.01, np.ones(100), False, 2)
    assert request is not None and request.first_gate == 2
    assert abs(request.anchor-.04) < 1e-12
    assert s.observe(.06, .06+np.arange(101)*.01, np.ones(100), False, 2) is None


def test_future_safe_island_and_crossing_do_not_trigger():
    s = Scheduler(); s.flag_risk()
    for now in np.arange(10)*.02:
        c = np.ones(100); c[:20] = -.01
        assert s.observe(now, now+np.arange(101)*.01, c, False, 0) is None
        assert s.observe(now, now+np.arange(101)*.01, np.ones(100), True, 0) is None


def test_late_or_wrong_order_reference_is_rejected():
    for delay, gate in ((.101, 0), (.08, 1)):
        s = Scheduler(); s.flag_risk()
        r = s.observe(0., np.arange(101)*.01, np.ones(100), False, 0)
        assert not s.finish(r, r.started+delay, gate, 0, 0, 0)
        assert s.pending and s.running is None


def test_deadline_floating_point_boundary_and_skipped_cycles():
    s = Scheduler(); s.flag_risk()
    r = s.observe(0., np.arange(101)*.01, np.ones(100), False, 0)
    assert s.finish(r, r.started+.10000000000000009, 0, 0., 0., 0.)
    s = Scheduler(); s.flag_risk()
    assert s.observe(.2, .2+np.arange(101)*.01, np.ones(100), False, 0) is not None


def test_fast_feedback_reads_are_not_new_control_cycles():
    s = Scheduler(); s.flag_risk()
    assert s.observe(0.,np.arange(101)*.01,np.ones(100),False,0) is not None
    for now in np.arange(1,40)*.001:
        assert s.observe(now, now+np.arange(101)*.01, np.ones(100), False, 0) is None
    assert s.running is not None


def test_gate_only_completes_after_full_exit():
    track, _ = build_track()
    for w in track.windows:
        w.motion = replace(w.motion, translation_enabled=False, rotation_enabled=False)
    progress = GateProgress(track)
    w = track.windows[0]; c, r, *_ = w.state_at(0.)
    n = r[:, 2]*progress.signs[0]
    assert not progress.update(c-n, 0., c+.1*n, .01, True)
    assert progress.first == 0 and progress.crossing
    assert progress.update(c+.1*n, .01, c+n, .02, True)
    assert progress.first == 1 and not progress.crossing


def test_crossing_outside_aperture_does_not_count():
    track, _ = build_track()
    progress = GateProgress(track)
    c, r, *_ = track.windows[0].state_at(0.)
    n = r[:, 2]*progress.signs[0]
    assert not progress.update(c+5*r[:, 0]-n, 0., c+5*r[:, 0]+n, .001, True)
    assert progress.first == 0
