import numpy as np
from convex_timevarying_window.global_gate_minco_probe.flight import Progress
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case
from convex_timevarying_window.single_gate_stop_racing.planner import route_signs


def fixture():
    track, _ = build_case(Case(4., 4., 0.)); w = track.windows[1]; sign = route_signs(track)[1]
    center, R, *_ = w.state_at(0.)
    def point(q, z): return center+R[:, :2]@np.array(q)+R[:, 2]*sign*z
    return w, point, Progress([w], [sign], .38594472, .085)


def test_remote_plane_transit_does_not_pass_or_abort_gate():
    w, point, progress = fixture()
    assert not progress.update(point([30., 0.], 1.), point([30., 0.], -1.), 0., 0.)
    assert progress.gate == 0 and not progress.crossed and progress.violation is None
    assert len(progress.outside_plane_transits) == 1
    # Coming back outside, then the actual designated forward opening passage.
    assert not progress.update(point([30., 0.], -1.), point([0., 0.], 1.), 0., 0.)
    assert progress.update(point([0., 0.], 1.), point([0., 0.], -1.), 0., 0.)
    assert progress.gate == 1 and len(progress.crossings) == 1 and len(progress.exits) == 1


def test_near_frame_miss_still_rejected_and_body_exit_not_skipped():
    w, point, progress = fixture()
    q = [w.aperture.radius+.1, 0.]
    assert not progress.update(point(q, 1.), point(q, -1.), 0., 0.)
    assert progress.violation == 'missed_current_opening' and progress.gate == 0
    w, point, progress = fixture()
    assert not progress.update(point([0., 0.], 1.), point([0., 0.], -.1), 0., 0.)
    assert progress.crossed and progress.gate == 0
    assert progress.update(point([0., 0.], -.1), point([0., 0.], -1.), 0., 0.)
