from dataclasses import replace

import casadi as ca
import numpy as np
import pytest

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.continuation_racing.guard import CheckedPlanner
from convex_timevarying_window.checked_soft_racing.planner import Config, Planner, clearance_penalty


def test_penalty_scale_monotonicity_and_hinge_derivatives():
    cfg = Config()
    hard = cfg.radius + cfg.frame_thickness + cfg.tracking_margin
    preferred = hard + cfg.soft_clearance_buffer
    d2 = ca.SX.sym('d2')
    cost = clearance_penalty(d2, cfg)
    evaluate = ca.Function('penalty', [d2], [cost, ca.jacobian(cost, d2), ca.hessian(cost, d2)[0]])
    assert float(evaluate(hard**2)[0]) == pytest.approx(1.)
    assert float(evaluate((hard-.1)**2)[0]) > 1.
    for distance in (preferred, preferred+.1):
        np.testing.assert_allclose([float(v) for v in evaluate(distance**2)], 0., atol=1e-14)
    values = [float(evaluate(d*d)[0]) for d in np.linspace(hard-.1, preferred+.1, 50)]
    assert np.all(np.diff(values) <= 0)


@pytest.mark.parametrize('kw', [dict(soft_clearance_buffer=0.), dict(soft_clearance_weight=-1.),
                                    dict(soft_clearance_weight=np.nan), dict(soft_clearance_buffer=np.inf)])
def test_bad_config_rejected(kw):
    with pytest.raises(ValueError):
        Config(**kw)


@pytest.fixture(scope='module')
def planners():
    track, _ = build_track()
    vary_track(track, .4, 0., 10., 4., 1.)
    cfg = Config(horizon=2, workspace_lower=(-28., -30., .65), workspace_upper=(28., 22., 12.))
    return CheckedPlanner(track, cfg), Planner(track, replace(cfg, soft_clearance_weight=0.)), Planner(track, cfg)


def test_constraints_unchanged_and_zero_weight_is_exact_ablation(planners):
    base, zero, soft = planners
    rng = np.random.default_rng(46)
    positive_delta = False
    for first in range(len(base.solvers)):
        b, z, s = (p.solvers[first] for p in planners)
        assert b[0] == z[0] == s[0]
        for i in range(3, 7):
            np.testing.assert_array_equal(b[i], z[i])
            np.testing.assert_array_equal(b[i], s[i])
        x = rng.uniform(-.2, .2, size=len(b[3]))
        x[::9] = 3.
        params = np.r_[.4, base.track.start, np.zeros(6), base.collocation_parameters(first, b[0])]
        for reference in (z, s):
            for left, right in zip(b[2](x, params), reference[2](x, params)):
                np.testing.assert_allclose(left, right, rtol=0., atol=0.)
        f = [float(p[1].get_function('nlp_f')(x, params)) for p in (b, z, s)]
        assert f[0] == f[1]
        assert f[2] >= f[0]
        positive_delta |= f[2] > f[0]
    assert positive_delta


def test_known_motion_is_evaluated_at_candidate_time(planners):
    soft = planners[2]
    t = ca.SX.sym('time')
    # A fixed world point near one actual sleeve at t=0 is not fixed relative
    # to that sleeve as the known moving-window function advances.
    frame = soft.frames[0]
    c, R, *_ = frame.state_at(0.)
    q = np.array([frame.aperture.radius, 0.]) if frame.aperture.kind == 'circle' else frame.aperture.physical_vertices[0]
    point = c + R[:, :2] @ q
    fn = ca.Function('moving_cost', [t], [soft.extra_stage_cost(ca.DM(point), t, 0, 0, 0)])
    costs = np.array([float(fn(time)) for time in np.linspace(0., 2., 41)])
    assert np.ptp(costs) > 1.
