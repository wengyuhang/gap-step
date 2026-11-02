from dataclasses import replace

import casadi as ca
import numpy as np
import pytest

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.single_gate_stop_racing.geometry import passage as old_passage
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.scenario import VERTICES
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.component_initialization.planner import Planner as Original
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.spin.scenario import (
    build_case, symbolic_pose, local_coordinates)
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.spin.bounds import (
    rotate_box, passage, value, warmup)
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.spin.planner import Planner


def test_pose_and_derivatives_include_true_spin():
    w = build_case(.7).windows[0]
    t = ca.MX.sym('t')
    c, R = symbolic_pose(w, t)
    f = ca.Function('pose_test', [t], [c, R, ca.jacobian(c, t), ca.jacobian(ca.vec(R), t)])
    q = np.array([1.4, -.2, .3])
    for at in (.0, .071, .23, 1.2):
        center, rotation, velocity, dR = w.state_at(at)
        raw = f(at)
        np.testing.assert_allclose(center, np.array(raw[0]).ravel(), atol=1e-12)
        np.testing.assert_allclose(rotation, raw[1], atol=1e-12)
        np.testing.assert_allclose(velocity, np.array(raw[2]).ravel(), atol=1e-11)
        np.testing.assert_allclose(dR.ravel(order='F'), np.array(raw[3]).ravel(), atol=1e-11)
        np.testing.assert_allclose(rotation[:, 2], w.base_rotation[:, 2], atol=1e-15)
        h = 1e-6
        numerical = (w.state_at(at+h)[1]-w.state_at(at-h)[1])/(2*h)
        np.testing.assert_allclose(dR, numerical, atol=1e-8)
        world = center+rotation@q
        np.testing.assert_allclose(local_coordinates(w, [world], [at])[0], q, atol=1e-12)


def test_rotated_boxes_enclose_samples_including_reverse_and_full_turn():
    rng = np.random.default_rng(20261013)
    for a, b in ((0., .3), (1.4, 1.9), (3.2, -2.1), (-5., 4.), (1., 1.)):
        lo, hi = np.array([-2., -.4, -.2]), np.array([.3, 1.8, .8])
        low, high = rotate_box(lo, hi, a, b)
        q = rng.uniform(lo, hi, size=(400, 3))
        theta = rng.uniform(min(a, b), max(a, b), size=len(q))
        c, s = np.cos(theta), np.sin(theta)
        rotated = np.column_stack((c*q[:, 0]+s*q[:, 1], -s*q[:, 0]+c*q[:, 1], q[:, 2]))
        assert np.all(rotated >= low-1e-12)
        assert np.all(rotated <= high+1e-12)


def straight_plan():
    C = np.zeros((2, 8, 3))
    C[0, 0] = [1.6, -4., 3.2]
    C[0, 1] = [0., 4., 0.]
    C[1, 0] = [1.6, 0., 3.2]
    C[1, 1] = [0., 4., 0.]
    return Plan(0., 0, np.ones(2), C, np.zeros(7), 0., 0., 'test', 1)


def test_rotation_checker_detects_collision_missed_by_freezing_orientation():
    warmup()
    plan = straight_plan()
    static, rotating = build_case(0., amplitude=0.).windows[0], build_case(.5, amplitude=0.).windows[0]
    for piece in (0, 1):
        assert old_passage(plan, static, .57, piece)['passed']
        assert passage(plan, static, .57, piece)['passed']
        result = passage(plan, rotating, .57, piece)
        assert not result['passed']
        assert result['outcome'] == 'violated'


def test_native_distance_matches_direct_moving_finite_segments():
    window = build_case(1.).windows[0]
    plan = straight_plan()
    R = window.base_rotation
    coeff = plan.coefficients[0]@R
    coeff[0] -= window.center0@R
    for u in np.linspace(0., 1., 31):
        center, rotation, *_ = window.state_at(u)
        q = rotation.T@(plan.sample(u)-center)
        expected = window.aperture.boundary_distance(q[:2])**2+q[2]**2
        actual = value(u, coeff, 0., 1., R.T@window.motion.translation_amplitude,
                       6*np.pi, 0., 2*np.pi, 0., VERTICES)
        assert actual == pytest.approx(expected, abs=1e-10)


def test_rotating_checker_can_accept_a_clear_fast_passage():
    plan = straight_plan()
    plan.coefficients[0, 0, 1] = -40.
    plan.coefficients[0, 1, 1] = 40.
    plan.coefficients[1, 1, 1] = 40.
    window = build_case(1., amplitude=0.).windows[0]
    for piece in (0, 1):
        assert passage(plan, window, .57, piece)['passed']


def test_zero_spin_nlp_equivalence_and_nonzero_spin_analytic_jacobian():
    from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.scenario import build_case as original_case
    mapping = SCDiskMap.fit(VERTICES)
    cfg = replace(Config(), compile_objective=False, nodes_per_piece=2)
    original = Original(original_case(), cfg, mapping)
    zero = Planner(build_case(0.), cfg, mapping)
    spin = Planner(build_case(.5), cfg, mapping)
    offsets = np.tile(np.r_[cfg.passage_seed_offsets,
                           np.zeros(cfg.witness_slots-len(cfg.passage_seed_offsets))], 2)
    par = np.r_[0., original.track.start, np.zeros(9), offsets]
    x = np.r_[np.log([1.411, 2.1]), original.seed_coordinates[1]['d'], [0., 5., 0.]]
    for a, b in zip(original.function(x, par), zero.function(x, par)):
        np.testing.assert_allclose(a, b, atol=1e-10)
    raw = spin.function(x, par)
    analytic = np.vstack([np.asarray(raw[1])[None, :], raw[3]])
    finite = np.zeros_like(analytic)
    for k in range(7):
        h = 2e-7*max(1., abs(x[k]))
        xp, xm = x.copy(), x.copy()
        xp[k] += h
        xm[k] -= h
        plus, minus = spin.function(xp, par), spin.function(xm, par)
        finite[:, k] = np.r_[float(plus[0])-float(minus[0]),
                              np.asarray(plus[2]-minus[2]).ravel()]/(2*h)
    np.testing.assert_allclose(analytic, finite, rtol=4e-4, atol=4e-5)
