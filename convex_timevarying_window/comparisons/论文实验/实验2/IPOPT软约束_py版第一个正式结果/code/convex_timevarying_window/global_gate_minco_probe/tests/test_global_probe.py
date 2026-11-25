import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.global_gate_minco_probe.trajectory import coefficients
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.two_gate_hard_mpc.joint_planner import coefficients as full_system


def test_reduced_system_same_c5_minco_not_new_curve():
    rng = np.random.default_rng(42)
    for n in (2, 3, 8):
        head, tail = ca.DM(rng.normal(size=(3, 4))), ca.DM(rng.normal(size=(3, 4)))
        points, velocities = ca.DM(rng.normal(size=(3, n-1))), ca.DM(rng.normal(size=(3, n-1)))
        T = ca.DM(rng.uniform(1., 3., n))
        C = np.asarray(ca.Function('reduced', [], [coefficients(head, tail, points, velocities, T)])()['o0'])
        original = np.asarray(ca.Function('full', [], [full_system(head, tail, points, velocities, T)])()['o0'])
        np.testing.assert_allclose(C, original, rtol=2e-6, atol=3e-7)
        def state(k, u, d):
            return C[8*k:8*k+8].T@np.asarray(basis(u, d)).ravel()/float(T[k])**d
        for k in range(n-1):
            for d in range(6):
                np.testing.assert_allclose(state(k, 1., d), state(k+1, 0., d), rtol=1e-6, atol=2e-6)
        for d in range(4):
            np.testing.assert_allclose(state(0, 0., d), np.asarray(head[:, d]).ravel(), atol=1e-7)
            np.testing.assert_allclose(state(n-1, 1., d), np.asarray(tail[:, d]).ravel(), atol=1e-7)


@pytest.mark.parametrize('position_only', [False, True])
def test_full_course_constraint_chain_rule(position_only):
    from convex_timevarying_window.global_gate_minco_probe.planner import Planner
    from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case
    track, _ = build_case(Case(4., 4., 0.))
    planner = Planner(track, position_only=position_only, seed_speed=6.5)
    assert planner.n == 8 and planner.m == 7 and planner.nvar == (33 if position_only else 54)
    x = planner.seeds()[0]
    length, derivative = planner.arc(x)
    assert float(length) > 0.
    for i in (0, 8, planner.nvar-1):
        delta = np.eye(planner.nvar)[i]*1e-6
        numerical = float(planner.arc(x+delta)[0]-planner.arc(x-delta)[0])/(2e-6)
        np.testing.assert_allclose(float(derivative[i]), numerical, rtol=3e-4, atol=2e-4)
    witnesses = planner.witnesses()
    witnesses[(0, 7)] = dict(kind='normalized', values=[.3, .8])
    g, jac, C = planner.evaluate(x, witnesses)
    # Only physical, frame safety and forward-crossing rows remain. No 2*m
    # seed-relative arrival bounds; broad per-piece duration bounds stay intact.
    expected = planner.n * planner.physical.size1_out(0) + planner.m
    expected += sum(planner.safety[gate].size1_out(0) for gate, _ in witnesses)
    assert len(g) == expected
    np.testing.assert_allclose(np.exp(planner.lower[:planner.n]), .3)
    np.testing.assert_allclose(np.exp(planner.upper[:planner.n]), 15.)
    assert np.all(np.isfinite(g)) and np.all(np.isfinite(jac))
    assert C.shape == (8, 8, 3)
    for i in (0, 2, 7, 8, 13, 32) if position_only else (0, 2, 7, 8, 13, 32, 33, 50):
        delta = np.eye(planner.nvar)[i]*1e-6
        numerical = (planner.evaluate(x+delta, witnesses)[0]-planner.evaluate(x-delta, witnesses)[0])/(2e-6)
        np.testing.assert_allclose(jac[:, i], numerical, rtol=3e-3, atol=3e-4)
    if position_only:
        T = np.exp(x[:8])
        for k in range(7):
            for d in range(7):
                left = C[k].T@np.asarray(basis(1., d)).ravel()/T[k]**d
                right = C[k+1].T@np.asarray(basis(0., d)).ravel()/T[k+1]**d
                np.testing.assert_allclose(left, right, atol=2e-5, rtol=2e-5)
    else:
        expanded = dict(witnesses)
        expanded[(0, 7)] = dict(kind='normalized', values=list(np.linspace(.1, .9, 25)))
        gg, jj, _ = planner.evaluate(x, expanded)
        assert len(gg) == len(g)+planner.safety[0].size1_out(0)
        delta = np.eye(planner.nvar)[0]*1e-6
        numerical = (planner.evaluate(x+delta, expanded)[0]-planner.evaluate(x-delta, expanded)[0])/(2e-6)
        np.testing.assert_allclose(jj[:, 0], numerical, rtol=3e-3, atol=3e-4)
