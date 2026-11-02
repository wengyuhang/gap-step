import casadi as ca
import numpy as np
from convex_timevarying_window.global_gate_minco_probe.trajectory import full_state_coefficients
from convex_timevarying_window.global_gate_minco_probe.planner import Planner
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case
from convex_timevarying_window.togt_minco_mpc.trajectory import basis


def test_full_pvaj_endpoints_and_shared_c3_states():
    rng = np.random.default_rng(6); n = 8
    nodes = rng.normal(size=(n+1, 3, 4)); T = rng.uniform(1., 4., n)
    C = full_state_coefficients(ca.DM(nodes[0]), ca.DM(nodes[-1]),
        *[ca.DM(nodes[1:-1, :, d].T) for d in range(4)], ca.DM(T))
    C = np.asarray(C).reshape(n, 8, 3)
    for k in range(n):
        for side in (0, 1):
            for d in range(4):
                value = C[k].T@np.asarray(basis(float(side), d)).ravel()/T[k]**d
                np.testing.assert_allclose(value, nodes[k+side, :, d], atol=1e-9)
    # Do not accidentally constrain snap continuity: this is the added freedom.
    left = C[0].T@np.asarray(basis(1., 4)).ravel()/T[0]**4
    right = C[1].T@np.asarray(basis(0., 4)).ravel()/T[1]**4
    assert np.linalg.norm(left-right) > 1.


def test_full_state_contains_pv_and_preserves_initial_curves_and_constraints():
    track, _ = build_case(Case(4., 4., 0.))
    pv, full = Planner(track), Planner(track, full_state=True)
    assert pv.nvar == 54 and full.nvar == 96
    for x, y in zip(pv.seeds(), full.seeds()):
        np.testing.assert_array_equal(y[:54], x)
        g, _, C = pv.evaluate(x, pv.witnesses())
        h, jac, D = full.evaluate(y, full.witnesses())
        np.testing.assert_allclose(D, C, atol=1e-7, rtol=1e-8)
        np.testing.assert_allclose(g, h, atol=2e-8, rtol=1e-7)
    for i in (0, 7, 13, 33, 54, 60, 75, 95):
        delta = np.eye(full.nvar)[i]*1e-6
        numeric = (full.evaluate(y+delta, full.witnesses())[0]-full.evaluate(y-delta, full.witnesses())[0])/(2e-6)
        np.testing.assert_allclose(jac[:, i], numeric, rtol=4e-3, atol=4e-4)
