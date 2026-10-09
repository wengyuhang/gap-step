from dataclasses import asdict
import numpy as np
from scipy.spatial.transform import Rotation
from convex_timevarying_window.simple_two_gate_soft_only_mpc.tracking_mpc import MPC, Config
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state, reference_states, LOWER, UPPER


def test_objective_has_only_tracking_and_smoothing():
    m = MPC(); x = hover_state()
    x[6:10] = Rotation.from_euler('x', 65, degrees=True).as_quat()[[3,0,1,2]]
    states = np.tile(x, (m.cfg.steps+1, 1))
    controls = np.tile(m.previous, (m.cfg.steps, 1))
    assert m.merit(states, controls, states, controls) == 0.


def test_only_physical_input_rows_and_no_tilt_or_trust_effect():
    m = MPC(); x = hover_state()
    x[6:10] = Rotation.from_euler('x', 65, degrees=True).as_quat()[[3,0,1,2]]
    controls = np.tile(m.previous, (m.cfg.steps, 1)); states = m.rollout(x, controls)
    pva = np.tile(np.r_[x[:3], np.zeros(6)], (m.cfg.steps+1, 1))
    refs, ff = reference_states(pva, m.cfg.dt, x[6:10])
    P, q, A, lo, hi = m.condense(x, states, controls, refs, ff)
    assert A.shape == (100, 100)
    np.testing.assert_array_equal(A.toarray(), np.eye(100))
    np.testing.assert_array_equal(lo, np.tile(LOWER/m.uscale, m.cfg.steps))
    np.testing.assert_array_equal(hi, np.tile(UPPER/m.uscale, m.cfg.steps))
    assert not any('tilt' in k or 'trust' in k for k in asdict(m.cfg))
    u = m.command(x, 0., lambda t: np.tile(np.r_[x[:3], np.zeros(6)], (len(t),1)))
    assert np.isfinite(u).all() and (u >= LOWER).all() and (u <= UPPER).all()
    assert m.logs[-1]['controller'] == 'trajectory_tracking_input_box_rti'
