import numpy as np
import pytest
from scipy.signal import butter
from convex_timevarying_window.single_gate_stop_racing import px4_model
from convex_timevarying_window.togt_style_mpc.aligned_session import MotorEstimate
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state


def test_d_term_uses_lowpass_rate_and_preserves_memory():
    x=px4_model.initial_state(hover_state()); x[10]=1.
    y=np.asarray(px4_model.native_dynamics()[0](x,[20.2438398154,0,0,0],.004)).ravel()
    b,_=butter(2,40,fs=250); alpha=.004/(.004+1/(2*np.pi*20))
    assert y[26]==pytest.approx(b[0]); assert y[29]==pytest.approx(alpha*b[0]/.004)


def test_motor_estimate_is_causal():
    m=MotorEstimate(); m.update(0.,np.full(4,500.)); m.update(.01,np.full(4,1000.))
    np.testing.assert_allclose(m.speed,500.)
    np.testing.assert_allclose(m.at(.015),1000-500*np.exp(-.005/.0125))
    m.update(.02,np.full(4,500.))
    np.testing.assert_allclose(m.speed,1000-500*np.exp(-.01/.0125))
    np.testing.assert_allclose(m.at(.005),500.)


@pytest.fixture(scope='module')
def controller():
    from convex_timevarying_window.togt_style_mpc.aligned_controller import MPC
    return MPC()


def reference(times):
    return np.tile([0.,0.,3.,0.,0.,0.,0.,0.,0.],(len(times),1))


def test_direct_ctbr_and_execution_clock(controller):
    calls=[]
    def recording(ts): calls.extend(ts); return reference(ts)
    cmd=controller.command(hover_state([0,0,3]),0.,recording,execution_time=.024)
    np.testing.assert_allclose(cmd,controller.controls[0])
    assert controller.predicted.shape==(26,33)
    assert calls[0]==pytest.approx(.024)
    assert controller.logs[-1]['execution_time_s']==pytest.approx(.024)
    assert np.linalg.norm(cmd[1:])<1e-6


def test_delivery_history_not_new_command_over_past(controller):
    controller.deliveries=[]
    cmd=np.array([22.,.1,.2,.3]); controller.command_sent(cmd,.036)
    assert controller.applied_at(.036)[0]!=22.
    np.testing.assert_allclose(controller.applied_at(.040),cmd)


def test_delivery_uses_global_pid_grid_between_physics_ticks(controller):
    from convex_timevarying_window.togt_style_mpc.aligned_controller import next_control_tick
    assert next_control_tick(.037)==pytest.approx(.040)
    assert next_control_tick(.039)==pytest.approx(.040)
    assert next_control_tick(.040)==pytest.approx(.044)
    controller.deliveries=[]
    cmd=np.array([22.,.1,.2,.3]);controller.command_sent(cmd,.037)
    assert controller.applied_at(.039)[0]!=22.
    np.testing.assert_allclose(controller.applied_at(.040),cmd)


def test_failure_does_not_execute_last_solution(controller,monkeypatch):
    monkeypatch.setattr(controller.solver,'solve',lambda: 4)
    with pytest.raises(RuntimeError,match='status=4'):
        controller.command(hover_state([0,0,3]),.1,reference)
