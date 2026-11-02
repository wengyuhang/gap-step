import numpy as np
import pytest
from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state,MASS,G


@pytest.fixture(scope='module')
def controller():return MPC()


def reference(times):return np.tile([0.,0.,3.,0.,0.,0.,0.,0.,0.],(len(times),1))


def test_hover_tracks_without_control_bias(controller):
    u=controller.command(hover_state(),0.,reference)
    np.testing.assert_allclose(u,[MASS*G,0.,0.,0.],atol=1e-5)


def test_perturbed_tracking_has_exact_dynamics_and_hard_inputs(controller):
    state=hover_state([-.12,.05,3.]);u=controller.command(state,.016,reference,execution_time=.04)
    assert np.linalg.norm(u[1:])>1e-4
    assert np.all(controller.controls>=controller.lower-1e-10)
    assert np.all(controller.controls<=controller.upper+1e-10)
    for k in range(controller.cfg.steps):
        expected=np.array(controller.step(controller.predicted[k],controller.controls[k],controller.cfg.dt)).ravel()
        np.testing.assert_allclose(controller.predicted[k+1],expected,atol=1e-9)
    assert controller.logs[-1]['final_cost']<=controller.logs[-1]['initial_cost']+1e-9


def test_invalid_feedback_is_rejected(controller):
    with pytest.raises(ValueError):controller.command(np.zeros(17),.1,reference)
    with pytest.raises(ValueError):controller.command(np.full(17,np.nan),.1,reference)
