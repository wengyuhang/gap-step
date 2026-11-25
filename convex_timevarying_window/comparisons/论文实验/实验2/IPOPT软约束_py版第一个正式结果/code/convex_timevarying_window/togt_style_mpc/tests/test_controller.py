import numpy as np
import casadi as ca
import pytest
from convex_timevarying_window.togt_style_mpc.model import (
    functions,quaternion_error,MASS,G,FMIN,FMAX,MIXER)
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state


def hover_reference(times):
    result=np.zeros((len(times),9)); result[:,:3]=[0.,0.,3.]
    return result


def test_hover_and_torque_signs():
    f,step,_=functions(); x=hover_state()[:13]
    u=np.full(4,MASS*G/4)
    np.testing.assert_allclose(np.asarray(f(x,u)).ravel(),0.,atol=1e-12)
    np.testing.assert_allclose(np.asarray(step(x,u,.04)).ravel(),x,atol=1e-12)
    for axis in range(3):
        control=u+.2*MIXER[axis+1]/np.max(abs(MIXER[axis+1]))
        assert np.asarray(f(x,control)).ravel()[10+axis]>0


def test_quaternion_cost_sign_invariance():
    q=np.array([.5,.5,.5,.5]); r=np.array([1.,0.,0.,0.])
    a=np.asarray(quaternion_error(ca.DM(q),ca.DM(r))).ravel()
    b=np.asarray(quaternion_error(ca.DM(-q),ca.DM(r))).ravel()
    assert np.dot(a,a)==np.dot(b,b)


@pytest.fixture(scope='module')
def controller():
    from convex_timevarying_window.togt_style_mpc.controller import MPC
    return MPC()


def test_hover_and_ctbr_conversion(controller):
    x=hover_state([0.,0.,3.])
    for i in range(5):
        command=controller.command(x,i*.01,hover_reference)
    np.testing.assert_allclose(command,[MASS*G,0.,0.,0.],atol=1e-6)
    assert controller.controls.shape==(25,4)
    assert controller.predicted.shape==(26,13)
    assert np.min(controller.controls)>=FMIN-1e-6
    assert np.max(controller.controls)<=FMAX+1e-6
    assert command[0]==pytest.approx(controller.controls[0].sum())
    np.testing.assert_allclose(command[1:],controller.predicted[1,10:13])


def test_position_error_produces_future_rate(controller):
    x=hover_state([-.15,0.,3.])
    command=controller.command(x,.1,hover_reference)
    assert np.linalg.norm(command[1:])>1e-3
    assert np.linalg.norm(controller.predicted[0,10:13])<1e-10
    assert abs(command[0]-controller.controls[0].sum())<1e-10


def test_bad_state_not_silently_executed(controller):
    with pytest.raises(ValueError):
        controller.command(np.zeros(13),.2,hover_reference)
    with pytest.raises(ValueError):
        controller.command(np.full(17,np.nan),.2,hover_reference)


def test_solver_failure_is_not_old_command_fallback(controller,monkeypatch):
    monkeypatch.setattr(controller.solver,'solve',lambda: 4)
    with pytest.raises(RuntimeError,match='status=4'):
        controller.command(hover_state([0.,0.,3.]),.2,hover_reference)


def test_smooth_smoke_reference_derivatives():
    from convex_timevarying_window.togt_style_mpc.gazebo_smoke import local_reference
    origin=np.array([-16.,4.,3.2]);h=1e-5
    for t in [0.,2.,2.5,4.,6.,9.,10.,12.]:
        q=local_reference([t],origin)[0]
        before,after=local_reference([t-h,t+h],origin)
        np.testing.assert_allclose(q[3:6],(after[:3]-before[:3])/(2*h),atol=1e-7)
        np.testing.assert_allclose(q[6:],(after[3:6]-before[3:6])/(2*h),atol=1e-5)


def test_multiple_shooting_warm_start_keeps_predicted_states(controller,monkeypatch):
    x=hover_state([0.,0.,3.]); controller.controls=None
    controller.command(x,1.,hover_reference)
    old=controller.predicted.copy();old[:,0]+=np.linspace(0.,.1,26)
    controller.predicted=old.copy();saved={};original=controller.solver.set
    def recording(stage,field,value):
        if field=='x':saved[stage]=np.asarray(value).copy()
        original(stage,field,value)
    monkeypatch.setattr(controller.solver,'set',recording)
    controller.command(x,1.01,hover_reference)
    np.testing.assert_allclose(saved[0],x[:13])
    assert saved[4][0]==pytest.approx(.75*old[4,0]+.25*old[5,0])
