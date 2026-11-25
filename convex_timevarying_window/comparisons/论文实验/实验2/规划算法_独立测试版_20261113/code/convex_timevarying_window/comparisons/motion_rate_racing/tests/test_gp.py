import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.comparisons.motion_rate_racing.gp_mppi import GPMppi,GPConfig


def controller():
    track,_=build_track();model=GPMppi(track,cfg=GPConfig(samples=32))
    state=np.r_[track.start,np.zeros(3),[1.,0,0,0],np.zeros(3),np.full(4,2.0643076923076924*9.8066/4)]
    return model,state


def test_hover_equilibrium():
    model,state=controller();old=state.copy()
    model.lib.integrate(state,np.zeros(4),.03)
    np.testing.assert_allclose(state,old,atol=1e-12)


def test_collective_increase_goes_up():
    model,state=controller();model.lib.integrate(state,np.ones(4),.03)
    assert state[5]>0
    np.testing.assert_allclose(state[10:13],0,atol=1e-12)


def test_rollout_matches_single_integration():
    model,state=controller();model.previous=np.tile(state,(21,1))
    u=np.full((1,20,4),.2);poses=model.poses(0)
    _,pred,_=model.evaluate(state,u,poses,0,save=True)
    for k in range(20):
        model.lib.integrate(state,u[0,k],.03)
        np.testing.assert_allclose(state,pred[0,k+1],atol=1e-12)


def test_seed_reproducible_and_controls_bounded():
    a,state=controller();b,_=controller()
    ua,pa,_=a.solve(state,0,0);ub,pb,_=b.solve(state,0,0)
    np.testing.assert_allclose(ua,ub);np.testing.assert_allclose(pa,pb)
    assert np.max(np.abs(a.u))<=10


def test_px4_actuator_output_units_are_rad_per_second():
    from convex_timevarying_window.comparisons.motion_rate_racing.gp_px4 import MotorEstimate
    m=MotorEstimate();hover=np.sqrt(2.0643076923076924*9.8066/(4*8.54858e-6))
    m.update(30.,[hover]*4)
    np.testing.assert_allclose(m.thrust.sum(),2.0643076923076924*9.8066)


def test_ctbr_hover_and_rollout_agree():
    track,_=build_track();model=GPMppi(track,cfg=GPConfig(samples=32,input_mode='ctbr'))
    _,state=controller();original=state.copy()
    model.previous=np.tile(state,(21,1))
    controls=np.tile([2.0643076923076924*9.8066,0.,0.,0.],(1,20,1))
    _,pred,_=model.evaluate(state,controls,model.poses(0),0,save=True)
    for k in range(20):
        model.lib.integrate_ctbr(state,controls[0,k],.03)
        np.testing.assert_allclose(state,pred[0,k+1],atol=1e-12)
    np.testing.assert_allclose(state,original,atol=1e-12)


def test_ctbr_positive_roll_rate_response():
    model,state=controller()
    model.lib.integrate_ctbr(state,np.array([2.0643076923076924*9.8066,.2,0.,0.]),.03)
    assert 0<state[10]<.2
    assert state[7]>0
