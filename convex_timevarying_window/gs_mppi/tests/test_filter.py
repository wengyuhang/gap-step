import numpy as np
import pytest
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.gs_mppi.engine import Engine,advance
from convex_timevarying_window.gs_mppi.progress import Progress
from dataclasses import replace

@pytest.fixture(scope='module')
def engine():
    track,_=build_track();return Engine(track,gpu=False)

def test_barrier_affine_derivative(engine):
    x=np.r_[engine.track.start+[.1,.2,.3],[1.,2.,-.5]];a=np.array([.2,-.8,.3]);t=.4;epsilon=1e-5
    b=engine.barrier(x,t)
    numerical=(engine.barrier(advance(x,a,epsilon),t+epsilon)[0]-engine.barrier(advance(x,a,-epsilon),t-epsilon)[0])/(2*epsilon)
    assert np.isclose(numerical,b[1:4]@a+b[4],rtol=2e-4,atol=2e-3)

def test_filter_bounded_and_constraint(engine):
    x=np.r_[engine.track.start,np.zeros(3)]
    for desired in ([200,0,0],[0,0,-200],[0,200,40],[0,0,0]):
        a,info=engine.filter(x,0.,desired)
        assert np.linalg.norm(a)<=6.5+1e-8
        assert info['status']==0 and info['residual']>=-1e-5

def test_filter_does_not_change_admissible_action(engine):
    x=np.r_[engine.track.start,np.zeros(3)];a,info=engine.filter(x,0.,[0,0,0])
    assert np.allclose(a,0) and info['mu']==0

def test_sagitta_covers_circle(engine):
    for gate,index in enumerate(engine.track.order):
        frame=engine.geometry.frames[index]
        if frame.kind!='circle':continue
        fs=engine.features[engine.features[:,0]==gate]
        for theta in np.linspace(0,2*np.pi,301):
            q=np.array([frame.radius*np.cos(theta),frame.radius*np.sin(theta),0.]);a=fs[:,1:4];edge=fs[:,4:7]-a
            f=np.clip(np.sum((q-a)*edge,axis=1)/np.sum(edge*edge,axis=1),0,1)
            assert np.min(np.linalg.norm(q-a-f[:,None]*edge,axis=1)-fs[:,7])<1e-9

def test_time_varying_derivative_changes():
    track,_=build_track();vary_track(track,0.,0.,10.,1.,1.);e=Engine(track,gpu=False)
    c,r,*_=track.windows[0].state_at(.3);x=np.r_[c-3*r[:,2],[1.,2.,0.]]
    b=e.barrier(x,.3);a=np.array([.1,.2,.3]);dt=1e-5
    n=(e.barrier(advance(x,a,dt),.3+dt)[0]-e.barrier(advance(x,a,-dt),.3-dt)[0])/(2*dt)
    assert np.isclose(n,b[1:4]@a+b[4],rtol=1e-4,atol=.01)

def test_cpu_gpu_rollout():
    track,_=build_track();e=Engine(track);state=np.r_[track.start,np.zeros(3)];steps=10
    controls=np.zeros((2,steps,3));ref=np.tile(np.r_[track.start,np.zeros(6)],(steps+1,1));poses=e.poses(np.arange(steps+1)*.02)
    gpu=e.rollout(state,controls,ref,poses,mode=0).get()
    cpu,_,_=e.rollout(state,controls[:1],ref,poses,mode=0,gpu=False)
    assert np.allclose(gpu,cpu[None],rtol=1e-4,atol=1e-3)

def static_engine():
    track,_=build_track()
    for w in track.windows:w.motion=replace(w.motion,translation_enabled=False,rotation_enabled=False)
    return Engine(track,gpu=False)

def test_actual_center_crossing_does_not_count_exit():
    e=static_engine();w=e.track.windows[0];c,r,*_=w.state_at(0);n=e.signs[0]*r[:,2]
    p=Progress(e.track,e.config,e.signs);slab=e.cfg[14]
    assert not p.update(c-n*(slab+.1),0,c-n*.2,.1)
    assert p.phase==1 and p.first==0
    assert not p.update(c-n*.2,.1,c+n*.2,.2)
    assert p.phase==2 and p.first==0 and len(p.centers)==1
    assert p.update(c+n*.2,.2,c+n*(slab+.1),.3)
    assert p.first==1 and p.phase==0

def test_actual_no_retreat_and_no_bypass():
    e=static_engine();w=e.track.windows[0];c,r,*_=w.state_at(0);n=e.signs[0]*r[:,2];s=e.cfg[14]
    p=Progress(e.track,e.config,e.signs)
    p.update(c-(s+.1)*n,0,c-.2*n,.1);p.update(c-.2*n,.1,c-(s+.1)*n,.2)
    assert p.violation=='forbidden_retreat' and p.first==0
    p=Progress(e.track,e.config,e.signs);shift=4*r[:,0]
    p.update(c+shift-(s+.1)*n,0,c+shift-.2*n,.1)
    assert p.violation=='outside_aperture' and p.first==0

def test_predicted_bypass_rejected():
    e=static_engine();w=e.track.windows[0];c,r,*_=w.state_at(0);n=e.signs[0]*r[:,2]
    x=np.r_[c+3*r[:,0]-(e.cfg[14]+.01)*n,3*n]
    ref=np.tile(np.r_[x[:3],np.zeros(6)],(2,1))
    m,_,_=e.rollout(x,np.zeros((1,1,3)),ref,e.poses([0,.02]),mode=1,gpu=False)
    assert int(m[1])&16 and m[4]==0

def test_current_exit_not_charged_next_gate_distance():
    e=static_engine();w=e.track.windows[0];c,r,*_=w.state_at(0);n=e.signs[0]*r[:,2]
    x=np.r_[c-1.2*n,3*n];steps=60;times=np.arange(steps+1)*.02
    ref=np.tile(np.r_[x[:3],np.zeros(6)],(steps+1,1));poses=e.poses(times);controls=np.zeros((1,steps,3))
    a,_,_=e.rollout(x,controls,ref,poses,gpu=False)
    poses[:,1,:3]+=1000
    b,_,_=e.rollout(x,controls,ref,poses,gpu=False)
    assert a[1]==0 and b[1]==0 and a[4]==b[4]==1
    assert np.isclose(a[0],b[0],atol=1e-8)

def test_infeasible_boundary_not_relaxed(engine):
    b=np.array([0.,0.,0.,0.,-1.,1.]);desired=np.zeros(3);a=np.empty(3);info=np.empty(2)
    status=engine.lib.gs_filter(desired.ctypes.data,b.ctypes.data,engine.cfg.ctypes.data,a.ctypes.data,info.ctypes.data)
    assert status!=0

def test_bounded_filter_matches_independent_optimizer(engine):
    from scipy.optimize import minimize
    b=np.array([.4,2.,-1.,.5,-15.,.8]);desired=np.array([-5.,2.,0.]);a=np.empty(3);info=np.empty(2)
    status=engine.lib.gs_filter(desired.ctypes.data,b.ctypes.data,engine.cfg.ctypes.data,a.ctypes.data,info.ctypes.data)
    objective=lambda z:.5*np.sum((z[:3]-desired)**2)+.5*engine.cfg[4]*z[3]**2
    residual=lambda z:b[1:4]@z[:3]+b[4]+engine.cfg[2]*b[0]+z[3]*b[0]
    answer=minimize(objective,np.r_[a,info[1]],method='SLSQP',constraints=[
        {'type':'ineq','fun':residual},{'type':'ineq','fun':lambda z:engine.cfg[5]**2-z[:3]@z[:3]}],options={'ftol':1e-10})
    assert status==0 and answer.success
    assert np.isclose(objective(np.r_[a,info[1]]),answer.fun,rtol=1e-6,atol=1e-6)

def test_transit_avoidance_log_is_serializable():
    import json
    from convex_timevarying_window.gs_mppi.controller import Controller
    track,_=build_track();c=Controller(track,samples=24,steps=10)
    # Near the floor, descending fast: current tracking action needs filtering.
    x=np.r_[track.start[:2],1.08,0.,0.,-.5]
    c.select(x,0.,len(track.order),0,None)
    assert c.last['avoiding'] is True
    json.dumps(c.last)
