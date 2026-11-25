import numpy as np
import pytest
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,MASS
from convex_timevarying_window.three_phase_mppi.controller import Controller,Progress,Policy,LinearReference,config
from convex_timevarying_window.three_phase_mppi.engine import source
from convex_timevarying_window.three_phase_mppi.events import reason,computation_safe

@pytest.fixture(scope='module')
def ctrl():
    track,_=build_track();vary_track(track,.4);return Controller(track,config(128),7)

def test_independent_kernel_has_no_old_policy_logic():
    s=source()
    for name in ('v3_rollout','v3_guided','v3_feedback_replay','passage_deadline'):
        assert name not in s

def test_future_geometry_matches_query(ctrl):
    poses=ctrl.engine.prepare(1.37,20)
    for k in (0,8,20):
        for i,wi in enumerate(ctrl.track.order):
            c,r,*_=ctrl.track.windows[wi].state_at(1.37+.01*k)
            np.testing.assert_allclose(poses[k,i,:3],c,atol=1e-12)
            np.testing.assert_allclose(poses[k,i,3:].reshape(3,3),r,atol=1e-12)

def points(ctrl,depths,times):
    w=ctrl.track.windows[ctrl.track.order[0]];sign=ctrl.engine.signs[0]
    return [w.state_at(t)[0]+w.state_at(t)[1][:,2]*sign*z for z,t in zip(depths,times)]

def test_complete_body_exit_not_center(ctrl):
    p=Progress(ctrl.track,ctrl.cfg,ctrl.engine.signs);slab=ctrl.cfg.radius+ctrl.cfg.error_margin+ctrl.cfg.frame_thickness
    ts=[0,.01,.02,.03];x=points(ctrl,[-slab-.1,-.1,.1,slab+.1],ts)
    assert not p.update(x[0],ts[0],x[1],ts[1]);assert p.phase==1
    assert not p.update(x[1],ts[1],x[2],ts[2]);assert p.phase==2 and p.first==0
    assert p.update(x[2],ts[2],x[3],ts[3]);assert p.first==1

@pytest.mark.parametrize('depths',[[-1.,-.1,-1.],[ -1.,.1,-.1]])
def test_forbidden_retreat(ctrl,depths):
    p=Progress(ctrl.track,ctrl.cfg,ctrl.engine.signs);ts=[0,.01,.02];x=points(ctrl,depths,ts)
    p.update(x[0],0,x[1],.01);p.update(x[1],.01,x[2],.02)
    assert p.violation=='forbidden_retreat';assert p.first==0

def test_remote_infinite_plane_not_gate_entry(ctrl):
    p=Progress(ctrl.track,ctrl.cfg,ctrl.engine.signs);ts=[0,.01];x=points(ctrl,[-1.,1.],ts)
    w=ctrl.track.windows[ctrl.track.order[0]]
    x=[q+30*w.state_at(t)[1][:,0] for q,t in zip(x,ts)]
    assert not p.update(x[0],0,x[1],.01);assert p.phase==0 and p.violation is None

def test_phase_lock_and_release(ctrl):
    x=hover_state(ctrl.track.start);ctrl.prepare_gate=0
    assert ctrl.choose_stage(x,0,0,0,None)=='PREPARE'
    assert ctrl.choose_stage(x,0,0,1,None)=='CROSS'
    assert ctrl.choose_stage(x,0,7,0,None)=='TRANSIT';assert ctrl.prepare_gate is None

def test_cpu_gpu_prediction_agree(ctrl):
    x=hover_state(ctrl.track.start);u=np.array([MASS*9.8066,0,0,0.]);pol=np.zeros((1,25))
    ref=ctrl.reference_array(None,0,20,x);poses=ctrl.engine.prepare(0,20)
    cpu,path,commands=ctrl.engine.evaluate(x,u,pol,ref,poses,0,0,0,False)
    gpu=ctrl.engine.evaluate(x,u,pol,ref,poses,0,0,0).get()[0]
    assert cpu[1]==gpu[1]==0;np.testing.assert_allclose(cpu,gpu,atol=2e-4)
    state=x.copy();prev=u.copy()
    for k in range(20):
        command=ctrl.engine.command(state,ref[k],prev);state=ctrl.engine.integrate(state,command)
        np.testing.assert_allclose(command,commands[k],atol=1e-10);prev=command
    np.testing.assert_allclose(state,path[-1],atol=1e-10)

def test_policy_age_and_normal_bias():
    x=np.zeros(17);ref=LinearReference(x,2.);v=np.zeros(25);v[21]=2
    p=Policy(v,2.,0,ref,4.,0)
    np.testing.assert_allclose(p.pva(2.5)[[0,3,6]],[.25,1,2])

def test_prefix_in_gate_has_no_scheduler_ban(ctrl):
    # Prefix mode does not require completing a gate in 100 ms; it checks
    # current executing motion, not a new TOGT reference.
    x=hover_state(points(ctrl,[-.2],[0.])[0]);u=np.array([MASS*9.8066,0,0,0.]);ref=LinearReference(x,0.,hold=True)
    p=Policy(np.zeros(25),0.,0,ref,1.,1)
    m,_,_=ctrl.prefix(p,x,0.,0,1,u)
    assert int(m[1])&512==0

@pytest.mark.parametrize('stage',['TRANSIT','PREPARE','CROSS'])
def test_actual_exit_always_requests_replan(stage):
    assert reason(stage,passed=True)=='gate_exit'

@pytest.mark.parametrize('stage',['PREPARE','CROSS'])
def test_autonomous_phase_is_not_a_replanning_event(stage):
    assert reason(stage,position_error=10,velocity_error=10,avoiding=True) is None

def test_transit_events_and_prefix_only():
    assert reason('TRANSIT',position_error=.6)=='tracking_error'
    assert reason('TRANSIT',avoiding=True)=='obstacle_recovery'
    assert reason('TRANSIT') is None
    m=np.zeros(10);m[5]=10;m[6]=1
    assert computation_safe(m)
    m[6]=0;assert not computation_safe(m)

def test_autonomous_prediction_does_not_track_togt(ctrl):
    x=hover_state(ctrl.track.start);u=np.array([MASS*9.8066,0,0,0.]);pol=np.zeros((1,25))
    pol[0,[0,6,12]]=x[:3];pol[0,18]=1.;pol[0,20]=1.
    poses=ctrl.engine.prepare(0,10)
    a,path_a,_=ctrl.engine.evaluate(x,u,pol,np.zeros((11,9)),poses,0,0,1,False)
    b,path_b,_=ctrl.engine.evaluate(x,u,pol,np.full((11,9),999.),poses,0,0,1,False)
    np.testing.assert_array_equal(a,b);np.testing.assert_array_equal(path_a,path_b)

def test_feedback_thrust_projection(ctrl):
    from scipy.spatial.transform import Rotation
    x=hover_state(ctrl.track.start);q=Rotation.from_euler('y',30,degrees=True).as_quat()
    x[6:10]=np.r_[q[3],q[:3]];previous=np.array([26.,0,0,0.])
    u=ctrl.engine.command(x,np.r_[x[:3],np.zeros(3),[0,0,20]],previous)
    axis=Rotation.from_quat(q).as_matrix()[:,2]
    assert np.linalg.norm(axis*u[0]/MASS-[0,0,9.8066])<=7.00001

def test_quaternion_sign_flip_is_not_attitude_jump():
    from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions
    q=np.array([[.1,0,0,.994987],[-.1,0,0,-.994987],[.1,0,0,.994987]])
    fixed=continuous_quaternions(q)
    np.testing.assert_allclose(fixed,np.broadcast_to(q[0],(3,4)))

def test_polynomial_feedback_execution_matches_admission(ctrl):
    x=hover_state(ctrl.track.start);u=np.array([MASS*9.8066,0,0,0.])
    vec=np.zeros(25,dtype=np.float32);vec[[0,6,12]]=x[:3];vec[18]=2.;vec[20]=1.
    vec[3:6]=[.6,-.2,.015];vec[9:12]=[-.5,.1,-.01]
    policy=Policy(vec,0.,0,None,1.,1)
    assert policy.vector.dtype==np.float64
    poses=ctrl.engine.prepare(0,40);ref=np.zeros((41,9))
    m,path,commands=ctrl.engine.evaluate(x,u,policy.vector[None],ref,poses,0,0,0,False)
    state=x.copy();previous=u.copy()
    for k in range(int(m[5])):
        action=ctrl.engine.command(state,policy.pva(k*.01),previous)
        np.testing.assert_allclose(action,commands[k],atol=1e-10)
        state=ctrl.engine.integrate(state,action);previous=action
    np.testing.assert_allclose(state,path[-1],atol=1e-10)

def test_zero_gate_reference_matches_actual_pvaj():
    from convex_timevarying_window.three_phase_mppi.reference import Planner
    from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
    track,opt=build_track();planner=Planner(track,opt)
    head=BoundaryState(np.array([-5.,4.,3.2]),np.array([3.,0.,0.]),np.array([1.,0.,0.]),np.array([2.,0.,0.]))
    ref=planner.solve(head,12.,7)
    assert ref is not None and ref.solve_ms<100 and len(ref.trajectory.durations)==3
    for d,expected in enumerate((head.position,head.velocity,head.acceleration,head.jerk)):
        np.testing.assert_allclose(ref.trajectory.evaluate(0,d),expected,atol=1e-7)

def test_return_invalid_line_search_is_a_rejected_trial():
    from convex_timevarying_window.three_phase_mppi.reference import Planner
    from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
    track,opt=build_track();planner=Planner(track,opt)
    def invalid(*args):raise RuntimeError('invalid duration')
    planner.core.value_and_gradient=invalid
    assert planner.solve(BoundaryState(track.start),0.,7) is None
