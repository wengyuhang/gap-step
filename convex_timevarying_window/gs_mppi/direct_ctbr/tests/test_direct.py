import numpy as np
import pytest
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import CTBRRollout,encode_ctbr
from convex_timevarying_window.gs_mppi.direct_ctbr.engine import Engine

@pytest.fixture(scope='module')
def engine():return Engine(build_track()[0],gpu=False)

def inputs(e,steps=10):
    x=hover_state(e.track.start);u=np.tile([MASS*9.8066,0,0,0.],(1,steps,1))
    ref=np.tile(np.r_[e.track.start,np.zeros(6)],(steps+1,1));poses=e.poses(np.arange(steps*4+1)*.005)
    return x,u,ref,poses

def test_motor_thrust_does_not_act_instantly(engine):
    x=hover_state(engine.track.start);u=np.array([28.,0.,0.,0.]);new=engine.integrate(x,u,.005)
    assert 0<(new[5]-x[5])/.005<28/MASS-9.8066
    assert np.all(new[13:]>x[13:])

def test_bodyrate_command_has_attitude_and_rate_response(engine):
    x=hover_state(engine.track.start);u=np.array([MASS*9.8066,1.,0.,0.]);new=engine.integrate(x,u,.005)
    assert 0<new[10]<1 and 0<abs(new[7])<.01
    assert np.linalg.norm(new[3:5])<.005

def test_matches_separately_compiled_ctbr_plant(engine):
    plant=CTBRRollout(engine.track,engine.config);x=hover_state(engine.track.start)
    rng=np.random.default_rng(4)
    for _ in range(80):
        u=np.r_[rng.uniform(16,26),rng.uniform(-.7,.7,3)]
        a=engine.integrate(x,u,.005);b=plant.integrate(x,u,.005)
        assert np.allclose(a,b,atol=1e-12,rtol=1e-12);x=a

def test_output_is_ctbr_and_execution_matches_rollout(engine):
    x,u,r,p=inputs(engine);u[0,:,1]=.1
    m,path,commands=engine.rollout(x,u,r,p,gpu=False)
    assert m[1]==0 and commands.shape==(10,4)
    for k,command in enumerate(commands):
        encode_ctbr(command)
        for _ in range(4):x=engine.integrate(x,command,.005)
        assert np.array_equal(x,path[k+1])

def test_cpu_gpu_same_model():
    e=Engine(build_track()[0]);x,u,r,p=inputs(e,20);u[0,:,1]=.07
    cpu,path,commands=e.rollout(x,u,r,p,gpu=False);gpu=e.rollout(x,u,r,p).get()[0]
    assert int(cpu[1])==int(gpu[1])==0
    assert np.allclose(cpu,gpu,atol=2e-4,rtol=1e-3)

def test_input_limits_applied_in_control_space(engine):
    x,u,r,p=inputs(engine,1);u[0,0]=[MASS*9.8066,10.,-10.,0.]
    m,path,commands=engine.rollout(x,u,r,p,gpu=False)
    assert m[1]==0
    assert commands[0,0]<=4*8.54858 and np.max(np.abs(commands[0,1:]))<=3.

def test_first_command_is_scored_against_actual_previous_output(engine):
    x,u,r,p=inputs(engine,3);u[0,:,1]=.12
    hover=MASS*9.8066
    matched,_,_=engine.rollout(x,u,r,p,gpu=False,previous_command=[hover,.12,0.,0.])
    reversed_rate,_,_=engine.rollout(x,u,r,p,gpu=False,previous_command=[hover,-.12,0.,0.])
    assert matched[1]==reversed_rate[1]==0
    assert np.isclose(reversed_rate[0]-matched[0],.4*.24**2,atol=1e-6)

def test_prediction_uses_sender_slew_budget_for_first_command(engine):
    x,u,r,p=inputs(engine,2);u[0,0,1]=2.;u[0,1,1]=2.
    check,_,actions=engine.rollout(x,u,r,p,gpu=False,
                                    previous_command=[MASS*9.8066,0.,0.,0.])
    assert check[1]==0 and len(actions)==2
    from convex_timevarying_window.gs_mppi.direct_ctbr.safety import SLEW_PER_SEND
    assert actions[0,1]<=SLEW_PER_SEND[1]+1e-9
    assert actions[1,1]-actions[0,1]<=2*SLEW_PER_SEND[1]+1e-9

def test_seed_turns_nose_toward_planned_horizontal_tangent(engine):
    x=hover_state(engine.track.start);x[3:6]=[0.,4.,0.]
    reference=np.tile(np.r_[x[:3],x[3:6],np.zeros(3)],(6,1))
    seed=engine.seed(x,reference,engine.poses(np.arange(21)*.005),0,0,.02)
    assert seed[0,3]>0.5

def test_seed_heading_uses_reference_not_sideways_measured_velocity(engine):
    x=hover_state(engine.track.start);x[3:6]=[0.,4.,0.]
    reference=np.tile(np.r_[x[:3],[0.,-4.,0.],np.zeros(3)],(3,1))
    seed=engine.seed(x,reference,engine.poses(np.arange(9)*.005),0,0,.02)
    assert seed[0,3]<-.5

def test_tangent_yaw_wraps_continuously_and_holds_at_zero_speed(engine):
    x=hover_state(engine.track.start);yaw=np.deg2rad(179.)
    x[6:10]=[np.cos(yaw/2),0.,0.,np.sin(yaw/2)]
    tangent=4*np.array([np.cos(np.deg2rad(-179.)),np.sin(np.deg2rad(-179.)),0.])
    reference=np.tile(np.r_[x[:3],tangent,np.zeros(3)],(3,1));poses=engine.poses(np.arange(9)*.005)
    seed=engine.seed(x,reference,poses,0,0,.02)
    assert 0.<seed[0,3]<.2
    reference[:,3:6]=0.
    held=engine.seed(x,reference,poses,0,0,.02)
    assert np.allclose(held[:,3],0.,atol=1e-12)

def test_transit_heading_cost_uses_trajectory_tangent_not_velocity(engine):
    x,u,r,p=inputs(engine,1);x[3:6]=[0.,4.,0.];r[:,3:6]=[4.,0.,0.]
    tangent_aligned,_,_=engine.rollout(x,u,r,p,gpu=False)
    x[6:10]=[np.sqrt(.5),0.,0.,np.sqrt(.5)]
    velocity_aligned,_,_=engine.rollout(x,u,r,p,gpu=False)
    assert tangent_aligned[1]==velocity_aligned[1]==0
    assert tangent_aligned[0]<velocity_aligned[0]-.005

def test_motor_acceleration_headroom_is_filtered_before_hard_limit(engine):
    x,u,r,p=inputs(engine,1);u[0,0]=[200.,0.,0.,0.]
    m,_,commands=engine.rollout(x,u,r,p,gpu=False,previous_command=u[0,0])
    assert m[1]==0 and m[2]<0 and len(commands)==1
    assert commands[0,0]<=4*8.54858

def test_high_rate_state_is_not_mistaken_for_hover(engine):
    x,u,r,p=inputs(engine,1);x[10]=4.5
    m,_,_=engine.rollout(x,u,r,p,gpu=False)
    assert int(m[1])&16

def test_outward_tilt_is_discouraged_before_hard_limit(engine):
    x,u,r,p=inputs(engine,1);angle=np.deg2rad(39.)
    x[6:10]=[np.cos(angle/2),np.sin(angle/2),0.,0.]
    x[10]=.3;u[0,0,1]=.3
    outward,_,_=engine.rollout(x,u,r,p,gpu=False,previous_command=u[0,0])
    x[10]=-.3;u[0,0,1]=-.3
    inward,_,_=engine.rollout(x,u,r,p,gpu=False,previous_command=u[0,0])
    assert outward[1]==inward[1]==0
    assert outward[0]>inward[0]+.01

def test_physical_collision_not_soft_penalty(engine):
    x,u,r,p=inputs(engine,1);w=engine.track.windows[0];c,rot,*_=w.state_at(0.)
    point=engine.geometry.frames[0].physical_vertices[0];x[:3]=c+rot[:,:2]@point
    m,_,commands=engine.rollout(x,u,r,p,gpu=False)
    assert int(m[1])&1 and len(commands)==0

def test_known_window_time_changes_prediction(engine):
    now=engine.poses(np.array([0.]));future=engine.poses(np.array([.5]))
    assert not np.allclose(now,future)

def test_later_rollout_failure_does_not_reject_a_safe_first_command(engine):
    x=hover_state(engine.track.start)
    u=np.tile([32.,0.,0.,0.],(1,100,1))
    ref=np.tile(np.r_[x[:3],np.zeros(6)],(101,1))
    poses=engine.poses(np.arange(401)*.005)
    strict,_,_=engine.rollout(x,u,ref,poses,gpu=False)
    graded,path,commands=engine.rollout(x,u,ref,poses,gpu=False,soft_future=True)
    assert strict[1]!=0 and strict[6]>1
    assert graded[1]==strict[1] and graded[7]==1
    assert np.isfinite(graded[0]) and len(commands)==int(graded[6])
    assert len(path)==len(commands)+1

def test_far_infinite_plane_sweep_is_not_gate_entry(engine):
    from convex_timevarying_window.gs_mppi.progress import Progress
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,2*np.pi/3,0.,10.,4.,1.)
    moving=Engine(track,gpu=False)
    # Regression from the high-motion case: the *stationary* vehicle is far
    # from W2's finite opening, while W2's rotating plane sweeps over it.
    p=np.array([11.13782745,-1.05420138,1.61535673])
    progress=Progress(track,moving.config,moving.signs);progress.first=1
    assert not progress.update(p,5.68,p,5.76)
    assert progress.phase==0 and progress.violation is None

def test_batch_converts_phase_degrees_to_radians(tmp_path,monkeypatch):
    from convex_timevarying_window.gs_mppi.direct_ctbr import batch
    import sys
    seen=[]
    def fake_run(args):
        seen.append(args.phase);return {'completed':True}
    monkeypatch.setattr(batch,'run',fake_run)
    monkeypatch.setattr(sys,'argv',['batch','--outdir',str(tmp_path/'matrix')])
    batch.main()
    assert len(seen)==12
    assert np.allclose(seen[-4:],[0.,2*np.pi/3,0.,2*np.pi/3])

def test_bodyframe_encoder_preserves_roll_and_flips_pitch_yaw():
    rate,thrust=encode_ctbr(np.array([MASS*9.8066,.2,.3,.4]))
    assert np.allclose(rate,[.2,-.3,-.4]) and 0<thrust<1

def test_activation_does_not_extend_prediction_horizon():
    from convex_timevarying_window.gs_mppi.direct_ctbr.px4 import remaining_tape
    tape=np.arange(400.).reshape(100,4)
    for elapsed in (0.,.02,.031,.055):
        remaining,skip=remaining_tape(tape,3.,3.+elapsed,.02)
        assert 3.+elapsed+len(remaining)*.02<=5.+1e-9
        assert np.array_equal(remaining,tape[skip:])

def test_actual_crossing_phase_and_state_have_one_timestamp():
    from convex_timevarying_window.gs_mppi.direct_ctbr.px4 import Observations
    class Progress:
        phase=1;violation=None
        def update(self,p0,t0,p1,t1):
            if p0[0]<=0<p1[0]:self.phase=2
            return False
    class Geometry:
        from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
        config=Config()
        def interval_clearances(self,*args,**kwargs):return np.array([1.])
    p=Progress();obs=Observations(p,Geometry())
    before=np.zeros(17);before[0]=-.1
    after=before.copy();after[0]=.1
    obs.update(1.,before)
    at,x,_=obs.update(1.02,after) # main thread can observe before sender
    assert p.phase==2 and at==1.02 and x[0]>.0
    at,x,_=obs.update(1.01,before) # late sender must not rewind events/state
    assert p.phase==2 and at==1.02 and x[0]>.0

def test_nonlinear_projection_can_require_second_iteration(engine):
    # Saved PX4 state: one linear correction left a negative acceleration
    # residual. A second nonlinear relinearization resolves it, without
    # loosening the 8 m/s^2 limit or modifying the sampling configuration.
    x=np.array([3.4661058938363567,-3.847687336616218,7.320210685729981,
        7.357523360856665,1.546422310355958,1.8325353384617784,
        .5996190432068598,-.20516975786300418,.1465037183950105,.7595386981441837,
        -.13220633566379547,.12822335958480835,.010952180251479149,
        925.4135929729156,951.1827411135903,943.5939726654588,923.6949473207906])
    u=np.array([30.4821625,-.110415168,.306950361,.00311407307])[None,None]
    r=np.tile(np.r_[x[:3],np.zeros(6)],(2,1));p=engine.poses(4.708+np.arange(5)*.005)
    a,_,_=engine.rollout(x,u,r,p,1,0,gpu=False,previous_command=u[0,0])
    b,_,action=engine.rollout(x,u,r,p,1,0,gpu=False,projection_iterations=4,previous_command=u[0,0])
    assert a[1]==0 and a[2]<0 and b[1]==0 and b[2]>=0 and len(action)==1
    assert engine.cfg[12]==1

def test_observed_chord_deducts_unknown_curvature():
    from convex_timevarying_window.gs_mppi.direct_ctbr.px4 import observed_clearance
    from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
    class Geometry:
        config=Config()
        def interval_clearances(self,p,v,t,acceleration_bound=None):
            assert acceleration_bound==0 and np.diff(t).max()<=.005+1e-12
            assert np.allclose(p[:,0],t/.024)
            return np.ones(len(t)-1)*.1
    a=np.zeros(17);b=a.copy();b[0]=1
    g=Geometry();value=observed_clearance(g,0.,a,.024,b)
    bound=4*g.config.max_rotor_thrust/MASS+9.8066+4*8.06428e-5*1000/MASS*g.config.speed
    assert np.isclose(value,.1-bound*.024**2/8) and value<.1

def test_distant_rotating_plane_does_not_take_over_intergate_transit(engine):
    from convex_timevarying_window.gs_mppi.direct_ctbr.controller import Controller
    # A moving plane can appear close in signed distance even when its finite
    # opening remains far away. The inter-gate TOGT reference stays active.
    x=np.zeros(17)
    x[:6]=[-17.29455899260938,-5.784524861723185,7.131136169433594,
           3.133490714870184,1.5794132548468658,.106192641040266]
    ctrl=Controller.__new__(Controller);ctrl.track=engine.track;ctrl.engine=engine;ctrl.prepare_gate=None
    assert ctrl.stage_at(x,18.004,4,0)=='TRANSIT'
    assert ctrl.stage_at(x,18.024,4,1)=='CROSS'

def test_prepare_candidates_include_a_full_horizon_speed_shedding_maneuver():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,0.,0.,10.,1.,1.)
    e=Engine(track,gpu=False)
    # Previously failed PX4 state at W2. The original prepare seed fails, but
    # the physically identical plant admits a full two-second CTBR tape.
    x=np.array([14.159872028511018,8.314174770959653,4.419070949554444,
        6.362006731162705,8.145233820445018,-1.787576449387714,
        .7089815032155694,-.03543049160390479,-.10498728125046784,.6964679311630453,
        -1.2280257940292358,-1.2052905559539795,-.02393939159810543,
        784.2303484907836,939.8349199714352,921.5042612927357,886.1621864755755])
    now=6.248;poses=e.poses(now+np.arange(401)*.005)
    reference=np.tile(np.r_[x[:3],np.zeros(6)],(101,1))
    aggressive=e.seed(x,reference,poses,1,1,.02)
    slow_reference=np.tile(np.r_[x[:3],.4*x[3:6],np.zeros(3)],(101,1))
    slower=e.seed(x,slow_reference,poses,1,0,.02)
    bad,_,_=e.rollout(x,aggressive[None],reference,poses,1,0,1,gpu=False)
    good,_,_=e.rollout(x,slower[None],reference,poses,1,0,1,gpu=False)
    assert bad[1]!=0 and good[1]==0 and good[6]==100

def test_transit_hazard_rendezvous_turns_away_from_workspace_face(engine):
    from convex_timevarying_window.gs_mppi.direct_ctbr.controller import Controller
    x=np.array([22.562618434196338,14.055227456614375,1.7594048881530764,
        4.855836202730892,-.9236856925086534,-.3701728843340379,
        .6091846313810566,.008539639957772988,-.05948802466124453,.7907479588088868,
        -.1942184716463089,.9070896506309509,.009005583822727203,
        794.3068468815869,768.6073665506864,803.7617813544562,792.0589256996783])
    ctrl=Controller.__new__(Controller);ctrl.track=engine.track;ctrl.steps=100;ctrl.dt=.02
    ref=ctrl.avoidance_reference(x,9.144,2)
    # The old constant-velocity escape would hit the x=28 m workspace face.
    assert x[0]+2*x[3]>engine.config.upper[0]
    assert ref[-1,0]<engine.config.upper[0]
    poses=engine.poses(9.144+np.arange(401)*.005)
    u=engine.seed(x,ref,poses,2,0,.02)
    check,_,_=engine.rollout(x,u[None],ref,poses,2,0,0,gpu=False)
    assert check[1]==0 and check[6]==100

def test_prepare_and_cross_generate_different_physical_controls(engine):
    x=np.zeros(17);x[:6]=[-.38076805,-4.18362326,3.14402699,7.35752336,1.54642231,1.83253534]
    x[6]=1.;x[13:]=850.
    t=20.252;poses=engine.poses(t+np.arange(401)*.005)
    ref=np.tile(np.r_[x[:3],np.zeros(6)],(101,1))
    prep=engine.seed(x,ref,poses,4,1,.02)
    cross=engine.seed(x,ref,poses,4,2,.02)
    assert prep.shape==cross.shape==(100,4)
    assert not np.allclose(prep[:10],cross[:10])

def test_fast_gate_rotation_does_not_send_distant_prepare_seed_away():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,0.,0.,10.,1.,1.)
    e=Engine(track,gpu=False);x=hover_state(track.start)
    poses=e.poses(np.arange(401)*.005)
    reference=np.tile(np.r_[x[:3],np.zeros(6)],(101,1))
    controls=e.seed(x,reference,poses,0,1,.02)
    _,start_z=track.windows[0].world_to_local(x[:3],0.)
    for k in range(50):
        for _ in range(4):x=e.integrate(x,controls[k],.005)
    _,end_z=track.windows[0].world_to_local(x[:3],1.)
    assert e.signs[0]*end_z>e.signs[0]*start_z
    assert np.linalg.norm(x[3:6])>2.

def test_aligned_prepare_seed_preserves_speed_near_entry():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track,gpu=False);c,r,*_=track.windows[0].state_at(0.)
    normal=e.signs[0]*r[:,2]
    x=hover_state(c-(sum(e.cfg[:3])+1.)*normal);x[3:6]=6*normal
    ref=np.tile(np.r_[x[:3],x[3:6],np.zeros(3)],(26,1))
    seed=e.seed(x,ref,e.poses(np.arange(101)*.005),0,1,.02)
    for command in seed[:15]:x=e.integrate(x,command,.02)
    # An aligned gate is not a stopping target one metre before its slab.
    assert np.dot(x[3:6],normal)>6.

def test_predicted_entry_uses_cross_objective_regardless_of_initial_mode():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track,gpu=False);c,r,*_=track.windows[0].state_at(0.)
    normal=e.signs[0]*r[:,2];x=hover_state(c-.1*normal);x[3:6]=3*normal
    ref=np.tile(np.r_[x[:3],x[3:6],np.zeros(3)],(11,1));poses=e.poses(np.arange(41)*.005)
    controls=np.tile([MASS*9.8066,0.,0.,0.],(1,10,1))
    prepare,_,_=e.rollout(x,controls,ref,poses,0,1,1,gpu=False)
    cross,_,_=e.rollout(x,controls,ref,poses,0,1,2,gpu=False)
    assert prepare[1]==cross[1]==0
    assert np.allclose(prepare,cross,atol=1e-12)

def test_exit_continuation_changes_value_when_next_gate_changes():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track,gpu=False);c,r,*_=track.windows[0].state_at(0.)
    normal=e.signs[0]*r[:,2]
    x=hover_state(c+(sum(e.cfg[:3])-.03)*normal);x[3:6]=4*normal
    ref=np.tile(np.r_[x[:3],x[3:6],np.zeros(3)],(11,1));poses=e.poses(np.arange(41)*.005)
    controls=np.tile([MASS*9.8066,0.,0.,0.],(1,10,1))
    # Both alternate gates stay remote and collision-free. Their different
    # future directions must nevertheless influence the exit-state value.
    poses[:,1,:3]=x[:3]+20*normal
    toward,_,_=e.rollout(x,controls,ref,poses,0,2,2,gpu=False)
    poses[:,1,:3]=x[:3]-20*normal
    away,_,_=e.rollout(x,controls,ref,poses,0,2,2,gpu=False)
    assert toward[1]==away[1]==0 and toward[3]==away[3]==1
    assert toward[0]<away[0]

def test_cpu_gpu_agree_across_predicted_center_and_exit_events():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track);c,r,*_=track.windows[0].state_at(0.)
    normal=e.signs[0]*r[:,2];x=hover_state(c-.1*normal);x[3:6]=3*normal
    controls=np.tile([MASS*9.8066,0.,0.,0.],(1,40,1))
    ref=np.tile(np.r_[x[:3],x[3:6],np.zeros(3)],(41,1));poses=e.poses(np.arange(161)*.005)
    cpu,_,_=e.rollout(x,controls,ref,poses,0,1,1,gpu=False)
    gpu=e.rollout(x,controls,ref,poses,0,1,1).get()[0]
    assert cpu[1]==gpu[1]==0 and cpu[3]==gpu[3]==1
    assert np.allclose(cpu,gpu,atol=2e-3,rtol=1e-3)

def test_finite_entry_guard_ignores_far_plane_but_preserves_near_bypass_rule():
    from convex_timevarying_window.gs_mppi.progress import Progress
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track,gpu=False);w=track.windows[0];c,r,*_=w.state_at(0.)
    normal=e.signs[0]*r[:,2];slab=sum(e.cfg[:3]);radius=np.linalg.norm(w.aperture.physical_vertices,axis=1).max()
    # Use the horizontal aperture axis so the far-field fixture stays inside
    # the real workspace; the vertical axis would exceed the ceiling.
    tangent=r[:,:2][:,np.argmin(np.abs(r[2,:2]))]
    far=c+10*tangent
    a=far-(slab+.05)*normal;b=far-(slab-.05)*normal
    legacy=Progress(track,e.config,e.signs)
    finite=Progress(track,e.config,e.signs,finite_entry_only=True)
    legacy.update(a,0.,b,.02);finite.update(a,0.,b,.02)
    assert legacy.violation=='outside_aperture'
    assert finite.violation is None and finite.first==finite.phase==0
    x=hover_state(a);x[3:6]=6*normal
    u=np.array([MASS*9.8066,0.,0.,0.])[None,None]
    ref=np.tile(np.r_[x[:3],x[3:6],np.zeros(3)],(2,1))
    check,_,_=e.rollout(x,u,ref,e.poses(np.arange(5)*.005),gpu=False)
    assert check[1]==0 and check[3]==check[4]==0
    near=c+(radius+.1)*tangent
    finite=Progress(track,e.config,e.signs,finite_entry_only=True)
    finite.update(near-(slab+.05)*normal,0.,near-(slab-.05)*normal,.02)
    assert finite.violation=='outside_aperture'

def test_curved_reference_prevents_false_early_ballistic_takeover():
    from types import SimpleNamespace
    from convex_timevarying_window.gs_mppi.direct_ctbr.controller import Controller
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track,gpu=False);c,r,*_=track.windows[0].state_at(0.)
    normal=e.signs[0]*r[:,2];tangent=r[:,0]
    class Curve:
        total_time=3.
        def evaluate(self,t,d=0):
            t=np.asarray(t)
            if d==0:return c+(t[...,None]-1.5)*8*normal+3*((1-t[...,None]/1.5)**2)*tangent
            return np.broadcast_to(8*normal,t.shape+(3,)).copy()
    trajectory=Curve();x=hover_state(trajectory.evaluate(0));x[3:6]=8*normal
    ref=SimpleNamespace(first=0,made_at=0.,crossing_times=np.array([1.5]),trajectory=trajectory)
    ctrl=Controller.__new__(Controller);ctrl.track=track;ctrl.engine=e;ctrl.prepare_gate=None;ctrl.racing_guidance=True
    assert ctrl.stage_at(x,0.,0,0)=='PREPARE'
    ctrl.prepare_gate=None
    assert ctrl.stage_at(x,0.,0,0,ref)=='TRANSIT'
    assert ctrl.handoff_hint['aligned']
    x[:3]=trajectory.evaluate(.8)
    assert ctrl.stage_at(x,.8,0,0,ref)=='PREPARE'
    assert ctrl.stage_at(x,.8,0,1,ref)=='CROSS'

def test_reference_hint_preserves_measured_tracking_offset():
    from types import SimpleNamespace
    from convex_timevarying_window.gs_mppi.direct_ctbr.controller import Controller
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track,gpu=False);c,r,*_=track.windows[0].state_at(0.)
    normal=e.signs[0]*r[:,2]
    class Curve:
        total_time=3.
        def evaluate(self,t,d=0):
            t=np.asarray(t)
            if d==0:return c+(t[...,None]-1.5)*8*normal
            return np.broadcast_to(8*normal,t.shape+(3,)).copy()
    curve=Curve();ref=SimpleNamespace(first=0,made_at=0.,crossing_times=np.array([1.5]),trajectory=curve)
    ctrl=Controller.__new__(Controller);ctrl.track=track;ctrl.engine=e
    x=hover_state(curve.evaluate(0));assert ctrl.reference_passage_hint(x,0.,0,ref)['aligned']
    x[:3]+=2*r[:,0]
    assert not ctrl.reference_passage_hint(x,0.,0,ref)['aligned']

def test_prepare_seed_brakes_for_reversal_but_not_straight_exit():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,translation_scale=0.,rotation_scale=0.)
    e=Engine(track,gpu=False,racing_guidance=True);c,r,*_=track.windows[0].state_at(0.);normal=e.signs[0]*r[:,2]
    x=hover_state(c-(sum(e.cfg[:3])+.4)*normal);x[3:6]=6*normal
    ref=np.tile(np.r_[x[:3],x[3:6],np.zeros(3)],(31,1));poses=e.poses(np.arange(121)*.005)
    projected=[]
    for direction in (normal,-normal):
        poses[:,1,:3]=c+20*direction
        controls=e.seed(x,ref,poses,0,1,.02);state=x.copy()
        for command in controls[:15]:state=e.integrate(state,command,.02)
        projected.append(np.dot(state[3:6],normal))
    assert projected[0]>projected[1]+.2

def test_experimental_racing_guidance_is_opt_in():
    from convex_timevarying_window.gs_mppi.direct_ctbr.experiment import parser
    assert not parser().parse_args(['--outdir','unused']).experimental_racing_guidance
    assert parser().parse_args(['--outdir','unused','--experimental-racing-guidance']).experimental_racing_guidance
    assert Engine(build_track()[0],gpu=False).cfg[16]==0.

def test_reference_jerk_generates_anticipatory_rate_seed():
    e=Engine(build_track()[0],gpu=False,coupled_racing=True)
    baseline=Engine(build_track()[0],gpu=False)
    x,u,reference,poses=inputs(e,20)
    reference[:,7]=2*np.arange(len(reference))*.02
    anticipatory=e.seed(x,reference,poses,0,0,.02)
    legacy=baseline.seed(x,reference,poses,0,0,.02)
    assert anticipatory[0,1]<legacy[0,1]-.1
    assert np.isfinite(anticipatory).all()

def test_gpu_cpu_coupled_soft_clearance_agree():
    e=Engine(build_track()[0],coupled_racing=True);x,u,reference,poses=inputs(e,20)
    cpu=e.rollout(x,u,reference,poses,mode=1,gpu=False)[0]
    gpu=e.rollout(x,e.cp.asarray(u,dtype=e.cp.float32),reference,poses,mode=1).get()[0]
    assert np.allclose(cpu,gpu,rtol=2e-4,atol=2e-4)
