import xml.etree.ElementTree as ET
import numpy as np
from convex_timevarying_window.comparisons.single_gate_constant_velocity.scenario import Case,build_case,write_world,original
from convex_timevarying_window.comparisons.single_gate_constant_velocity.planner import Planner,Config
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.dynamic_passage_minco.safety import passage


def test_exogenous_motion_and_plane():
    a,_=build_case(Case(20));b,_=build_case(Case(-20));w=a.windows[0];R=w.state_at(0)[1]
    assert w.motion.anchor_time==b.windows[0].motion.anchor_time
    np.testing.assert_allclose(w.state_at(w.motion.anchor_time)[0],w.center0)
    for t in (0.,.1,3.,20.):
        c,r,v,*_=w.state_at(t)
        np.testing.assert_allclose(r,R)
        assert abs((c-w.center0)@R[:,2])<1e-10
        np.testing.assert_allclose(v,w.motion.velocity)


def test_world_matches_query(tmp_path):
    track,_=build_case(Case(-15));w=track.windows[0]
    root=ET.parse(write_world(track,original.ASSETS,tmp_path))
    models=[m for m in root.findall('.//world/model') if m.get('name','').startswith('gate_')]
    assert len(models)==1
    p=models[0].find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
    assert p.findtext('motion_start_time')=='30'
    v=np.fromstring(p.findtext('linear_velocity'),sep=' ');at=float(p.findtext('linear_anchor_time'))
    for t in (0.,2.,5.):np.testing.assert_allclose(w.center0+v*(t-at),w.state_at(t)[0])


def test_affine_bound_detects_sweeping_frame():
    track,_=build_case(Case(40));w=track.windows[0];R=w.state_at(0)[1]
    at=w.motion.anchor_time
    C=np.zeros((1,8,3));C[0,0]=w.center0+5*R[:,2];C[0,1]=-10*R[:,2]
    plan=Plan(at-.5,0,np.array([1.]),C,np.zeros(7),0.,0.,'test',1)
    vl=R.T@w.motion.velocity
    result=passage(plan,w,.57,translation_bounds=lambda a,b:(vl*(a-at),vl*(b-at)))
    assert not result['passed'] and result['minimum_sample_distance_m']<.57


def test_linear_pose_gradient_and_center_only_difference():
    track,_=build_case(Case(15));p=Planner(track,Config(compile_objective=False))
    x=np.r_[np.log([2.9,1.7]),.1,.2,10.,3.,.4]
    par=np.r_[.15,track.start,np.zeros(9),np.tile(np.linspace(0,.2,p.cfg.witness_slots),2)]
    out=p.function(x,par)
    for i in (0,1,2,3):
        dx=np.eye(7)[i]*1e-6;a,b=p.function(x+dx,par),p.function(x-dx,par)
        np.testing.assert_allclose(np.asarray(out[3])[:,i],np.asarray(a[2]-b[2]).ravel()/2e-6,atol=2e-4,rtol=1e-3)
    q=Planner(track,Config(compile_objective=False,free_points=False))
    for a,b in zip(out,q.function(x,par)):np.testing.assert_allclose(a,b)
    assert np.all(q.lower[2:4]==0) and np.all(q.upper[2:4]==0)
    assert p.arrival_bounds(track.windows[0],0,3,0,np.inf)==(.3,15.)
