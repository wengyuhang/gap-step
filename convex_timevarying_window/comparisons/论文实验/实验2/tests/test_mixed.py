import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.论文实验.实验2.scenario import Case,build_case,pose,manifest,write_world,ASSETS
from convex_timevarying_window.comparisons.论文实验.实验2.planner import Planner
from convex_timevarying_window.togt_minco_mpc.trajectory import basis

@pytest.mark.parametrize('layout',['original','slalom','height'])
def test_shared_motion_queries_and_bounds(layout):
    track,_=build_case(Case(layout,'high'));assert len(track.windows)==7
    assert [w.motion.kind for w in track.windows].count('swing')==3
    assert np.array_equal(track.goal,track.start)
    for i,w in enumerate(track.windows):
        t=ca.MX.sym('t');c,R=pose(w,t);fn=ca.Function('p'+str(i),[t],[c,ca.jacobian(c,t)])
        for at in (.01,.137,1.75,12.234):
            c,v=fn(at);cn,rn,vn,_=w.state_at(at)
            np.testing.assert_allclose(np.array(c).ravel(),cn,atol=1e-12)
            np.testing.assert_allclose(np.array(v).ravel(),vn,atol=1e-10)
        for a,b in ((0,.003),(.137,.193),(1.1,1.7)):
            lo,hi=w.motion.local_bounds(a,b);q=w.motion.local(np.linspace(a,b,301))[0]
            assert np.all(q>=lo-1e-12) and np.all(q<=hi+1e-12)
        assert w.aperture.boundary_distance([100.,100.])>1


def test_world_seven_independent_contact_sensors():
    tr,_=build_case(Case('slalom','high'))
    with tempfile.TemporaryDirectory() as tmp:
        tree=ET.parse(write_world(tr,ASSETS,Path(tmp)));world=tree.find('world')
        gates=[m for m in world.findall('model') if m.get('name','').startswith('gate_')]
        assert len(gates)==7 and len(world.findall("model/link/sensor[@type='contact']"))==20
        assert all(m.find('plugin').get('name')=='paper_experiment::MixedGateMotion' for m in gates)
        assert all(s.findtext('update_rate')=='1000' and len(s.findall('contact/collision'))==1 for s in world.findall("model/link/sensor[@type='contact']"))

@pytest.mark.parametrize('free',[False,True])
def test_global_eight_segments_exact_centres_and_boundary(free):
    tr,_=build_case(Case('slalom','medium'));p=Planner(tr,free)
    assert p.m==7 and p.n==8 and p.full_state
    seed=p.seeds()[1];T=np.exp(seed[:8]);points=np.array(p.point_function(seed)).T
    centres=np.array([w.state_at(t)[0] for w,t in zip(tr.windows,np.cumsum(T)[:7])]);np.testing.assert_allclose(points,centres,atol=1e-10)
    if not free:
        moved=seed.copy();moved[p.velocity_start:]+=.1
        np.testing.assert_array_equal(p.point_function(moved),p.point_function(seed))
    C=np.array(p.curve(seed)[0]).ravel().reshape(8,24).reshape(8,3,8).transpose(0,2,1)
    for d in range(4):
        a=C[0].T@np.array(basis(0,d)).ravel()/T[0]**d;b=C[-1].T@np.array(basis(1,d)).ravel()/T[-1]**d
        np.testing.assert_allclose(a,tr.start if d==0 else 0,atol=1e-8)
        np.testing.assert_allclose(b,tr.goal if d==0 else 0,atol=1e-8)
    for k in range(7):
        for d in range(4):np.testing.assert_allclose(C[k].T@np.array(basis(1,d)).ravel()/T[k]**d,C[k+1].T@np.array(basis(0,d)).ravel()/T[k+1]**d,atol=1e-8)


def test_swing_safety_is_not_a_frozen_centre_check():
    from convex_timevarying_window.comparisons.论文实验.实验2.safety import passage
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
    for level,expected in [('static',True),('high',False)]:
        track,_=build_case(Case('slalom',level));w=track.windows[1]
        C=np.zeros((2,8,3));C[0,0]=w.center0-2*w.R[:,2];C[0,1]=2*w.R[:,2];C[1,0]=w.center0;C[1,1]=2*w.R[:,2]
        plan=Plan(0.,0,np.ones(2),C,np.zeros(1),0.,0.,'test',1)
        results=[passage(plan,w,.38594472+.085+.1,k) for k in (0,1)]
        assert all(r['passed'] for r in results)==expected


def test_wang_external_core_keeps_N2_M4_and_dynamic_query():
    from convex_timevarying_window.comparisons.论文实验.实验2.baseline import Config,Planner as Wang
    from convex_timevarying_window.wang_paper_aligned_mpc.trajectory import Plan as WangPlan
    track,_=build_case(Case('slalom','medium'));p=Wang(track,Config())
    assert p.cfg.horizon==1 and p.cfg.pieces_per_leg==4
    assert p.cfg.lambda_thrust==100 and p.cfg.lambda_body_rate==100 and p.cfg.solver_seconds==.055
    assert not p.cfg.free_points
    for first in (0,1):
        head=np.r_[track.start,np.zeros(9)];x=p.initial(head,.15,first);par=np.r_[.15,head,np.tile([.15,.35,.65,.85],8)]
        _,C,_=p.solvers[first][1](x,par)
        plan=WangPlan(.15,first,np.repeat(np.exp(x[:2])/4,4),np.array(C).reshape(8,6,3),x,0,0,'test',1,terminal_goal=False)
        at=plan.crossing_times[0];np.testing.assert_allclose(plan.sample(at),track.windows[first].state_at(at)[0],atol=1e-8)
        assert len(plan.durations)==8
