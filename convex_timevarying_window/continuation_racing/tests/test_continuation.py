import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.planner import Config, coefficients, Plan
from convex_timevarying_window.continuation_racing.planner import Planner


def lightweight():
    p=Planner.__new__(Planner);p.track,_=build_track();p.windows=list(p.track.windows)
    p.cfg=Config(workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
    p.frames=[]
    return p


def test_each_horizon_has_terminal_rest_node():
    p=lightweight();n=len(p.windows)
    for first in range(n+1):
        ids=p.horizon_ids(first)
        assert ids[-1]==n
        assert len(ids)<=p.cfg.horizon+1
        assert len(ids)==len(set(ids))
    np.testing.assert_allclose(p.terminal_position(n),p.track.goal)


def test_terminal_rule_uses_geometry_not_window_id():
    p=lightweight()
    for first in range(len(p.windows)-p.cfg.horizon):
        last=first+p.cfg.horizon-1
        np.testing.assert_allclose(p.terminal_position(first),.5*(p.windows[last].center0+p.windows[last+1].center0))


def test_last_segment_checks_its_own_adjacent_frames():
    p=lightweight()
    assert list(p.segment_frames(0,len(p.windows),3))==[2,3]
    assert list(p.segment_frames(2,len(p.windows),3))==[4,5]


def test_workspace_margin_is_part_of_prefix_clearance():
    p=lightweight();point=np.array([27.8,0.,5.]);z=np.zeros(3)
    c=coefficients(point,z,z,point,z,z,1.)
    plan=Plan(0.,0,np.ones(1),c[None],np.zeros(9),0.,0.,'test')
    assert abs(p.prefix_clearance(plan,0.)-.2)<1e-10
    assert p.prefix_clearance(plan,0.)<p.cfg.tracking_margin


def test_rest_endpoint_is_holdable_after_plan_end():
    z=np.zeros(3);p=np.array([3.,2.,5.]);c=coefficients(z,np.ones(3),z,p,z,z,2.)
    plan=Plan(0.,0,np.array([2.]),c[None],np.zeros(9),0.,0.,'test')
    np.testing.assert_allclose(plan.sample(10.),p,atol=1e-10)
    for d in (1,2):np.testing.assert_allclose(plan.sample(10.,d),z,atol=1e-10)


def test_terminal_projection_avoids_another_windows_swept_ball():
    from types import SimpleNamespace
    from dataclasses import replace
    p=lightweight();p.cfg=replace(p.cfg,horizon=1)
    p.windows[0].center0=np.array([0.,0.,5.]);p.windows[1].center0=np.array([10.,0.,5.])
    p.frames=[SimpleNamespace(center0=np.array([5.,0.,5.]),
        aperture=SimpleNamespace(kind='circle',radius=1.),
        motion=SimpleNamespace(translation_enabled=False))]
    target=p.terminal_position(0)
    center,radius=next(p.swept_balls())
    assert np.linalg.norm(target-center)>=radius+.049999
    assert 0<target[0]<10


def test_unexecuted_candidate_does_not_lose_travel_time():
    p=lightweight();p.warm_start_duals=False;p.previous=None
    values=np.array([3.,0.,0.,1.,0.,0.,0.,0.,0.])
    p.search=Plan(0.,0,np.array([3.]),np.zeros((1,6,3)),values,1.,1.,'infeasible')
    p.horizon_ids=lambda first:[0]
    class Solver:
        def __call__(self,**kwargs):
            self.guess=kwargs['x0'];return {'x':values}
        def stats(self):return {'return_status':'test'}
    solver=Solver()
    p.solvers=[([0],solver,lambda x,par:(np.array([1.]),np.zeros((6,3))),
                np.full(9,-100.),np.full(9,100.),np.array([0.]),np.array([0.]))]
    p.solve(np.r_[p.track.start,np.zeros(6)],1.,0)
    assert solver.guess[0]==3.
    p.commit(p.search)
    p.solve(np.r_[p.track.start,np.zeros(6)],1.5,0)
    assert solver.guess[0]==2.5


def test_command_preview_does_not_shift_physical_reference_clock():
    from types import SimpleNamespace
    from convex_timevarying_window.receding_gate_racing.px4_experiment import command_plan
    active=SimpleNamespace(made_at=0.);pending=SimpleNamespace(made_at=1.)
    selected,t=command_plan(active,pending,.96,.05)
    assert selected is pending and abs(t-1.01)<1e-12
    assert active.made_at==0. and pending.made_at==1.
    selected,t=command_plan(active,pending,.94,.05)
    assert selected is active and abs(t-.99)<1e-12
    selected,t=command_plan(active,pending,.96,0.)
    assert selected is active and t==.96


def test_anticipation_uses_a_fixed_anchor_and_not_the_terminal_parking_node():
    from convex_timevarying_window.receding_gate_racing.px4_experiment import planning_anchor
    p=Plan(0.,0,np.array([3.,4.,2.]),np.zeros((3,6,3)),np.zeros(27),0,0,'test')
    assert planning_anchor(p,0,2.3,.15,2,7,.8)==(1,3.15)
    assert planning_anchor(p,0,2.7,.15,2,7,.8)==(1,3.15)
    assert planning_anchor(p,1,6.4,.15,2,7,.8)==(2,7.15)
    assert planning_anchor(p,2,8.5,.15,2,7,.8)==(2,8.65)
