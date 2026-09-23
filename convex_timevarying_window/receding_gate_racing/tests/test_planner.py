import casadi as ca
import numpy as np
import pytest

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.planner import (
    Config, Plan, Planner, coefficients, poly, symbolic_pose, symbolic_clearance_sq,
)


def test_quintic_matches_both_boundary_states():
    rng=np.random.default_rng(2)
    p0,v0,a0,p1,v1,a1=rng.normal(size=(6,3)); T=2.7
    C=coefficients(p0,v0,a0,p1,v1,a1,T)
    for k,(left,right) in enumerate(zip((p0,v0,a0),(p1,v1,a1))):
        np.testing.assert_allclose(poly(C,0,T,k),left,atol=1e-12)
        np.testing.assert_allclose(poly(C,1,T,k),right,atol=1e-12)


def test_symbolic_geometry_agrees_with_independent_window_model():
    track,_=build_track(); t=ca.SX.sym('t'); p=ca.SX.sym('p',3)
    rng=np.random.default_rng(9)
    for i,w in enumerate(track.windows):
        c,R=symbolic_pose(w,t)
        f=ca.Function(f'geometry{i}',[p,t],[c,R,symbolic_clearance_sq(w,p,t)])
        for time in (0.,1.7,8.1,17.3):
            point=w.center0+rng.normal(size=3)
            cc,rr,dd=f(point,time); cn,rn,*_=w.state_at(time)
            np.testing.assert_allclose(np.asarray(cc).ravel(),cn,atol=1e-12)
            np.testing.assert_allclose(rr,rn,atol=1e-12)
            q,z=w.world_to_local(point,time)
            np.testing.assert_allclose(float(dd),w.aperture.boundary_distance(q)**2+z*z,atol=1e-10)


def test_plan_uses_absolute_time_and_clamps_final_state():
    zero=np.zeros(3); end=np.array([3.,1.,2.])
    C=coefficients(zero,zero,zero,end,zero,zero,2.)
    plan=Plan(11.,0,np.array([2.]),C[None],np.zeros(9),0.,0.,'test')
    np.testing.assert_allclose(plan.sample(11.),zero)
    np.testing.assert_allclose(plan.sample(13.),end)
    np.testing.assert_allclose(plan.sample(15.,1),zero,atol=1e-12)


def test_failed_solve_cannot_poison_feasible_shifted_warm_start():
    track,_=build_track()
    planner=Planner.__new__(Planner)
    planner.track=track; planner.windows=list(track.windows); planner.cfg=Config()
    values=np.array([3.,0.,0.,1.,0.,0.,0.,0.,0.])
    old=Plan(0.,0,np.array([3.]),np.zeros((1,6,3)),values,1.,0.,'feasible')
    planner.previous=old
    class FailedSolver:
        def __call__(self,**kwargs): return {'x':values}
        def stats(self): return {'return_status':'Maximum_CpuTime_Exceeded'}
    planner.solvers=[([0],FailedSolver(),lambda x,p:(np.array([100.]),np.zeros((6,3))),
                      np.full(9,-100.),np.full(9,100.),np.array([0.]),np.array([0.]))]
    failed=planner.solve(np.r_[track.start,np.zeros(6)],.2,0)
    assert failed.violation==100.
    assert planner.previous is old


def test_empty_eroded_circle_is_rejected_instead_of_squaring_negative_radius():
    from convex_timevarying_window.geometry import ConvexAperture
    track,_=build_track()
    track.windows[0].aperture=ConvexAperture('circle',radius=.4)
    with pytest.raises(ValueError,match='no circular opening'):
        Planner(track)


def test_prefix_rejects_environment_boundary_even_without_nearby_gate():
    planner=Planner.__new__(Planner)
    planner.cfg=Config(workspace_lower=(-2.,-2.,0.),workspace_upper=(2.,2.,5.))
    planner.frames=[]
    C=np.zeros((1,6,3)); C[0,0]=[3.,0.,2.]
    plan=Plan(0.,0,np.array([1.]),C,np.zeros(9),0.,0.,'test')
    assert planner.prefix_clearance(plan,0.)==-np.inf


def test_local_sweep_has_consistent_bounds_at_tenfold_frequency():
    from convex_timevarying_window.receding_gate_racing.scenario import vary_track
    track,_=build_track();vary_track(track,motion_rate=10)
    planner=Planner(track,Config(crossing_constraint='local_sweep'))
    for ids,solver,evaluate,lbx,ubx,lbg,ubg in planner.solvers:
        assert np.all(lbx<=ubx)
        assert np.all(lbg<=ubg)
    # The constraints depend on the absolute motion phase, not just its bounds.
    x=np.tile([3.,0.,0.,4.,0.,0.,0.,0.,0.],3)
    p=np.r_[0.,track.start,np.zeros(6)]
    first=planner.solvers[0][2](x,p)[0]
    p[0]=.17
    second=planner.solvers[0][2](x,p)[0]
    assert np.max(np.abs(np.asarray(first)-np.asarray(second)))>.01


@pytest.mark.parametrize('center_violation,free_violation,free_cost,winner',[
    (0.,0.,6.,'center'),(0.,0.,4.,'free'),(1.,0.,6.,'free'),(0.,1.,4.,'center')])
def test_nested_refinement_never_replaces_feasible_incumbent_by_worse_or_invalid(
        center_violation,free_violation,free_cost,winner):
    from types import SimpleNamespace
    from convex_timevarying_window.receding_gate_racing.nested import NestedPlanner
    def candidate(violation):
        return Plan(0.,0,np.ones(1),np.zeros((1,6,3)),np.zeros(9),0.,violation,'test')
    center=candidate(center_violation); free=candidate(free_violation)
    nested=NestedPlanner.__new__(NestedPlanner)
    nested.center=SimpleNamespace(solve=lambda *args:center,solvers=[(None,SimpleNamespace(cost=5.))])
    nested.free=SimpleNamespace(solve=lambda *args:free,solvers=[(None,SimpleNamespace(cost=free_cost))])
    nested.selection_counts={'center':0,'free':0,'infeasible':0}
    result=nested.solve(np.zeros(9),0.,0)
    assert result is (center if winner=='center' else free)
