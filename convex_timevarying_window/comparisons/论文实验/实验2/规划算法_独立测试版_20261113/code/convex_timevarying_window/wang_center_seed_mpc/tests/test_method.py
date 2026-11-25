from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest

from 复现.polynomial_dynamic_racing_2023.minco import coefficients as original_coefficients
from convex_timevarying_window.comparisons.fixed_plane_wang.scenario import Case, build_case
from convex_timevarying_window.wang_center_seed_mpc.planner import Planner, Config
from convex_timevarying_window.wang_center_seed_mpc.trajectory import coefficients, Plan
from convex_timevarying_window.wang_center_seed_mpc.reference import Reference
from convex_timevarying_window.wang_center_seed_mpc.events import plan_event_check


@pytest.fixture(scope='module')
def planner():
    track, _ = build_case(Case(.8, 1.6, .4))
    return Planner(track, Config(workspace_lower=(-28.,-30.,.65), workspace_upper=(28.,22.,12.)),
        build_indices=[0, 3, 4, 6, 7])


def values(p, first=0):
    state = np.r_[p.track.start, np.zeros(9)]
    x = p.initial(state, .15, first)
    gates, legs, *_ = p.layout(first)
    parameters = np.r_[.15, state, np.tile([.15,.35,.65,.85], 2*legs)]
    return x, parameters


def make_plan(p, x, par, first=0):
    gates, legs, *_ = p.layout(first)
    _, C, _ = p.solvers[first][1](x, par)
    return Plan(float(par[0]), first, np.repeat(np.exp(x[:legs])/2,2),
        np.array(C).reshape(2*legs,6,3), x, 0., 0., 'test', gates, terminal_goal=p.goal_in_horizon(first))


@pytest.mark.parametrize('duration',[.35,1.,3.])
def test_direct_wang_matrix_exact(duration):
    rng = np.random.default_rng(42)
    head, tail, middle = rng.normal(size=(3,3)), rng.normal(size=(3,3)), rng.normal(size=3)
    actual = np.array(coefficients(head.T, tail.T, middle, duration)).reshape(2,6,3)
    np.testing.assert_allclose(actual, original_coefficients(head, tail, [middle], duration), atol=1e-11)


@pytest.mark.parametrize('first,gates,legs,finish',[(0,3,3,False),(3,3,3,False),(4,3,4,True),(6,1,2,True),(7,0,1,True)])
def test_no_intermediate_extra_segment(planner, first, gates, legs, finish):
    assert planner.layout(first)[:2] == (gates, legs)
    x, par = values(planner, first); plan = make_plan(planner, x, par, first)
    assert len(plan.durations) == 2*legs
    assert plan.gate_count == gates and plan.terminal_goal == finish
    if finish:
        np.testing.assert_allclose(plan.sample(plan.end_time), planner.track.goal, atol=1e-8)
        np.testing.assert_allclose(plan.sample(plan.end_time,1), 0., atol=1e-8)
    else:
        assert abs(plan.crossing_times[-1]-plan.end_time)<1e-12
        assert np.linalg.norm(plan.sample(plan.end_time,1))>1.
        with pytest.raises(ValueError, match='No intermediate'):
            planner.terminal_position(first)


def test_last_gate_pva_free(planner):
    x, par = values(planner); gates, legs, vi, ai, mi, qi = planner.layout(0)
    x[qi:]=[.1,-.12,-.15,.05,.21,-.13]
    x[vi+6:vi+9]=[.3,-.2,.1]; x[ai+6:ai+9]=[.11,.2,-.1]
    plan=make_plan(planner,x,par)
    center,R,*_=planner.windows[2].state_at(plan.end_time)
    np.testing.assert_allclose(plan.sample(plan.end_time),center+R[:,:2]@x[qi+4:qi+6],atol=1e-8)
    np.testing.assert_allclose(plan.sample(plan.end_time,1),planner.cfg.speed*x[vi+6:vi+9],atol=1e-8)
    np.testing.assert_allclose(plan.sample(plan.end_time,2),planner.cfg.acceleration*x[ai+6:ai+9],atol=1e-8)


def test_center_seed_ignores_old_offsets_and_failed_candidate(planner):
    x,par=values(planner); old=make_plan(planner,x,par)
    qi=planner.layout(0)[-1];old.x[qi:]=.5
    planner.admitted=old;planner.search[0]=SimpleNamespace(x=np.full_like(x,999))
    try:
        seeded=planner.initial(par[1:13],.35,0)
        np.testing.assert_array_equal(seeded[qi:],0.)
        assert np.isfinite(seeded).all() and np.max(np.abs(seeded))<100
        new_par=par.copy();new_par[0]=.35;plan=make_plan(planner,seeded,new_par)
        for wi,at in enumerate(plan.crossing_times):
            np.testing.assert_allclose(plan.sample(at),planner.windows[wi].state_at(at)[0],atol=1e-8)
    finally:
        planner.admitted=None;planner.search.clear()


@pytest.mark.parametrize('index',[0,3,12,21,30,35])
def test_dynamic_gradient(planner,index):
    x,par=values(planner);fg=planner.solvers[0][0].fg;_,gradient,_=fg(x,par)
    e=np.eye(len(x))[index]*1e-6
    numerical=float(fg(x+e,par)[0]-fg(x-e,par)[0])/2e-6
    np.testing.assert_allclose(float(gradient[index]),numerical,rtol=5e-4,atol=.2)


def test_no_time_regression_veto(planner):
    x,par=values(planner);old=make_plan(planner,x,par)
    ref=Reference(planner.track.start);ref.gate=0;ref.active=old
    x[:3]+=np.log(1.4);candidate=make_plan(planner,x,par)
    assert candidate.crossing_times[0]>old.crossing_times[0]+.02
    assert ref.candidate_reason(candidate,0,'approach') is None
    assert ref.candidate_reason(candidate,1,'approach')=='stale_gate_stage'
    assert ref.candidate_reason(candidate,0,'exit')=='stale_gate_stage'
    with pytest.raises(RuntimeError,match='unextended_flying_horizon'):
        ref.state_at(old.end_time+.1)


def test_flying_last_endpoint_is_not_claimed_body_exit():
    track,_=build_case(Case(.8,0.,0.))
    object.__setattr__(track,'start',np.array([-2.,0.,4.]))
    for i,w in enumerate(track.windows):
        w.center0=np.array([4.+6*i,0.,4.]);w.angles0_rpy=np.array([np.pi/2,0.,np.pi/2])
    p=Planner(track,Config(),build_indices=[])
    C=np.zeros((6,6,3));C[:,0,:]=[[-2+3*i,0,4] for i in range(6)];C[:,1,0]=3.
    plan=Plan(0.,0,np.full(6,.5),C,np.zeros(36),0.,0.,'straight',3,terminal_goal=False)
    assert plan_event_check(plan,p.windows,p.signs,p.cfg)[0]
    assert np.linalg.norm(plan.sample(plan.end_time,1))==6.
    # This truncated plan does NOT demonstrate the last body's exit.
    plan.terminal_goal=True
    assert not plan_event_check(plan,p.windows,p.signs,p.cfg)[0]


def test_solver_has_no_deadline_parameters(planner):
    x,par=values(planner)
    assert planner.solvers[0][0].fg.size1_in(1)==len(par)==37
    np.testing.assert_allclose(planner.solvers[0][3][:3],np.log(15.))
