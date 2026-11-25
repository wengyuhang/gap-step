from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from 复现.polynomial_dynamic_racing_2023.minco import coefficients as original, basis
from convex_timevarying_window.comparisons.fixed_plane_wang.scenario import Case, build_case
from convex_timevarying_window.wang_paper_aligned_mpc.planner import Planner, Config
from convex_timevarying_window.wang_paper_aligned_mpc.trajectory import Plan, coefficients
from 复现.polynomial_dynamic_racing_2023.paper_aligned.planner import Config as WangConfig


@pytest.fixture(scope='module')
def planner():
    track, _ = build_case(Case(.8, 1.6, .4))
    return Planner(track, Config(workspace_lower=(-28.,-30.,.65), workspace_upper=(28.,22.,12.)))


def make(p, first=0, now=.15, state=None, variables=None):
    state = np.r_[p.track.start, np.zeros(9)] if state is None else state
    x = p.initial(state, now, first) if variables is None else variables
    par = np.r_[now, state, np.tile([.15,.35,.65,.85], 8)]
    _, C, _ = p.solvers[first][1](x, par)
    plan = Plan(now, first, np.repeat(np.exp(x[:2])/4,4), np.array(C).reshape(8,6,3),
                x, 0., 0., 'test', 1, terminal_goal=p.goal_in_horizon(first))
    return x, par, plan


@pytest.mark.parametrize('duration',[.35,1.,3.])
def test_four_piece_minco_and_c4(duration):
    rng = np.random.default_rng(42)
    head, tail, mid = rng.normal(size=(3,3)), rng.normal(size=(3,3)), rng.normal(size=(3,3))
    C = np.array(coefficients(head.T, tail.T, mid.T, duration)).reshape(4,6,3)
    np.testing.assert_allclose(C, original(head,tail,mid,duration), atol=1e-10)
    for i in range(3):
        for d in range(5):
            np.testing.assert_allclose(basis(1.,d)@C[i],basis(0.,d)@C[i+1],atol=1e-9)


@pytest.mark.parametrize('first',range(7))
def test_N2_fixed_forward_tail_and_dynamic_center(planner,first):
    x, _, plan = make(planner,first)
    assert planner.layout(first) == (1,2,2,5,8,26)
    assert len(plan.durations)==8 and len(x)==28 and len(plan.crossing_times)==1
    at=plan.crossing_times[0]
    np.testing.assert_allclose(plan.sample(at),planner.windows[first].state_at(at)[0],atol=1e-8)
    np.testing.assert_allclose(plan.sample(at,1),12*x[2:5],atol=1e-8)
    np.testing.assert_allclose(plan.sample(at,2),8*x[5:8],atol=1e-8)
    for d in range(3):
        np.testing.assert_allclose(plan.sample(plan.end_time,d),planner.terminal_state(first)[:,d],atol=1e-8)
    assert (np.linalg.norm(plan.sample(plan.end_time,1))<1e-7)==(first==6)


def test_warm_retimes_internal_points_and_preserves_VA_but_not_q(planner):
    x,_,old=make(planner)
    x[2:8]=[.63,.1,-.05,.1,-.06,.02];x[26:]=[.15,.12]
    _,_,old=make(planner,variables=x)
    now=.35; state=np.concatenate([old.sample(now,d) for d in range(4)])
    planner.commit(old);planner.search[0]=SimpleNamespace(x=np.full(28,999.))
    try:
        seed,_,new=make(planner,now=now,state=state)
        np.testing.assert_allclose(seed[2:8],x[2:8],atol=1e-9)
        np.testing.assert_array_equal(seed[26:],0.)
        np.testing.assert_allclose(new.crossing_times,old.crossing_times,atol=1e-9)
        expected=old.sample_many(now+(old.crossing_times[0]-now)*np.arange(1,4)/4)
        np.testing.assert_allclose(seed[8:17].reshape(3,3)*10,expected,atol=1e-9)
        assert all(r['state_source']=='admitted' for r in planner.seed_provenance)
    finally:
        planner.commit(None);planner.search.clear()


def test_baseline_is_center_constrained_and_global_seed_only():
    track,_=build_case(Case(.8,1.6,.4));p=Planner(track,WangConfig(),build_indices=[0])
    x,_,old=make(p);old.x[:]=99;p.commit(old)
    seed,_,_=make(p,now=.35)
    assert np.max(np.abs(seed))<10
    assert p.solvers[0][2][-2:].tolist()==p.solvers[0][3][-2:].tolist()==[0.,0.]
    assert all(r['state_source']=='global' for r in p.seed_provenance)


@pytest.mark.parametrize('index',[0,1,2,5,8,12,17,25,26,27])
def test_analytic_dynamic_gradient(planner,index):
    x,par,_=make(planner);fg=planner.solvers[0][0].fg;_,gradient,_=fg(x,par)
    e=np.eye(len(x))[index]*1e-6
    numerical=float(fg(x+e,par)[0]-fg(x-e,par)[0])/2e-6
    np.testing.assert_allclose(float(gradient[index]),numerical,rtol=1e-3,atol=.5)


def test_published_and_adaptation_values():
    cfg=Config()
    assert cfg.horizon==1 and cfg.pieces_per_leg==4
    assert cfg.lambda_thrust==cfg.lambda_body_rate==100.
    assert cfg.soft_clearance_weight==0. and cfg.solver_seconds==.055
