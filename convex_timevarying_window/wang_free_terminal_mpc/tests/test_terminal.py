from dataclasses import replace
import casadi as ca
import numpy as np
import pytest
from 复现.polynomial_dynamic_racing_2023.minco import basis
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case, build_case
from convex_timevarying_window.comparisons.original_fixed_plane_ours import adapter
from convex_timevarying_window.wang_paper_aligned_mpc import planner as old_module, events
from convex_timevarying_window.wang_paper_aligned_mpc.trajectory import coefficients, Plan
from convex_timevarying_window.wang_free_terminal_mpc import planner as module
from convex_timevarying_window.wang_free_terminal_mpc.trajectory import from_terminal_jerk


@pytest.fixture(scope='module')
def planner():
    with pytest.MonkeyPatch.context() as patch:
        for target in (old_module, module):
            for name in ('symbolic_pose', 'symbolic_clearance_sq'):
                patch.setattr(target, name, getattr(adapter, name))
        patch.setattr(old_module, 'frame_clearances', adapter.frame_clearances)
        patch.setattr(events, 'local_coordinates', adapter.local_coordinates)
        track, _ = build_case(Case(1., 2., 0.))
        yield module.Planner(track, module.Config(workspace_lower=(-28.,-30.,.65), workspace_upper=(28.,22.,12.)),
                             build_indices=[0,1,6])


def make(p, first=0, now=.15, state=None, variables=None):
    state = np.r_[p.track.start, np.zeros(9)] if state is None else state
    x = p.initial(state, now, first) if variables is None else variables.copy()
    par = np.r_[now, state, np.tile([.15,.35,.65,.85],8)]
    g, C, _ = p.solvers[first][1](x, par)
    plan = Plan(now, first, np.repeat(np.exp(x[:2])/4,4), np.asarray(C).reshape(8,6,3), x, 0., 0., 'test',1,
                terminal_goal=p.goal_in_horizon(first))
    return x, par, plan


@pytest.mark.parametrize('duration',[.35,1.,3.])
def test_jerk_coordinates_preserve_minco_family_and_C4(duration):
    rng = np.random.default_rng(23)
    head, tail, middle = [rng.normal(size=(3,3)) for _ in range(3)]
    C = np.asarray(coefficients(head,tail,middle,duration)).reshape(4,6,3)
    jerk = basis(1.,3)@C[-1]/(duration/4)**3
    actual = np.asarray(from_terminal_jerk(head,tail,middle[:,:2],jerk,duration)).reshape(4,6,3)
    np.testing.assert_allclose(actual,C,atol=2e-10)
    for i in range(3):
        for d in range(5):
            np.testing.assert_allclose(basis(1.,d)@actual[i],basis(0.,d)@actual[i+1],atol=1e-9)


def test_all_twelve_terminal_components_independent(planner):
    x,_,plan = make(planner)
    indices = list(range(28,37))+list(range(23,26))
    scales = [10.]*3+[planner.cfg.speed]*3+[planner.cfg.acceleration]*3+[planner.cfg.jerk]*3
    original = np.concatenate([plan.sample(plan.end_time,d) for d in range(4)])
    for k,(index,scale) in enumerate(zip(indices,scales)):
        z=x.copy();z[index]+=.013
        _,_,changed=make(planner,variables=z)
        expected=original.copy();expected[k]+=.013*scale
        actual=np.concatenate([changed.sample(changed.end_time,d) for d in range(4)])
        np.testing.assert_allclose(actual,expected,atol=2e-8)
        np.testing.assert_allclose(changed.sample(changed.crossing_times[0]),plan.sample(plan.crossing_times[0]),atol=1e-9)


def test_bad_old_terminal_is_not_a_constraint(planner):
    old = old_module.Planner.terminal_state(planner,1)
    assert old[2,0] < .4
    x,par,plan=make(planner,first=1)
    x[30]=.3  # independently request z=3 metres
    _,_,moved=make(planner,first=1,variables=x)
    assert moved.sample(moved.end_time)[2] == pytest.approx(3.,abs=1e-8)
    # Full feasibility is NOT claimed by changing one coordinate.
    assert len(x)==37


def test_warm_terminal_PVAJ_and_center_seed(planner):
    x,_,_=make(planner)
    x[28:37]=[1.,.2,.4,.5,.1,0.,.02,.01,0.];x[23:26]=[.02,-.03,.01];x[26:28]=[.1,.12]
    _,_,old=make(planner,variables=x)
    planner.commit(old)
    try:
        now=.35;state=np.concatenate([old.sample(now,d) for d in range(4)])
        seed,_,plan=make(planner,now=now,state=state)
        np.testing.assert_array_equal(seed[26:28],0.)
        np.testing.assert_allclose(seed[28:37],x[28:37],atol=1e-8)
        np.testing.assert_allclose(seed[23:26],x[23:26],atol=1e-8)
        assert plan.end_time==pytest.approx(old.end_time)
        assert planner.seed_provenance[-1]['terminal_state_source']=='admitted'
    finally: planner.commit(None)


def test_actual_return_goal_is_not_freed(planner):
    x,_,plan=make(planner,first=6)
    assert len(x)==28
    np.testing.assert_allclose(plan.sample(plan.end_time),planner.track.goal,atol=1e-8)
    for d in (1,2):np.testing.assert_allclose(plan.sample(plan.end_time,d),0.,atol=1e-8)
    np.testing.assert_allclose(plan.sample(plan.end_time,3),planner.cfg.jerk*x[23:26],atol=1e-8)


@pytest.mark.parametrize('index',[0,1,23,24,25,28,29,30,31,34,36])
def test_analytic_gradient(planner,index):
    x,par,_=make(planner);fg=planner.solvers[0][0].fg
    _,g,_=fg(x,par); e=np.eye(len(x))[index]*1e-6
    numerical=float(fg(x+e,par)[0]-fg(x-e,par)[0])/2e-6
    np.testing.assert_allclose(float(g[index]),numerical,rtol=2e-3,atol=.5)
