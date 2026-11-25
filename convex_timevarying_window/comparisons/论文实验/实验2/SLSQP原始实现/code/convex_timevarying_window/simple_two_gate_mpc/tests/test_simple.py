from dataclasses import asdict,replace
import numpy as np
import pytest
from convex_timevarying_window.simple_two_gate_mpc.planner import Planner,Config
from convex_timevarying_window.simple_two_gate_mpc.scheduler import Scheduler
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


@pytest.fixture(scope='module')
def problem():
    install();track,_=build_case(Case(1.,2.,0.));cfg=Config(compile_objective=False)
    a=Planner(track,cfg,build_indices=[0,6])
    b=Planner(track,replace(cfg,free_points=False),build_indices=[0])
    zero=Planner(track,replace(cfg,soft_clearance_weight=0.),build_indices=[0])
    return a,b,zero,np.r_[track.start,np.zeros(9)]


def test_only_point_freedom_differs(problem):
    a,b,_,state=problem
    x,y=asdict(a.cfg),asdict(b.cfg);assert x.pop('free_points') and not y.pop('free_points');assert x==y
    xx=a.initial(state,.15,0);np.testing.assert_array_equal(xx,b.initial(state,.15,0))
    par=np.r_[.15,state,np.tile([.15,.35,.65,.85],6)]
    for f,g in zip(a.solvers[0][0].fg(xx,par),b.solvers[0][0].fg(xx,par)):np.testing.assert_array_equal(f,g)
    qi=a.offsets(2)[3]
    assert np.all(b.solvers[0][2][qi:]==0) and np.all(b.solvers[0][3][qi:]==0)
    assert np.any(a.solvers[0][3][qi:]>0)


def test_objective_only_time_and_violation_and_safety(problem):
    a,_,zero,state=problem;x=a.initial(state,.15,0)
    par=np.r_[.15,state,np.tile([.15,.35,.65,.85],6)]
    solver,ev,_,_,lo,hi=zero.solvers[0];g,_,obj=ev(x,par);g=np.asarray(g).ravel()
    violations=np.maximum(lo-g,0)**3+np.maximum(g-hi,0)**3
    expected=sum(np.exp(x[:3]))+zero.cfg.constraint_penalty*sum(violations)
    assert float(obj)==pytest.approx(sum(np.exp(x[:3])))
    assert float(solver.fg(x,par)[0])==pytest.approx(expected,rel=1e-10)
    assert float(a.solvers[0][0].fg(x,par)[0])>=float(solver.fg(x,par)[0])


@pytest.mark.parametrize('first',[0,6])
def test_real_third_or_return_endpoint_and_all_leg_time(problem,first):
    a,_,_,state=problem;gates=len(a.ids(first));legs=gates+1
    x=a.initial(state,.15,first);par=np.r_[.15,state,np.tile([.15,.35,.65,.85],2*legs)]
    _,C,_=a.solvers[first][1](x,par);C=np.asarray(C).reshape(2*legs,6,3)[-1]
    end=.15+sum(np.exp(x[:legs]));T=np.exp(x[gates])/2
    target=a.windows[first+gates].state_at(end)[0] if first+gates<7 else a.track.goal
    np.testing.assert_allclose(C.sum(0),target,atol=1e-8)
    np.testing.assert_allclose(np.arange(6)@C/T,0,atol=1e-8)
    assert not a.cfg.exclude_postrace_time and a.cfg.jerk_weight==0


def test_only_body_exit_and_startup_trigger_replans():
    s=Scheduler();s.observe(0,0,False,0);s.submitted(0);s.resolved(.01,True,None)
    s.observe(.2,0,False,99);assert not s.due(.2)
    s.observe(1.,1,True,0);assert s.due(1.)
    s.submitted(1.);s.resolved(1.01,False,'optimizer_residual')
    assert not s.due(1.1) and s.due(1.3)
    assert [e['reason'] for e in s.log if e['action']=='trigger']==['initial','body_exit']


def test_reject_old_objective_options():
    with pytest.raises(ValueError):Config(jerk_weight=.001)
    with pytest.raises(ValueError):Config(exclude_postrace_time=True)
