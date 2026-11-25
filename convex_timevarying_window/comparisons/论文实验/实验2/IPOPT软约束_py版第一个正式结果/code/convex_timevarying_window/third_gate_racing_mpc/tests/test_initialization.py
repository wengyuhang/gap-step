import numpy as np
import pytest
from convex_timevarying_window.third_gate_racing_mpc.planner import Planner,Config
from convex_timevarying_window.third_gate_stop_mpc.planner import Planner as Original
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


@pytest.fixture(scope='module')
def problem():
    install();track,_=build_case(Case(1.,2.,0.));cfg=Config(compile_objective=False)
    a=Original(track,cfg,build_indices=[0,1]);b=Planner(track,cfg,build_indices=[0,1])
    state=np.r_[track.start,np.zeros(9)];x=a.initial(state,.15,0);par=np.r_[.15,state,np.tile([.15,.35,.65,.85],6)]
    _,C,_=a.solvers[0][1](x,par)
    plan=Plan(.15,0,np.repeat(np.exp(x[:3])/2,2),np.asarray(C).reshape(6,6,3),x,0.,0.,'test',2)
    return a,b,state,par,plan


def test_no_change_to_objective_or_constraints(problem):
    a,b,state,par,_=problem;x=a.initial(state,.15,0)
    for f,g in zip(a.solvers[0][0].fg(x,par),b.solvers[0][0].fg(x,par)):np.testing.assert_allclose(f,g)


def test_cold_seed_uses_racing_velocity_without_changing_limit(problem):
    _,b,state,_,_=problem;x=b.initial(state,.15,0)
    vi,_,_,_=b.offsets(2)
    for k in range(2):assert np.linalg.norm(x[vi+3*k:vi+3*k+3])==pytest.approx(.75)
    assert b.cfg.speed==12.


def test_shared_absolute_crossing_retained_and_new_legs_shorter(problem):
    a,b,_,_,old=problem;now=old.crossing_times[0]+.2
    state=np.concatenate([old.sample(now,d) for d in range(4)])
    try:
        a.commit(old);b.commit(old);x=a.initial(state,now,1);y=b.initial(state,now,1)
        assert now+np.exp(y[0])==pytest.approx(old.crossing_times[1])
        assert np.exp(y[1])<=np.exp(x[1]) and np.exp(y[2])<=np.exp(x[2])
        np.testing.assert_allclose(y[3:6],x[3:6])
        pp=np.r_[now,state,np.tile([.15,.35,.65,.85],6)];_,C,_=b.solvers[1][1](y,pp)
        C=np.asarray(C).reshape(6,6,3);T=np.exp(y[2])/2;end=now+np.exp(y[:3]).sum()
        np.testing.assert_allclose(C[-1].sum(axis=0),b.windows[3].state_at(end)[0],atol=1e-8)
        np.testing.assert_allclose(np.arange(6)@C[-1]/T,0,atol=1e-8)
    finally:a.commit(None);b.commit(None)


def test_retry_seed_uses_absolute_committed_time_not_failed_relative_time(problem):
    _,b,_,_,old=problem;now=old.crossing_times[0]+.2
    state=np.concatenate([old.sample(now,d) for d in range(4)])
    try:
        b.commit(old)
        first=b.initial(state,now,1)
        from types import SimpleNamespace
        failed=SimpleNamespace(x=first.copy())
        b.search[1]=failed
        later=now+.2;newstate=np.concatenate([old.sample(later,d) for d in range(4)])
        retry=b.initial(newstate,later,1)
        assert later+np.exp(retry[0])==pytest.approx(old.crossing_times[1])
        assert b.search[1] is failed  # Refinement evidence must not be discarded.
        assert not np.array_equal(first,retry)
    finally:b.commit(None);b.search.clear()
