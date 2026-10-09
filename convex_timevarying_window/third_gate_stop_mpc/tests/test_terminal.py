import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from convex_timevarying_window.third_gate_stop_mpc.planner import Planner,Config
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan,basis


@pytest.fixture(scope='module')
def problem():
    install();track,_=build_case(Case(1.,2.,0.))
    p=Planner(track,Config(compile_objective=False,workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.)),build_indices=[0,3,5])
    state=np.r_[track.start,np.zeros(9)]
    return p,state,np.r_[.15,state,np.tile([.15,.35,.65,.85],6)]


def endpoint(p,x,par,first):
    _,C,_=p.solvers[first][1](x,par);C=np.asarray(C).reshape(6,6,3)
    T=np.exp(x[2])/2
    return np.array([basis(1.,d)@C[-1]/T**d for d in range(3)])


def test_fixed_size_two_crossings_and_one_terminal(problem):
    p,state,par=problem
    for first in [0,3,5]:
        x=p.initial(state,.15,first)
        assert len(x)==28 and len(p.ids(first))==2
        assert p.solvers[first][1](x,par)[1].shape==(36,3)


@pytest.mark.parametrize('shift',[0.,.1,.31])
def test_moving_third_center_and_zero_world_va(problem,shift):
    p,state,par=problem;x=p.initial(state,.15,0);x[2]+=shift
    end=.15+np.exp(x[:3]).sum();y=endpoint(p,x,par,0)
    np.testing.assert_allclose(y[0],p.windows[2].state_at(end)[0],atol=1e-8)
    np.testing.assert_allclose(y[1:],0,atol=1e-8)
    assert np.linalg.norm(y[0]-p.windows[2].state_at(.15)[0])>.01


def test_only_real_return_goal_stops_after_final_gate(problem):
    p,state,par=problem;x=p.initial(state,.15,5);y=endpoint(p,x,par,5)
    np.testing.assert_allclose(y[0],p.track.goal,atol=1e-8)
    np.testing.assert_allclose(y[1:],0,atol=1e-8)


def test_time_gradient_includes_moving_terminal(problem):
    p,state,par=problem;x=p.initial(state,.15,0);fg=p.solvers[0][0].fg
    analytic=np.asarray(fg(x,par)[1]).ravel()
    for i in [0,1,2,24,25,26,27]:
        d=np.zeros(len(x));d[i]=1e-5
        numeric=(float(fg(x+d,par)[0])-float(fg(x-d,par)[0]))/2e-5
        assert abs(numeric-analytic[i])<.003*max(1.,abs(numeric))


def test_warm_start_can_roll_to_next_two_gates(problem):
    p,state,par=problem;x=p.initial(state,.15,0);_,C,_=p.solvers[0][1](x,par)
    old=Plan(.15,0,np.repeat(np.exp(x[:3])/2,2),np.asarray(C).reshape(6,6,3),x,0.,0.,'test',2)
    try:
        p.commit(old)
        same=p.initial(state,.20,0)
        assert len(same)==28 and np.isfinite(same).all()
        end=.20+np.exp(same[:3]).sum()
        pp=par.copy();pp[0]=.20
        np.testing.assert_allclose(endpoint(p,same,pp,0)[0],p.windows[2].state_at(end)[0],atol=1e-8)
    finally:p.commit(None)
