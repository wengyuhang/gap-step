from types import SimpleNamespace
import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.exit_value_mpc.value import travel_time
from convex_timevarying_window.exit_value_mpc.planner import Planner,Config
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


def test_velocity_direction_changes_continuation_cost():
    cfg=Config();p=ca.DM.zeros(3);target=ca.DM([20.,0.,0.])
    forward=float(travel_time(p,ca.DM([6.,0.,0.]),target,cfg))
    backward=float(travel_time(p,ca.DM([-6.,0.,0.]),target,cfg))
    lateral=float(travel_time(p,ca.DM([0.,6.,0.]),target,cfg))
    assert forward<backward and forward<lateral


@pytest.fixture(scope='module')
def problem():
    install();track,_=build_case(Case(1.,2.,0.))
    p=Planner(track,Config(workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.)),build_indices=[0])
    return p,track


def test_short_optimization_not_seven_gate_constraints(problem):
    p,track=problem;x=p.initial(np.r_[track.start,np.zeros(9)],.15,0)
    assert len(p.ids(0))==2 and len(x)==37
    assert p.free_terminal(0)


def test_proxy_state_phase_and_gradient(problem):
    p,_=problem;z=np.r_[3.25,[-1.,-2.,6.5],[8.,2.,1.],[.2,.3,-.2]]
    x=ca.SX.sym('z',10);v=p.values[0](x)
    f=ca.Function('proxy_test',[x],[v,ca.gradient(v,x)])
    value,g=f(z);g=np.asarray(g).ravel()
    assert np.isfinite(float(value)) and float(value)>0
    for k in (0,1,4,7):
        d=np.zeros(10);d[k]=1e-6
        numeric=(float(f(z+d)[0])-float(f(z-d)[0]))/2e-6
        assert abs(numeric-g[k])<.005*max(1.,abs(numeric))
    assert abs(g[0])>1e-5 and np.linalg.norm(g[4:7])>1e-5


def test_objective_gradient(problem):
    p,track=problem;x=p.initial(np.r_[track.start,np.zeros(9)],.15,0)
    par=np.r_[.15,track.start,np.zeros(9),np.tile([.15,.35,.65,.85],6)]
    fn=p.solvers[0][0].fg;_,g,_=fn(x,par);g=np.asarray(g).ravel()
    for k in (0,1,3,24,28):
        d=np.zeros(len(x));d[k]=1e-6
        numeric=(float(fn(x+d,par)[0])-float(fn(x-d,par)[0]))/2e-6
        assert abs(numeric-g[k])<.01*max(1.,abs(numeric))
