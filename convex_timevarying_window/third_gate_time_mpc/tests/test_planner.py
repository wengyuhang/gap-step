from types import SimpleNamespace
from dataclasses import replace
import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from convex_timevarying_window.third_gate_time_mpc.planner import Planner,Config,ConvergedSolver,reach_residual


@pytest.fixture(scope='module')
def problem():
    install();track,_=build_case(Case(1.,2.,0.))
    p=Planner(track,Config(workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.)),build_indices=[0,3,5])
    state=np.r_[track.start,np.zeros(9)]
    return p,state,np.r_[.15,state,np.tile([.15,.35,.65,.85],6)]


def test_two_gate_size_not_remaining_course(problem):
    p,state,par=problem
    assert p.ids(0)==[0,1] and p.ids(3)==[3,4]
    assert len(p.initial(state,.15,0))==len(p.initial(state,.15,3))==38
    assert len(p.initial(state,.15,5))==28
    assert p.solvers[0][0].fg.size1_in(1)==len(par)


def test_gradient_including_time_and_tau(problem):
    p,state,par=problem;x=p.initial(state,.15,0);f=p.solvers[0][0].fg
    g=np.asarray(f(x,par)[1]).ravel()
    for i in (0,1,2,28,31,37):
        d=np.zeros(len(x));d[i]=1e-4
        numeric=(float(f(x+d,par)[0])-float(f(x-d,par)[0]))/2e-4
        # The cold infeasible seed has a very large penalty; account explicitly
        # for floating-point cancellation when checking an O(1) tau derivative.
        roundoff=8*np.spacing(abs(float(f(x,par)[0])))/(2e-4)
        assert abs(numeric-g[i])<.003*max(1.,abs(numeric))+roundoff


def test_far_windows_do_not_enter_third_estimate(problem):
    p,state,par=problem;x=p.initial(state,.15,0)
    # Coefficients contain only 2 gate legs plus physical exit tail.
    assert p.solvers[0][1](x,par)[1].shape==(36,3)


def test_outgoing_velocity_matters():
    target=ca.DM([10.,0.,0.]);p=ca.DM.zeros(3)
    good=np.asarray(reach_residual(p,ca.DM([5.,0.,0.]),target,1.,8.,12.)).ravel()
    bad=np.asarray(reach_residual(p,ca.DM([-5.,0.,0.]),target,1.,8.,12.)).ravel()
    assert good[0]<bad[0]


def test_stationary_acceleration_bound():
    r=np.asarray(reach_residual(ca.DM.zeros(3),ca.DM.zeros(3),ca.DM([4.,0.,0.]),1.,8.,12.)).ravel()
    assert r[0]==pytest.approx(0.) and r[1]<0


def test_final_iterate_not_best_point(monkeypatch):
    import convex_timevarying_window.third_gate_time_mpc.planner as m
    x=ca.SX.sym('q');p=ca.SX.sym('p')
    f=ca.Function('test_final',[x,p],[x*x,2*x,0.])
    def fake(fun,x0,**kw):
        fun(np.array([0.]))
        return SimpleNamespace(x=np.array([.5]),success=True,nit=2,nfev=3,message='converged',fun=.25)
    monkeypatch.setattr(m,'minimize',fake)
    s=ConvergedSolver(f,Config());out=s(x0=[1.],p=[0.],lbx=[-2.],ubx=[2.])
    assert float(out['x'])==.5 and s.stats()['converged']


def test_failed_convergence_flag():
    x=ca.SX.sym('q',2);p=ca.SX.sym('p')
    cost=(x[0]-1)**2+10*(x[1]-2)**2
    f=ca.Function('test_limit',[x,p],[cost,ca.gradient(cost,x),0.])
    s=ConvergedSolver(f,replace(Config(),max_iterations=1))
    s(x0=[-1.,-1.],p=[0.],lbx=[-5.,-5.],ubx=[5.,5.])
    assert not s.stats()['converged']
