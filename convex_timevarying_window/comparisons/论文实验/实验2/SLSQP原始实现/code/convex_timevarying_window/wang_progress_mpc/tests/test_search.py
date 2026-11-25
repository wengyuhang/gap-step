from dataclasses import replace
from types import SimpleNamespace
import casadi as ca
import numpy as np

from convex_timevarying_window.comparisons.fixed_plane_wang.scenario import Case,build_case
from convex_timevarying_window.wang_progress_mpc.planner import Planner,Config


def planner(first=3):
    track,_=build_case(Case(.8,1.6,.4))
    return Planner(track,replace(Config(),workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.)),build_indices=[first])


def test_deadline_cost_gradient_matches_finite_difference():
    p=planner();wrapped=p.solvers[3][0]
    fg=wrapped.solver.fg
    x=np.zeros(fg.size1_in(0));x[:3]=np.log([2.,2.,2.])
    par=np.zeros(fg.size1_in(1));par[0]=7.;par[-2:]=[8.5,10.]
    value,g,_=fg(x,par);g=np.array(g).ravel()
    for i in (0,1):
        delta=np.zeros(len(x));delta[i]=1e-6
        numerical=(float(fg(x+delta,par)[0])-float(fg(x-delta,par)[0]))/2e-6
        np.testing.assert_allclose(g[i],numerical,rtol=2e-5,atol=1e-3)


def test_solver_receives_real_first_leg_bound_and_all_common_deadlines():
    p=planner();wrapped=p.solvers[3][0]
    p.admitted=SimpleNamespace(first_gate=2,gate_count=2,crossing_times=np.array([7.,9.]))
    solver,evaluate,lo,hi,lbg,ubg=p.solvers[3]
    seen={}
    def fake(**args):seen.update(args);return dict(x=ca.DM(args['x0']))
    wrapped.solver=fake
    par=np.zeros(37);par[0]=7.4
    seed=np.zeros(len(lo));seed[:3]=np.log([5.,3.,2.])
    wrapped(x0=seed,p=par,lbx=lo,ubx=hi,lbg=lbg,ubg=ubg)
    assert abs(np.exp(seen['ubx'][0])-1.62)<1e-10
    assert np.exp(seen['x0'][0])<=1.62+1e-10
    np.testing.assert_allclose(seen['p'][-2:],[9.02,1007.4])


def test_no_window_terminal_model_still_builds():
    p=planner(7)
    assert p.solvers[7][0].ids==[]
