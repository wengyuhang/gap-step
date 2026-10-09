import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from convex_timevarying_window.center_continuation_mpc.planner import Planner,Config


@pytest.fixture(scope='module')
def problem():
    install();track,_=build_case(Case(1.,2.,0.))
    p=Planner(track,Config(workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.)),build_indices=[0])
    return p,track


def test_only_two_window_points_are_free(problem):
    p,_=problem;_,_,lo,hi,_,_=p.solvers[0];qi=p.offsets(7)[3]
    assert np.all(lo[qi:qi+4]==-2.) and np.all(hi[qi:qi+4]==2.)
    assert np.all(lo[qi+4:qi+14]==0.) and np.all(hi[qi+4:qi+14]==0.)
    assert len(p.ids(0))==7


def test_initial_center_and_final_boundary(problem):
    p,track=problem;x=p.initial(np.r_[track.start,np.zeros(9)],.15,0)
    qi=p.offsets(7)[3];np.testing.assert_array_equal(x[qi:qi+14],0.)
    par=np.r_[.15,track.start,np.zeros(9),np.tile([.15,.35,.65,.85],16)]
    _,C,_=p.solvers[0][1](x,par);C=np.array(C)[-6:];T=np.exp(x[7])/2
    np.testing.assert_allclose(C.sum(axis=0),track.goal,atol=1e-7)
    np.testing.assert_allclose(np.arange(6)@C/T,0.,atol=1e-7)


def test_value_gradient_includes_late_leg_time(problem):
    p,track=problem;x=p.initial(np.r_[track.start,np.zeros(9)],.15,0)
    par=np.r_[.15,track.start,np.zeros(9),np.tile([.15,.35,.65,.85],16)]
    fg=p.solvers[0][0].fg;_,g,_=fg(x,par);g=np.array(g).ravel()
    for k in (0,4,6,p.offsets(7)[0]):
        delta=np.zeros_like(x);delta[k]=1e-6
        numeric=(float(fg(x+delta,par)[0])-float(fg(x-delta,par)[0]))/2e-6
        assert abs(numeric-g[k])<.01*max(1.,abs(numeric))
