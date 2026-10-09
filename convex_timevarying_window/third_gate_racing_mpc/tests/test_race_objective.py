from dataclasses import replace
import numpy as np
from convex_timevarying_window.third_gate_stop_mpc.planner import Planner
from convex_timevarying_window.third_gate_racing_mpc.planner import Config
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


def test_only_postrace_time_removed_not_safety_or_smoothing():
    install();track,_=build_case(Case(1.,2.,0.));cfg=Config(compile_objective=False)
    old=Planner(track,replace(cfg,exclude_postrace_time=False),build_indices=[6])
    new=Planner(track,cfg,build_indices=[6])
    state=np.r_[track.start,np.zeros(9)];x=old.initial(state,.15,6)
    par=np.r_[.15,state,np.tile([.15,.35,.65,.85],4)]
    a=old.solvers[6][0].fg(x,par);b=new.solvers[6][0].fg(x,par)
    T=np.exp(x[1]);assert abs(float(a[0]-b[0])-T)<1e-5
    delta=np.asarray(a[1]-b[1]).ravel();expected=np.zeros(len(x));expected[1]=T
    np.testing.assert_allclose(delta,expected,atol=1e-5)
    np.testing.assert_array_equal(a[2],b[2])
    a=old.solvers[6][1](x,par);b=new.solvers[6][1](x,par)
    np.testing.assert_array_equal(a[0],b[0]);np.testing.assert_array_equal(a[1],b[1])
