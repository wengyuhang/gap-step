from dataclasses import replace
import casadi as ca
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.togt_minco_mpc.planner import Config,Planner,ScalarSolver


def test_moving_window_scalar_gradient():
    track,_=build_track()
    cfg=replace(Config(),workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
    p=Planner(track,cfg,build_indices=[0]);state=np.r_[track.start,np.zeros(9)]
    x=p.initial(state,.15,0);parameters=np.r_[.15,state,np.tile([.15,.35,.65,.85],6)]
    fg=p.solvers[0][0].fg;_,grad,_=fg(x,parameters)
    for j in (0,3,7,15,18):
        delta=np.eye(len(x))[j]*1e-5
        fd=float(fg(x+delta,parameters)[0]-fg(x-delta,parameters)[0])/2e-5
        np.testing.assert_allclose(float(grad[j]),fd,rtol=3e-4,atol=2e-3)


def test_solver_retains_a_feasible_trial():
    x=ca.MX.sym('x');p=ca.MX.sym('p')
    f=(x-1)**2;fg=ca.Function('solver_fixture',[x,p],[f,ca.gradient(f,x),ca.fmax(-x,0)])
    s=ScalarSolver(fg,None,Config())
    result=s(x0=np.array([.2]),p=0,lbx=np.array([0.]),ubx=np.array([2.]),lbg=np.array([0.]),ubg=np.array([np.inf]))
    np.testing.assert_allclose(float(result['x']),1.,atol=1e-7)
