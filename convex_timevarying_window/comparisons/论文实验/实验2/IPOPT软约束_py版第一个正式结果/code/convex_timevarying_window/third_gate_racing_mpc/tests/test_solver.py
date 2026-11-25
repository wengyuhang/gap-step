import casadi as ca
import numpy as np
from convex_timevarying_window.third_gate_racing_mpc.planner import Config
from convex_timevarying_window.third_gate_racing_mpc.solver import BufferedSolver
from convex_timevarying_window.third_gate_time_mpc.planner import ConvergedSolver


def test_buffer_preserves_final_iterate_and_stopping():
    x=ca.SX.sym('x',4);p=ca.SX.sym('p',4)
    cost=ca.sumsqr(x-p)+.1*ca.sum1(ca.sin(3*x))
    fg=ca.Function('test_buffer',[x,p],[cost,ca.gradient(cost,x),ca.DM(0)])
    a,b=ConvergedSolver(fg,Config()),BufferedSolver(fg,Config())
    args=dict(x0=np.ones(4),p=np.array([.3,-.2,.5,.7]),lbx=np.full(4,-3.),ubx=np.full(4,3.))
    np.testing.assert_array_equal(np.asarray(a(**args)['x']),np.asarray(b(**args)['x']))
    for k in ('iter_count','evaluations','converged','return_status','final_objective'):
        assert a.stats()[k]==b.stats()[k]
