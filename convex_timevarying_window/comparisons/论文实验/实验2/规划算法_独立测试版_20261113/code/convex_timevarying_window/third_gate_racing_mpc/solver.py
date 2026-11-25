"""Normal L-BFGS-B final iterate, using CasADi's allocation-free evaluator."""
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from convex_timevarying_window.third_gate_time_mpc.planner import ConvergedSolver


class BufferedSolver(ConvergedSolver):
    def __call__(self,**args):
        began=time.perf_counter()
        buffer,evaluate=self.fg.buffer()
        xbuf=np.empty(self.fg.numel_in(0),dtype=np.float64)
        pbuf=np.asarray(args['p'],dtype=np.float64).ravel().copy()
        outputs=[np.empty(self.fg.nnz_out(i),dtype=np.float64) for i in range(3)]
        assert self.fg.nnz_out(1)==len(xbuf)
        buffer.set_arg(0,memoryview(xbuf));buffer.set_arg(1,memoryview(pbuf))
        for i,out in enumerate(outputs):buffer.set_res(i,memoryview(out))
        def fun(x):
            np.copyto(xbuf,x);evaluate()
            if buffer.ret()!=0:raise RuntimeError('objective evaluation failed')
            # SciPy may retain a previous gradient; do not return reused storage.
            return float(outputs[0][0]),outputs[1].copy()
        result=minimize(fun,np.asarray(args['x0']),jac=True,method='L-BFGS-B',
            bounds=list(zip(args['lbx'],args['ubx'])),options=dict(maxiter=self.cfg.max_iterations,
            ftol=self.cfg.optimizer_ftol,gtol=self.cfg.optimizer_gtol,maxls=20,maxcor=12))
        self._stats=dict(iter_count=int(result.nit),evaluations=int(result.nfev),
            converged=bool(result.success),return_status=str(result.message),
            optimization_ms=1000*(time.perf_counter()-began),final_objective=float(result.fun))
        return dict(x=ca.DM(result.x))
