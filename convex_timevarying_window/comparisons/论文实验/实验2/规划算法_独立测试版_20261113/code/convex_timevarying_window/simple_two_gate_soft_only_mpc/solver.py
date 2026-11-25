"""L-BFGS-B with recovery from a zero-step, nonstationary line search.

Same unscaled objective, gradient, bounds and initial guess. A projected
Armijo descent step repairs numerical stagnation; it is not a safety check.
"""
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from convex_timevarying_window.third_gate_racing_mpc.solver import BufferedSolver


def projected_gradient(x, g, lo, hi):
    result = g.copy()
    result[(x <= lo) & (g > 0)] = 0.
    result[(x >= hi) & (g < 0)] = 0.
    return result


def guarded_minimize(fun, x, lo, hi, cfg):
    x = np.clip(np.asarray(x, dtype=float), lo, hi)
    calls = 0; iterations = 0; repairs = []; attempts = []
    def counted(z):
        nonlocal calls
        calls += 1
        return fun(z)
    f, g = counted(x); initial_f = f
    if not np.isfinite(f) or not np.isfinite(g).all():
        raise ValueError('nonfinite initial objective/gradient')
    success = False; message = 'iteration budget exhausted'
    for _ in range(9):
        previous = x.copy(); last_step = None
        def callback(z):
            nonlocal previous, last_step
            last_step = float(np.max(np.abs(z-previous)))
            previous = z.copy()
        result = minimize(counted, x, jac=True, method='L-BFGS-B', callback=callback,
             bounds=list(zip(lo, hi)), options=dict(maxiter=max(1,cfg.max_iterations-iterations),
             ftol=cfg.optimizer_ftol, gtol=cfg.optimizer_gtol, maxls=20, maxcor=12))
        iterations += int(result.nit)
        x = result.x.copy(); f = float(result.fun); g = np.asarray(result.jac)
        pg = float(np.linalg.norm(projected_gradient(x,g,lo,hi), ord=np.inf))
        stagnant = (last_step is None or last_step <= 8*np.finfo(float).eps*max(1.,np.linalg.norm(x,ord=np.inf))) and pg > cfg.optimizer_gtol
        attempts.append(dict(status=str(result.message),iterations=int(result.nit),
                             objective=f,last_step=last_step,projected_gradient_inf=pg,stagnant=bool(stagnant)))
        if not stagnant:
            success = bool(result.success); message = str(result.message)
            break
        message = 'NUMERICAL_STAGNATION: nonzero projected gradient, zero step'
        if len(repairs)>=8 or iterations>=cfg.max_iterations:break
        direction = np.clip(x-g,lo,hi)-x
        slope = float(g@direction); alpha = 1.; accepted = False
        for _ in range(50):
            trial = np.clip(x+alpha*direction,lo,hi)
            value, grad = counted(trial)
            if (np.isfinite(value) and np.isfinite(grad).all() and value < f
                    and value <= f+1e-4*alpha*slope):
                repairs.append(dict(before=f,after=float(value),alpha=alpha))
                x,f,g = trial,float(value),grad; accepted = True
                break
            alpha *= .5
        if not accepted:break
    return x,dict(iter_count=iterations,evaluations=calls,converged=success,
                  return_status=message,initial_objective=float(initial_f),final_objective=float(f),
                  projected_gradient_inf=float(np.linalg.norm(projected_gradient(x,g,lo,hi),ord=np.inf)),
                  stagnation_repairs=repairs,attempts=attempts)


class Solver(BufferedSolver):
    def __call__(self, **args):
        began = time.perf_counter()
        buffer,evaluate = self.fg.buffer()
        xb = np.empty(self.fg.numel_in(0),dtype=np.float64)
        pb = np.asarray(args['p'],dtype=np.float64).ravel().copy()
        outputs = [np.empty(self.fg.nnz_out(i),dtype=np.float64) for i in range(3)]
        buffer.set_arg(0,memoryview(xb));buffer.set_arg(1,memoryview(pb))
        for i,out in enumerate(outputs):buffer.set_res(i,memoryview(out))
        def fun(x):
            np.copyto(xb,x);evaluate()
            if buffer.ret()!=0:raise RuntimeError('objective evaluation failed')
            return float(outputs[0][0]),outputs[1].copy()
        x,self._stats = guarded_minimize(fun,args['x0'],args['lbx'],args['ubx'],self.cfg)
        self._stats['optimization_ms'] = 1000*(time.perf_counter()-began)
        return dict(x=ca.DM(x))
