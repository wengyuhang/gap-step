"""Always nonempty input box, soft tilt, same model and nonlinear RTI merit.

Eliminate elastic tilt slack analytically: rho*max(b-Tu,0)^2. The resulting
convex piecewise quadratic has only the physical actuator box as a constraint.
L-BFGS-B solves this small (100 input variables) subproblem; no hard-tilt QP
or artificial per-iteration trust box can make it infeasible.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy.optimize import minimize
from convex_timevarying_window.checked_soft_mpc_isolated.fast.mpc import MPC as Parent
from convex_timevarying_window.checked_soft_mpc_racing.mpc import Config as BaseConfig, LOWER, UPPER, NU, reference_states


@dataclass(frozen=True)
class Config(BaseConfig):
    elastic_tilt_weight: float = 1e6  # Same penalty already present in nonlinear merit.
    box_iterations: int = 120
    box_ftol: float = 1e-9
    box_gtol: float = 1e-5


def elastic_value_gradient(u, H, q, T, b, weight):
    residual = np.minimum(T@u-b, 0.)
    return (float(.5*u@H@u+q@u+weight*(residual@residual)),
            H@u+q+2*weight*T.T@residual)


class MPC(Parent):
    def __init__(self, cfg=Config()):
        super().__init__(cfg)

    def solve_elastic(self, P, q, A, lower, upper, initial):
        size = self.cfg.steps*NU
        upper_triangle = P.toarray()
        H = upper_triangle+upper_triangle.T-np.diag(np.diag(upper_triangle))
        T = A[size:].toarray(); b = lower[size:]
        lo = np.tile(LOWER/self.uscale, self.cfg.steps)
        hi = np.tile(UPPER/self.uscale, self.cfg.steps)
        seed = np.clip(np.asarray(initial).ravel(), lo, hi)
        best = seed.copy()
        best_value = np.inf
        def objective(u):
            nonlocal best, best_value
            value, gradient = elastic_value_gradient(u, H, q, T, b, self.cfg.elastic_tilt_weight)
            if np.isfinite(value) and np.isfinite(gradient).all() and value < best_value:
                best, best_value = np.clip(u, lo, hi).copy(), value
            return value, gradient
        if not all(np.isfinite(v).all() for v in (H,q,T,b,seed)):
            raise ValueError('nonfinite MPC model/reference')
        initial_value = objective(seed)[0]
        result = minimize(objective, seed, method='L-BFGS-B', jac=True, bounds=list(zip(lo,hi)),
                          options=dict(maxiter=self.cfg.box_iterations, maxls=20, maxcor=12,
                                       ftol=self.cfg.box_ftol, gtol=self.cfg.box_gtol))
        # Numerical termination is not infeasibility. Every retained iterate
        # satisfies the true input box; actual nonlinear merit chooses the step.
        return best, dict(status=str(result.message), converged=bool(result.success),
                          iterations=int(result.nit), evaluations=int(result.nfev),
                          initial_value=float(initial_value), final_value=float(best_value),
                          max_soft_tilt_residual=float(np.maximum(b-T@best,0.).max()))

    def command(self, state, now, reference):
        began=time.perf_counter();cfg=self.cfg;n=cfg.steps
        if not np.isfinite(state).all():raise ValueError('nonfinite MPC state')
        x=np.asarray(self.step(state,self.previous,cfg.state_prediction_s)).ravel()
        at=now+cfg.state_prediction_s
        refs,ff=reference_states(reference(at+np.arange(n+1)*cfg.dt),cfg.dt,x[6:10])
        if self.controls is None:
            controls=ff.copy()
        else:
            offset=max(0.,(now-self.last_time)/cfg.dt);knots=np.arange(n)
            controls=np.column_stack([np.interp(knots+offset,knots,self.controls[:,j]) for j in range(NU)])
        controls=np.clip(controls,LOWER,UPPER)
        statistics=[];steps=[]
        for _ in range(cfg.iterations):
            states=self.rollout(x,controls)
            problem=self.condense(x,states,controls,refs,ff)
            vector,stats=self.solve_elastic(*problem,(controls/self.uscale).ravel())
            proposal=np.clip(vector.reshape(n,NU)*self.uscale,LOWER,UPPER)
            best_cost=self.merit(states,controls,refs,ff);selected=controls;selected_alpha=0.
            for alpha in (1.,.5,.25,.125,.0625):
                # A convex combination can overshoot a floating-point endpoint
                # by one ULP; the CTBR encoder uses exact inclusive bounds.
                trial=np.clip(controls+alpha*(proposal-controls),LOWER,UPPER)
                cost=self.merit(self.rollout(x,trial),trial,refs,ff)
                if np.isfinite(cost) and cost<best_cost:
                    selected=trial;best_cost=cost;selected_alpha=alpha
            controls=selected;statistics.append(stats);steps.append(selected_alpha)
        self.controls=np.clip(controls,LOWER,UPPER);self.last_time=now
        self.previous=self.controls[0].copy()
        elapsed=1000*(time.perf_counter()-began)
        self.logs.append(dict(time=float(now),cycle_ms=elapsed,controller='elastic_input_box_rti',
            qp_status='not_used_elastic_box',qp_iterations=sum(s['iterations'] for s in statistics),
            box_solves=statistics,nonlinear_steps=steps,command=self.previous.tolist(),
            position_error_m=float(np.linalg.norm(state[:3]-reference(np.array([now]))[0,:3])),
            tilt_deg=float(np.degrees(np.arccos(np.clip(1-2*np.sum(state[7:9]**2),-1.,1.))))))
        # User requests physical simulation outcomes, not a 100 ms abort.
        return self.previous.copy()
