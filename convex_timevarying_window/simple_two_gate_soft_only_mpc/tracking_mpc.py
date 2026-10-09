"""Trajectory tracking and input smoothing, with physical input bounds only.

Reuse the quadrotor model, state elimination and RTI numerical iteration.
No tilt penalty/constraint, gate geometry, safety filter or path decisions.
"""
from dataclasses import dataclass
import numpy as np
from scipy import sparse
from scipy.optimize import minimize
from .elastic_mpc import MPC as NumericalRTI
from convex_timevarying_window.checked_soft_mpc_racing.mpc import LOWER, UPPER, NU


@dataclass(frozen=True)
class Config:
    steps: int = 25
    dt: float = .04
    update_period: float = .02
    state_prediction_s: float = .015
    position_weight: float = 2000.
    velocity_weight: float = 30.
    attitude_weight: float = 20.
    terminal_multiplier: float = 4.
    iterations: int = 2
    box_iterations: int = 120
    box_ftol: float = 1e-9
    box_gtol: float = 1e-5


class MPC(NumericalRTI):
    def __init__(self, cfg=Config()):
        super().__init__(cfg)

    def merit(self, states, controls, refs, ff):
        error = (states-refs).ravel()
        delta = np.diff(np.vstack((self.previous, controls)), axis=0)
        return float(np.dot(self.qdiag, error**2)
                     + np.sum(self.R*(controls-ff)**2) + np.sum(self.S*delta**2))

    def condense(self, x, states, controls, refs, ff):
        n = self.cfg.steps
        offset, mapping = self.affine_prediction(x, states, controls)
        H = mapping.T@(self._w[:, None]*mapping)+self._pu
        H = H+H.T
        P = sparse.csc_matrix((H[self._upper], self._upper), shape=(n*NU, n*NU))
        d0 = np.zeros(n*NU); d0[:NU] = self.previous
        qu = -2*(np.tile(self.R, n)*ff.ravel()+self.D.T@(self.SD@d0))*np.tile(self.uscale, n)
        q = 2*mapping.T@(self._w*(offset-refs.ravel()/np.tile(self.xscale, n+1)))+qu
        # No attitude or trust-region rows. Only physical input limits.
        return (P, q, sparse.eye(n*NU, format='csc'),
                np.tile(LOWER/self.uscale, n), np.tile(UPPER/self.uscale, n))

    def solve_elastic(self, P, q, A, lower, upper, initial):
        # Method name retained only for the inherited numerical RTI loop.
        triangle = P.toarray()
        H = triangle+triangle.T-np.diag(np.diag(triangle))
        seed = np.clip(np.asarray(initial).ravel(), lower, upper)
        if not all(np.isfinite(v).all() for v in (H, q, lower, upper, seed)):
            raise ValueError('nonfinite MPC model/reference')
        best = seed.copy(); best_value = np.inf
        def objective(u):
            nonlocal best, best_value
            value = float(.5*u@H@u+q@u); gradient = H@u+q
            if np.isfinite(value) and np.isfinite(gradient).all() and value < best_value:
                best, best_value = np.clip(u, lower, upper).copy(), value
            return value, gradient
        initial_value = objective(seed)[0]
        result = minimize(objective, seed, method='L-BFGS-B', jac=True,
                          bounds=list(zip(lower, upper)), options=dict(maxiter=self.cfg.box_iterations,
                          maxls=20, maxcor=12, ftol=self.cfg.box_ftol, gtol=self.cfg.box_gtol))
        return best, dict(status=str(result.message), converged=bool(result.success),
                          iterations=int(result.nit), evaluations=int(result.nfev),
                          initial_value=initial_value, final_value=best_value)

    def command(self, state, now, reference):
        command = super().command(state, now, reference)
        self.logs[-1].update(controller='trajectory_tracking_input_box_rti', qp_status='tracking_box')
        return command
