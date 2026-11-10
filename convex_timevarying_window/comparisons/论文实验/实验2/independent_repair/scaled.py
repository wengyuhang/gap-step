"""Positive row preconditioning of the unchanged constrained problem."""
import inspect
import numpy as np
from scipy.optimize import minimize, least_squares, OptimizeResult
from convex_timevarying_window.global_gate_minco_probe import planner as core
from .. import planner as original
from .planner import Planner as ConfiguredPlanner


class Planner(ConfiguredPlanner):
    def scaled_minimize(self, fun, initial, *, jac, method, bounds, constraints, callback, options):
        evaluate = inspect.getclosurevars(constraints['fun']).nonlocals['evaluate']
        progress = inspect.getclosurevars(evaluate).nonlocals['progress']
        y = initial.copy()
        iterations = 0

        def literal(at):
            value, gradient = fun(at)
            return np.expm1(value), np.exp(value)*gradient

        for attempt in range(3):
            norm = np.linalg.norm(constraints['jac'](y), axis=1)
            row_scale = np.clip(np.where(norm > 1e-12, 1./np.maximum(norm, 1e-3), 1.), 1e-6, 1e6)

            def residual(at):
                return row_scale*np.minimum(constraints['fun'](at), 0.)

            def residual_jacobian(at):
                return (row_scale*(constraints['fun'](at) < 0.))[:, None]*constraints['jac'](at)

            restored = least_squares(residual, y, jac=residual_jacobian,
                bounds=np.asarray(bounds).T, x_scale='jac', max_nfev=400,
                ftol=1e-10, xtol=1e-10, gtol=1e-10)
            minimum = float(constraints['fun'](restored.x).min())
            row = dict(stage='scaled_restoration', attempt=attempt, evaluations=int(restored.nfev),
                minimum_constraint=minimum, scale_min=float(row_scale.min()), scale_max=float(row_scale.max()))
            self.optimizer_calls.append(row)
            progress(row)
            result = minimize(literal, restored.x, jac=True, method='SLSQP', bounds=bounds,
                constraints=dict(type='ineq', fun=lambda at: row_scale*constraints['fun'](at),
                    jac=lambda at: row_scale[:, None]*constraints['jac'](at)),
                callback=callback, options=dict(options, maxiter=self.max_iterations, ftol=1e-9))
            iterations += int(result.nit)
            final_minimum = float(constraints['fun'](result.x).min())
            self.optimizer_calls.append(dict(stage='scaled_sqp', attempt=attempt,
                success=bool(result.success), status=str(result.message),
                minimum_constraint=final_minimum, iterations=int(result.nit)))
            if result.success and final_minimum >= -1e-5:
                result.nit = iterations
                return result
            y = result.x.copy() if final_minimum > minimum else restored.x.copy()
        return OptimizeResult(x=y, success=False, message='scaled SQP did not converge under original tolerances', nit=iterations)

    def solve_all(self, *args, **kwargs):
        previous = core.minimize
        core.minimize = self.scaled_minimize
        self.optimizer_calls = []
        try:
            plan, result = original.Planner.solve_all(self, *args, **kwargs)
            result.update(nonlinear_solver='row-scaled restoration and SLSQP',
                optimizer_calls=self.optimizer_calls, independent_cold_start=True,
                cross_condition_warm_start=False, amplitude_continuation=False,
                attitude_model='original fixed world Y', numerical_objective='sum(T)')
            return plan, result
        finally:
            core.minimize = previous
