"""Same-target augmented-Lagrangian restoration followed by original SQP."""
import inspect
import numpy as np
from scipy.optimize import minimize, OptimizeResult
from convex_timevarying_window.global_gate_minco_probe import planner as core
from .. import planner as original
from .planner import Planner as ConfiguredPlanner


class Planner(ConfiguredPlanner):
    def augmented_minimize(self, fun, initial, *, jac, method, bounds, constraints, callback, options):
        evaluate = inspect.getclosurevars(constraints['fun']).nonlocals['evaluate']
        progress = inspect.getclosurevars(evaluate).nonlocals['progress']
        y = initial.copy()
        multiplier = np.zeros_like(constraints['fun'](y))
        penalty = 100.
        previous_violation = max(0., -float(constraints['fun'](y).min()))
        iterations = 0
        interior_margin = 1e-4
        best_feasible = None
        last_refined_time = np.inf

        def literal(at):
            value, gradient = fun(at)
            return np.expm1(value), np.exp(value)*gradient

        for outer in range(16):
            def lagrangian(at):
                nonlocal best_feasible
                value, gradient = literal(at)
                g = constraints['fun'](at)-interior_margin
                if g.min() >= -1e-5 and (best_feasible is None or value < best_feasible[0]):
                    best_feasible = value, at.copy()
                dual = np.maximum(multiplier-penalty*g, 0.)
                value += float(np.where(dual > 0., .5*penalty*g*g-multiplier*g,
                    -multiplier*multiplier/(2*penalty)).sum())
                gradient -= constraints['jac'](at).T@dual
                return value, gradient

            result = minimize(lagrangian, y, jac=True, method='L-BFGS-B', bounds=bounds,
                callback=callback, options=dict(maxiter=120, maxls=100, maxcor=30,
                    ftol=1e-10, gtol=max(1e-7, 1e-3/(10**outer)), maxfun=4000))
            y = result.x.copy()
            iterations += int(result.nit)
            g = constraints['fun'](y)-interior_margin
            violation = max(0., -float(g.min()))
            row = dict(stage='augmented_lagrangian', round=outer, penalty=penalty,
                minimum_constraint=float(g.min()+interior_margin), time_s=float(literal(y)[0]),
                initialization_interior_margin=interior_margin,
                best_strictly_feasible_time_s=best_feasible[0] if best_feasible else None,
                inner_converged=bool(result.success), inner_status=str(result.message),
                inner_iterations=int(result.nit))
            self.optimizer_calls.append(row)
            progress(row)
            if best_feasible is not None and best_feasible[0] < last_refined_time-1e-6:
                last_refined_time = best_feasible[0]
                normalization = max(1., last_refined_time)
                def normalized_time(at):
                    value, gradient = literal(at)
                    return value/normalization, gradient/normalization
                refined = minimize(normalized_time, best_feasible[1], jac=True, method='SLSQP', bounds=bounds,
                    constraints=constraints, callback=callback,
                    options=dict(options, maxiter=self.max_iterations, ftol=1e-5))
                iterations += int(refined.nit)
                self.optimizer_calls.append(dict(stage='sqp_refinement', status=str(refined.message),
                    success=bool(refined.success), iterations=int(refined.nit),
                    minimum_constraint=float(constraints['fun'](refined.x).min())))
                if refined.success and constraints['fun'](refined.x).min() >= -1e-5:
                    refined.nit = iterations
                    return refined
            multiplier = np.maximum(multiplier-penalty*g, 0.)
            if violation > .25*previous_violation:
                penalty = min(1e10, penalty*10.)
            previous_violation = violation
        return OptimizeResult(x=y, success=False, message='augmented restoration did not reach SQP convergence', nit=iterations)

    def solve_all(self, *args, **kwargs):
        previous = core.minimize
        core.minimize = self.augmented_minimize
        self.optimizer_calls = []
        try:
            plan, result = original.Planner.solve_all(self, *args, **kwargs)
            result.update(nonlinear_solver='augmented Lagrangian plus converged SQP',
                optimizer_calls=self.optimizer_calls, independent_cold_start=True,
                cross_condition_warm_start=False, amplitude_continuation=False,
                attitude_model='original fixed world Y', numerical_objective='sum(T)',
                sqp_normalized_ftol=1e-5, original_node_acceptance_tolerance=1e-5)
            return plan, result
        finally:
            core.minimize = previous
