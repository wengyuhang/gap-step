"""Direct free-point SQP with same-problem feasibility restoration."""
import numpy as np
from scipy.optimize import minimize
from ..independent_repair.smooth_augmented import Planner as BasePlanner


class Planner(BasePlanner):
    def augmented_minimize(self, fun, initial, *, jac, method, bounds, constraints, callback, options):
        normalization = max(1., float(np.expm1(fun(initial)[0])))

        def objective(at):
            value, gradient = fun(at)
            return np.expm1(value)/normalization, np.exp(value)*gradient/normalization

        result = minimize(objective, initial, jac=jac, method=method, bounds=bounds,
            constraints=constraints, callback=callback,
            options=dict(options, maxiter=self.max_iterations, ftol=1e-5))
        minimum = float(constraints['fun'](result.x).min())
        self.optimizer_calls.append(dict(stage='direct_free_sqp', success=bool(result.success),
            status=str(result.message), iterations=int(result.nit), minimum_constraint=minimum))
        if result.success and minimum >= -1e-5:
            return result
        return super().augmented_minimize(fun, initial, jac=jac, method=method, bounds=bounds,
            constraints=constraints, callback=callback, options=options)

    def solve_all(self, *args, **kwargs):
        plan, result = super().solve_all(*args, **kwargs)
        result.update(nonlinear_solver='direct normalized SQP with same-problem augmented restoration',
            initialization='four analytic free-point cold starts',
            center_solution_used=False, center_subproblem_solved=False,
            crossing_points_free_from_first_iteration=bool(self.free_points))
        return plan, result
