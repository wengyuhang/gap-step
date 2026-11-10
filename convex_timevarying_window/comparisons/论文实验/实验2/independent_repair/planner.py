"""Cold-start global solve with arrival-time SQP coordinates and literal time."""
import inspect
import numpy as np
from scipy.optimize import minimize, least_squares, Bounds, NonlinearConstraint, LinearConstraint, OptimizeResult
from scipy.sparse import csr_matrix
from convex_timevarying_window.global_gate_minco_probe import planner as core
from .. import planner as original
from .coordinates import ArrivalCoordinates


class Planner(original.Planner):
    def __init__(self, track, free_points=True, *, pose_function, max_iterations=2000):
        self.max_iterations = max_iterations
        self.pose_function = pose_function
        previous = original.pose
        original.pose = pose_function
        try:
            super().__init__(track, free_points)
        finally:
            original.pose = previous

    def arrival_minimize(self, fun, initial, *, jac, method, bounds, constraints, callback, options):
        assert jac is True and method == 'SLSQP' and constraints['type'] == 'ineq'
        context = inspect.getclosurevars(fun).nonlocals
        coordinates = ArrivalCoordinates(context['origin'], context['scale'], self.n, bounds)

        def objective(z):
            y, derivative = coordinates.unpack(z)
            value, gradient = fun(y)
            return np.expm1(value), np.exp(value)*gradient@derivative

        def values(z):
            return constraints['fun'](coordinates.unpack(z)[0])

        def derivatives(z):
            y, derivative = coordinates.unpack(z)
            return constraints['jac'](y)@derivative

        def residual(z):
            return np.r_[np.minimum(values(z)-1e-4, 0.), np.minimum(coordinates.margins(z), 0.)]

        def residual_jacobian(z):
            return np.vstack((derivatives(z)*(values(z) < 1e-4)[:, None],
                              coordinates.margin_jacobian(z)*(coordinates.margins(z) < 0.)[:, None]))

        restored = least_squares(residual, coordinates.pack(initial), jac=residual_jacobian,
            bounds=np.array(coordinates.bounds).T, x_scale='jac', max_nfev=400,
            ftol=1e-10, xtol=1e-10, gtol=1e-10)
        initial_minimum = float(values(restored.x).min())

        def step(z, state):
            callback(coordinates.unpack(z)[0])

        bounds = np.array(coordinates.bounds)
        nonlinear = NonlinearConstraint(values, 0., np.inf,
            jac=lambda z: csr_matrix(derivatives(z)), keep_feasible=initial_minimum >= 0.)
        temporal = LinearConstraint(csr_matrix(coordinates.difference),
            coordinates.duration_lower, coordinates.duration_upper,
            keep_feasible=float(coordinates.margins(restored.x).min()) >= 0.)
        result = minimize(objective, restored.x, jac=True, hess=lambda z: np.zeros((len(z), len(z))),
            method='trust-constr', bounds=Bounds(bounds[:, 0], bounds[:, 1], keep_feasible=True),
            constraints=(nonlinear, temporal), callback=step,
            options=dict(maxiter=self.max_iterations, gtol=1e-6, xtol=1e-10, barrier_tol=1e-8,
                sparse_jacobian=True, factorization_method='AugmentedSystem',
                initial_barrier_parameter=.01, initial_tr_radius=.1))
        temporal_minimum = float(coordinates.margins(result.x).min())
        self.optimizer_calls.append(dict(status=str(result.message), iterations=int(result.nit),
            success=bool(result.success), temporal_minimum=temporal_minimum,
            restoration_evaluations=int(restored.nfev), restored_node_minimum=initial_minimum,
            optimality=float(result.optimality), constraint_violation=float(result.constr_violation)))
        if temporal_minimum < -1e-7:
            result.success = False
            result.message = 'arrival-coordinate duration bounds violated'
        result.x = coordinates.unpack(result.x)[0]
        return result

    def solve_all(self, *args, **kwargs):
        previous = core.minimize, core.least_squares
        core.minimize = self.arrival_minimize
        core.least_squares = lambda fun, x, **kwargs: OptimizeResult(x=x.copy())
        self.optimizer_calls = []
        try:
            plan, result = super().solve_all(*args, **kwargs)
            result.update(independent_cold_start=True, cross_condition_warm_start=False,
                amplitude_continuation=False, numerical_objective='sum(T)',
                numerical_time_coordinates='absolute arrivals inside trust-constr',
                nonlinear_solver='sparse trust-region interior-point with same-condition restoration',
                attitude_model='original fixed world Y', optimizer_calls=self.optimizer_calls)
            return plan, result
        finally:
            core.minimize, core.least_squares = previous
