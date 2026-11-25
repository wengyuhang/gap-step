"""Independent original-model solves using sparse exact first/second derivatives."""
import inspect
import time
import casadi as ca
import numpy as np
from scipy.optimize import OptimizeResult
from convex_timevarying_window.global_gate_minco_probe import planner as core
from .. import planner as original
from .planner import Planner as ArrivalPlanner
from .coordinates import ArrivalCoordinates
from .symbolic import constraints as original_constraints, Progress


class Planner(ArrivalPlanner):
    def arrival_minimize(self, fun, initial, *, jac, method, bounds, constraints: dict, callback, options):
        # The core evaluator owns the original geometry witnesses and deadline.
        context = inspect.getclosurevars(fun).nonlocals
        evaluate = inspect.getclosurevars(constraints['fun']).nonlocals['evaluate']
        evaluation = inspect.getclosurevars(evaluate).nonlocals
        coordinates = ArrivalCoordinates(context['origin'], context['scale'], self.n, bounds)
        z = ca.MX.sym('absolute_arrivals_and_scaled_states', self.nvar)
        duration = ca.DM(coordinates.difference)@z
        log_time = ca.log(ca.fmax(duration, 1e-8))
        x = ca.vertcat(log_time, ca.DM(context['origin'][self.n:])+ca.DM(context['scale'][self.n:])*z[self.n:])
        original_g, upper = original_constraints(self, x, evaluation['witnesses'])
        g = ca.vertcat(original_g, duration)
        lower = np.r_[np.zeros(len(upper)), coordinates.duration_lower]
        upper = np.r_[upper, coordinates.duration_upper]
        progress = Progress(self.nvar, lower, upper, evaluation['deadline'], evaluation['progress'])
        began = time.perf_counter()
        remaining = max(.01, evaluation['deadline']-began)
        solver = ca.nlpsol('independent_exact_exchange', 'ipopt', dict(x=z, f=z[self.n-1], g=g),
            {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': False,
             'ipopt.max_iter': self.max_iterations, 'ipopt.max_cpu_time': remaining,
             'ipopt.tol': 1e-6, 'ipopt.constr_viol_tol': 1e-7,
             'ipopt.dual_inf_tol': 1e-5, 'ipopt.compl_inf_tol': 1e-6,
             'ipopt.acceptable_iter': 0, 'ipopt.bound_relax_factor': 0.,
             'ipopt.mu_strategy': 'adaptive', 'ipopt.hessian_approximation': 'exact',
             'iteration_callback': progress, 'error_on_fail': False})
        variable_bounds = np.asarray(coordinates.bounds)
        result = solver(x0=coordinates.pack(initial), lbx=variable_bounds[:, 0], ubx=variable_bounds[:, 1], lbg=lower, ubg=upper)
        stats = solver.stats()
        z_final = np.asarray(result['x']).ravel()
        status = stats['return_status']
        success = status == 'Solve_Succeeded' and coordinates.margins(z_final).min() >= -1e-7
        self.optimizer_calls.append(dict(status=status, success=bool(success), iterations=int(stats['iter_count']),
            elapsed_s=time.perf_counter()-began, compressed_rows=len(lower),
            last_dual_infeasibility=stats.get('iterations', {}).get('inf_du', [])[-1:],
            last_primal_infeasibility=stats.get('iterations', {}).get('inf_pr', [])[-1:]))
        return OptimizeResult(x=coordinates.unpack(z_final)[0], success=success,
            message=status, nit=int(stats['iter_count']), fun=float(result['f']))

    def solve_all(self, *args, **kwargs):
        previous = core.minimize
        core.minimize = self.arrival_minimize
        self.optimizer_calls = []
        try:
            plan, result = original.Planner.solve_all(self, *args, **kwargs)
            result.update(nonlinear_solver='sparse IPOPT with exact Hessian',
                numerical_time_coordinates='absolute arrivals inside IPOPT',
                constraints_compressed_without_relaxation=True, optimizer_calls=self.optimizer_calls,
                independent_cold_start=True, cross_condition_warm_start=False,
                amplitude_continuation=False, attitude_model='original fixed world Y',
                numerical_objective='sum(T)', restoration='original least_squares max_nfev 200')
            return plan, result
        finally:
            core.minimize = previous
