"""Analytic first-order SLSQP backend for the direct soft trajectory objective."""
import time

import casadi as ca
import numpy as np
from scipy.optimize import minimize

from convex_timevarying_window.global_gate_minco_probe.planner import Deadline


def solve(variables, objective, constraints, initial, lower_x, upper_x,
          lower_g, upper_g, max_iterations, deadline, progress):
    value_function = ca.Function('slsqp_soft_values', [variables], [objective, constraints])
    derivative_function = ca.Function('slsqp_soft_derivatives', [variables],
        [ca.gradient(objective, variables), ca.jacobian(constraints, variables)])
    equality = np.isfinite(lower_g) & (lower_g == upper_g)
    lower_rows = np.isfinite(lower_g) & ~equality
    upper_rows = np.isfinite(upper_g) & ~equality
    cached_value_point = cached_derivative_point = None
    cached_values = cached_derivatives = None
    began = last_report = time.perf_counter()
    iterations = 0

    def values(point):
        nonlocal cached_value_point, cached_values
        if time.perf_counter() >= deadline:
            raise Deadline
        if cached_value_point is None or not np.array_equal(point, cached_value_point):
            try:
                value, margins = value_function(point)
            except RuntimeError as exc:
                progress(dict(stage='slsqp_evaluation_failure', kind='value',
                    trial_variables=point.tolist(), error=str(exc), elapsed_s=time.perf_counter()-began))
                raise
            cached_value_point = point.copy()
            cached_values = float(value), np.asarray(margins).ravel()
        return cached_values

    def derivatives(point):
        nonlocal cached_derivative_point, cached_derivatives
        if time.perf_counter() >= deadline:
            raise Deadline
        if cached_derivative_point is None or not np.array_equal(point, cached_derivative_point):
            try:
                gradient, jacobian = derivative_function(point)
            except RuntimeError as exc:
                progress(dict(stage='slsqp_evaluation_failure', kind='derivative',
                    trial_variables=point.tolist(), error=str(exc), elapsed_s=time.perf_counter()-began))
                raise
            cached_derivative_point = point.copy()
            cached_derivatives = np.asarray(gradient).ravel(), np.asarray(jacobian)
        return cached_derivatives

    def inequality_values(point):
        margins = values(point)[1]
        return np.r_[margins[lower_rows]-lower_g[lower_rows],
            upper_g[upper_rows]-margins[upper_rows]]

    def inequality_jacobian(point):
        jacobian = derivatives(point)[1]
        return np.vstack([jacobian[lower_rows], -jacobian[upper_rows]])

    def callback(point):
        nonlocal iterations, last_report
        iterations += 1
        now = time.perf_counter()
        if now-last_report >= 5.:
            value, margins = values(point)
            minimum = float(np.minimum(margins-lower_g, upper_g-margins).min())
            progress(dict(stage='exact_slsqp', iteration=iterations,
                elapsed_s=now-began, objective=value, minimum_constraint=minimum))
            last_report = now

    rows = []
    if equality.any():
        rows.append(dict(type='eq',
            fun=lambda point: values(point)[1][equality]-lower_g[equality],
            jac=lambda point: derivatives(point)[1][equality]))
    if lower_rows.any() or upper_rows.any():
        rows.append(dict(type='ineq', fun=inequality_values, jac=inequality_jacobian))
    progress(dict(stage='slsqp_started', solver_variables=int(variables.numel()),
        constraint_rows=int(constraints.numel()), equality_rows=int(equality.sum()),
        inequality_rows=int(lower_rows.sum()+upper_rows.sum()), ftol=1e-8,
        max_iterations=max_iterations, analytical_first_derivatives=True))
    result = minimize(lambda point: values(point)[0], initial,
        jac=lambda point: derivatives(point)[0], method='SLSQP',
        bounds=list(zip(lower_x, upper_x)), constraints=rows, callback=callback,
        options=dict(maxiter=max_iterations, ftol=1e-8, disp=False))
    value, margins = values(result.x)
    returned = dict(x=result.x, f=value, g=margins)
    stats = dict(return_status=str(result.message), iter_count=int(result.nit),
        success=bool(result.success), status_code=int(result.status),
        function_evaluations=int(result.nfev), gradient_evaluations=int(result.njev))
    return returned, stats
