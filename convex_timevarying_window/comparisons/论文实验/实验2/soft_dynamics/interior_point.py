"""Shared trajectory coordinates with direct SLSQP and lifted IPOPT backends."""
import inspect
import time

import casadi as ca
import numpy as np
from scipy.optimize import OptimizeResult

from ..independent_repair.symbolic import Progress
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations, symbolic_durations_from_k
from convex_timevarying_window.togt_minco_mpc.trajectory import basis, minco_coefficients
from ..safety import symbolic_constraints
from .symbolic_penalty import instantaneous, positive_penalty, objective as direct_objective


def geometric_constraints(planner, x, witnesses):
    """Remove duplicate rows and combine workspace lower/upper pairs exactly."""
    times = ca.exp(x[:planner.n])
    starts = ca.vertcat(0., ca.cumsum(times)[:-1])
    coefficients = planner.curve_values(x)
    rows, upper = [], []
    span = (np.asarray(planner.cfg.workspace_upper)-planner.cfg.workspace_lower)/30.
    for piece in range(planner.n):
        curve = ca.reshape(coefficients[24*piece:24*(piece+1)], 8, 3)
        for u in np.linspace(0., 1., planner.cfg.nodes_per_piece+1):
            position = curve.T@basis(float(u))
            rows.append((position-ca.DM(planner.cfg.workspace_lower))/30.)
        upper.extend(np.tile(span, planner.cfg.nodes_per_piece+1).tolist())
    for (gate, piece), item in sorted(witnesses.items()):
        for begin in range(0, len(item['values']), planner.cfg.witness_slots):
            offsets = np.resize(item['values'][begin:begin+planner.cfg.witness_slots], planner.cfg.witness_slots)
            clipped = ca.fmin(ca.DM(offsets), times[piece])/times[piece]
            u = 1.-clipped if item['kind'] == 'before' else clipped if item['kind'] == 'after' else ca.DM(offsets)
            curve = ca.reshape(coefficients[24*piece:24*(piece+1)], 8, 3)
            radius = planner.cfg.radius+planner.cfg.frame_thickness+planner.cfg.passage_margin+planner.cfg.witness_slack
            for index in sorted(np.unique(offsets, return_index=True)[1]):
                center, rotation = planner.pose_function(planner.track.windows[gate], starts[piece]+u[index]*times[piece])
                local = rotation.T@(curve.T@basis(u[index])-center)
                values = symbolic_constraints(planner.track.windows[gate].aperture, local, radius)
                rows.append(ca.vertcat(*values))
                upper.extend([np.inf]*len(values))
    for gate, (window, sign) in enumerate(zip(planner.track.windows, planner.signs)):
        normal = ca.DM(-sign*window.state_at(0.)[1][:, 2])
        offset = planner.velocity_start+3*gate
        rows.append(ca.dot(normal, x[offset:offset+3])-.1)
        upper.append(np.inf)
    return ca.vertcat(*rows), np.asarray(upper)


def minimize(planner, fun, initial, *, bounds, constraints, callback, **kwargs):
    context = inspect.getclosurevars(fun).nonlocals
    origin, scale = context['origin'], context['scale']
    evaluate = inspect.getclosurevars(constraints['fun']).nonlocals['evaluate']
    evaluation = inspect.getclosurevars(evaluate).nonlocals
    normalization = max(1., float(np.exp(origin[:planner.n]).sum()))
    z = ca.MX.sym('soft_trajectory_state', planner.nvar)
    if planner.solver == 'slsqp':
        evaluation_times = symbolic_durations_from_k(z[:planner.n])
    else:
        times = ca.vertcat(z[0], z[1:planner.n]-z[:planner.n-1])
        evaluation_times = ca.fmax(times, 1e-8)
    x = ca.vertcat(ca.log(evaluation_times),
        ca.DM(origin[planner.n:])+ca.DM(scale[planner.n:])*z[planner.n:])
    times = ca.exp(x[:planner.n])
    head = ca.horzcat(ca.DM(planner.track.start), ca.DM.zeros(3, 3))
    tail = ca.horzcat(ca.DM(planner.track.goal), ca.DM.zeros(3, 3))
    minco = minco_coefficients(head, tail, planner.point_function(x), times)
    baseline = ca.vertcat(*[minco[8*gate:8*(gate+1), :].T@basis(1., derivative)/times[gate]**derivative
        for derivative in (1, 2, 3) for gate in range(planner.m)])
    baseline_at = ca.Function('current_minco_state', [z], [baseline])
    x = ca.vertcat(x[:planner.velocity_start],
        baseline+ca.DM(scale[planner.velocity_start:])*z[planner.velocity_start:])
    g, geometric_upper = geometric_constraints(planner, x, evaluation['witnesses'])
    extras, extra_lower, extra_upper = [], [], []
    if planner.solver != 'slsqp':
        original_duration = ca.vertcat(z[0], z[1:planner.n]-z[:planner.n-1])
        extras.append(original_duration)
        extra_lower.extend(np.exp(planner.lower[:planner.n]))
        extra_upper.extend(np.exp(planner.upper[:planner.n]))
    if planner.free_points:
        for window, selection in zip(planner.track.windows, planner.slices):
            if window.aperture.kind != 'circle':
                extras.append(ca.sumsqr(x[selection])-1.)
                extra_lower.append(0.)
                extra_upper.append(0.)
    original_count = int(g.numel())
    g = ca.vertcat(g, *extras)
    lower = np.r_[np.zeros(original_count), extra_lower]
    upper = np.r_[geometric_upper, extra_upper]
    variables = z
    slack = None
    if planner.solver == 'slsqp':
        f = direct_objective(planner, x)
    else:
        slack = ca.MX.sym('soft_epigraph_violations', planner.n*33*6)
        coefficients = planner.curve_values(x)
        physical = []
        from . import kernels
        if planner.dynamics_model == 'x500':
            from .x500_dynamics import instantaneous as x500_instantaneous
            native = x500_instantaneous(planner.dynamic_limit_fraction)
        else:
            native = instantaneous(limit_fraction=planner.dynamic_limit_fraction)
        residual_scale = ca.DM(kernels.residual_scale(planner.dynamics_model, planner.dynamic_limit_fraction))
        f = ca.sum1(times)
        for piece in range(planner.n):
            curve = ca.reshape(coefficients[24*piece:24*(piece+1)], 8, 3)
            for node in range(33):
                orders = (1, 2, 3, 4) if planner.dynamics_model == 'x500' else (2, 3, 4)
                states = [curve.T@basis(node/32., order)/times[piece]**order for order in orders]
                residual = native(*states)[3]/residual_scale
                offset = (piece*33+node)*6
                violation = slack[offset:offset+6]
                physical.append(residual)
                weight = .5 if node in (0, 32) else 1.
                f += 1000.*times[piece]/32.*weight*sum(
                    positive_penalty(value) for value in ca.vertsplit(violation*residual_scale))
        residuals = ca.vertcat(*physical)
        initial_residuals = ca.Function('initial_epigraph_residuals', [z], [residuals])
        g = ca.vertcat(g, slack-residuals)
        lower = np.r_[lower, np.zeros(int(slack.numel()))]
        upper = np.r_[upper, np.full(int(slack.numel()), np.inf)]
        variables = ca.vertcat(z, slack)
    started = time.perf_counter()
    original_bounds = np.asarray(bounds).copy()
    if planner.solver == 'slsqp':
        original_bounds[:planner.n, 0] = k_from_durations(np.exp(planner.lower[:planner.n]))
        original_bounds[:planner.n, 1] = k_from_durations(np.exp(planner.upper[:planner.n]))
    else:
        original_bounds[:planner.n] = [-np.inf, np.inf]
    lbx, ubx = original_bounds[:, 0], original_bounds[:, 1]
    coordinate_initial = initial.copy()
    initial_times = np.exp(origin[:planner.n]+scale[:planner.n]*initial[:planner.n])
    coordinate_initial[:planner.n] = (k_from_durations(initial_times) if planner.solver == 'slsqp'
        else np.cumsum(initial_times))
    actual_states = origin[planner.velocity_start:]+scale[planner.velocity_start:]*initial[planner.velocity_start:]
    coordinate_initial[planner.velocity_start:] = (actual_states-np.asarray(baseline_at(coordinate_initial)).ravel())/scale[planner.velocity_start:]
    starting = coordinate_initial
    if slack is not None:
        lbx = np.r_[lbx, np.zeros(int(slack.numel()))]
        ubx = np.r_[ubx, np.full(int(slack.numel()), np.inf)]
        initial_slack = np.maximum(np.asarray(initial_residuals(coordinate_initial)).ravel(), 0.)
        starting = np.r_[coordinate_initial, initial_slack]
    if planner.solver == 'slsqp':
        from .slsqp_solver import solve
        result, stats = solve(variables, f/normalization, g, starting, lbx, ubx,
            lower, upper, max(5000, planner.max_iterations), evaluation['deadline'], evaluation['progress'])
        success = stats['success']
    else:
        progress = Progress(int(variables.numel()), lower, upper, evaluation['deadline'], evaluation['progress'])
        solver = ca.nlpsol('independent_soft_epigraph', 'ipopt', dict(x=variables, f=f/normalization, g=g), {
            'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': False, 'error_on_fail': False,
            'ipopt.max_iter': max(5000, planner.max_iterations), 'ipopt.tol': 1e-8,
            'ipopt.constr_viol_tol': 1e-7, 'ipopt.dual_inf_tol': 1e-5,
            'ipopt.compl_inf_tol': 1e-9, 'ipopt.acceptable_iter': 0,
            'ipopt.bound_relax_factor': 0., 'ipopt.mu_strategy': 'adaptive', 'ipopt.nlp_scaling_method': 'none',
            'ipopt.hessian_approximation': 'exact', 'iteration_callback': progress})
        result = solver(x0=starting, lbx=lbx, ubx=ubx, lbg=lower, ubg=upper)
        stats = solver.stats()
        success = stats['return_status'] == 'Solve_Succeeded'
    solver_point = np.asarray(result['x']).ravel()
    unpack = ca.Function('epigraph_original_coordinates', [z], [x])
    original_point = np.asarray(unpack(solver_point[:planner.nvar])).ravel()
    y = (original_point-origin)/scale
    minimum = float(constraints['fun'](y).min())
    native_value, _, _ = planner.objective(origin+scale*y)
    objective_difference = float(result['f'])*normalization-native_value
    gap = objective_difference if slack is not None else None
    actual_g = np.asarray(result['g']).ravel()
    original_violation = float(np.maximum(np.maximum(lower-actual_g, actual_g-upper), 0.).max())
    stationarity_inf = None
    if planner.solver != 'slsqp':
        objective_gradient = ca.Function('epigraph_final_gradient', [variables], [ca.gradient(f/normalization, variables)])
        constraint_jacobian = ca.Function('epigraph_final_jacobian', [variables], [ca.jacobian(g, variables)])
        stationarity = np.asarray(objective_gradient(solver_point)).ravel()
        stationarity += np.asarray(constraint_jacobian(solver_point)).T@np.asarray(result['lam_g']).ravel()
        stationarity += np.asarray(result['lam_x']).ravel()
        stationarity_inf = float(abs(stationarity).max())
    eliminated_error = None
    if slack is not None:
        reduced = ca.substitute(f, slack, ca.fmax(residuals, 0.))
        reduced_value = float(ca.Function('eliminated_soft_epigraph_cost', [z], [reduced])(solver_point[:planner.nvar]))
        eliminated_error = abs(reduced_value-native_value)
    stage = 'direct_soft_slsqp' if planner.solver == 'slsqp' else 'equivalent_soft_epigraph'
    row = dict(stage=stage, success=bool(success), status=stats['return_status'],
        solver=planner.solver, solver_variables=int(variables.numel()),
        auxiliary_variables=int(slack.numel()) if slack is not None else 0, constraint_rows=int(g.numel()),
        iterations=int(stats['iter_count']), elapsed_s=time.perf_counter()-started,
        minimum_constraint=minimum, native_objective=native_value, epigraph_gap=gap,
        direct_objective_error=abs(objective_difference) if slack is None else None,
        constraint_violation_inf=original_violation,
        epigraph_constraint_violation_inf=original_violation if slack is not None else None,
        lagrangian_stationarity_inf=stationarity_inf,
        original_variables=original_point.tolist(), eliminated_objective_error=eliminated_error,
        solver_coordinates=('TOGT positive durations and full MINCO-centered PVAJ deviations with polygon gauge'
            if planner.solver == 'slsqp' else 'absolute arrivals and full MINCO-centered PVAJ deviations with polygon gauge'),
        last_primal_infeasibility=stats.get('iterations', {}).get('inf_pr', [])[-1:],
        last_dual_infeasibility=stats.get('iterations', {}).get('inf_du', [])[-1:])
    planner.optimizer_calls.append(row)
    evaluation['progress'](row)
    callback(y)
    return OptimizeResult(x=y.copy(), success=bool(success), message=stats['return_status'], nit=int(stats['iter_count']))
