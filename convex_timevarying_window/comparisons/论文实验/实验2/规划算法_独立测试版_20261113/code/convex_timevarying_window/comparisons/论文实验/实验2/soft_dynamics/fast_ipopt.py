"""Factorized finite SIP subproblems with stable full-PVAJ coordinates."""
import inspect
import time

import casadi as ca
import numpy as np
from scipy.optimize import OptimizeResult

from ..independent_repair.symbolic import Progress
from . import kernels


def coordinate_parameters(planner):
    frequencies = np.array([window.motion.frequency for window in planner.track.windows])
    angular = np.r_[2.*np.pi*frequencies, 2.*np.pi*frequencies.mean()]
    origin = planner._fast_time_origin
    scale = 1./np.maximum(1., angular)
    return np.r_[planner._fast_normalization, origin, scale]


def canonical_circle_warm_start(planner, variables):
    """Choose equivalent, well-scaled TOGT preimages of the same warm curve."""
    point = variables.copy()
    changed = []
    for gate, (window, selection) in enumerate(zip(planner.track.windows,
            planner.slices if planner.free_points else [])):
        if window.aperture.kind == 'circle':
            value = point[selection].copy()
            squared = float(value@value)
            if squared > 1.:
                point[selection] = value/squared
                changed.append(gate)
    return point, changed


def build_problem(planner, witnesses, progress, deadline, warm):
    began = time.perf_counter()
    z = ca.MX.sym('finite_trajectory', planner.nvar)
    slack = ca.MX.sym('finite_soft_violations', planner.n*33*6)
    slack_scale = np.tile(.01/kernels.residual_scale(planner.dynamics_model, planner.dynamic_limit_fraction), planner.n*33)
    scaled_slack = ca.DM(slack_scale)*slack
    parameters = ca.MX.sym('coordinate_parameters', 1+2*planner.n)
    normalization = parameters[0]
    time_origin, time_scale = parameters[1:planner.n+1], parameters[planner.n+1:]
    arrivals = time_origin+time_scale*z[:planner.n]
    durations = ca.vertcat(arrivals[0], arrivals[1:]-arrivals[:-1])
    state_scale = np.r_[np.full(3*planner.m, planner.cfg.speed),
        np.full(3*planner.m, planner.cfg.acceleration), np.full(3*planner.m, planner.cfg.jerk)]
    x = ca.vertcat(ca.log(ca.fmax(durations, 1e-8)), z[planner.n:planner.velocity_start],
        ca.DM(state_scale)*z[planner.velocity_start:])
    times = ca.exp(x[:planner.n])
    baseline = kernels.minco_states(planner.n)(planner.point_function(x), times,
        ca.DM(np.column_stack([planner.track.start, planner.track.goal])))
    x = ca.vertcat(x[:planner.velocity_start], baseline+x[planner.velocity_start:])
    coefficients = kernels.curve_coefficients(planner, x)
    geometry, geometric_upper = kernels.geometric_constraints(planner, x, witnesses, coefficients)
    extras, extra_lower, extra_upper = [durations], list(np.exp(planner.lower[:planner.n])), list(np.exp(planner.upper[:planner.n]))
    for window, selection in zip(planner.track.windows, planner.slices if planner.free_points else []):
        if window.aperture.kind != 'circle':
            extras.append(ca.sumsqr(x[selection])-1.)
            extra_lower.append(0.)
            extra_upper.append(0.)
    residuals = []
    objective = ca.sum1(times)
    for piece in range(planner.n):
        residuals.append(kernels.dynamic_residuals(planner.dynamics_model, planner.dynamic_limit_fraction)(ca.vertcat(
            coefficients[24*piece:24*(piece+1)], times[piece])))
        segment_values = scaled_slack[198*piece:198*(piece+1)]
        objective += kernels.slack_integral(planner.dynamics_model, planner.dynamic_limit_fraction)(segment_values, times[piece])
    residuals = ca.vertcat(*residuals)
    constraints = ca.vertcat(geometry, *extras, scaled_slack-residuals)
    lower = np.r_[np.zeros(int(geometry.numel())), extra_lower, np.zeros(int(slack.numel()))]
    upper = np.r_[geometric_upper, extra_upper, np.full(int(slack.numel()), np.inf)]
    variables = ca.vertcat(z, slack)
    callback = Progress(int(variables.numel()), lower, upper, deadline, progress)
    options = {'ipopt.print_level': 0, 'ipopt.sb': 'yes', 'print_time': False,
        'error_on_fail': False, 'record_time': True,
        'ipopt.max_iter': max(5000, planner.max_iterations),
        'ipopt.tol': 1e-8, 'ipopt.constr_viol_tol': 1e-7,
        'ipopt.dual_inf_tol': 1e-5, 'ipopt.compl_inf_tol': 1e-9,
        'ipopt.acceptable_iter': 0, 'ipopt.bound_relax_factor': 0.,
        'ipopt.mu_strategy': 'adaptive', 'ipopt.nlp_scaling_method': 'none',
        'ipopt.hessian_approximation': 'limited-memory', 'ipopt.limited_memory_max_history': 6,
        'iteration_callback': callback}
    if warm:
        options.update({'ipopt.warm_start_init_point': 'yes',
            'ipopt.warm_start_bound_push': 1e-8, 'ipopt.warm_start_mult_bound_push': 1e-8,
            'ipopt.warm_start_slack_bound_push': 1e-8})
    solver = ca.nlpsol('factorized_finite_sip', 'ipopt',
        dict(x=variables, p=parameters, f=objective/normalization, g=constraints), options)
    keys = [('workspace', piece, node, axis) for piece in range(planner.n)
        for node in range(planner.cfg.nodes_per_piece+1) for axis in range(3)]
    for (gate, piece), item in sorted(witnesses.items()):
        edges = 1 if planner.track.windows[gate].aperture.kind == 'circle' else len(planner.track.windows[gate].aperture.physical_vertices)
        for begin in range(0, len(item['values']), planner.cfg.witness_slots):
            offsets = np.resize(item['values'][begin:begin+planner.cfg.witness_slots], planner.cfg.witness_slots)
            offsets = offsets[sorted(np.unique(offsets, return_index=True)[1])]
            keys.extend(('frame', gate, piece, item['kind'], float(offset), edge)
                for offset in offsets for edge in range(edges))
    keys.extend(('forward', gate) for gate in range(planner.m))
    keys.extend(('duration', piece) for piece in range(planner.n))
    keys.extend(('gauge', gate) for gate, window in enumerate(planner.track.windows)
        if planner.free_points and window.aperture.kind != 'circle')
    keys.extend(('soft', index) for index in range(int(slack.numel())))
    assert len(keys) == int(constraints.numel())
    return dict(solver=solver, callback=callback, lower=lower, upper=upper, keys=keys,
        unpack=ca.Function('finite_original_variables', [z, parameters], [x]),
        baseline=ca.Function('finite_minco_baseline', [z, parameters], [baseline]),
        residuals=ca.Function('finite_initial_residuals', [z, parameters], [residuals]),
        gradient=ca.Function('finite_objective_gradient', [variables, parameters],
            [ca.gradient(objective/normalization, variables)]),
        jacobian=ca.Function('finite_constraint_jacobian', [variables, parameters], [ca.jacobian(constraints, variables)]),
        state_scale=state_scale, slack_scale=slack_scale,
        build_s=time.perf_counter()-began,
        variables=int(variables.numel()), warm=warm)


def minimize(planner, fun, initial, *, bounds, constraints, callback, **kwargs):
    began = time.perf_counter()
    context = inspect.getclosurevars(fun).nonlocals
    origin, scale = context['origin'], context['scale']
    evaluate = inspect.getclosurevars(constraints['fun']).nonlocals['evaluate']
    evaluation = inspect.getclosurevars(evaluate).nonlocals
    witnesses = evaluation['witnesses']
    warm = getattr(planner, '_fast_warm', None)
    signature = (bool(warm), tuple((gate, piece, item['kind'], tuple(item['values']))
        for (gate, piece), item in sorted(witnesses.items())))
    cache_name = '_fast_problem' if warm else '_fast_cold_problem'
    cached = getattr(planner, cache_name, None)
    if cached is None or cached[0] != signature:
        problem = build_problem(planner, witnesses, evaluation['progress'], evaluation['deadline'], bool(warm))
        setattr(planner, cache_name, (signature, problem))
    else:
        problem = cached[1]
    progress = problem['callback']
    progress.began = progress.last = time.perf_counter()
    progress.count = 0
    progress.progress = evaluation['progress']
    progress.deadline = evaluation['deadline']
    actual = origin+scale*initial
    parameters = coordinate_parameters(planner)
    time_origin, time_scale = parameters[1:planner.n+1], parameters[planner.n+1:]
    point = np.r_[(np.cumsum(np.exp(actual[:planner.n]))-time_origin)/time_scale, actual[planner.n:planner.velocity_start],
        actual[planner.velocity_start:]/problem['state_scale']]
    point[planner.velocity_start:] -= np.asarray(problem['baseline'](point, parameters)).ravel()/problem['state_scale']
    residuals = np.asarray(problem['residuals'](point, parameters)).ravel()
    initial_slack = np.maximum(residuals, 0.)/problem['slack_scale']
    starting = np.r_[point, initial_slack]
    lower_x = np.r_[np.full(planner.nvar, -np.inf), np.zeros(len(initial_slack))]
    upper_x = np.full(len(starting), np.inf)
    arguments = dict(x0=starting, p=parameters,
        lbx=lower_x, ubx=upper_x, lbg=problem['lower'], ubg=problem['upper'])
    canonicalized = []
    if warm:
        warm_point, canonicalized = canonical_circle_warm_start(planner, warm['variables'])
        arguments.update(x0=warm_point, lam_x0=warm['lam_x'],
            lam_g0=np.array([warm['lam_g'].get(key, 0.) for key in problem['keys']]))
    solve_began = time.perf_counter()
    result = problem['solver'](**arguments)
    solve_s = time.perf_counter()-solve_began
    statistics = problem['solver'].stats()
    returned = np.asarray(result['x']).ravel()
    original = np.asarray(problem['unpack'](returned[:planner.nvar], parameters)).ravel()
    y = (original-origin)/scale
    minimum = float(constraints['fun'](y).min())
    native, _, penalty = planner.objective(original)
    g = np.asarray(result['g']).ravel()
    violation = float(np.maximum(np.maximum(problem['lower']-g, g-problem['upper']), 0.).max())
    lam_x, lam_g = np.asarray(result['lam_x']).ravel(), np.asarray(result['lam_g']).ravel()
    stationarity = np.asarray(problem['gradient'](returned, parameters)).ravel()
    stationarity += np.asarray(problem['jacobian'](returned, parameters)).T@lam_g+lam_x
    planner._fast_warm = dict(variables=returned.copy(), lam_x=lam_x.copy(),
        lam_g=dict(zip(problem['keys'], lam_g)))
    success = bool(statistics['success'])
    row = dict(stage='factorized_finite_ipopt', solver='ipopt', success=success,
        status=statistics['return_status'], iterations=int(statistics['iter_count']),
        elapsed_s=time.perf_counter()-began, solver_s=solve_s,
        problem_build_s=problem['build_s'] if cached is None or cached[0] != signature else 0.,
        warm_start_used=bool(warm), solver_variables=problem['variables'],
        canonicalized_circle_gates=canonicalized,
        compiled_cold_graph_reused=bool(not warm and cached is not None and cached[0] == signature),
        auxiliary_variables=len(initial_slack), constraint_rows=len(g),
        formulation='equivalent residual epigraph', hessian='limited-memory', history=6,
        slack_coordinates='physical residual divided by original penalty smoothing width 0.01',
        minimum_constraint=minimum, native_objective=native, dynamic_soft_integral=penalty,
        epigraph_gap=float(result['f'])*planner._fast_normalization-native,
        constraint_violation_inf=violation, lagrangian_stationarity_inf=float(abs(stationarity).max()),
        original_variables=original.tolist(),
        timing={key:value for key,value in statistics.items()
            if key.startswith(('t_wall_', 'n_call_'))},
        solver_coordinates='phase-scaled arrival times and full reduced-MINCO PVAJ deviations')
    planner.optimizer_calls.append(row)
    evaluation['progress'](row)
    callback(y)
    return OptimizeResult(x=y.copy(), success=success, message=statistics['return_status'],
        nit=int(statistics['iter_count']), verification_ready=bool(np.isfinite(y).all()))
