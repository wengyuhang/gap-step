from types import SimpleNamespace

import casadi as ca
import numpy as np
from scipy.optimize import OptimizeResult

from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import fast_ipopt
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.interior_point import geometric_constraints
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.symbolic_penalty import objective
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario


def test_factorized_geometry_and_eliminated_soft_cost_preserve_values_and_derivatives(monkeypatch):
    monkeypatch.setattr(kernels, 'local_options', lambda name: {'cse': True, 'never_inline': True})
    kernels.node_residuals.cache_clear()
    kernels.node_penalty.cache_clear()
    kernels.dynamic_residuals.cache_clear()
    kernels.slack_integral.cache_clear()
    kernels.segment_coefficients.cache_clear()
    track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
    planner = Planner(track, pose_function=scenario.pose)
    x = ca.MX.sym('factorized_equivalence', planner.nvar)
    witnesses = planner.witnesses()
    witnesses[(0, 7)] = dict(kind='normalized', values=[.2, .7])
    original, original_upper = geometric_constraints(planner, x, witnesses)
    factorized, factorized_upper = kernels.geometric_constraints(planner, x, witnesses)
    np.testing.assert_equal(factorized_upper, original_upper)
    geometry_error = ca.Function('factorized_geometry_error', [x],
        [factorized-original, ca.jacobian(factorized-original, x)])
    coefficients = kernels.curve_coefficients(planner, x)
    curve_difference = coefficients-planner.curve_values(x)
    curve_error = ca.Function('factorized_curve_error', [x],
        [curve_difference, ca.jacobian(curve_difference, x)])
    times = ca.exp(x[:planner.n])
    cost = ca.sum1(times)
    for piece in range(planner.n):
        residuals = kernels.dynamic_residuals()(ca.vertcat(
            coefficients[24*piece:24*(piece+1)], times[piece]))
        cost += kernels.slack_integral()(ca.fmax(residuals, 0.), times[piece])
    difference = cost-objective(planner, x)
    cost_error = ca.Function('factorized_cost_error', [x],
        [difference, ca.gradient(difference, x)])
    for seed in planner.seeds()[:2]:
        values, jacobian = curve_error(seed)
        np.testing.assert_allclose(values, 0., atol=1e-9)
        np.testing.assert_allclose(jacobian, 0., atol=1e-7)
        values, jacobian = geometry_error(seed)
        np.testing.assert_allclose(values, 0., atol=1e-9)
        np.testing.assert_allclose(jacobian, 0., atol=1e-7)
        value, gradient = cost_error(seed)
        np.testing.assert_allclose(value, 0., atol=1e-6)
        np.testing.assert_allclose(gradient, 0., atol=1e-4)


def test_unfinished_finite_candidate_can_add_a_safety_witness(monkeypatch):
    observed_witnesses = []
    cfg = SimpleNamespace(speed=10., acceleration=10., jerk=10.)
    dummy = SimpleNamespace(n=1, m=0, nvar=1, velocity_start=1, full_state=False,
        track=SimpleNamespace(windows=[SimpleNamespace(motion=SimpleNamespace(translation_period=1.))]),
        cfg=cfg, lower=np.array([-10.]), upper=np.array([10.]), witnesses=lambda: {})

    def evaluate(x, witnesses):
        observed_witnesses.append({key: list(item['values']) for key, item in witnesses.items()})
        return np.array([1.]), np.zeros((1, 1)), np.zeros((1, 8, 3))

    checks = []

    def verify(plan, deadline):
        checks.append(True)
        passed = len(checks) > 1
        return [dict(gate=0, piece=0, passed=passed, witness_u=.5)], passed

    calls = []

    def minimize(fun, initial, **kwargs):
        calls.append(True)
        return OptimizeResult(x=initial, success=len(calls) > 1,
            verification_ready=True, message='test finite endpoint', nit=1)

    dummy.evaluate = evaluate
    dummy.verify = verify
    monkeypatch.setattr(core, 'least_squares', lambda fun, x, **kwargs: OptimizeResult(x=x))
    monkeypatch.setattr(core, 'minimize', minimize)
    plan, result = core.Planner.solve(dummy, np.inf, initial_seeds=[np.zeros(1)])
    assert result['found'] and plan is not None
    assert len(calls) == len(checks) == 2
    assert any(row.get((0, 0)) == [.5] for row in observed_witnesses)


def test_reduced_minco_states_preserve_the_analytic_initializer_and_time_derivatives(monkeypatch):
    monkeypatch.setattr(kernels, 'local_options', lambda name: {'cse': True, 'never_inline': True})
    kernels.minco_states.cache_clear()
    track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
    planner = Planner(track, pose_function=scenario.pose)
    x = ca.MX.sym('minco_coordinate_check', planner.nvar)
    states = kernels.minco_states(planner.n)(planner.point_function(x),
        ca.exp(x[:planner.n]), ca.DM(np.column_stack([track.start, track.goal])))
    evaluate = ca.Function('minco_coordinate_check', [x], [states, ca.jacobian(states, x)])
    for seed in planner.seeds()[:2]:
        actual, jacobian = evaluate(seed)
        np.testing.assert_allclose(np.asarray(actual).ravel(), seed[planner.velocity_start:],
            rtol=1e-6, atol=1e-6)
        for column in (0, 4, 8, 24):
            step = np.eye(planner.nvar)[column]*1e-4
            numerical = (np.asarray(evaluate(seed+step)[0])-np.asarray(evaluate(seed-step)[0]))/2e-4
            np.testing.assert_allclose(np.asarray(jacobian)[:, column], numerical.ravel(),
                rtol=1e-4, atol=1e-5)


def test_normal_ipopt_problem_round_trips_free_and_center_initial_states(monkeypatch):
    monkeypatch.setattr(kernels, 'local_options', lambda name: {'cse': True, 'never_inline': True})
    for function in (kernels.minco_states, kernels.segment_coefficients, kernels.node_residuals,
            kernels.dynamic_residuals, kernels.node_penalty, kernels.slack_integral):
        function.cache_clear()
    dimensions = []

    def inspect_solver(name, plugin, problem, options):
        assert plugin == 'ipopt'
        assert options['ipopt.max_iter'] == 5000
        assert options['ipopt.hessian_approximation'] == 'limited-memory'
        dimensions.append((int(problem['x'].numel()), int(problem['g'].numel())))
        return SimpleNamespace()

    monkeypatch.setattr(ca, 'nlpsol', inspect_solver)
    track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
    for free_points in (True, False):
        planner = Planner(track, free_points, solver='ipopt', pose_function=scenario.pose)
        seed = planner.seeds()[0]
        planner._fast_time_origin = np.cumsum(np.exp(seed[:planner.n]))
        planner._fast_normalization = float(np.exp(seed[:planner.n]).sum())
        witnesses = planner.witnesses()
        assert sum(len(item['values']) for item in witnesses.values()) == 7
        problem = fast_ipopt.build_problem(planner, witnesses, lambda row: None, np.inf, False)
        point = np.r_[np.zeros(planner.n), seed[planner.n:planner.velocity_start],
            seed[planner.velocity_start:]/problem['state_scale']]
        parameters = fast_ipopt.coordinate_parameters(planner)
        point[planner.velocity_start:] -= np.asarray(problem['baseline'](point, parameters)).ravel()/problem['state_scale']
        np.testing.assert_allclose(np.asarray(problem['unpack'](point, parameters)).ravel(), seed, atol=1e-9)
    assert dimensions == [(1680, 2033), (1655, 2029)]


def test_cold_starts_reuse_only_the_compiled_graph_and_reset_primal_dual_initialization(monkeypatch):
    monkeypatch.setattr(kernels, 'local_options', lambda name: {'cse': True, 'never_inline': True})
    for function in (kernels.minco_states, kernels.segment_coefficients, kernels.node_residuals,
            kernels.dynamic_residuals, kernels.node_penalty, kernels.slack_integral):
        function.cache_clear()
    factories, inputs = [], []

    class Solver:
        def __init__(self, problem, options):
            assert 'ipopt.warm_start_init_point' not in options
            self.evaluate = ca.Function('cold_graph_probe', [problem['x'], problem['p']],
                [problem['f'], problem['g']])
            self.rows = int(problem['g'].numel())

        def __call__(self, **arguments):
            assert 'lam_x0' not in arguments and 'lam_g0' not in arguments
            inputs.append(arguments['p'].copy())
            objective, values = self.evaluate(arguments['x0'], arguments['p'])
            return dict(x=arguments['x0'], f=objective, g=values,
                lam_x=ca.DM.zeros(len(arguments['x0'])), lam_g=ca.DM.zeros(self.rows))

        def stats(self):
            return dict(success=False, return_status='test cold initialization', iter_count=0)

    def factory(name, plugin, problem, options):
        factories.append(True)
        return Solver(problem, options)

    monkeypatch.setattr(ca, 'nlpsol', factory)
    track, _ = scenario.build_case(scenario.Case(level='static'))
    planner = Planner(track, solver='ipopt', pose_function=scenario.pose)
    monkeypatch.setattr(planner, 'verify', lambda plan, deadline: ([], False))
    plan, result = planner.solve_all()
    assert plan is None and result['all_four_attempted']
    assert len(factories) == 1 and len(inputs) == 4
    assert len({tuple(parameters) for parameters in inputs}) == 4
    assert [row['compiled_cold_graph_reused'] for row in result['optimizer_calls']] == [False, True, True, True]
    assert all(not row['warm_start_used'] for row in result['optimizer_calls'])


def test_extra_temporal_initializers_preserve_the_original_four_physical_curves():
    scales = (1., .8, 1.2, 1.5, .5, .6, .7, .9)
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    for free_points in (True, False):
        original = Planner(track, free_points, solver='ipopt', pose_function=scenario.pose)
        extended = Planner(track, free_points, solver='ipopt', pose_function=scenario.pose,
            temporal_scales=scales)
        original_seeds, extended_seeds = original.seeds(), extended.seeds()
        assert len(original_seeds) == 4 and len(extended_seeds) == 8
        for previous, current in zip(original_seeds, extended_seeds):
            np.testing.assert_array_equal(previous, current)
        assert len({tuple(seed) for seed in extended_seeds}) == 8
        assert extended.nvar == (96 if free_points else 71)


def test_equivalent_circle_warm_preimages_preserve_the_physical_curve(monkeypatch):
    monkeypatch.setattr(kernels, 'local_options', lambda name: {'cse': True, 'never_inline': True})
    monkeypatch.setattr(ca, 'nlpsol', lambda *args: SimpleNamespace())
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    planner = Planner(track, True, solver='ipopt', pose_function=scenario.pose)
    seed = planner.seeds()[0]
    planner._fast_time_origin = np.cumsum(np.exp(seed[:planner.n]))
    planner._fast_normalization = float(np.exp(seed[:planner.n]).sum())
    problem = fast_ipopt.build_problem(planner, planner.witnesses(), lambda row: None, np.inf, False)
    point = np.r_[np.zeros(planner.n), seed[planner.n:planner.velocity_start],
        seed[planner.velocity_start:]/problem['state_scale']]
    parameters = fast_ipopt.coordinate_parameters(planner)
    for gate, (window, selection) in enumerate(zip(track.windows, planner.slices)):
        if window.aperture.kind == 'circle':
            point[selection] = [3000., 2000.] if gate == 3 else [2., 3.]
    before = point.copy()
    after, changed = fast_ipopt.canonical_circle_warm_start(planner, point)
    assert changed == [1, 3, 5]
    np.testing.assert_array_equal(point, before)
    x_before = np.asarray(problem['unpack'](before, parameters)).ravel()
    x_after = np.asarray(problem['unpack'](after, parameters)).ravel()
    np.testing.assert_allclose(planner.point_function(x_before), planner.point_function(x_after), atol=1e-12)
    np.testing.assert_allclose(planner.curve_values(x_before), planner.curve_values(x_after), atol=1e-8)
    np.testing.assert_allclose(planner.objective(x_before)[0], planner.objective(x_after)[0], rtol=1e-10)
    again, changed = fast_ipopt.canonical_circle_warm_start(planner, after)
    assert not changed
    np.testing.assert_array_equal(again, after)
    center = Planner(track, False, solver='ipopt', pose_function=scenario.pose)
    unchanged, changed = fast_ipopt.canonical_circle_warm_start(center, np.zeros(center.nvar))
    assert not changed
    np.testing.assert_array_equal(unchanged, np.zeros(center.nvar))
