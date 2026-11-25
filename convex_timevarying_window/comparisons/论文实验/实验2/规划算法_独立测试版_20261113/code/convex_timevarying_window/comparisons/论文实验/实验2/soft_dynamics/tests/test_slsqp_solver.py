import casadi as ca
import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.slsqp_solver import solve
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import slsqp_solver
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    durations_from_k, duration_jacobian_diagonal, k_from_durations, symbolic_durations_from_k)


def test_slsqp_uses_equalities_two_sided_constraints_and_variable_bounds():
    x = ca.MX.sym('slsqp_adapter_test', 3)
    objective = (x[0]-2.)**2+(x[1]-2.)**2+10.*x[2]**2
    constraints = ca.vertcat(x[0]+x[1], x[0], x[2]-(x[1]-.5))
    result, stats = solve(x, objective, constraints, np.array([1., 1., .5]),
        np.array([-np.inf, -np.inf, 0.]), np.full(3, np.inf),
        np.array([2., 0., 0.]), np.array([2., 1.5, np.inf]),
        100, np.inf, lambda row: None)
    assert stats['success'] and stats['status_code'] == 0
    np.testing.assert_allclose(result['x'], [1.4166666667, .5833333333, .0833333333], atol=1e-6)
    np.testing.assert_allclose(result['g'][0], 2., atol=1e-9)


def test_symbolic_togt_time_mapping_matches_values_derivatives_and_inverse():
    k = ca.MX.sym('togt_positive_time', 9)
    times = symbolic_durations_from_k(k)
    evaluate = ca.Function('togt_positive_time_check', [k], [times, ca.jacobian(times, k)])
    sample = np.array([-1000., -10., -2., -.1, 0., .1, 2., 10., 1000.])
    values, jacobian = evaluate(sample)
    values = np.asarray(values).ravel()
    assert (values > 0.).all()
    np.testing.assert_allclose(values, durations_from_k(sample), rtol=1e-14)
    np.testing.assert_allclose(np.diag(np.asarray(jacobian)), duration_jacobian_diagonal(sample), rtol=1e-14)
    np.testing.assert_allclose(k_from_durations(values), sample, atol=1e-13)


def test_slsqp_constructs_only_trajectory_variables_and_the_original_soft_objective(monkeypatch):
    track, _ = scenario.build_case(scenario.Case(level='static'))
    planner = Planner(track, pose_function=scenario.pose)
    calls = []
    def inspect_problem(variables, objective, constraints, initial, lower_x, upper_x,
                        lower_g, upper_g, max_iterations, deadline, progress):
        assert variables.numel() == planner.nvar == 96
        assert len(initial) == len(lower_x) == len(upper_x) == 96
        assert all('violations' not in symbol.name() for symbol in ca.symvar(objective))
        evaluate = ca.Function('direct_slsqp_construction_test', [variables], [objective, constraints])
        value, margins = evaluate(initial)
        trial = initial.copy()
        trial[:8] = np.maximum(lower_x[:8], np.minimum(upper_x[:8],
            [-4., 2., -3., .5, -1., -2., 1., -3.]))
        trial_value, trial_margins = evaluate(trial)
        gradient = ca.Function('direct_slsqp_trial_gradient', [variables], [ca.gradient(objective, variables)])
        assert np.isfinite(float(trial_value)) and np.isfinite(np.asarray(trial_margins)).all()
        assert np.isfinite(np.asarray(gradient(trial))).all()
        assert (durations_from_k(trial[:8]) > 0.).all()
        np.testing.assert_allclose(durations_from_k(lower_x[:8]), np.exp(planner.lower[:8]), rtol=1e-14)
        np.testing.assert_allclose(durations_from_k(upper_x[:8]), np.exp(planner.upper[:8]), rtol=1e-14)
        calls.append(int(constraints.numel()))
        return dict(x=initial, f=float(value), g=np.asarray(margins).ravel()), dict(
            success=False, return_status='test_only_no_optimization', iter_count=0)
    monkeypatch.setattr(slsqp_solver, 'solve', inspect_problem)
    plan, result = planner.solve_all()
    assert plan is None and len(calls) == 4
    assert result['auxiliary_variables'] == 0
    assert all(row['solver_variables'] == 96 and row['auxiliary_variables'] == 0
        and row['direct_objective_error'] < 1e-7 for row in result['optimizer_calls'])
