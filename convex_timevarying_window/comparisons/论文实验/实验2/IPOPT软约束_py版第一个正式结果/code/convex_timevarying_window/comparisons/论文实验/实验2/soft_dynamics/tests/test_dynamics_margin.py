from types import SimpleNamespace

import casadi as ca
import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import kernels, fast_ipopt
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.native import Penalty
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.symbolic_penalty import instantaneous, objective
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario


def test_margin_changes_only_existing_upper_limits():
    a, j, s = np.array([5., 1., 2.]), np.array([3., 2., 4.]), np.array([8., -2., 3.])
    base = instantaneous()(a, j, s)
    margin = instantaneous(limit_fraction=.9)(a, j, s)
    np.testing.assert_array_equal(base[1], margin[1])
    np.testing.assert_array_equal(base[2], margin[2])
    rates, rotors = np.asarray(margin[1]).ravel(), np.asarray(margin[2]).ravel()
    residuals = np.r_[sum(rates[:2]**2)-14.74348422*.9**2,
        rates[2]**2-12.18469679*.9**2, rotors*(rotors-8.54858*.9)]
    np.testing.assert_allclose(margin[3], residuals[:, None], atol=1e-12)


def test_margin_preserves_geometry_initializers_and_finite_problem_size(monkeypatch):
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    base = Planner(track, solver='ipopt', pose_function=scenario.pose)
    margin = Planner(track, solver='ipopt', pose_function=scenario.pose, dynamic_limit_fraction=.9)
    for original, initial in zip(base.seeds(), margin.seeds()):
        np.testing.assert_array_equal(initial, original)
        a = base.evaluate(initial, base.witnesses())
        b = margin.evaluate(initial, margin.witnesses())
        for left, right in zip(a, b):
            np.testing.assert_array_equal(left, right)
    dimensions = []

    def capture(name, plugin, problem, options):
        dimensions.append((int(problem['x'].numel()), int(problem['g'].numel())))
        return SimpleNamespace()

    monkeypatch.setattr(ca, 'nlpsol', capture)
    for planner in (base, margin):
        initial = planner.seeds()[0]
        planner._fast_time_origin = np.cumsum(np.exp(initial[:planner.n]))
        planner._fast_normalization = np.exp(initial[:planner.n]).sum()
        fast_ipopt.build_problem(planner, planner.witnesses(), lambda row: None, np.inf, False)
    assert dimensions == [(1680, 2033), (1680, 2033)]


def test_parameterized_polynomial_penalty_and_full_gradient():
    rng = np.random.default_rng(93)
    coefficients = rng.normal(size=(2, 8, 3))
    times = np.array([1.2, 1.5])
    expected = Penalty().evaluate(coefficients, times)
    actual = kernels.PolynomialPenalty('togt').evaluate(coefficients, times)
    for left, right in zip(actual, expected):
        np.testing.assert_allclose(left, right, rtol=1e-9, atol=.005)
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    planner = Planner(track, solver='ipopt', pose_function=scenario.pose, dynamic_limit_fraction=.9)
    initial = planner.seeds()[1]
    variable = ca.MX.sym('margin_trajectory', planner.nvar)
    expression = objective(planner, variable)
    evaluate = ca.Function('margin_objective', [variable], [expression, ca.gradient(expression, variable)])
    value, gradient, _ = planner.objective(initial)
    expected, derivative = evaluate(initial)
    np.testing.assert_allclose(value, expected, rtol=1e-10, atol=1e-6)
    np.testing.assert_allclose(gradient, np.asarray(derivative).ravel(), rtol=1e-8, atol=.01)
    direction = rng.normal(size=planner.nvar)
    step = 1e-7
    difference = (planner.objective(initial+step*direction)[0]-planner.objective(initial-step*direction)[0])/(2*step)
    np.testing.assert_allclose(difference, gradient@direction, rtol=2e-4, atol=.1)
