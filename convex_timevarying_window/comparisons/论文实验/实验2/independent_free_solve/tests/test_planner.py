import numpy as np
from ..planner import Planner, BasePlanner


def test_direct_sqp_solves_original_problem_without_restoration(monkeypatch):
    def unexpected(*args, **kwargs):
        raise AssertionError('Directly converged SQP must return its own solution')
    monkeypatch.setattr(BasePlanner, 'augmented_minimize', unexpected)
    p = Planner.__new__(Planner)
    p.max_iterations = 2000
    p.optimizer_calls = []

    def objective(x):
        value = (x[0]-2.)**2
        return np.log1p(value), np.array([2*(x[0]-2.)/(1+value)])

    result = p.augmented_minimize(objective, np.array([0.]), jac=True, method='SLSQP',
        bounds=[(-5., 5.)], constraints=dict(type='ineq', fun=lambda x: 1.-x,
            jac=lambda x: -np.ones((1, 1))), callback=lambda x: None,
        options=dict(maxiter=400, ftol=1e-7, disp=False))
    assert result.success
    np.testing.assert_allclose(result.x, [1.], atol=1e-6)
    assert p.optimizer_calls[0]['stage'] == 'direct_free_sqp'


def test_free_seeds_match_original_analytic_initialization():
    from ...basic_group.whole_course_difficulty import scenario
    track, _ = scenario.build_case(scenario.Case(level='a50_f6'))
    p = Planner(track, True, pose_function=scenario.pose)
    original = BasePlanner(track, True, pose_function=scenario.pose)
    assert p.nvar == 96
    for expected, actual in zip(original.seeds(), p.seeds()):
        np.testing.assert_array_equal(expected, actual)
