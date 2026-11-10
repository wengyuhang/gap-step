import numpy as np
from ..augmented import Planner


def test_restoration_refines_to_original_hard_constraint_optimum():
    p = Planner.__new__(Planner)
    p.max_iterations = 2000
    p.optimizer_calls = []
    messages = []
    progress = messages.append
    def evaluate(x):
        _ = progress
        return np.array([1.-x[0]])
    def values(x):
        return evaluate(x)
    def objective(x):
        value = (x[0]-2.)**2
        return np.log1p(value), np.array([2*(x[0]-2.)/(1+value)])
    result = p.augmented_minimize(objective, np.array([0.]), jac=True, method='SLSQP',
        bounds=[(-5., 5.)], constraints=dict(type='ineq', fun=values, jac=lambda x: -np.ones((1, 1))),
        callback=lambda x: None, options=dict(maxiter=400, ftol=1e-7, disp=False))
    assert result.success and values(result.x).min() >= -1e-5
    np.testing.assert_allclose(result.x, [1.], atol=1e-6)
    assert p.optimizer_calls[-1]['stage'] == 'sqp_refinement'
