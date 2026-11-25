import casadi as ca
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.symbolic_penalty import instantaneous, objective
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario


def test_symbolic_instantaneous_matches_native_physics_and_cost():
    rng = np.random.default_rng(61)
    states = rng.normal(size=(40, 5, 3))
    states[:, 2, :] *= 12.
    states[:, 3:, :] *= 10.
    states[:20, 2, 2] = np.clip(states[:20, 2, 2], -3., 3.)
    states[-1, 2, :] = [0., 0., -20.]
    native = AnalyticTOGTCore().sample_dynamics(states)
    symbolic = instantaneous()
    for index, state in enumerate(states):
        cost, omega, thrust, _ = symbolic(*state[2:])
        np.testing.assert_allclose(float(cost), native['instantaneous_penalty'][index], rtol=1e-8, atol=1e-5)
        if native['regular_branch'][index]:
            np.testing.assert_allclose(np.asarray(omega).ravel(), native['body_rate'][index], rtol=1e-9, atol=1e-8)
            np.testing.assert_allclose(np.asarray(thrust).ravel(), native['rotor_thrusts'][index], rtol=1e-9, atol=1e-8)


def test_symbolic_full_objective_and_gradient_match_native():
    track, _ = scenario.build_case(scenario.Case(level='mixed_base'))
    planner = Planner(track, pose_function=scenario.pose)
    x = ca.MX.sym('symbolic_penalty_probe', planner.nvar)
    value = objective(planner, x)
    evaluate = ca.Function('symbolic_penalty_probe', [x], [value, ca.gradient(value, x)])
    for seed in planner.seeds()[:2]:
        native, gradient, _ = planner.objective(seed)
        actual, derivative = evaluate(seed)
        np.testing.assert_allclose(float(actual), native, rtol=1e-8, atol=1e-5)
        np.testing.assert_allclose(np.asarray(derivative).ravel(), gradient, rtol=1e-6, atol=.01)
