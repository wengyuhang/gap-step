import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.native import Penalty
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics.planner import Planner
from convex_timevarying_window.comparisons.论文实验.实验2.independent_free_solve.planner import Planner as HardPlanner
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.single_gate_stop_racing import point_mapping


def test_native_polynomial_penalty_matches_frozen_togt_cost():
    head, tail = BoundaryState(np.zeros(3)), BoundaryState(np.array([9., 1., 2.]))
    times = np.array([1.813, 1.937])
    points = np.array([[4., -2., 1.]])
    curve = MincoSnap(head, tail, points, times)
    coefficients = curve.coefficients*times[:, None, None]**np.arange(8)[None, :, None]
    original, _, _ = AnalyticTOGTCore().value_and_gradient(head.matrix.T, tail.matrix.T, points, times)
    penalty, _, _ = Penalty().evaluate(coefficients, times)
    np.testing.assert_allclose(penalty+times.sum(), original, rtol=2e-10, atol=1e-7)


def test_polynomial_partial_gradients_include_normalized_time_chain():
    rng = np.random.default_rng(52)
    coefficients = rng.normal(size=(2, 8, 3))
    coefficients[:, 2:] *= 3.
    times = np.array([1.813, 2.123])
    backend = Penalty()
    _, gradient, time_gradient = backend.evaluate(coefficients, times)
    for _ in range(3):
        dc, dt = rng.normal(size=coefficients.shape), rng.normal(size=times.shape)
        epsilon = 2e-6
        plus = backend.evaluate(coefficients+epsilon*dc, times+epsilon*dt)[0]
        minus = backend.evaluate(coefficients-epsilon*dc, times-epsilon*dt)[0]
        exact = float(np.sum(dc*gradient)+dt@time_gradient)
        np.testing.assert_allclose((plus-minus)/(2*epsilon), exact, rtol=2e-5, atol=.02)


def test_only_dynamic_hard_rows_are_replaced_and_initializers_stay_independent():
    track, _ = scenario.build_case(scenario.Case(level='static'))
    old = HardPlanner(track, pose_function=scenario.pose)
    new = Planner(track, pose_function=scenario.pose)
    for a, b in zip(old.seeds(), new.seeds()):
        np.testing.assert_allclose(a[:old.n], b[:new.n], atol=1e-13, rtol=0.)
    seed = old.seeds()[0]
    hard, _, curve = old.evaluate(seed, old.witnesses())
    soft, _, same_curve = new.evaluate(seed, new.witnesses())
    np.testing.assert_array_equal(curve, same_curve)
    count = old.n*(old.cfg.nodes_per_piece+1)
    np.testing.assert_array_equal(hard[:count*17].reshape(count, 17)[:, 11:].ravel(), soft[:count*6])
    np.testing.assert_array_equal(hard[count*17:], soft[count*6:])
    assert old.cfg.radius == new.cfg.radius == .38594472


def test_initial_pvaj_reconstructs_current_geometry_minco_without_optimization(monkeypatch):
    track, _ = scenario.build_case(scenario.Case(level='static'))
    planner = Planner(track, pose_function=scenario.pose)
    def forbidden(*args, **kwargs):
        raise AssertionError('Analytic initialization must not run an optimizer')
    monkeypatch.setattr(planner, 'solve_all', forbidden)
    for seed in planner.seeds():
        times = np.exp(seed[:planner.n])
        points = np.asarray(planner.point_function(seed)).T
        curve = MincoSnap(BoundaryState(track.start), BoundaryState(track.goal), points, times)
        _, _, coefficients = planner.evaluate(seed, planner.witnesses())
        expected = curve.coefficients*times[:, None, None]**np.arange(8)[None, :, None]
        np.testing.assert_allclose(coefficients, expected, atol=1e-7, rtol=1e-10)
    assert planner.objective(planner.seeds()[0])[2] == 0.


def test_dynamic_initial_points_use_only_current_geometry_and_remain_in_the_opening():
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    planner = Planner(track, pose_function=scenario.pose)
    for seed in planner.seeds():
        times = np.cumsum(np.exp(seed[:planner.n]))[:-1]
        points = np.asarray(planner.point_function(seed)).T
        for window, selection, point, at in zip(track.windows, planner.slices, points, times):
            local, plane = window.world_to_local(point, at)
            assert abs(plane) < 1e-8 and window.aperture.contains(local)
            np.testing.assert_allclose(point_mapping.numeric(window.aperture,
                point_mapping.inverse(window.aperture, local)), local, atol=1e-10)
            np.testing.assert_allclose(point_mapping.numeric(window.aperture, seed[selection]), local, atol=1e-10)


def test_full_pvaj_objective_gradient():
    track, _ = scenario.build_case(scenario.Case(level='mixed_base'))
    planner = Planner(track, pose_function=scenario.pose)
    x = planner.seeds()[0]
    value, gradient, penalty = planner.objective(x)
    assert penalty >= 0. and value == np.exp(x[:8]).sum()+penalty
    rng = np.random.default_rng(41)
    for _ in range(3):
        direction = rng.normal(size=len(x))
        direction[:8] *= .001
        direction[planner.velocity_start:] *= .1
        epsilon = 2e-6
        plus = planner.objective(x+epsilon*direction)[0]
        minus = planner.objective(x-epsilon*direction)[0]
        np.testing.assert_allclose((plus-minus)/(2*epsilon), gradient@direction, rtol=3e-4, atol=.03)


def test_quadrature_has_no_node_count_jump_at_duration_threshold():
    coefficients = np.zeros((1, 8, 3))
    coefficients[0, 2, 0] = 10.
    backend = Penalty()
    differences = []
    for epsilon in (1e-7, 1e-8, 1e-9):
        plus = backend.evaluate(coefficients, np.array([.9+epsilon]))[0]
        minus = backend.evaluate(coefficients, np.array([.9-epsilon]))[0]
        differences.append(abs(plus-minus))
    np.testing.assert_allclose(np.asarray(differences[1:])/differences[:-1], [.1, .1], rtol=.01)
