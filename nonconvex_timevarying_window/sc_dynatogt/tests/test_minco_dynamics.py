"""Numerical tests for TOGT MINCO and quadrotor flatness."""

from __future__ import annotations

import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicCheckSampling,
    DynamicLimits,
    FlatnessState,
    ObjectiveWeights,
    PenaltyWeights,
    QuadrotorParameters,
    cubic_positive_part,
    constant_yaw_profile,
    dynamic_check_interval_count,
    flatness_map,
    instantaneous_constraint_penalty,
    integrated_dynamic_penalty,
    objective_with_gradient,
    smoothed_l1,
    trajectory_objective,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import (
    BoundaryState,
    MincoSnap,
)


def _trajectory_data():
    start = BoundaryState(
        [0.0, 0.0, 0.0],
        velocity=[0.1, -0.05, 0.0],
        acceleration=[0.0, 0.0, 0.0],
        jerk=[0.0, 0.0, 0.0],
    )
    finish = BoundaryState([2.0, 0.2, 0.1])
    points = np.array([[0.8, 0.4, 0.15], [1.4, -0.2, 0.2]])
    durations = np.array([1.1, 0.9, 1.2])
    return start, finish, points, durations


def _central_difference_points(function, points, step=1.0e-6):
    gradient = np.zeros_like(points)
    for index in np.ndindex(points.shape):
        plus = points.copy()
        minus = points.copy()
        plus[index] += step
        minus[index] -= step
        gradient[index] = (function(plus) - function(minus)) / (2.0 * step)
    return gradient


def _central_difference_times(function, durations, step=1.0e-6):
    gradient = np.zeros_like(durations)
    for index in range(durations.size):
        plus = durations.copy()
        minus = durations.copy()
        plus[index] += step
        minus[index] -= step
        gradient[index] = (function(plus) - function(minus)) / (2.0 * step)
    return gradient


def test_minco_is_degree_seven_and_satisfies_pvaj_and_waypoints():
    start, finish, points, durations = _trajectory_data()
    trajectory = MincoSnap(start, finish, points, durations)

    assert trajectory.coefficients.shape == (3, 8, 3)
    # This is not a quintic/Hermite stand-in: the seventh-order terms are used.
    assert np.linalg.norm(trajectory.coefficients[:, 6:, :]) > 1.0e-6
    np.testing.assert_allclose(trajectory.evaluate(0.0), start.position, atol=1.0e-11)
    np.testing.assert_allclose(
        trajectory.evaluate(trajectory.total_time), finish.position, atol=1.0e-10
    )
    for derivative, expected_start, expected_finish in (
        (1, start.velocity, finish.velocity),
        (2, start.acceleration, finish.acceleration),
        (3, start.jerk, finish.jerk),
    ):
        np.testing.assert_allclose(
            trajectory.evaluate(0.0, derivative), expected_start, atol=2.0e-10
        )
        np.testing.assert_allclose(
            trajectory.evaluate(trajectory.total_time, derivative),
            expected_finish,
            atol=2.0e-9,
        )

    np.testing.assert_allclose(
        trajectory.evaluate(np.cumsum(durations)[:-1]), points, atol=2.0e-10
    )
    # The original MincoSnap system is continuous beyond jerk; verify all
    # junction equations, including snap/crackle/sixth derivative.
    for junction in range(trajectory.num_segments - 1):
        for derivative in range(7):
            np.testing.assert_allclose(
                trajectory.evaluate_segment(junction, durations[junction], derivative),
                trajectory.evaluate_segment(junction + 1, 0.0, derivative),
                atol=2.0e-8,
                rtol=2.0e-9,
            )


def test_continuous_sampling_exposes_jerk_snap_and_exact_energy():
    # One degree-seven rest-to-rest segment has a closed-form scaling law:
    # E(lambda*T) = E(T) / lambda**7.
    start = BoundaryState([0.0, 0.0, 0.0])
    finish = BoundaryState([1.0, 2.0, -0.5])
    trajectory = MincoSnap(start, finish, np.empty((0, 3)), np.array([2.0]))
    stretched = MincoSnap(start, finish, np.empty((0, 3)), np.array([4.0]))
    assert trajectory.snap_energy() > 0.0
    assert stretched.snap_energy() == pytest.approx(
        trajectory.snap_energy() / 2.0**7, rel=2.0e-11
    )

    samples = trajectory.sample(samples_per_segment=11)
    assert samples.time.shape == (11,)
    assert samples.jerk.shape == (11, 3)
    assert samples.snap.shape == (11, 3)
    assert samples.crackle.shape == (11, 3)
    assert np.all(np.isfinite(samples.snap))


def test_minco_energy_gradients_match_centered_finite_differences():
    start, finish, points, durations = _trajectory_data()
    trajectory = MincoSnap(start, finish, points, durations)
    value, point_gradient, time_gradient = trajectory.energy_with_grad()
    assert value == pytest.approx(trajectory.snap_energy())

    finite_points = _central_difference_points(
        lambda candidate: MincoSnap(
            start, finish, candidate, durations
        ).snap_energy(),
        points,
    )
    finite_times = _central_difference_times(
        lambda candidate: MincoSnap(
            start, finish, points, candidate
        ).snap_energy(),
        durations,
    )
    np.testing.assert_allclose(point_gradient, finite_points, rtol=2.0e-6, atol=1.0e-4)
    np.testing.assert_allclose(time_gradient, finite_times, rtol=2.0e-7, atol=1.0e-4)


def test_flatness_hover_and_yaw_rate_recover_expected_inputs():
    parameters = QuadrotorParameters()
    hover = flatness_map(
        np.zeros(3), np.zeros(3), np.zeros(3), parameters=parameters
    )
    np.testing.assert_allclose(hover.rotation, np.eye(3), atol=1.0e-13)
    np.testing.assert_allclose(hover.body_rate, np.zeros(3), atol=1.0e-13)
    np.testing.assert_allclose(
        hover.collective_thrust, parameters.mass * parameters.gravity, atol=1.0e-13
    )
    np.testing.assert_allclose(
        hover.rotor_thrusts,
        np.full(4, parameters.mass * parameters.gravity / 4.0),
        atol=1.0e-12,
    )

    spinning_hover = flatness_map(
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        yaw=0.4,
        yaw_rate=1.7,
        yaw_acceleration=-0.3,
        parameters=parameters,
    )
    np.testing.assert_allclose(
        spinning_hover.body_rate, [0.0, 0.0, 1.7], atol=1.0e-12
    )
    np.testing.assert_allclose(
        spinning_hover.body_rate_derivative, [0.0, 0.0, -0.3], atol=1.0e-12
    )
    assert spinning_hover.rotor_thrusts.shape == (4,)
    np.testing.assert_allclose(
        parameters.mixing_matrix @ spinning_hover.rotor_thrusts,
        np.r_[spinning_hover.collective_thrust, spinning_hover.torque],
        atol=1.0e-12,
    )


def test_cubic_positive_part_remains_available_as_the_paper_formula():
    residual = np.array([-2.0, 0.0, 0.5, 2.0])
    np.testing.assert_allclose(cubic_positive_part(residual), [0.0, 0.0, 0.125, 8.0])
    # Complex-step continuation must preserve the active residual derivative.
    value = cubic_positive_part(2.0 + 1.0e-30j)
    assert np.imag(value) / 1.0e-30 == pytest.approx(12.0)


def test_smoothed_l1_matches_the_bundled_cpp_formula_value_by_value():
    mu = 1.0e-2
    residual = np.array([-0.003, 0.0, 0.002, 0.005, 0.01, 0.02])
    expected = np.array(
        [
            0.0,
            0.0,
            (mu - 0.5 * 0.002) * (0.002 / mu) ** 3,
            (mu - 0.5 * 0.005) * (0.005 / mu) ** 3,
            0.01 - 0.5 * mu,
            0.02 - 0.5 * mu,
        ]
    )
    np.testing.assert_allclose(smoothed_l1(residual), expected, atol=1.0e-16)

    # This is the derivative expression in TrajSolver::smoothedL1.
    x = 0.004
    xdmu = x / mu
    expected_derivative = xdmu**2 * (
        -0.5 * xdmu + 3.0 * (mu - 0.5 * x) / mu
    )
    value = smoothed_l1(x + 1.0e-30j)
    assert np.imag(value) / 1.0e-30 == pytest.approx(expected_derivative)
    assert np.imag(smoothed_l1(0.02 + 1.0e-30j)) / 1.0e-30 == pytest.approx(1.0)
    assert np.imag(smoothed_l1(-0.02 + 1.0e-30j)) == 0.0


def test_thrust_penalties_use_cpp_center_and_radius_squared_residuals():
    def state(collective, rotors):
        return FlatnessState(
            rotation=np.eye(3),
            body_rate=np.zeros(3),
            body_rate_derivative=np.zeros(3),
            collective_thrust=collective,
            torque=np.zeros(3),
            rotor_thrusts=np.asarray(rotors, dtype=float),
            body_z=np.array([0.0, 0.0, 1.0]),
        )

    limits = DynamicLimits(
        min_collective_thrust=8.0,
        max_collective_thrust=12.0,
        min_rotor_thrust=2.0,
        max_rotor_thrust=6.0,
    )
    weights = PenaltyWeights(
        velocity=0.0,
        collective_thrust=1.0,
        body_rate=0.0,
        rotor_thrust=1.0,
    )
    below = instantaneous_constraint_penalty(
        np.zeros(3), state(7.0, np.full(4, 1.0)), limits=limits, weights=weights
    )
    above = instantaneous_constraint_penalty(
        np.zeros(3), state(13.0, np.full(4, 7.0)), limits=limits, weights=weights
    )
    # Both symmetric violations have (f-mean)^2-radius^2 = 5.
    expected = smoothed_l1(5.0)
    assert below.collective_thrust == pytest.approx(expected)
    assert above.collective_thrust == pytest.approx(expected)
    assert below.rotor_thrust == pytest.approx(4.0 * expected)
    assert above.rotor_thrust == pytest.approx(4.0 * expected)

    # Collective thrust is an optional check and is disabled by an open bound.
    optional_limits = DynamicLimits(
        min_collective_thrust=0.0,
        max_collective_thrust=np.inf,
        min_rotor_thrust=2.0,
        max_rotor_thrust=6.0,
    )
    optional = instantaneous_constraint_penalty(
        np.zeros(3), state(100.0, np.full(4, 4.0)),
        limits=optional_limits, weights=weights
    )
    assert optional.collective_thrust == 0.0
    assert optional.rotor_thrust == 0.0


def test_complete_objective_gradients_match_centered_finite_differences():
    start, finish, points, durations = _trajectory_data()
    trajectory = MincoSnap(start, finish, points, durations)
    kwargs = dict(
        parameters=QuadrotorParameters(),
        limits=DynamicLimits(
            max_velocity=1.0,
            min_collective_thrust=9.5,
            max_collective_thrust=10.0,
            max_body_rate_xy=0.8,
            max_body_rate_z=0.1,
            min_rotor_thrust=2.3,
            max_rotor_thrust=2.6,
        ),
        penalty_weights=PenaltyWeights(
            velocity=0.03,
            collective_thrust=0.02,
            body_rate=0.04,
            rotor_thrust=0.05,
        ),
        objective_weights=ObjectiveWeights(time=0.7, snap_energy=1.0e-5),
        samples_per_segment=7,
    )
    # Exercise every requested constraint family in the gradient check.
    breakdown = integrated_dynamic_penalty(
        trajectory,
        parameters=kwargs["parameters"],
        limits=kwargs["limits"],
        weights=kwargs["penalty_weights"],
        samples_per_segment=kwargs["samples_per_segment"],
        return_breakdown=True,
    )
    assert breakdown.velocity > 0.0
    assert breakdown.collective_thrust > 0.0
    assert breakdown.body_rate > 0.0
    assert breakdown.rotor_thrust > 0.0
    value, point_gradient, time_gradient = objective_with_gradient(
        trajectory, **kwargs
    )

    def objective(candidate_points, candidate_times):
        candidate = MincoSnap(
            start, finish, candidate_points, candidate_times
        )
        return float(np.real(trajectory_objective(candidate, **kwargs)))

    finite_points = _central_difference_points(
        lambda candidate: objective(candidate, durations), points
    )
    finite_times = _central_difference_times(
        lambda candidate: objective(points, candidate), durations
    )
    assert value == pytest.approx(objective(points, durations), rel=1.0e-12)
    np.testing.assert_allclose(point_gradient, finite_points, rtol=3.0e-5, atol=2.0e-7)
    np.testing.assert_allclose(time_gradient, finite_times, rtol=3.0e-5, atol=2.0e-7)


def test_reverse_mode_and_complex_step_backends_agree_for_all_constraints():
    start, finish, points, durations = _trajectory_data()
    trajectory = MincoSnap(start, finish, points, durations)
    kwargs = dict(
        parameters=QuadrotorParameters(),
        limits=DynamicLimits(
            max_velocity=1.0,
            min_collective_thrust=9.5,
            max_collective_thrust=10.0,
            max_body_rate_xy=0.8,
            max_body_rate_z=0.1,
            min_rotor_thrust=2.3,
            max_rotor_thrust=2.6,
        ),
        penalty_weights=PenaltyWeights(0.03, 0.02, 0.04, 0.05),
        objective_weights=ObjectiveWeights(time=0.7, snap_energy=1.0e-5),
        samples_per_segment=7,
        yaw_profile=constant_yaw_profile(0.37),
    )
    reverse = objective_with_gradient(
        trajectory, gradient_backend="autodiff", **kwargs
    )
    oracle = objective_with_gradient(
        trajectory, gradient_backend="complex_step", **kwargs
    )
    assert reverse[0] == pytest.approx(oracle[0], rel=2.0e-14)
    np.testing.assert_allclose(reverse[1], oracle[1], rtol=2.0e-10, atol=2.0e-11)
    np.testing.assert_allclose(reverse[2], oracle[2], rtol=2.0e-10, atol=2.0e-11)


def test_reverse_mode_uses_one_differentiable_solve_independent_of_dimension(
    monkeypatch,
):
    torch = pytest.importorskip("torch")
    start = BoundaryState([0.0, 0.0, 0.0])
    finish = BoundaryState([2.0, 0.0, 0.0])
    point_count = 9
    points = np.column_stack(
        (
            np.linspace(0.2, 1.8, point_count),
            0.1 * np.sin(np.arange(point_count)),
            0.05 * np.cos(np.arange(point_count)),
        )
    )
    trajectory = MincoSnap(
        start, finish, points, np.full(point_count + 1, 0.55)
    )

    solve_calls = 0
    original_solve = torch.linalg.solve

    def counted_solve(*args, **kwargs):
        nonlocal solve_calls
        solve_calls += 1
        return original_solve(*args, **kwargs)

    monkeypatch.setattr(torch.linalg, "solve", counted_solve)
    value, point_gradient, time_gradient = objective_with_gradient(
        trajectory,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=1.0e-7),
        penalty_weights=PenaltyWeights(0.0, 0.0, 0.0, 0.0),
        samples_per_segment=3,
    )
    assert solve_calls == 1
    assert np.isfinite(value)
    assert point_gradient.shape == points.shape
    assert time_gradient.shape == (point_count + 1,)


def test_one_segment_reverse_mode_returns_empty_point_gradient():
    trajectory = MincoSnap(
        BoundaryState([0.0, 0.0, 0.0]),
        BoundaryState([1.0, 0.2, -0.1]),
        np.empty((0, 3)),
        np.array([1.4]),
    )
    value, point_gradient, time_gradient = objective_with_gradient(
        trajectory,
        objective_weights=ObjectiveWeights(time=0.4, snap_energy=1.0e-4),
        penalty_weights=PenaltyWeights(0.0, 0.0, 0.0, 0.0),
        samples_per_segment=3,
    )
    assert np.isfinite(value)
    assert point_gradient.shape == (0, 3)
    assert time_gradient.shape == (1,)
    finite_time = _central_difference_times(
        lambda duration: trajectory_objective(
            MincoSnap(
                trajectory.start_state,
                trajectory.end_state,
                np.empty((0, 3)),
                duration,
            ),
            objective_weights=ObjectiveWeights(time=0.4, snap_energy=1.0e-4),
            penalty_weights=PenaltyWeights(0.0, 0.0, 0.0, 0.0),
            samples_per_segment=3,
        ),
        trajectory.durations,
    )
    np.testing.assert_allclose(time_gradient, finite_time, rtol=2.0e-7, atol=2.0e-8)


def test_dynamic_const_check_counts_and_default_gradients_away_from_thresholds(
    monkeypatch,
):
    durations = np.array([0.39, 0.61, 1.71])
    points = np.array([[0.35, 0.08, 0.03], [0.95, -0.08, 0.08]])
    trajectory = MincoSnap(
        BoundaryState([0.0, 0.0, 0.0]),
        BoundaryState([1.5, 0.0, 0.1]),
        points,
        durations,
    )
    # These exercise the lower clamp, unclamped interior and upper clamp.
    interval_counts = [dynamic_check_interval_count(value) for value in durations]
    assert interval_counts == [8, 12, 32]
    assert dynamic_check_interval_count(
        0.61,
        DynamicCheckSampling(
            check_time_sec=0.1, min_num_check=3, max_num_check=5
        ),
    ) == 5

    # The original loop is inclusive: K intervals produce K + 1 nodes.
    import nonconvex_timevarying_window.sc_dynatogt.dynamics as dynamics_module

    original_flatness = dynamics_module.flatness_map
    flatness_calls = 0

    def counted_flatness(*args, **kwargs):
        nonlocal flatness_calls
        flatness_calls += 1
        return original_flatness(*args, **kwargs)

    monkeypatch.setattr(dynamics_module, "flatness_map", counted_flatness)
    integrated_dynamic_penalty(
        trajectory, weights=PenaltyWeights(0.0, 0.0, 0.0, 0.0)
    )
    assert flatness_calls == sum(count + 1 for count in interval_counts)
    flatness_calls = 0
    integrated_dynamic_penalty(
        trajectory,
        weights=PenaltyWeights(0.0, 0.0, 0.0, 0.0),
        samples_per_segment=5,
    )
    assert flatness_calls == 5 * trajectory.num_segments

    kwargs = dict(
        limits=DynamicLimits(max_velocity=1.0),
        penalty_weights=PenaltyWeights(1.0e-5, 0.0, 0.0, 0.0),
        objective_weights=ObjectiveWeights(time=0.3, snap_energy=1.0e-8),
    )
    reverse = objective_with_gradient(trajectory, **kwargs)
    oracle = objective_with_gradient(
        trajectory, gradient_backend="complex_step", **kwargs
    )
    assert reverse[0] == pytest.approx(oracle[0], rel=2.0e-14)
    np.testing.assert_allclose(reverse[1], oracle[1], rtol=2.0e-10, atol=2.0e-11)
    np.testing.assert_allclose(reverse[2], oracle[2], rtol=2.0e-10, atol=2.0e-11)

    def objective(candidate_points, candidate_times):
        return float(
            trajectory_objective(
                MincoSnap(
                    trajectory.start_state,
                    trajectory.end_state,
                    candidate_points,
                    candidate_times,
                ),
                **kwargs,
            )
        )

    finite_points = _central_difference_points(
        lambda candidate: objective(candidate, durations), points
    )
    finite_times = _central_difference_times(
        lambda candidate: objective(points, candidate), durations
    )
    np.testing.assert_allclose(reverse[1], finite_points, rtol=2.0e-7, atol=2.0e-9)
    np.testing.assert_allclose(reverse[2], finite_times, rtol=2.0e-7, atol=2.0e-9)


def test_rejects_nonpositive_durations_and_flatness_singularities():
    start = BoundaryState([0.0, 0.0, 0.0])
    finish = BoundaryState([1.0, 0.0, 0.0])
    with pytest.raises(ValueError, match="strictly positive"):
        MincoSnap(start, finish, np.empty((0, 3)), [0.0])
    with pytest.raises(ValueError, match="specific force"):
        flatness_map(
            [0.0, 0.0, -9.8066],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )


def test_pva_endpoint_arrays_default_jerk_to_zero():
    start_pva = np.array(
        [[0.0, 0.0, 0.0], [0.2, 0.0, 0.0], [0.0, 0.1, 0.0]]
    )
    finish_pva = np.array(
        [[1.0, 0.0, 0.0], [0.0, -0.1, 0.0], [0.0, 0.0, 0.2]]
    )
    trajectory = MincoSnap(
        start_pva, finish_pva, np.empty((0, 3)), np.array([1.5])
    )
    np.testing.assert_allclose(trajectory.evaluate(0.0, 2), start_pva[2])
    np.testing.assert_allclose(
        trajectory.evaluate(0.0, 3), np.zeros(3), atol=1.0e-12
    )
    np.testing.assert_allclose(
        trajectory.evaluate(trajectory.total_time, 2), finish_pva[2], atol=1.0e-10
    )
    np.testing.assert_allclose(
        trajectory.evaluate(trajectory.total_time, 3), np.zeros(3), atol=1.0e-10
    )


def test_three_by_three_waypoint_array_prefers_python_row_layout():
    points = np.array(
        [[0.4, 0.1, 0.2], [0.9, -0.3, 0.25], [1.4, 0.2, -0.1]]
    )
    durations = np.array([0.8, 1.0, 0.9, 1.1])
    trajectory = MincoSnap(
        BoundaryState([0.0, 0.0, 0.0]),
        BoundaryState([2.0, 0.0, 0.0]),
        points,
        durations,
    )
    np.testing.assert_allclose(
        trajectory.evaluate(np.cumsum(durations)[:-1]), points, atol=2.0e-9
    )
