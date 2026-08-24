import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile,
    SCDynamicWindow,
    SCWindowTrack,
    rotation_and_derivative,
)
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    JointTOGTObjective,
    OptimizationConfig,
    _PastCostStoppingCriterion,
    optimize_track,
)
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCMappingError


def _window(dynamic=True):
    # A deliberately non-convex local safe polygon in the gate's y-z plane.
    polygon = np.array(
        [[-1.2, -1.0], [1.2, -1.0], [1.2, 1.0], [0.2, 1.0], [0.2, 0.1], [-1.2, 0.1]]
    )
    sc_map = SCDiskMap.fit(polygon, quadrature_order=32)
    motion = (
        MotionProfile(
            translation_amplitude=np.array([0.05, 0.12, 0.08]),
            rotation_amplitude=np.array([0.08, 0.06, 0.04]),
            scale_amplitude=0.08,
            translation_period=5.0,
            rotation_period=6.0,
            scale_period=7.0,
        )
        if dynamic
        else MotionProfile.static()
    )
    return SCDynamicWindow(
        "L",
        sc_map,
        polygon,
        center0=np.array([0.0, 0.0, 1.0]),
        angles0=np.zeros(3),
        motion=motion,
    )


def _track(dynamic=True):
    return SCWindowTrack(
        "one_gate",
        start=np.array([-1.5, 0.0, 1.0]),
        goal=np.array([1.5, 0.0, 1.0]),
        windows=(_window(dynamic),),
        order=(0,),
    )


def _config(**kwargs):
    values = dict(
        max_iterations=2,
        samples_per_segment=4,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=1.0e-4),
        penalty_weights=PenaltyWeights(velocity=0.0, collective_thrust=0.0, body_rate=0.0, rotor_thrust=0.0),
        dynamic_limits=DynamicLimits(),
    )
    values.update(kwargs)
    return OptimizationConfig(**values)


def test_standard_togt_lbfgs_defaults_and_past_ring_semantics():
    config = OptimizationConfig()
    assert config.memory_size == 256
    assert config.past_iterations == 32
    assert config.max_line_search_steps == 64
    assert config.max_iterations == 0
    assert config.function_tolerance == 1.0e-5
    assert config.gradient_tolerance == 0.0

    # With past=3, iteration four compares against iteration one (9.0),
    # exactly like pf[k % past] in the bundled C++ implementation.
    criterion = _PastCostStoppingCriterion(past=3, tolerance=0.01)
    criterion.record_initial(10.0)
    assert not criterion.record_iteration(9.0)
    assert not criterion.record_iteration(8.5)
    assert not criterion.record_iteration(8.0)
    assert criterion.record_iteration(8.95)
    np.testing.assert_allclose(criterion.relative_reduction, 0.05 / 8.95)


def test_rotation_derivative_matches_centered_difference():
    angles = np.array([0.2, -0.1, 0.3])
    rates = np.array([0.4, -0.2, 0.1])
    rotation, derivative = rotation_and_derivative(angles, rates)
    h = 1.0e-6
    plus, _ = rotation_and_derivative(angles + h * rates, rates)
    minus, _ = rotation_and_derivative(angles - h * rates, rates)
    np.testing.assert_allclose(derivative, (plus - minus) / (2.0 * h), rtol=2e-9, atol=2e-10)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)


def test_dynamic_scale_converts_fixed_world_clearance_to_conservative_local_inset():
    motion = MotionProfile(
        translation_amplitude=np.zeros(3),
        rotation_amplitude=np.zeros(3),
        scale_amplitude=0.4,
        scale_period=2.0,
    )
    assert motion.minimum_scale == pytest.approx(0.6)
    assert motion.maximum_scale == pytest.approx(1.4)
    local = motion.local_offset_for_world_clearance(0.315)
    assert local == pytest.approx(0.525)
    times = np.linspace(0.0, motion.scale_period, 1001)
    world_offsets = np.asarray([motion.scale(float(t))[0] * local for t in times])
    assert world_offsets.min() == pytest.approx(0.315, abs=1.0e-12)


def test_dynamic_window_space_and_time_jacobians():
    window = _window(dynamic=True)
    d = np.array([0.24, -0.31])
    t = 1.37
    point, _, jac_d, derivative_t = window.point_and_jacobians(d, t)
    h = 1.0e-6
    numeric_d = np.column_stack(
        [
            (window.to_point(d + h * np.eye(2)[j], t) - window.to_point(d - h * np.eye(2)[j], t)) / (2 * h)
            for j in range(2)
        ]
    )
    numeric_t = (window.to_point(d, t + h) - window.to_point(d, t - h)) / (2 * h)
    assert np.all(np.isfinite(point))
    np.testing.assert_allclose(jac_d, numeric_d, rtol=2e-5, atol=2e-7)
    np.testing.assert_allclose(derivative_t, numeric_t, rtol=2e-7, atol=2e-9)


def test_full_joint_gradient_matches_centered_difference():
    objective = JointTOGTObjective(_track(dynamic=True), _config())
    x = objective.initial_guess()
    x[-2:] = [0.13, -0.17]
    value, analytic = objective.value_and_gradient(x)
    h = 1.0e-6
    numeric = np.empty_like(x)
    for index in range(len(x)):
        direction = np.zeros_like(x)
        direction[index] = h
        numeric[index] = (
            objective.evaluate(x + direction).cost - objective.evaluate(x - direction).cost
        ) / (2.0 * h)
    assert np.isfinite(value)
    np.testing.assert_allclose(analytic, numeric, rtol=2e-4, atol=2e-5)


def test_time_gradient_ablation_removes_only_window_motion_chain():
    full = JointTOGTObjective(_track(dynamic=True), _config(include_window_time_gradient=True))
    ablated = JointTOGTObjective(_track(dynamic=True), _config(include_window_time_gradient=False))
    x = full.initial_guess()
    full_eval = full.evaluate(x)
    ablated_eval = ablated.evaluate(x)
    np.testing.assert_allclose(full_eval.gradient[-2:], ablated_eval.gradient[-2:])
    assert np.linalg.norm(full_eval.gradient[:2] - ablated_eval.gradient[:2]) > 1.0e-9
    np.testing.assert_allclose(ablated_eval.traversal_time_gradient, 0.0)


def test_optimizer_returns_valid_ordered_crossing():
    track = _track(dynamic=False)
    result = optimize_track(track, config=_config(max_iterations=3))
    assert result.d.shape == (1, 2)
    assert result.durations.shape == (2,)
    assert np.all(result.durations > 0.0)
    assert track.windows[0].contains(result.waypoints[0], result.traversal_times[0])
    np.testing.assert_allclose(
        result.trajectory.evaluate(result.traversal_times[0]), result.waypoints[0], atol=2e-8
    )
    assert result.to_dict()["full_time_gradient"] is True


def test_scipy_line_search_wrapper_turns_numeric_domain_error_into_finite_trial(monkeypatch):
    objective = JointTOGTObjective(_track(dynamic=True), _config())
    x = objective.initial_guess()

    def fail(_values):
        raise SCMappingError("synthetic near-boundary quadrature failure")

    monkeypatch.setattr(objective, "value_and_gradient", fail)
    cost, gradient = objective.scipy_value_and_gradient(x)
    assert np.isfinite(cost)
    assert np.all(np.isfinite(gradient))
    assert objective.invalid_trial_count == 1
