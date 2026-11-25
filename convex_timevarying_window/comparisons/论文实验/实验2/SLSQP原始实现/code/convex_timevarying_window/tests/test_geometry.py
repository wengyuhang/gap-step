import numpy as np

from convex_timevarying_window.experiment import BODY_RADIUS, WINDOW_MARGIN_FACTOR, build_track
from convex_timevarying_window.native_objective import NativeJointTOGTObjective


def test_all_togt_mappings_stay_in_convex_apertures():
    track, _ = build_track()
    for window in track.windows:
        base = window.aperture.initial_d()
        for d in (base, base + np.linspace(0.2, 0.7, len(base))):
            _, local, _, _ = window.point_and_jacobians(d, 3.1)
            assert window.aperture.contains(local, safe=True)


def test_periodic_window_time_derivative_matches_finite_difference():
    track, _ = build_track()
    window = track.windows[2]
    d = window.aperture.initial_d() + np.linspace(0.1, 0.5, window.aperture.dimension)
    time = 2.7
    point, _, _, derivative = window.point_and_jacobians(d, time)
    step = 1e-6
    finite = (window.to_point(d, time + step) - window.to_point(d, time - step)) / (2 * step)
    np.testing.assert_allclose(point, window.to_point(d, time), atol=1e-14)
    np.testing.assert_allclose(derivative, finite, rtol=2e-7, atol=2e-8)


def test_nominal_motion_is_fully_three_dimensional():
    track, _ = build_track()
    for window in track.windows:
        assert np.all(np.abs(window.motion.translation_amplitude) > 0.0)
        assert np.all(np.abs(window.motion.rotation_amplitude) > 0.0)


def test_paper_shape_dimensions_and_scaled_body_diameter_margins():
    track, _ = build_track()
    assert [w.aperture.dimension for w in track.windows] == [4, 2, 5, 2, 6, 2, 4]
    assert all(w.aperture.margin == WINDOW_MARGIN_FACTOR * 2.0 * BODY_RADIUS for w in track.windows)


def test_togt_standard_objective_configuration_is_frozen():
    _, config = build_track()
    assert config.objective_weights.time == 1.0
    assert config.objective_weights.snap_energy == 0.0
    assert config.penalty_weights.velocity == 0.0
    assert config.penalty_weights.collective_thrust == 0.0
    assert config.penalty_weights.body_rate == 1.0
    assert config.penalty_weights.rotor_thrust == 1.0
    assert config.samples_per_segment is None
    assert config.memory_size == 256
    assert config.past_iterations == 32
    assert config.max_line_search_steps == 64
    assert config.max_iterations == 0


def test_native_full_gradient_matches_centered_directional_difference():
    track, config = build_track()
    objective = NativeJointTOGTObjective(track, config)
    x = objective.initial_guess()
    cost, gradient = objective.value_and_gradient(x)
    direction = np.random.default_rng(4).normal(size=x.size)
    direction /= np.linalg.norm(direction)
    step = 1e-4
    finite = (
        objective.value_and_gradient(x + step * direction)[0]
        - objective.value_and_gradient(x - step * direction)[0]
    ) / (2.0 * step)
    assert np.isfinite(cost)
    np.testing.assert_allclose(gradient @ direction, finite, rtol=2e-7, atol=2e-7)
