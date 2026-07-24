from __future__ import annotations

from dataclasses import replace

import numpy as np

from closed_loop_deformable_window.fapp_ppo.config import ExperimentConfig
from closed_loop_deformable_window.fapp_ppo.dynamics import QuadrotorDynamics
from closed_loop_deformable_window.fapp_ppo.geometry import DeformableWindow, radial_boundary
from closed_loop_deformable_window.fapp_ppo.scenario import build_scenario


def _static_window() -> DeformableWindow:
    times = np.array([0.0, 1.0, 2.0])
    boundary = radial_boundary(
        vertices=64,
        radius_x=1.0,
        radius_y=0.8,
        coefficients=np.array([0.08, 0.12, 0.06, 0.02, 0.05]),
    )
    return DeformableWindow(
        name="test",
        keyframe_times=times,
        centers=np.zeros((3, 3)),
        rotation_vectors=np.zeros((3, 3)),
        boundary_keyframes=np.stack([boundary] * 3),
        safe_margin=0.15,
        frame_thickness=0.1,
    )


def test_deformation_stays_simple_connected_and_nonempty():
    config = ExperimentConfig()
    scenario = build_scenario(
        seed=18,
        stage="full",
        environment=config.environment,
        quadrotor=config.quadrotor,
    )
    for window in scenario.windows:
        areas = []
        for time in np.linspace(0.0, scenario.horizon, 41):
            state = window.state(float(time))
            assert state.polygon.is_valid
            assert state.safe_polygon.geom_type == "Polygon"
            assert state.safe_polygon.area > 0.1
            areas.append(state.polygon.area)
        assert np.ptp(areas) > 1.0e-3


def test_safe_crossing_and_frame_collision_are_distinguished():
    window = _static_window()
    safe = window.crossing_event(
        np.array([0.0, 0.0, -1.0]),
        0.0,
        np.array([0.0, 0.0, 1.0]),
        0.1,
    )
    assert safe.occurred and safe.safe and not safe.frame_collision
    boundary_x = float(np.max(window.state(0.0).boundary[:, 0]))
    collision = window.crossing_event(
        np.array([boundary_x, 0.0, -1.0]),
        0.0,
        np.array([boundary_x, 0.0, 1.0]),
        0.1,
    )
    assert collision.occurred and collision.frame_collision and not collision.safe


def test_neutral_ctbr_hovers_and_respects_rotor_limits():
    config = ExperimentConfig()
    dynamics = QuadrotorDynamics(config.quadrotor)
    state = dynamics.hover_state(np.array([0.0, 0.0, 1.5]))
    for _ in range(100):
        state, diagnostics = dynamics.step(state, np.zeros(4), config.environment.dt)
        assert np.all(diagnostics.rotor_thrusts >= config.quadrotor.rotor_thrust_min)
        assert np.all(diagnostics.rotor_thrusts <= config.quadrotor.rotor_thrust_max)
    assert np.linalg.norm(state.position - np.array([0.0, 0.0, 1.5])) < 1.0e-6
    assert np.linalg.norm(state.velocity) < 1.0e-6
    assert np.allclose(state.rotation, np.eye(3), atol=1.0e-8)


def test_full_scenarios_preserve_window_separation_for_many_seeds():
    config = ExperimentConfig()
    for seed in range(12):
        scenario = build_scenario(
            seed=seed,
            stage="full",
            environment=config.environment,
            quadrotor=config.quadrotor,
        )
        assert len(scenario.windows) == config.environment.full_windows
        first = scenario.windows[0]
        # Fixed future, but not a periodic/constant first-to-last state.
        start = first.state(0.0)
        end = first.state(scenario.horizon)
        pose_change = np.linalg.norm(start.center - end.center)
        shape_change = np.linalg.norm(start.boundary - end.boundary)
        assert pose_change + shape_change > 1.0e-4


def test_time_critical_window_is_physical_but_temporarily_impassable():
    config = ExperimentConfig()
    config.environment.episode_seconds = 22.0
    config.environment.route_radius = 5.2
    config.environment.workspace_radius = 14.0
    config.environment.opportunity_mode = "single_shot"
    config.environment.opportunity_width = 1.1
    config.environment.opportunity_transition = 0.32
    config.environment.opportunity_closed_scale = 0.16
    config.environment.opportunity_open_scale = 1.05
    config.environment.opportunity_schedule_jitter = 0.22
    config.environment.motion_amplitude_multiplier = 1.8
    config.environment.deformation_amplitude_multiplier = 2.0
    scenario = build_scenario(
        seed=51,
        stage="full",
        environment=config.environment,
        quadrotor=config.quadrotor,
    )
    for window in scenario.windows:
        start, end = window.planned_opportunities[0]
        closed = window.state(0.0)
        opened = window.state(0.5 * (start + end))
        assert closed.polygon.is_valid and closed.polygon.area > 0.0
        assert closed.safe_polygon.is_empty
        assert not window.is_passable_state(closed)
        assert opened.polygon.is_valid and opened.polygon.area > closed.polygon.area
        assert window.is_passable_state(opened)
        features = window.opportunity_features(0.0, scenario.horizon)
        assert features.shape == (4,)
        assert features[0] == 0.0


def test_window_schedules_are_independent_of_route_and_vehicle_speed():
    config = ExperimentConfig()
    base = replace(
        config.environment,
        episode_seconds=22.0,
        route_radius=5.2,
        workspace_radius=14.0,
        opportunity_mode="irregular_repeated",
        opportunity_width=1.1,
        opportunity_schedule_jitter=0.22,
        opportunity_rescue_delay=4.0,
        motion_amplitude_multiplier=1.8,
        deformation_amplitude_multiplier=2.0,
    )
    changed_vehicle_assumptions = replace(
        base,
        cruise_speed=0.8,
        route_radius=6.0,
        workspace_radius=16.0,
    )
    first = build_scenario(
        seed=207,
        stage="full",
        environment=base,
        quadrotor=config.quadrotor,
    )
    second = build_scenario(
        seed=207,
        stage="full",
        environment=changed_vehicle_assumptions,
        quadrotor=config.quadrotor,
    )
    schedules = [window.planned_opportunities for window in first.windows]
    assert len(set(schedules)) == len(schedules)
    assert schedules == [window.planned_opportunities for window in second.windows]


def test_window_random_components_use_isolated_streams():
    config = ExperimentConfig()
    base = replace(
        config.environment,
        episode_seconds=22.0,
        route_radius=5.2,
        workspace_radius=14.0,
        opportunity_mode="irregular_repeated",
        opportunity_width=1.1,
        motion_amplitude_multiplier=1.8,
        deformation_amplitude_multiplier=2.0,
    )
    wider = replace(base, opportunity_width=1.7)
    first = build_scenario(
        seed=311,
        stage="full",
        environment=base,
        quadrotor=config.quadrotor,
    )
    second = build_scenario(
        seed=311,
        stage="full",
        environment=wider,
        quadrotor=config.quadrotor,
    )
    for left, right in zip(first.windows, second.windows):
        for time in (0.0, 3.7, 9.1, 17.3):
            left_state = left.state(time)
            right_state = right.state(time)
            assert np.allclose(left_state.center, right_state.center)
            assert np.allclose(left_state.rotation, right_state.rotation)
