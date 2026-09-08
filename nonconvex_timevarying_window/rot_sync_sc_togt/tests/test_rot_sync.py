from __future__ import annotations

import json

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.rot_sync_sc_togt.collision import (
    cuboid_vertices,
    cuboid_window_collision,
    sample_collision_report,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.experiments import run_scenario
from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import (
    RotSyncObjective,
    RotSyncOptimizationConfig,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import (
    REALISTIC_BODY,
    REALISTIC_RHO,
    REALISTIC_SHAPE_SCALES,
    build_formal_scenarios,
    build_multi_scenarios,
    build_realistic_extreme_scenario,
    build_smoke_scenario,
    preprocess_shape_catalog,
    scenario_difficulty_metrics,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.trajectory import RotationSyncSegment


@pytest.fixture(scope="module")
def catalog():
    return preprocess_shape_catalog(
        vertex_count=32,
        quadrature_order=32,
        shape_names=("L", "U", "star"),
    )


@pytest.fixture(scope="module")
def formal_catalog():
    return preprocess_shape_catalog(vertex_count=256, quadrature_order=32)


@pytest.fixture(scope="module")
def realistic_catalog():
    return preprocess_shape_catalog(
        rho=REALISTIC_RHO,
        vertex_count=256,
        quadrature_order=32,
        shape_scales=REALISTIC_SHAPE_SCALES,
    )


def test_sync_analytic_derivatives_and_gate_frame_lock(catalog) -> None:
    scenario = build_smoke_scenario(catalog)
    window = scenario.windows[0]
    q = window.local_point(np.asarray((0.25, -0.15)))
    sync = RotationSyncSegment(window, q, entry_time=1.7, duration=0.8)
    instant, step = 0.37, 1.0e-5
    for derivative in range(3):
        numeric = (
            sync.evaluate(instant + step, derivative)
            - sync.evaluate(instant - step, derivative)
        ) / (2.0 * step)
        assert np.allclose(numeric, sync.evaluate(instant, derivative + 1), atol=2.0e-7)

    times = np.linspace(0.0, sync.duration, 21)
    recovered = []
    for tau, position in zip(times, sync.evaluate(times)):
        z = -window.clearance_distance + 2.0 * window.clearance_distance * tau / sync.duration
        recovered.append(
            window.rotated_basis(sync.entry_time + tau).T
            @ (position - window.center - window.normal * z)
        )
    assert np.max(np.linalg.norm(np.asarray(recovered) - q, axis=1)) < 1.0e-12
    assert Polygon(window.safe_polygon).covers(Point(q))


def test_smoke_builds_exact_minco_sync_minco_and_c3(catalog) -> None:
    scenario = build_smoke_scenario(catalog)
    objective = RotSyncObjective(
        scenario,
        RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3),
    )
    forward = objective.forward(objective.initial_guess())
    assert forward.trajectory.segment_kinds == ("minco", "sync", "minco")
    assert forward.trajectory.interface_residuals().shape == (2, 4)
    assert np.max(forward.trajectory.interface_residuals()) < 1.0e-9
    crossing = forward.crossing_times[0]
    assert np.allclose(
        forward.trajectory.evaluate(crossing),
        scenario.windows[0].world_point(forward.local_points[0], crossing),
        atol=1.0e-10,
    )
    half = np.asarray(scenario.body.half_extents)
    assert half[0] == pytest.approx(half[1])
    assert half[2] < half[0]
    assert scenario.windows[0].rho == pytest.approx(scenario.body.circumscribed_radius)
    vertices = cuboid_vertices(np.zeros(3), np.eye(3), scenario.body)
    assert np.ptp(vertices, axis=0) == pytest.approx(2.0 * half)


def test_oriented_cuboid_collision_and_sampled_rate(catalog) -> None:
    scenario = build_smoke_scenario(catalog)
    objective = RotSyncObjective(
        scenario,
        RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3),
    )
    forward = objective.forward(objective.initial_guess())
    report = sample_collision_report(scenario, forward.trajectory, samples=121)
    assert report.sample_count == 121
    assert report.sampled_collision_rate == pytest.approx(0.0)
    window = scenario.windows[0]
    boundary_center = window.world_point(window.physical_polygon[0], 0.0)
    collision, clearance = cuboid_window_collision(
        window, 0.0, boundary_center, np.eye(3), scenario.body
    )
    assert collision
    assert clearance == pytest.approx(0.0)


def test_multi_tracks_are_distinct_closed_l_u_star_courses(catalog) -> None:
    scenarios = build_multi_scenarios(catalog)
    assert tuple(scenario.name for scenario in scenarios) == (
        "closed_triangle", "closed_slalom", "closed_spatial"
    )
    for scenario in scenarios:
        assert scenario.closed
        assert np.array_equal(scenario.start_state.matrix, scenario.goal_state.matrix)
        assert tuple(window.name for window in scenario.windows) == ("L", "U", "star")
        objective = RotSyncObjective(
            scenario,
            RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3),
        )
        forward = objective.forward(objective.initial_guess())
        assert forward.trajectory.segment_kinds == (
            "minco", "sync", "minco", "sync", "minco", "sync", "minco"
        )
        assert np.max(forward.trajectory.interface_residuals()) < 1.0e-8


def test_formal_tracks_cover_six_shapes_and_increase_difficulty(formal_catalog) -> None:
    scenarios = build_formal_scenarios(formal_catalog)
    assert tuple(scenario.name for scenario in scenarios) == (
        "D1_compact_planar",
        "D2_spatial_slalom",
        "D3_uzh_irregular",
        "D4_split_s_endurance",
    )
    metrics = [scenario_difficulty_metrics(scenario) for scenario in scenarios]
    assert [item["gate_count"] for item in metrics] == [3, 4, 6, 6]
    assert [item["maximum_abs_omega"] for item in metrics] == pytest.approx(
        [0.44, 0.82, 1.18, 1.52]
    )
    assert all(
        left["altitude_range"] < right["altitude_range"]
        for left, right in zip(metrics, metrics[1:])
    )
    assert {
        window.name for scenario in scenarios for window in scenario.windows
    } == {"L", "U", "star", "limacon", "wavy", "line_bezier"}
    for scenario in scenarios:
        assert scenario.closed
        assert np.array_equal(scenario.start_state.matrix, scenario.goal_state.matrix)
    objective = RotSyncObjective(
        scenarios[-1],
        RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3),
    )
    forward = objective.forward(objective.initial_guess())
    assert forward.trajectory.segment_kinds == tuple(
        kind for _ in range(6) for kind in ("minco", "sync")
    ) + ("minco",)
    assert np.max(forward.trajectory.interface_residuals()) < 1.0e-8


def test_realistic_extreme_track_has_large_flat_body_tight_gates_and_long_legs(
    realistic_catalog,
) -> None:
    scenario = build_realistic_extreme_scenario(realistic_catalog)
    assert scenario.closed
    assert scenario.body is REALISTIC_BODY
    assert 2.0 * np.asarray(scenario.body.half_extents) == pytest.approx((0.60, 0.60, 0.18))
    assert scenario.body.circumscribed_radius == pytest.approx(REALISTIC_RHO)
    assert tuple(window.name for window in scenario.windows) == (
        "star", "line_bezier", "L", "wavy", "U", "limacon"
    )
    assert all(window.rho == pytest.approx(REALISTIC_RHO) for window in scenario.windows)
    centers = np.asarray([window.center for window in scenario.windows])
    separations = np.linalg.norm(centers[:, None] - centers[None, :], axis=2)
    separations[separations == 0.0] = np.inf
    assert np.min(separations) > 10.0
    assert scenario_difficulty_metrics(scenario)["nominal_route_length"] > 90.0
    assert max(np.max(np.ptp(window.physical_polygon, axis=0)) for window in scenario.windows) < 3.3
    objective = RotSyncObjective(
        scenario,
        RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3),
    )
    forward = objective.forward(objective.initial_guess())
    assert np.max(forward.trajectory.interface_residuals()) < 1.0e-8


def test_smoke_run_exports_research_artifacts(catalog, tmp_path) -> None:
    scenario = build_smoke_scenario(catalog)
    result, validation = run_scenario(
        scenario,
        tmp_path,
        config=RotSyncOptimizationConfig(
            max_iterations=2,
            samples_per_segment=3,
            audit_max_step=0.01,
        ),
        make_animation=False,
        collision_samples=301,
    )
    assert result.total_time > 0.0
    assert validation["all_q_in_safe_opening"]
    assert validation["c3_continuous"]
    assert validation["dynamic_audit_max_step"] <= 0.01
    for name in (
        "config.json", "result.json", "trajectory.csv", "trajectory_3d.png",
        "sync_closeups.png",
    ):
        assert (tmp_path / name).is_file()
    saved = json.loads((tmp_path / "result.json").read_text(encoding="utf-8"))
    assert "selected_q" in saved
    assert saved["max_velocity"] > 0.0
    assert saved["max_acceleration"] > 0.0
    assert saved["collision"]["body_model"] == "oriented_square_bottom_cuboid"
    assert saved["total_time"] / (saved["collision"]["sample_count"] - 1) <= 0.01
    assert 0.0 <= saved["collision"]["sampled_collision_rate"] <= 1.0
