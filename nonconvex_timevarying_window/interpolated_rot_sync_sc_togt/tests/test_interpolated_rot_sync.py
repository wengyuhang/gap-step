from __future__ import annotations

import json
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.experiments import (
    run_scenario,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.optimizer import (
    InterpolatedRotSyncObjective,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.paper_penalty import (
    integrated_paper_dynamic_penalty,
    instantaneous_paper_penalty,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.togt_code_penalty import (
    instantaneous_togt_code_penalty,
    integrated_togt_code_penalty,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.scenarios import (
    build_smoke_scenario,
    preprocess_shape_catalog,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.trajectory import (
    SCInputInterpolatedSyncSegment,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import (
    RotSyncOptimizationConfig,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    PenaltyWeights,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.trajectory import (
    RotationSyncSegment,
)


@pytest.fixture(scope="module")
def catalog():
    return preprocess_shape_catalog(
        vertex_count=32,
        quadrature_order=32,
        shape_names=("L",),
    )


def test_equal_inputs_recover_original_sync(catalog) -> None:
    window = build_smoke_scenario(catalog).windows[0]
    latent = np.asarray((0.22, -0.17))
    original = RotationSyncSegment(
        window, window.local_point(latent), entry_time=1.4, duration=0.83
    )
    interpolated = SCInputInterpolatedSyncSegment(
        window, latent, latent, entry_time=1.4, duration=0.83
    )
    times = np.linspace(0.0, original.duration, 101)
    for derivative in range(5):
        assert np.allclose(
            interpolated.evaluate(times, derivative),
            original.evaluate(times, derivative),
            atol=2.0e-11,
        )
    assert interpolated.snap_energy() == pytest.approx(
        original.snap_energy(), rel=2.0e-12
    )


def test_zero_thickness_sync_endpoints_are_sphere_plane_tangencies(catalog) -> None:
    original_scenario = build_smoke_scenario(catalog)
    window = replace(original_scenario.windows[0], thickness=0.0)
    assert window.clearance_distance == pytest.approx(window.rho)
    latent = np.asarray((0.22, -0.17))
    fixed = RotationSyncSegment(
        window, window.local_point(latent), entry_time=1.4, duration=0.83
    )
    interpolated = SCInputInterpolatedSyncSegment(
        window, latent, latent, entry_time=1.4, duration=0.83
    )
    for segment in (fixed, interpolated):
        signed_entry = float((segment.evaluate(0.0) - window.center) @ window.normal)
        signed_exit = float(
            (segment.evaluate(segment.duration) - window.center) @ window.normal
        )
        assert signed_entry == pytest.approx(-window.rho, abs=1.0e-12)
        assert signed_exit == pytest.approx(window.rho, abs=1.0e-12)

    times = np.linspace(0.0, fixed.duration, 101)
    for derivative in range(5):
        assert np.allclose(
            fixed.evaluate(times, derivative),
            interpolated.evaluate(times, derivative),
            atol=2.0e-11,
        )


def test_unequal_inputs_have_correct_derivatives_and_safe_path(catalog) -> None:
    window = build_smoke_scenario(catalog).windows[0]
    entry = np.asarray((-0.24, 0.13))
    exit = np.asarray((0.31, -0.21))
    segment = SCInputInterpolatedSyncSegment(
        window, entry, exit, entry_time=1.7, duration=0.79
    )
    instant, step = 0.37, 1.0e-5
    for derivative in range(4):
        numeric = (
            segment.evaluate(instant + step, derivative)
            - segment.evaluate(instant - step, derivative)
        ) / (2.0 * step)
        assert np.allclose(
            numeric, segment.evaluate(instant, derivative + 1), atol=2.0e-6
        )

    times = np.linspace(0.0, segment.duration, 51)
    expected = entry + (times[:, None] / segment.duration) * (exit - entry)
    assert np.allclose(segment.latent_at(times), expected, atol=1.0e-14)
    local = segment.local_point_at(times)
    safe = Polygon(window.safe_polygon).buffer(1.0e-10)
    assert all(safe.covers(Point(point)) for point in local)
    midpoint = segment.local_point_at(0.5 * segment.duration)
    assert np.allclose(
        midpoint, window.local_point(0.5 * (entry + exit)), atol=1.0e-12
    )
    assert np.linalg.norm(midpoint - 0.5 * (local[0] + local[-1])) > 1.0e-5


def test_pvaj_interfaces_are_c3(catalog) -> None:
    scenario = build_smoke_scenario(catalog)
    objective = InterpolatedRotSyncObjective(
        scenario,
        RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3),
    )
    x = objective.initial_guess()
    x[3:5] = (-0.20, 0.14)
    x[5:7] = (0.28, -0.11)
    forward = objective.forward(x)
    assert forward.trajectory.segment_kinds == ("minco", "sync", "minco")
    assert np.max(forward.trajectory.interface_residuals()) < 1.0e-8
    sync = forward.trajectory.sync_segments[0]
    before, after = forward.trajectory.free_segments
    for derivative in range(4):
        assert np.allclose(
            before.evaluate(before.total_time, derivative),
            sync.evaluate(0.0, derivative),
            atol=1.0e-8,
        )
        assert np.allclose(
            sync.evaluate(sync.duration, derivative),
            after.evaluate(0.0, derivative),
            atol=1.0e-8,
        )


def test_new_folder_experiment_exports(catalog, tmp_path) -> None:
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
    assert validation["all_sampled_sync_path_in_safe_opening"]
    assert validation["c3_continuous"]
    assert validation["dynamic_audit_max_step"] <= 0.01
    for name in (
        "config.json",
        "result.json",
        "trajectory.csv",
        "trajectory_3d.png",
        "sync_closeups.png",
    ):
        assert (tmp_path / name).is_file()
    saved = json.loads((tmp_path / "result.json").read_text(encoding="utf-8"))
    assert saved["entry_latent_points"]
    assert saved["exit_latent_points"]
    assert saved["collision"]["body_model"] == "oriented_square_bottom_cuboid"


def test_togt_paper_penalty_is_cubic_positive_part() -> None:
    flatness = SimpleNamespace(
        collective_thrust=0.0,
        body_rate=np.zeros(3),
        rotor_thrusts=np.ones(4),
    )
    breakdown = instantaneous_paper_penalty(
        np.asarray((2.0, 0.0, 0.0)),
        flatness,
        limits=DynamicLimits(max_velocity=1.0),
        weights=PenaltyWeights(
            velocity=1.0,
            collective_thrust=0.0,
            body_rate=0.0,
            rotor_thrust=0.0,
        ),
    )
    # h_v = ||v||^2 - v_max^2 = 3, so max(h_v, 0)^3 = 27.
    assert breakdown.velocity == pytest.approx(27.0)
    assert breakdown.total == pytest.approx(27.0)

    class ConstantVelocityTrajectory:
        durations = np.asarray((1.0,))
        coefficients = np.zeros((1, 1, 3))

        @staticmethod
        def evaluate_segment(segment, local_time, derivative=0):
            assert segment == 0
            if derivative == 1:
                return np.asarray((2.0, 0.0, 0.0))
            return np.zeros(3)

    integrated = integrated_paper_dynamic_penalty(
        ConstantVelocityTrajectory(),
        limits=DynamicLimits(max_velocity=1.0),
        weights=PenaltyWeights(
            velocity=1.0,
            collective_thrust=0.0,
            body_rate=0.0,
            rotor_thrust=0.0,
        ),
        samples_per_segment=3,
    )
    # Equation (12) uses three full Delta-t weights: 3 * (1/2) * 27.
    assert integrated == pytest.approx(40.5)


def test_counterexample_objective_excludes_snap_and_collision(
    catalog, monkeypatch
) -> None:
    from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import (
        compare_fixed_wp_counterexample as comparison,
    )

    def collision_must_not_run(*args, **kwargs):
        raise AssertionError("collision penalty was evaluated")

    monkeypatch.setattr(
        comparison._EXPERIMENT,
        "sphere_frame_collision_penalty",
        collision_must_not_run,
    )

    scenario = build_smoke_scenario(catalog)
    objective = comparison.CodeTOGTInterpolatedObjective(
        scenario,
        RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3),
    )
    forward = objective.forward(objective.initial_guess())
    cost = objective.cost_breakdown(forward)
    assert cost.collision_penalty == 0.0
    assert cost.smoothness > 0.0
    assert cost.weighted_total == pytest.approx(
        cost.total_time + cost.dynamic_penalty
    )


def test_fixed_sync_is_exact_subset_of_interpolated_objective(catalog) -> None:
    from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import (
        compare_fixed_wp_counterexample as comparison,
    )

    base = build_smoke_scenario(catalog)
    scenario = replace(
        base,
        windows=(replace(base.windows[0], thickness=0.0),),
    )
    config = RotSyncOptimizationConfig(max_iterations=1, samples_per_segment=3)
    fixed = comparison.CodeTOGTRotSyncObjective(scenario, config)
    interpolated = comparison.CodeTOGTInterpolatedObjective(scenario, config)
    fixed_x = fixed.initial_guess()
    interpolated_x = np.concatenate((fixed_x[:3], fixed_x[3:], fixed_x[3:]))
    fixed_forward = fixed.forward(fixed_x)
    interpolated_forward = interpolated.forward(interpolated_x)
    times = np.linspace(0.0, fixed_forward.trajectory.total_time, 101)
    for derivative in range(5):
        assert np.allclose(
            fixed_forward.trajectory.evaluate(times, derivative),
            interpolated_forward.trajectory.evaluate(times, derivative),
            atol=1.0e-9,
        )
    assert interpolated.cost_breakdown(
        interpolated_forward
    ).weighted_total == pytest.approx(
        fixed.cost_breakdown(fixed_forward).weighted_total,
        abs=1.0e-9,
    )


def test_sc_dynatogt_free_waypoint_exactly_contains_fixed_waypoint() -> None:
    from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import (
        compare_sc_dynatogt_fixed_wp as comparison,
    )

    _, config, _, _, track, _, fixed_d = comparison._build_problem()
    fixed = comparison.FixedWaypointObjective(track, config, fixed_d)
    free = comparison.FreeSCWaypointObjective(track, config)
    fixed_x = fixed.initial_guess()
    free_x = np.concatenate((fixed_x, fixed_d))
    fixed_forward = fixed.forward(fixed_x)
    free_forward = free.forward(free_x)
    times = np.linspace(0.0, fixed_forward.trajectory.total_time, 101)
    for derivative in range(5):
        assert np.array_equal(
            fixed_forward.trajectory.evaluate(times, derivative),
            free_forward.trajectory.evaluate(times, derivative),
        )
    assert fixed.cost_breakdown(
        fixed_forward
    ).weighted_total == pytest.approx(
        free.cost_breakdown(free_forward).weighted_total,
        abs=1.0e-12,
    )


def test_togt_code_penalty_uses_adaptive_trapezoid_and_smoothed_l1() -> None:
    class ConstantVelocityTrajectory:
        durations = np.asarray((1.0,))
        coefficients = np.zeros((1, 1, 3))

        @staticmethod
        def evaluate_segment(segment, local_time, derivative=0):
            assert segment == 0
            if derivative == 1:
                return np.asarray((2.0, 0.0, 0.0))
            return np.zeros(3)

    weights = PenaltyWeights(
        velocity=1.0,
        collective_thrust=0.0,
        body_rate=0.0,
        rotor_thrust=0.0,
    )
    integrated = integrated_togt_code_penalty(
        ConstantVelocityTrajectory(),
        limits=DynamicLimits(max_velocity=1.0),
        weights=weights,
    )
    # h=3 > mu=0.01, so smoothedL1(h)=h-mu/2=2.995.  The
    # 20-interval trapezoid integrates the constant value over exactly 1 s.
    assert integrated == pytest.approx(2.995)

    singular = instantaneous_togt_code_penalty(
        np.zeros(3),
        np.asarray((0.0, 0.0, -2.0 * 9.8066)),
        np.zeros(3),
        np.zeros(3),
        limits=DynamicLimits(),
        weights=PenaltyWeights(
            velocity=0.0,
            collective_thrust=0.0,
            body_rate=1.0,
            rotor_thrust=1.0,
        ),
    )
    assert singular.total == pytest.approx(0.0)
