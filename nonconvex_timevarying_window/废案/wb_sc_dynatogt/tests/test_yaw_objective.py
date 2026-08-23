from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.wb_sc_dynatogt.collider import CuboidCollider
from nonconvex_timevarying_window.wb_sc_dynatogt.config import WBSCOptimizationConfig
from nonconvex_timevarying_window.wb_sc_dynatogt.optimizer import WBSCObjective
from nonconvex_timevarying_window.wb_sc_dynatogt.objective import torch_polygon_boundary_clearance
from nonconvex_timevarying_window.wb_sc_dynatogt.yaw import YawTrajectory, yaw_from_unconstrained


def test_yaw_mapping_and_minimum_snap_continuity() -> None:
    trajectory = YawTrajectory(np.array([0.5, -0.35]), np.array([0.8, 1.1, 0.9]))
    assert np.all(np.abs(yaw_from_unconstrained(np.array([-1.0e6, 0.0, 1.0e6]))) < np.pi)
    for derivative in range(4):
        left = trajectory.evaluate_segment(0, trajectory.durations[0], derivative)
        right = trajectory.evaluate_segment(1, 0.0, derivative)
        assert np.isclose(left, right, atol=2.0e-8)
        left = trajectory.evaluate_segment(1, trajectory.durations[1], derivative)
        right = trajectory.evaluate_segment(2, 0.0, derivative)
        assert np.isclose(left, right, atol=2.0e-8)


def test_nonconvex_window_boundary_clearance_and_gradient() -> None:
    torch = __import__("torch")
    polygon = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, -0.2], [0.2, -0.2], [0.2, 1.0], [-1.0, 1.0]])
    points = torch.tensor([[0.0, 0.0], [0.7, 0.7]], dtype=torch.float64, requires_grad=True)
    signed = torch_polygon_boundary_clearance(points, polygon)
    assert signed[0] > 0.0
    assert signed[1] < 0.0
    signed.sum().backward()
    assert torch.isfinite(points.grad).all()


def test_joint_attitude_and_window_time_gradient(dynamic_track) -> None:
    config = WBSCOptimizationConfig(
        max_iterations=1,
        samples_per_segment=2,
        optimize_yaw=True,
        yaw_snap_weight=1.0e-5,
    )
    objective = WBSCObjective(
        dynamic_track,
        config,
        CuboidCollider(config.collider),
        0.5,
    )
    x = objective.initial_guess(yaw_waypoints=np.array([0.25]))
    evaluation = objective.evaluate(x)
    direction = np.array([0.35, -0.25, 0.10, -0.20, 0.30])
    direction /= np.linalg.norm(direction)
    step = 1.0e-6
    finite = (objective.evaluate(x + step * direction).cost - objective.evaluate(x - step * direction).cost) / (2.0 * step)
    analytic = float(evaluation.gradient @ direction)
    assert np.isclose(analytic, finite, rtol=2.0e-4, atol=2.0e-5)
    assert np.isfinite(evaluation.projected_min_clearance)


def test_hard_cuboid_constraint_vector_is_feasible_for_wide_square(static_track) -> None:
    config = WBSCOptimizationConfig(
        max_iterations=1,
        samples_per_segment=2,
        hard_collision_constraints=True,
    )
    objective = WBSCObjective(static_track, config, CuboidCollider(config.collider))
    residuals = objective.collision_constraints(objective.initial_guess())
    assert residuals.shape == (92,)
    assert np.min(residuals) > 0.0


def test_default_decision_vector_jointly_includes_yaw(static_track) -> None:
    config = WBSCOptimizationConfig(max_iterations=1, samples_per_segment=2)
    objective = WBSCObjective(static_track, config, CuboidCollider(config.collider))
    gate_count = len(static_track.order)
    assert config.optimize_yaw
    assert config.hard_collision_constraints
    assert objective.dimension == (gate_count + 1) + 3 * gate_count
    assert objective.initial_guess().shape == (objective.dimension,)


def test_pose_constraint_changes_inside_planning_variables(static_track) -> None:
    config = WBSCOptimizationConfig(max_iterations=1, samples_per_segment=2)
    objective = WBSCObjective(static_track, config, CuboidCollider(config.collider))
    x = objective.initial_guess()
    baseline = objective.collision_constraints(x)

    yaw_trial = x.copy()
    yaw_trial[-1] = 0.5
    yaw_changed = objective.collision_constraints(yaw_trial)

    point_trial = x.copy()
    first_d = len(static_track.order) + 1
    point_trial[first_d] = 0.2
    point_changed = objective.collision_constraints(point_trial)

    assert not np.allclose(yaw_changed, baseline)
    assert not np.allclose(point_changed, baseline)
