"""Differentiable position/yaw/whole-body objective for WBSC-DynaTOGT."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
import shapely

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicCheckSampling,
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
    QuadrotorParameters,
    _quadrature_interval_count,
    _torch_centered_interval_penalty,
    _torch_normalize_with_derivatives,
    _torch_optional_centered_interval_penalty,
    _torch_polynomial_derivative,
    _torch_smoothed_l1,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap

from .collider import CuboidCollider
from .config import WBSCOptimizationConfig
from .yaw import YawTrajectory


@dataclass(frozen=True)
class JointGradientResult:
    cost: float
    point_gradient: np.ndarray
    yaw_gradient: np.ndarray
    duration_gradient: np.ndarray
    minimum_projected_clearance: float
    collision_cost: float


def _torch_flatness_general(
    acceleration,
    jerk,
    snap,
    yaw,
    yaw_rate,
    yaw_acceleration,
    parameters: QuadrotorParameters,
):
    """Torch equivalent of the complete yaw-aware flatness map."""

    torch = __import__("torch")
    gravity = acceleration.new_tensor([0.0, 0.0, parameters.gravity])
    specific_force = acceleration + gravity
    body_z, body_z_dot, body_z_ddot = _torch_normalize_with_derivatives(
        specific_force,
        jerk,
        snap,
        name="specific force",
        epsilon=parameters.singularity_epsilon,
    )
    zero = acceleration.new_zeros(())
    heading_x = torch.stack((torch.cos(yaw), torch.sin(yaw), zero))
    heading_y = torch.stack((-torch.sin(yaw), torch.cos(yaw), zero))
    heading_y_dot = -yaw_rate * heading_x
    heading_y_ddot = -yaw_acceleration * heading_x - yaw_rate**2 * heading_y
    raw_x = torch.linalg.cross(heading_y, body_z, dim=0)
    raw_x_dot = torch.linalg.cross(heading_y_dot, body_z, dim=0) + torch.linalg.cross(
        heading_y, body_z_dot, dim=0
    )
    raw_x_ddot = (
        torch.linalg.cross(heading_y_ddot, body_z, dim=0)
        + 2.0 * torch.linalg.cross(heading_y_dot, body_z_dot, dim=0)
        + torch.linalg.cross(heading_y, body_z_ddot, dim=0)
    )
    body_x, body_x_dot, body_x_ddot = _torch_normalize_with_derivatives(
        raw_x,
        raw_x_dot,
        raw_x_ddot,
        name="heading/body-z cross product",
        epsilon=parameters.singularity_epsilon,
    )
    body_y = torch.linalg.cross(body_z, body_x, dim=0)
    body_y_dot = torch.linalg.cross(body_z_dot, body_x, dim=0) + torch.linalg.cross(
        body_z, body_x_dot, dim=0
    )
    body_y_ddot = (
        torch.linalg.cross(body_z_ddot, body_x, dim=0)
        + 2.0 * torch.linalg.cross(body_z_dot, body_x_dot, dim=0)
        + torch.linalg.cross(body_z, body_x_ddot, dim=0)
    )
    rotation = torch.stack((body_x, body_y, body_z), dim=1)
    rotation_dot = torch.stack((body_x_dot, body_y_dot, body_z_dot), dim=1)
    rotation_ddot = torch.stack((body_x_ddot, body_y_ddot, body_z_ddot), dim=1)
    omega_hat = rotation.T @ rotation_dot
    omega_hat_dot = rotation_dot.T @ rotation_dot + rotation.T @ rotation_ddot
    body_rate = 0.5 * torch.stack(
        (
            omega_hat[2, 1] - omega_hat[1, 2],
            omega_hat[0, 2] - omega_hat[2, 0],
            omega_hat[1, 0] - omega_hat[0, 1],
        )
    )
    body_rate_derivative = 0.5 * torch.stack(
        (
            omega_hat_dot[2, 1] - omega_hat_dot[1, 2],
            omega_hat_dot[0, 2] - omega_hat_dot[2, 0],
            omega_hat_dot[1, 0] - omega_hat_dot[0, 1],
        )
    )
    collective = parameters.mass * torch.linalg.vector_norm(specific_force)
    inertia = acceleration.new_tensor(np.asarray(parameters.inertia, dtype=float))
    torque = inertia @ body_rate_derivative + torch.linalg.cross(
        body_rate, inertia @ body_rate, dim=0
    )
    allocation = acceleration.new_tensor(
        np.asarray(parameters.allocation_matrix, dtype=float)
    )
    rotor_thrusts = allocation @ torch.cat((collective.reshape(1), torque))
    return rotation, collective, body_rate, rotor_thrusts


def _torch_rotation_rpy(angles):
    torch = __import__("torch")
    roll, pitch, yaw = angles.unbind()
    cr, sr = torch.cos(roll), torch.sin(roll)
    cp, sp = torch.cos(pitch), torch.sin(pitch)
    cy, sy = torch.cos(yaw), torch.sin(yaw)
    zero, one = roll.new_zeros(()), roll.new_ones(())
    rz = torch.stack(
        (
            torch.stack((cy, -sy, zero)),
            torch.stack((sy, cy, zero)),
            torch.stack((zero, zero, one)),
        )
    )
    ry = torch.stack(
        (
            torch.stack((cp, zero, sp)),
            torch.stack((zero, one, zero)),
            torch.stack((-sp, zero, cp)),
        )
    )
    rx = torch.stack(
        (
            torch.stack((one, zero, zero)),
            torch.stack((zero, cr, -sr)),
            torch.stack((zero, sr, cr)),
        )
    )
    return rz @ ry @ rx


def _torch_window_state(window, time):
    motion = window.motion
    translation = time.new_zeros((3,))
    if motion.translation_enabled:
        omega = 2.0 * math.pi / motion.translation_period
        phases = time.new_tensor([0.0, 0.7, 1.4]) + motion.phase
        amplitude = time.new_tensor(np.asarray(motion.translation_amplitude, dtype=float))
        translation = amplitude * __import__("torch").sin(omega * time + phases)
    angles = time.new_tensor(np.asarray(window.angles0, dtype=float))
    if motion.rotation_enabled:
        omega = 2.0 * math.pi / motion.rotation_period
        phases = time.new_tensor([0.0, 0.9, 1.8]) + motion.phase
        amplitude = time.new_tensor(np.asarray(motion.rotation_amplitude, dtype=float))
        angles = angles + amplitude * __import__("torch").sin(omega * time + phases)
    scale = time.new_ones(())
    if motion.scale_enabled:
        omega = 2.0 * math.pi / motion.scale_period
        scale = 1.0 + motion.scale_amplitude * __import__("torch").sin(
            omega * time + motion.phase
        )
    rotation = _torch_rotation_rpy(angles)
    center = time.new_tensor(np.asarray(window.center0, dtype=float)) + translation
    return center, rotation[:, :2], scale


def torch_polygon_boundary_clearance(points, polygon):
    """Metric clearance to polygon edges, positive only for interior points.

    This is an analytic collision residual for sampled cuboid boundary points.  It
    does not model or predict the window motion.
    """

    torch = __import__("torch")
    vertices = points.new_tensor(np.asarray(polygon, dtype=float))
    following = torch.roll(vertices, shifts=-1, dims=0)
    edges = following - vertices
    relative = points[:, None, :] - vertices[None, :, :]
    denominator = (edges * edges).sum(dim=1).clamp_min(1.0e-18)
    fraction = (relative * edges[None, :, :]).sum(dim=2) / denominator[None, :]
    closest = vertices[None, :, :] + fraction.clamp(0.0, 1.0)[:, :, None] * edges[None, :, :]
    distance = torch.sqrt(((points[:, None, :] - closest) ** 2).sum(dim=2) + 1.0e-24)
    unsigned = distance.min(dim=1).values

    py = points[:, 1:2]
    px = points[:, 0:1]
    ay = vertices[None, :, 1]
    by = following[None, :, 1]
    crossings = (ay > py) != (by > py)
    dy = by - ay
    safe_dy = torch.where(torch.abs(dy) > 1.0e-15, dy, torch.ones_like(dy))
    x_cross = (
        (following[None, :, 0] - vertices[None, :, 0])
        * (py - ay)
        / safe_dy
        + vertices[None, :, 0]
    )
    inside = ((crossings & (px < x_cross)).sum(dim=1) % 2) == 1
    return torch.where(inside, unsigned, -unsigned)


def polygon_boundary_clearance(points: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """NumPy counterpart used by the nonlinear hard constraints."""

    query = np.asarray(points, dtype=float)
    vertices = np.asarray(polygon, dtype=float)
    following = np.roll(vertices, -1, axis=0)
    edges = following - vertices
    relative = query[:, None, :] - vertices[None, :, :]
    denominator = np.maximum(np.sum(edges * edges, axis=1), 1.0e-18)
    fraction = np.sum(relative * edges[None, :, :], axis=2) / denominator[None, :]
    closest = vertices[None, :, :] + np.clip(fraction, 0.0, 1.0)[:, :, None] * edges[None, :, :]
    unsigned = np.sqrt(np.sum((query[:, None, :] - closest) ** 2, axis=2)).min(axis=1)
    inside = shapely.contains_xy(shapely.Polygon(vertices), query[:, 0], query[:, 1])
    return np.where(inside, unsigned, -unsigned)


def _collision_terms(
    position_coefficients,
    yaw_coefficients,
    durations,
    *,
    track: SCWindowTrack,
    collider: CuboidCollider,
    body_scale: float,
    parameters: QuadrotorParameters,
    weight: float,
):
    # A non-convex aperture can contain every corner while a cuboid edge cuts
    # across a reflex notch.  Sample all 12 edges in the soft term; the final
    # validator still checks the complete projected convex hull.
    points_body = durations.new_tensor(
        body_scale * collider.edge_points(samples_per_edge=9)
    )
    cost = durations.new_zeros(())
    clearances = []
    for crossing_index in range(len(track.order)):
        segment = crossing_index
        local_time = durations[segment]
        global_time = durations[: segment + 1].sum()
        position = _torch_polynomial_derivative(position_coefficients[segment], local_time, 0)
        acceleration = _torch_polynomial_derivative(position_coefficients[segment], local_time, 2)
        jerk = _torch_polynomial_derivative(position_coefficients[segment], local_time, 3)
        snap = _torch_polynomial_derivative(position_coefficients[segment], local_time, 4)
        yaw = _torch_polynomial_derivative(yaw_coefficients[segment], local_time, 0)[0]
        yaw_rate = _torch_polynomial_derivative(yaw_coefficients[segment], local_time, 1)[0]
        yaw_acceleration = _torch_polynomial_derivative(yaw_coefficients[segment], local_time, 2)[0]
        body_rotation, _, _, _ = _torch_flatness_general(
            acceleration,
            jerk,
            snap,
            yaw,
            yaw_rate,
            yaw_acceleration,
            parameters,
        )
        window = track.windows[track.order[crossing_index]]
        center, basis, scale = _torch_window_state(window, global_time)
        world_points = position[None, :] + points_body @ body_rotation.T
        local_points = (world_points - center[None, :]) @ basis / scale
        polygon = window.physical_boundary
        if polygon is None:
            polygon = window.safe_polygon
        signed = torch_polygon_boundary_clearance(local_points, polygon) * scale
        clearances.append(signed.min())
        cost = cost + weight * _torch_smoothed_l1(collider.config.clearance - signed).mean()
    if clearances:
        minimum = __import__("torch").stack(clearances).min()
    else:
        minimum = durations.new_tensor(float("inf"))
    return cost, minimum


def joint_objective_with_gradients(
    trajectory: MincoSnap,
    yaw_trajectory: YawTrajectory,
    *,
    track: SCWindowTrack,
    collider: CuboidCollider,
    body_scale: float,
    config: WBSCOptimizationConfig,
) -> JointGradientResult:
    """Evaluate one reverse-mode graph over position, yaw, time and collision."""

    torch = __import__("torch")
    position_points = torch.tensor(
        trajectory.intermediate_points, dtype=torch.float64, requires_grad=True
    )
    yaw_points_array = np.zeros((len(yaw_trajectory.waypoints), 3), dtype=float)
    yaw_points_array[:, 0] = yaw_trajectory.waypoints
    yaw_points = torch.tensor(yaw_points_array, dtype=torch.float64, requires_grad=True)
    durations = torch.tensor(trajectory.durations, dtype=torch.float64, requires_grad=True)
    position_coefficients = trajectory._torch_solve_coefficients(position_points, durations)
    yaw_coefficients = yaw_trajectory.minco._torch_solve_coefficients(yaw_points, durations)

    params = config.quadrotor
    limits: DynamicLimits = config.dynamic_limits
    penalties: PenaltyWeights = config.penalty_weights
    weights: ObjectiveWeights = config.objective_weights
    sampling = DynamicCheckSampling()
    dynamic_cost = durations.new_zeros(())
    for segment, duration in enumerate(durations.unbind()):
        interval_count = _quadrature_interval_count(
            float(duration.detach().cpu()), config.samples_per_segment, sampling
        )
        step = duration / interval_count
        for node_index, fraction in enumerate(np.linspace(0.0, 1.0, interval_count + 1)):
            local_time = duration * float(fraction)
            velocity = _torch_polynomial_derivative(position_coefficients[segment], local_time, 1)
            acceleration = _torch_polynomial_derivative(position_coefficients[segment], local_time, 2)
            jerk = _torch_polynomial_derivative(position_coefficients[segment], local_time, 3)
            snap = _torch_polynomial_derivative(position_coefficients[segment], local_time, 4)
            yaw = _torch_polynomial_derivative(yaw_coefficients[segment], local_time, 0)[0]
            yaw_rate = _torch_polynomial_derivative(yaw_coefficients[segment], local_time, 1)[0]
            yaw_acceleration = _torch_polynomial_derivative(yaw_coefficients[segment], local_time, 2)[0]
            _, collective, body_rate, rotor_thrusts = _torch_flatness_general(
                acceleration,
                jerk,
                snap,
                yaw,
                yaw_rate,
                yaw_acceleration,
                params,
            )
            point_cost = penalties.velocity * _torch_smoothed_l1(
                (velocity * velocity).sum() - limits.max_velocity**2
            )
            point_cost = point_cost + penalties.collective_thrust * (
                _torch_optional_centered_interval_penalty(
                    collective, limits.min_collective_thrust, limits.max_collective_thrust
                )
            )
            point_cost = point_cost + penalties.body_rate * (
                _torch_smoothed_l1((body_rate[:2] ** 2).sum() - limits.max_body_rate_xy**2)
                + _torch_smoothed_l1(body_rate[2] ** 2 - limits.max_body_rate_z**2)
            )
            rotor_cost = durations.new_zeros(())
            for thrust in rotor_thrusts.unbind():
                rotor_cost = rotor_cost + _torch_centered_interval_penalty(
                    thrust, limits.min_rotor_thrust, limits.max_rotor_thrust
                )
            point_cost = point_cost + penalties.rotor_thrust * rotor_cost
            quadrature_weight = 0.5 if node_index in (0, interval_count) else 1.0
            dynamic_cost = dynamic_cost + quadrature_weight * step * point_cost

    collision_cost, minimum_clearance = _collision_terms(
        position_coefficients,
        yaw_coefficients,
        durations,
        track=track,
        collider=collider,
        body_scale=body_scale,
        parameters=params,
        weight=config.collision_weight,
    )
    objective = (
        weights.time * durations.sum()
        + weights.snap_energy * trajectory._torch_snap_energy(position_coefficients, durations)
        + config.yaw_snap_weight * yaw_trajectory.minco._torch_snap_energy(yaw_coefficients, durations)
        + dynamic_cost
        + collision_cost
    )
    gradients = torch.autograd.grad(
        objective, (position_points, yaw_points, durations), allow_unused=True
    )
    point_gradient = (
        np.zeros_like(trajectory.intermediate_points)
        if gradients[0] is None
        else gradients[0].detach().cpu().numpy()
    )
    yaw_gradient = (
        np.zeros(len(yaw_trajectory.waypoints))
        if gradients[1] is None
        else gradients[1].detach().cpu().numpy()[:, 0]
    )
    duration_gradient = (
        np.zeros_like(trajectory.durations)
        if gradients[2] is None
        else gradients[2].detach().cpu().numpy()
    )
    return JointGradientResult(
        cost=float(objective.detach().cpu()),
        point_gradient=point_gradient,
        yaw_gradient=yaw_gradient,
        duration_gradient=duration_gradient,
        minimum_projected_clearance=float(minimum_clearance.detach().cpu()),
        collision_cost=float(collision_cost.detach().cpu()),
    )


__all__ = [
    "JointGradientResult",
    "joint_objective_with_gradients",
    "polygon_boundary_clearance",
    "torch_polygon_boundary_clearance",
]
