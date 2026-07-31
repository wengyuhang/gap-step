"""Dense MDG trajectory, gate, frame, and quadrotor validation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np
from shapely.geometry import Point

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    QuadrotorParameters,
    flatness_from_trajectory,
)

from .backend_adapter import BackendResult, SelectedDiscConstraint
from .config import MDGConfig
from .dynamic_gate import Scenario


def _point_polyline_distance_3d(point: np.ndarray, vertices: np.ndarray) -> float:
    following = np.roll(vertices, -1, axis=0)
    segments = following - vertices
    denominator = np.einsum("ij,ij->i", segments, segments)
    fraction = np.divide(
        np.einsum("ij,ij->i", point - vertices, segments),
        denominator,
        out=np.zeros(len(vertices)),
        where=denominator > 0.0,
    )
    closest = vertices + np.clip(fraction, 0.0, 1.0)[:, None] * segments
    return float(np.min(np.linalg.norm(closest - point, axis=1)))


@dataclass
class ValidationReport:
    success: bool
    gate_plane_error: float
    minimum_gate_clearance: float
    maximum_speed: float
    maximum_acceleration: float
    maximum_body_rate: float
    maximum_rotor_thrust: float
    minimum_rotor_thrust: float
    closed_loop_position_error: float
    closed_loop_velocity_error: float
    closed_loop_attitude_error_deg: float
    closed_loop_body_rate_error: float
    selected_point_offset_ratio: list[float]
    designated_order_legal: bool
    exactly_once: bool
    dynamics_legal: bool
    frame_collision_free: bool
    interval_legal: bool
    trajectory_continuous: bool
    worst_segment: int
    failure_reasons: list[str]
    detected_crossings: list[tuple[float, int]]

    def to_dict(self) -> dict[str, Any]:
        return self.__dict__.copy()


def validate_trajectory(
    scenario: Scenario,
    constraints: list[SelectedDiscConstraint],
    backend: BackendResult,
    config: MDGConfig,
) -> ValidationReport:
    settings = config.validation
    limits = DynamicLimits()
    params = QuadrotorParameters()
    trajectory = backend.trajectory
    plane_errors: list[float] = []
    clearances: list[float] = []
    order_legal = bool(
        len(backend.traversal_times) == len(scenario.gates)
        and np.all(np.diff(backend.traversal_times) > 0.0)
        and backend.traversal_times[0] > 0.0
        and backend.traversal_times[-1] < backend.total_time
    )
    interval_legal = (
        backend.interval_violation <= settings.interval_time_tolerance
    )
    designated = order_legal
    for index, (constraint, time) in enumerate(
        zip(constraints, backend.traversal_times)
    ):
        point = np.real(trajectory.evaluate(float(time)))
        local, plane_error = constraint.gate.world_to_local(point, float(time))
        plane_errors.append(plane_error)
        safe = constraint.gate.safe_polygon(
            float(time), config.safety.safety_radius
        )
        inside = bool(safe.buffer(1.0e-9).covers(Point(float(local[0]), float(local[1]))))
        waypoint_error = float(np.linalg.norm(point - backend.waypoints[index]))
        designated &= (
            inside
            and plane_error <= settings.gate_plane_error_max
            and waypoint_error <= settings.waypoint_tolerance
            and constraint.track.active_at(
                float(time), tolerance=settings.interval_time_tolerance
            )
        )
        physical = constraint.gate.local_polygon(float(time))
        clearances.append(
            float(Point(float(local[0]), float(local[1])).distance(physical.boundary))
            - config.safety.safety_radius
        )

    samples = trajectory.sample(
        samples_per_segment=config.backend.validation_samples_per_segment
    )
    speed = np.linalg.norm(np.real(samples.velocity), axis=1)
    acceleration = np.linalg.norm(np.real(samples.acceleration), axis=1)
    maximum_body_rate = 0.0
    maximum_rotor = -float("inf")
    minimum_rotor = float("inf")
    worst_segment = 0
    worst_score = -float("inf")
    for segment, duration in enumerate(backend.durations):
        local_times = np.linspace(
            0.0, float(duration), config.backend.validation_samples_per_segment
        )
        segment_score = 0.0
        for local_time in local_times:
            velocity = np.real(trajectory.evaluate_segment(segment, local_time, 1))
            state = flatness_from_trajectory(
                trajectory,
                float(np.sum(backend.durations[:segment]) + local_time),
                yaw=scenario.start.yaw,
                parameters=params,
            )
            body = float(np.max(np.abs(np.real(state.body_rate))))
            rotor_values = np.real(state.rotor_thrusts)
            maximum_body_rate = max(maximum_body_rate, body)
            maximum_rotor = max(maximum_rotor, float(np.max(rotor_values)))
            minimum_rotor = min(minimum_rotor, float(np.min(rotor_values)))
            segment_score = max(
                segment_score,
                np.linalg.norm(velocity) / limits.max_velocity,
                body / max(limits.max_body_rate_xy, limits.max_body_rate_z),
                float(np.max(rotor_values)) / limits.max_rotor_thrust,
                limits.min_rotor_thrust
                / max(float(np.min(rotor_values)), 1.0e-9),
            )
        if segment_score > worst_score:
            worst_score = segment_score
            worst_segment = segment
    dynamic_tolerance = settings.dynamic_relative_tolerance
    dynamics_legal = bool(
        float(np.max(speed))
        <= limits.max_velocity * (1.0 + dynamic_tolerance)
        and maximum_body_rate
        <= max(limits.max_body_rate_xy, limits.max_body_rate_z)
        * (1.0 + dynamic_tolerance)
        and maximum_rotor
        <= limits.max_rotor_thrust * (1.0 + dynamic_tolerance)
        and minimum_rotor
        >= limits.min_rotor_thrust * (1.0 - dynamic_tolerance)
    )

    endpoint_position = np.real(trajectory.evaluate(backend.total_time, 0))
    endpoint_velocity = np.real(trajectory.evaluate(backend.total_time, 1))
    position_error = float(
        np.linalg.norm(endpoint_position - scenario.start.position)
    )
    velocity_error = float(
        np.linalg.norm(endpoint_velocity - scenario.start.velocity)
    )
    start_flat = flatness_from_trajectory(
        trajectory, 0.0, yaw=scenario.start.yaw, parameters=params
    )
    end_flat = flatness_from_trajectory(
        trajectory,
        backend.total_time,
        yaw=scenario.start.yaw,
        parameters=params,
    )
    relative_rotation = np.real(start_flat.rotation).T @ np.real(end_flat.rotation)
    attitude_error = math.degrees(
        math.acos(float(np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)))
    )
    body_rate_error = float(
        np.linalg.norm(np.real(start_flat.body_rate) - np.real(end_flat.body_rate))
    )
    continuous = True
    elapsed = 0.0
    for segment, duration in enumerate(backend.durations[:-1]):
        elapsed += float(duration)
        for derivative in range(4):
            left = trajectory.evaluate_segment(segment, float(duration), derivative)
            right = trajectory.evaluate_segment(segment + 1, 0.0, derivative)
            continuous &= bool(np.allclose(left, right, atol=2.0e-7))

    # The physical entity is a zero-thickness planar frame.  Its swept-sphere
    # collision condition is evaluated at every refined plane intersection;
    # the safety inset already includes vehicle and tracking radii.
    detected: list[tuple[float, int]] = []
    crossing_clearances: list[float] = []
    grid = np.linspace(0.0, backend.total_time, max(501, 50 * len(scenario.gates)))
    positions = np.asarray([trajectory.evaluate(float(time)) for time in grid]).real
    for gate_index, gate in enumerate(scenario.gates):
        signed = []
        for time, point in zip(grid, positions):
            center, rotation = gate.pose(float(time))
            signed.append(float((point - center) @ rotation[:, 2]))
        signed_array = np.asarray(signed)
        for index in np.flatnonzero(signed_array[:-1] * signed_array[1:] <= 0.0):
            denominator = abs(signed_array[index]) + abs(signed_array[index + 1])
            fraction = 0.5 if denominator <= 1.0e-12 else abs(signed_array[index]) / denominator
            time = float(grid[index] + fraction * (grid[index + 1] - grid[index]))
            point = trajectory.evaluate(time).real
            local, plane = gate.world_to_local(point, time)
            if plane <= 0.01 and gate.local_polygon(time).buffer(1.0e-7).covers(
                Point(float(local[0]), float(local[1]))
            ):
                detected.append((time, gate_index))
                crossing_clearances.append(
                    float(Point(float(local[0]), float(local[1])).distance(
                        gate.local_polygon(time).boundary
                    ))
                    - config.safety.safety_radius
                )
    detected.sort()
    deduplicated: list[tuple[float, int]] = []
    for item in detected:
        if not deduplicated or item[1] != deduplicated[-1][1] or item[0] - deduplicated[-1][0] > 0.1:
            deduplicated.append(item)
    extra_crossings = [
        item
        for item in deduplicated
        if not any(
            item[1] == gate_index
            and abs(item[0] - float(backend.traversal_times[gate_index]))
            <= 0.10
            for gate_index in scenario.order
        )
    ]
    exactly_once = bool(designated and not extra_crossings)
    if extra_crossings:
        cumulative = np.cumsum(backend.durations)
        worst_segment = int(
            np.clip(
                np.searchsorted(cumulative, extra_crossings[0][0], side="right"),
                0,
                len(backend.durations) - 1,
            )
        )
    frame_clearance = min(crossing_clearances, default=float("inf"))
    frame_collision_free = bool(
        exactly_once
        and frame_clearance >= settings.minimum_clearance_tolerance
    )

    max_plane = max(plane_errors, default=float("inf"))
    min_clearance = min(clearances + crossing_clearances, default=-float("inf"))
    closed_state_legal = (
        position_error <= settings.closed_loop_position_error_max
        and velocity_error <= settings.closed_loop_velocity_error_max
        and attitude_error <= settings.closed_loop_attitude_error_deg_max
        and body_rate_error <= settings.closed_loop_body_rate_error_max
    )
    reasons: list[str] = []
    for condition, reason in (
        (backend.success, "optimizer_failed"),
        (designated, "designated_crossing_invalid"),
        (exactly_once, "window_crossing_count_or_order_invalid"),
        (dynamics_legal, "sampled_dynamic_limit_violation"),
        (frame_collision_free, "gate_frame_collision"),
        (interval_legal, "disc_track_interval_violation"),
        (continuous, "trajectory_discontinuity"),
        (closed_state_legal, "closed_loop_state_error"),
        (min_clearance >= settings.minimum_clearance_tolerance, "clearance_violation"),
    ):
        if not condition:
            reasons.append(reason)
    return ValidationReport(
        success=not reasons,
        gate_plane_error=max_plane,
        minimum_gate_clearance=min_clearance,
        maximum_speed=float(np.max(speed)),
        maximum_acceleration=float(np.max(acceleration)),
        maximum_body_rate=maximum_body_rate,
        maximum_rotor_thrust=maximum_rotor,
        minimum_rotor_thrust=minimum_rotor,
        closed_loop_position_error=position_error,
        closed_loop_velocity_error=velocity_error,
        closed_loop_attitude_error_deg=attitude_error,
        closed_loop_body_rate_error=body_rate_error,
        selected_point_offset_ratio=backend.selected_point_offset_ratio.tolist(),
        designated_order_legal=designated,
        exactly_once=exactly_once,
        dynamics_legal=dynamics_legal,
        frame_collision_free=frame_collision_free,
        interval_legal=interval_legal,
        trajectory_continuous=continuous,
        worst_segment=worst_segment,
        failure_reasons=reasons,
        detected_crossings=deduplicated,
    )


__all__ = ["ValidationReport", "validate_trajectory"]
