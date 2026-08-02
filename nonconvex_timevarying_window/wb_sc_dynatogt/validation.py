"""Straightforward cuboid-at-crossing validation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from shapely.geometry import MultiPoint, Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.dynamics import flatness_from_trajectory
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack

from .collider import CuboidCollider


@dataclass(frozen=True)
class SafetyViolation:
    crossing_index: int
    window_name: str
    time: float
    clearance: float
    reason: str

    def to_dict(self) -> dict[str, object]:
        return {
            "crossing_index": self.crossing_index,
            "window_name": self.window_name,
            "time": self.time,
            "clearance": self.clearance,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class CuboidCrossingCheck:
    crossing_index: int
    window_name: str
    time: float
    roll: float
    pitch: float
    yaw: float
    clearance: float
    safe: bool
    local_corners: tuple[tuple[float, float], ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "crossing_index": self.crossing_index,
            "window_name": self.window_name,
            "time": self.time,
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "clearance": self.clearance,
            "safe": self.safe,
            "local_corners": [list(point) for point in self.local_corners],
        }


@dataclass(frozen=True)
class AttitudeSafetyReport:
    safe: bool
    minimum_clearance: float
    checks: tuple[CuboidCrossingCheck, ...]
    violations: tuple[SafetyViolation, ...]
    sampling_statement: str = (
        "oriented cuboid projection checked at each prescribed crossing; "
        "not a continuous-time collision certificate"
    )

    @property
    def checked_time_count(self) -> int:
        return len(self.checks)

    def to_dict(self) -> dict[str, object]:
        return {
            "safe": self.safe,
            "minimum_clearance": self.minimum_clearance,
            "checked_time_count": self.checked_time_count,
            "checks": [item.to_dict() for item in self.checks],
            "violations": [item.to_dict() for item in self.violations],
            "sampling_statement": self.sampling_statement,
        }


@dataclass(frozen=True)
class SphereCrossingCheck:
    crossing_index: int
    window_name: str
    time: float
    clearance: float
    safe: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "crossing_index": self.crossing_index,
            "window_name": self.window_name,
            "time": self.time,
            "clearance": self.clearance,
            "safe": self.safe,
        }


@dataclass(frozen=True)
class SphereSafetyReport:
    """Safety under the legacy SC model's own 0.315 m local inset."""

    safe: bool
    minimum_clearance: float
    checks: tuple[SphereCrossingCheck, ...]
    violations: tuple[SafetyViolation, ...]
    sampling_statement: str = (
        "trajectory center checked against the legacy 0.315 m local gate inset "
        "at each prescribed crossing"
    )

    @property
    def checked_time_count(self) -> int:
        return len(self.checks)

    def to_dict(self) -> dict[str, object]:
        return {
            "safe": self.safe,
            "minimum_clearance": self.minimum_clearance,
            "checked_time_count": self.checked_time_count,
            "checks": [item.to_dict() for item in self.checks],
            "violations": [item.to_dict() for item in self.violations],
            "sampling_statement": self.sampling_statement,
        }


def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
    roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return roll, pitch, yaw


def _corner_clearance(corners: np.ndarray, polygon: Polygon, margin: float) -> float:
    values = []
    for corner in corners:
        point = Point(float(corner[0]), float(corner[1]))
        distance = float(polygon.boundary.distance(point))
        values.append((distance if polygon.covers(point) else -distance) - margin)
    return float(min(values))


def validate_whole_body(
    track: SCWindowTrack,
    result: Any,
    *,
    collider: CuboidCollider | None = None,
) -> AttitudeSafetyReport:
    """Check the full projected cuboid at every prescribed crossing time."""

    body = CuboidCollider(result.config.collider) if collider is None else collider
    checks: list[CuboidCrossingCheck] = []
    violations: list[SafetyViolation] = []
    minimum = float("inf")
    for crossing_index, window_index in enumerate(track.order):
        time = float(result.traversal_times[crossing_index])
        position = np.asarray(result.trajectory.evaluate(time), dtype=float)
        yaw = float(result.yaw_trajectory.evaluate(time))
        state = flatness_from_trajectory(
            result.trajectory,
            time,
            yaw=yaw,
            yaw_rate=float(result.yaw_trajectory.evaluate(time, 1)),
            yaw_acceleration=float(result.yaw_trajectory.evaluate(time, 2)),
            parameters=result.config.quadrotor,
        )
        rotation = np.asarray(np.real(state.rotation), dtype=float)
        roll, pitch, actual_yaw = _rpy(rotation)
        window = track.windows[window_index]
        center, basis, scale, *_ = window.state_at(time)
        world_corners = position[None, :] + body.corners @ rotation.T
        local_corners = (world_corners - center[None, :]) @ basis / scale
        boundary = window.physical_boundary if window.physical_boundary is not None else window.safe_polygon
        polygon = Polygon(boundary)
        local_margin = body.config.clearance / float(scale)
        allowed = polygon.buffer(-local_margin, join_style="mitre")
        projection = MultiPoint(local_corners).convex_hull
        safe = bool(not allowed.is_empty and allowed.covers(projection))
        clearance = float(scale) * _corner_clearance(local_corners, polygon, local_margin)
        if not safe:
            clearance = min(clearance, -1.0e-12)
            violations.append(
                SafetyViolation(
                    crossing_index,
                    window.name,
                    time,
                    clearance,
                    "projected cuboid is not fully contained in the physical aperture",
                )
            )
        minimum = min(minimum, clearance)
        checks.append(
            CuboidCrossingCheck(
                crossing_index,
                window.name,
                time,
                roll,
                pitch,
                actual_yaw,
                clearance,
                safe,
                tuple(tuple(float(value) for value in point) for point in local_corners),
            )
        )
    return AttitudeSafetyReport(not violations, minimum, tuple(checks), tuple(violations))


def validate_legacy_sphere(
    track: SCWindowTrack,
    result: Any,
    *,
    local_radius: float = 0.315,
) -> SphereSafetyReport:
    """Validate the original algorithm using its own fixed local inset."""

    checks: list[SphereCrossingCheck] = []
    violations: list[SafetyViolation] = []
    minimum = float("inf")
    for crossing_index, window_index in enumerate(track.order):
        time = float(result.traversal_times[crossing_index])
        position = np.asarray(result.trajectory.evaluate(time), dtype=float)
        window = track.windows[window_index]
        center, basis, scale, *_ = window.state_at(time)
        local_center = (position - center) @ basis / scale
        boundary = (
            window.physical_boundary
            if window.physical_boundary is not None
            else window.safe_polygon
        )
        polygon = Polygon(boundary)
        point = Point(float(local_center[0]), float(local_center[1]))
        signed_local = float(polygon.boundary.distance(point))
        if not polygon.covers(point):
            signed_local = -signed_local
        clearance = float(scale) * (signed_local - float(local_radius))
        safe = bool(clearance >= -1.0e-10)
        if not safe:
            violations.append(
                SafetyViolation(
                    crossing_index,
                    window.name,
                    time,
                    clearance,
                    "trajectory center violates the legacy 0.315 m local sphere inset",
                )
            )
        minimum = min(minimum, clearance)
        checks.append(
            SphereCrossingCheck(
                crossing_index,
                window.name,
                time,
                clearance,
                safe,
            )
        )
    return SphereSafetyReport(not violations, minimum, tuple(checks), tuple(violations))


__all__ = [
    "AttitudeSafetyReport",
    "CuboidCrossingCheck",
    "SafetyViolation",
    "SphereCrossingCheck",
    "SphereSafetyReport",
    "validate_legacy_sphere",
    "validate_whole_body",
]
