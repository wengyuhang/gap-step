"""Experiment-B attitude-induced whole-body collision counterexample."""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile,
    rotation_and_derivative,
)
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import l_shape_boundary

from .geometry import Cuboid, GateFrame, IntersectionMetrics, PlaneSection, exact_intersection_metrics, plane_section


FloatArray = NDArray[np.float64]
WORLD_CLEARANCE = 0.315
COLLISION_AREA_EPSILON = 1.0e-6


@dataclass(frozen=True)
class MethodSnapshot:
    method: str
    time: float
    center_world: FloatArray
    center_plane: FloatArray
    rotation: FloatArray
    section: PlaneSection
    metrics: IntersectionMetrics
    center_legal: bool
    center_normal_distance: float
    boundary_distance: float
    support_half_width: float


@dataclass(frozen=True)
class StressCase:
    boundary_local: FloatArray
    cuboid: Cuboid
    motion: MotionProfile
    center0: FloatArray
    angles0: FloatArray
    crossing_time: float
    collision_time: float
    total_time: float
    start: FloatArray
    old_center_local: FloatArray
    ours_center_local: FloatArray
    normal_amplitude: float

    @property
    def goal(self) -> FloatArray:
        """The requested closed demonstration uses exactly the same endpoint."""

        return self.start.copy()

    def frame_at(self, time: float) -> GateFrame:
        translation, _ = self.motion.translation(float(time))
        angle_delta, angle_rate = self.motion.rotation(float(time))
        scale, _ = self.motion.scale(float(time))
        rotation, _ = rotation_and_derivative(self.angles0 + angle_delta, angle_rate)
        basis = rotation[:, :2]
        normal = np.cross(basis[:, 0], basis[:, 1])
        return GateFrame(self.center0 + translation, basis, normal, scale)

    def body_rotation(self, method: str, time: float) -> FloatArray:
        """Point the prism's longitudinal body-x axis along flight velocity."""

        instant = float(time)
        step = 1.0e-4
        left = max(0.0, instant - step)
        right = min(self.total_time, instant + step)
        velocity = self.planned_position(method, right) - self.planned_position(method, left)
        speed = float(np.linalg.norm(velocity))
        if speed <= 1.0e-10:
            forward = self.frame_at(instant).normal.copy()
        else:
            forward = velocity / speed
        reference = self.frame_at(instant).basis[:, 1]
        transverse = reference - forward * float(reference @ forward)
        if np.linalg.norm(transverse) <= 1.0e-8:
            reference = np.array([0.0, 0.0, 1.0])
            transverse = reference - forward * float(reference @ forward)
        transverse /= np.linalg.norm(transverse)
        third = np.cross(forward, transverse)
        third /= np.linalg.norm(third)
        transverse = np.cross(third, forward)
        return np.column_stack((forward, transverse, third))

    def local_center(self, method: str) -> FloatArray:
        if method == "Old-0.315":
            return self.old_center_local.copy()
        if method == "Ours":
            return self.ours_center_local.copy()
        raise ValueError(f"unknown method {method!r}")

    def crossing_center_world(self, method: str) -> FloatArray:
        frame = self.frame_at(self.crossing_time)
        return frame.center + frame.basis @ (frame.scale * self.local_center(method))

    def world_inset_at(self, time: float) -> FloatArray:
        """Return the scale-aware local polygon for a fixed world 0.315 m inset."""

        frame = self.frame_at(time)
        physical_world_2d = Polygon(frame.boundary_world_2d(self.boundary_local))
        inset = physical_world_2d.buffer(-WORLD_CLEARANCE, join_style="mitre")
        if not isinstance(inset, Polygon) or inset.is_empty:
            raise RuntimeError("fixed world inset is empty or disconnected")
        return np.asarray(inset.exterior.coords[:-1], dtype=float)

    def planned_position(self, method: str, time: float) -> FloatArray:
        """Uninterrupted planned path used to expose pre/post-center collision.

        This is deliberately not reported as an optimized MINCO trajectory.
        It visualizes the same geometric task while the full optimizer study is
        reserved by the experiment protocol.
        """

        instant = float(time)
        if method not in {"Old-0.315", "Ours"}:
            raise ValueError(f"unknown method {method!r}")
        phase = instant / self.total_time
        envelope = np.sin(np.pi * phase) ** 2
        crossing_frame = self.frame_at(self.crossing_time)
        if method == "Old-0.315":
            # Old only chooses the nominal waypoint at t_i.  It does not track
            # the moving/rotating/scaling aperture while the body is crossing.
            target = self.crossing_center_world(method)
        else:
            # Ours completes its lateral correction before the final approach.
            # The terminal approach therefore points the longitudinal body axis
            # through the aperture instead of sliding the prism sideways while
            # it intersects the gate plane.
            target = self.crossing_center_world(method)
        normal_offset = self.normal_amplitude * np.sin(
            2.0 * np.pi * (instant - self.crossing_time) / self.total_time
        )
        return (
            self.start
            + envelope * (target - self.start)
            + envelope * normal_offset * crossing_frame.normal
        )

    def trajectory_position(self, method: str, time: float) -> FloatArray:
        """Executed path; Old freezes at the first detected collision."""

        instant = float(time)
        if method == "Old-0.315":
            instant = min(instant, self.collision_time)
        return self.planned_position(method, instant)

    def snapshot(
        self,
        method: str,
        time: float | None = None,
        *,
        executed: bool = True,
    ) -> MethodSnapshot:
        instant = self.crossing_time if time is None else float(time)
        frame = self.frame_at(instant)
        center = (
            self.trajectory_position(method, instant)
            if executed
            else self.planned_position(method, instant)
        )
        rotation = self.body_rotation(method, instant)
        section = plane_section(self.cuboid, center, rotation, frame)
        boundary_world_2d = frame.boundary_world_2d(self.boundary_local)
        metrics = exact_intersection_metrics(
            section,
            boundary_world_2d,
            area_epsilon=COLLISION_AREA_EPSILON,
        )
        center_plane = frame.world_to_plane(center)
        center_normal_distance = abs(float((center - frame.center) @ frame.normal))
        inset = Polygon(self.world_inset_at(instant))
        center_legal = bool(inset.covers(Point(float(center_plane[0]), float(center_plane[1]))))
        physical = Polygon(boundary_world_2d)
        center_point = Point(float(center_plane[0]), float(center_plane[1]))
        unsigned_distance = float(physical.boundary.distance(center_point))
        boundary_distance = unsigned_distance if physical.covers(center_point) else -unsigned_distance
        boundary_normal_world = frame.basis[:, 0]
        support = float(
            np.sum(
                self.cuboid.half_extents
                * np.abs(rotation.T @ boundary_normal_world)
            )
        )
        return MethodSnapshot(
            method,
            instant,
            center,
            center_plane,
            rotation,
            section,
            metrics,
            center_legal,
            center_normal_distance,
            float(boundary_distance),
            support,
        )


def build_stress_case() -> StressCase:
    """Build one deterministic member of the canonical E4 motion family."""

    boundary = l_shape_boundary().vertices.copy()
    motion = MotionProfile(
        translation_amplitude=np.array([0.12, 0.28, 0.20]),
        rotation_amplitude=np.array([0.16, 0.11, 0.09]),
        scale_amplitude=0.12,
        translation_period=6.5,
        rotation_period=7.5,
        scale_period=8.5,
        phase=0.0,
    )
    # At this phase the moving frame changes enough during a deliberately slow
    # body crossing to expose the difference between a waypoint-only constraint
    # and continuous whole-body safety.
    crossing = 8.8
    total = 2.0 * crossing
    cuboid = Cuboid.repository_default()
    scale, _ = motion.scale(crossing)
    old_distance = WORLD_CLEARANCE + 0.001
    # The square rotor-plane footprint needs clearance on both in-plane axes.
    # Ours completes this lateral shift before the nose-first final approach.
    ours_distance = 0.800
    old_local = np.array([2.0 - old_distance / scale, -1.60])
    ours_local = np.array([2.0 - ours_distance / scale, -1.20])
    provisional = StressCase(
        boundary_local=boundary,
        cuboid=cuboid,
        motion=motion,
        center0=np.array([0.0, 0.0, 1.8]),
        angles0=np.array([0.0, np.pi / 2.0, 0.0]),
        crossing_time=crossing,
        collision_time=total,
        total_time=total,
        start=np.array([-4.6, -3.6, 3.0]),
        old_center_local=old_local,
        ours_center_local=ours_local,
        # About 0.07 m/s normal speed at t_i, leaving the body and plane in
        # contact long enough for the canonical E4 gate motion to matter.
        normal_amplitude=0.196,
    )

    nominal = provisional.snapshot("Old-0.315", crossing, executed=False)
    if not nominal.center_legal or nominal.boundary_distance < WORLD_CLEARANCE:
        raise RuntimeError("nominal Old waypoint violates the fixed world clearance")
    if nominal.metrics.whole_body_collision:
        raise RuntimeError("nominal Old center crossing must be whole-body safe")

    grid = np.linspace(crossing - 1.0, crossing, 1001)
    flags = [
        provisional.snapshot("Old-0.315", float(time), executed=False).metrics.whole_body_collision
        for time in grid
    ]
    first = next((index for index, value in enumerate(flags) if value), None)
    if first is None or first == 0:
        raise RuntimeError("failed to bracket a first pre-crossing dynamic collision")
    left, right = float(grid[first - 1]), float(grid[first])
    for _ in range(50):
        middle = 0.5 * (left + right)
        collision = provisional.snapshot(
            "Old-0.315", middle, executed=False
        ).metrics.whole_body_collision
        if collision:
            right = middle
        else:
            left = middle
    return replace(provisional, collision_time=right)


__all__ = [
    "COLLISION_AREA_EPSILON",
    "WORLD_CLEARANCE",
    "MethodSnapshot",
    "StressCase",
    "build_stress_case",
]
