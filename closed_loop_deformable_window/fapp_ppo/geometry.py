"""Continuous non-periodic window motion, deformation, and crossing geometry."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry


Array = np.ndarray


def normalize(vector: Array, eps: float = 1.0e-9) -> Array:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm <= eps:
        raise ValueError("cannot normalize a near-zero vector")
    return value / norm


def rotation_from_normal(normal: Array, up_hint: Array | None = None) -> Array:
    """Return local-to-world rotation whose local z axis is ``normal``."""

    z_axis = normalize(normal)
    hint = np.array([0.0, 0.0, 1.0]) if up_hint is None else normalize(up_hint)
    if abs(float(np.dot(z_axis, hint))) > 0.95:
        hint = np.array([0.0, 1.0, 0.0])
    x_axis = normalize(np.cross(hint, z_axis))
    y_axis = normalize(np.cross(z_axis, x_axis))
    return np.column_stack((x_axis, y_axis, z_axis))


def signed_margin(polygon: BaseGeometry, point: Array) -> float:
    if polygon.is_empty:
        return float("-inf")
    query = Point(float(point[0]), float(point[1]))
    distance = float(polygon.boundary.distance(query))
    return distance if polygon.covers(query) else -distance


def _largest_polygon(geometry: BaseGeometry) -> Polygon:
    if geometry.is_empty:
        return Polygon()
    if geometry.geom_type == "Polygon":
        return geometry
    if geometry.geom_type == "MultiPolygon":
        return max(geometry.geoms, key=lambda item: item.area)
    raise ValueError(f"unsupported safety geometry {geometry.geom_type!r}")


@dataclass(frozen=True)
class WindowState:
    center: Array
    rotation: Array
    boundary: Array
    polygon: Polygon
    safe_polygon: Polygon

    @property
    def normal(self) -> Array:
        return self.rotation[:, 2]

    @property
    def safe_anchor_local(self) -> Array:
        region = self.polygon if self.safe_polygon.is_empty else self.safe_polygon
        centroid = region.centroid
        if not region.covers(centroid):
            centroid = region.representative_point()
        return np.array([centroid.x, centroid.y], dtype=float)

    @property
    def safe_anchor_world(self) -> Array:
        local = np.array([*self.safe_anchor_local, 0.0])
        return self.center + self.rotation @ local

    def world_to_local(self, point: Array) -> Array:
        return self.rotation.T @ (np.asarray(point, dtype=float) - self.center)


@dataclass(frozen=True)
class CrossingEvent:
    occurred: bool
    time: float | None = None
    point: Array | None = None
    local_point: Array | None = None
    safe: bool = False
    frame_collision: bool = False
    margin: float = float("-inf")


class DeformableWindow:
    """Planar window from continuously interpolated poses and boundary samples.

    The simulator fixes all keyframes at reset. Natural cubic splines make the
    pose and every boundary point continuously queryable between keyframes.
    Procedural keyframes use a positive radial graph, which preserves a simple,
    connected, hole-free polygon while allowing local non-rigid deformation.
    """

    def __init__(
        self,
        *,
        name: str,
        keyframe_times: Array,
        centers: Array,
        rotation_vectors: Array,
        boundary_keyframes: Array,
        safe_margin: float,
        frame_thickness: float,
        planned_opportunities: tuple[tuple[float, float], ...] | None = None,
        minimum_safe_area: float = 1.0e-4,
    ):
        self.name = str(name)
        self.keyframe_times = np.asarray(keyframe_times, dtype=float)
        self.centers = np.asarray(centers, dtype=float)
        self.rotation_vectors = np.asarray(rotation_vectors, dtype=float)
        self.boundary_keyframes = np.asarray(boundary_keyframes, dtype=float)
        self.safe_margin = float(safe_margin)
        self.frame_thickness = float(frame_thickness)
        self.minimum_safe_area = float(minimum_safe_area)
        keyframes = len(self.keyframe_times)
        if keyframes < 3 or np.any(np.diff(self.keyframe_times) <= 0.0):
            raise ValueError("window keyframe times must be strictly increasing")
        if self.centers.shape != (keyframes, 3):
            raise ValueError("centers must have shape [keyframes, 3]")
        if self.rotation_vectors.shape != (keyframes, 3):
            raise ValueError("rotation_vectors must have shape [keyframes, 3]")
        if self.boundary_keyframes.ndim != 3 or self.boundary_keyframes.shape[:1] != (keyframes,):
            raise ValueError("boundary_keyframes must have shape [keyframes, vertices, 2]")
        if self.boundary_keyframes.shape[1] < 8 or self.boundary_keyframes.shape[2] != 2:
            raise ValueError("each boundary requires at least eight 2D vertices")
        if self.safe_margin <= 0.0 or self.frame_thickness <= 0.0:
            raise ValueError("window safety and frame widths must be positive")
        if self.minimum_safe_area < 0.0:
            raise ValueError("minimum_safe_area must be non-negative")
        horizon = (float(self.keyframe_times[0]), float(self.keyframe_times[-1]))
        opportunities = (
            (horizon,)
            if planned_opportunities is None
            else tuple((float(start), float(end)) for start, end in planned_opportunities)
        )
        if any(
            start < horizon[0] or end > horizon[1] or end <= start
            for start, end in opportunities
        ):
            raise ValueError("planned opportunities must be ordered intervals inside the horizon")
        if any(
            opportunities[index][1] > opportunities[index + 1][0]
            for index in range(len(opportunities) - 1)
        ):
            raise ValueError("planned opportunities must not overlap")
        self.planned_opportunities = opportunities
        self._center_spline = CubicSpline(self.keyframe_times, self.centers, axis=0, bc_type="natural")
        self._rotation_spline = CubicSpline(
            self.keyframe_times, self.rotation_vectors, axis=0, bc_type="natural"
        )
        self._boundary_spline = CubicSpline(
            self.keyframe_times, self.boundary_keyframes, axis=0, bc_type="natural"
        )
        self._validate_keyframes()

    def _validate_keyframes(self) -> None:
        sample_count = max(25, 2 * len(self.keyframe_times))
        has_passable_state = False
        for time in np.linspace(
            self.keyframe_times[0], self.keyframe_times[-1], sample_count
        ):
            boundary = np.asarray(self._boundary_spline(float(time)), dtype=float)
            polygon = Polygon(boundary)
            if not polygon.is_valid or polygon.is_empty or polygon.area <= 1.0e-4:
                raise ValueError(f"{self.name}: deformation produced invalid topology at t={time:.3f}")
            safe = _largest_polygon(polygon.buffer(-self.safe_margin, join_style=2))
            has_passable_state = has_passable_state or safe.area >= self.minimum_safe_area
        if self.planned_opportunities and not has_passable_state:
            raise ValueError(f"{self.name}: no sampled state has a usable safe traversal region")

    def state(self, time: float) -> WindowState:
        clipped = float(np.clip(time, self.keyframe_times[0], self.keyframe_times[-1]))
        center = np.asarray(self._center_spline(clipped), dtype=float)
        rotvec = np.asarray(self._rotation_spline(clipped), dtype=float)
        rotation = Rotation.from_rotvec(rotvec).as_matrix()
        boundary = np.asarray(self._boundary_spline(clipped), dtype=float)
        polygon = Polygon(boundary)
        safe = _largest_polygon(polygon.buffer(-self.safe_margin, join_style=2))
        if not polygon.is_valid:
            raise RuntimeError(f"{self.name}: invalid interpolated state at t={time:.3f}")
        return WindowState(center, rotation, boundary, polygon, safe)

    def is_passable_state(self, state: WindowState) -> bool:
        return bool(
            not state.safe_polygon.is_empty
            and state.safe_polygon.area >= self.minimum_safe_area
        )

    def is_passable(self, time: float) -> bool:
        return self.is_passable_state(self.state(time))

    def next_opportunity(self, time: float) -> tuple[float, float] | None:
        for interval in self.planned_opportunities:
            if interval[1] >= time:
                return interval
        return None

    def containing_opportunity(self, time: float) -> tuple[float, float] | None:
        for interval in self.planned_opportunities:
            if interval[0] <= time <= interval[1]:
                return interval
        return None

    def opportunity_features(self, time: float, horizon: float) -> Array:
        interval = self.next_opportunity(time)
        passable = float(self.is_passable(time))
        if interval is None:
            return np.array([passable, 0.0, 1.0, 0.0], dtype=float)
        start, end = interval
        active = float(start <= time <= end)
        time_to_open = max(0.0, start - time) / max(horizon, 1.0e-6)
        time_to_close = max(0.0, end - time) / max(horizon, 1.0e-6)
        return np.array(
            [
                passable,
                active,
                np.clip(time_to_open, 0.0, 1.0),
                np.clip(time_to_close, 0.0, 1.0),
            ],
            dtype=float,
        )

    def center_velocity(self, time: float) -> Array:
        clipped = float(np.clip(time, self.keyframe_times[0], self.keyframe_times[-1]))
        return np.asarray(self._center_spline(clipped, 1), dtype=float)

    def boundary_signature(self, time: float, samples: int = 8) -> Array:
        state = self.state(time)
        anchor = state.safe_anchor_local
        relative = state.boundary - anchor[None, :]
        angles = np.arctan2(relative[:, 1], relative[:, 0])
        distances = np.linalg.norm(relative, axis=1)
        signature = []
        for angle in np.linspace(-np.pi, np.pi, samples, endpoint=False):
            delta = np.abs(np.angle(np.exp(1j * (angles - angle))))
            signature.append(float(distances[int(np.argmin(delta))]))
        return np.asarray(signature, dtype=float)

    def crossing_event(
        self,
        previous_point: Array,
        previous_time: float,
        point: Array,
        time: float,
    ) -> CrossingEvent:
        previous_state = self.state(previous_time)
        current_state = self.state(time)
        d0 = float(np.dot(previous_point - previous_state.center, previous_state.normal))
        d1 = float(np.dot(point - current_state.center, current_state.normal))
        if d0 == 0.0 and d1 == 0.0:
            return CrossingEvent(False)
        if d0 * d1 > 0.0:
            return CrossingEvent(False)
        denominator = abs(d0) + abs(d1)
        alpha = 0.5 if denominator <= 1.0e-9 else abs(d0) / denominator
        crossing_time = float(previous_time + alpha * (time - previous_time))
        crossing_point = np.asarray(previous_point + alpha * (point - previous_point), dtype=float)
        crossing_state = self.state(crossing_time)
        local3 = crossing_state.world_to_local(crossing_point)
        local2 = local3[:2]
        margin = signed_margin(crossing_state.safe_polygon, local2)
        safe = margin > 0.0 and self.is_passable_state(crossing_state)
        query = Point(float(local2[0]), float(local2[1]))
        near_frame = crossing_state.polygon.boundary.distance(query) <= self.frame_thickness
        unsafe_inside = crossing_state.polygon.covers(query) and not safe
        return CrossingEvent(
            occurred=True,
            time=crossing_time,
            point=crossing_point,
            local_point=local2,
            safe=safe,
            frame_collision=bool(near_frame or unsafe_inside),
            margin=float(margin),
        )


def radial_boundary(
    *,
    vertices: int,
    radius_x: float,
    radius_y: float,
    coefficients: Array,
) -> Array:
    """Build a topology-preserving, locally deformable non-convex boundary."""

    theta = np.linspace(0.0, 2.0 * np.pi, vertices, endpoint=False)
    c = np.asarray(coefficients, dtype=float)
    if c.shape != (5,):
        raise ValueError("radial boundary expects five deformation coefficients")
    radial = (
        1.0
        + c[0] * np.cos(2.0 * theta)
        + c[1] * np.sin(3.0 * theta)
        + c[2] * np.cos(5.0 * theta)
        + c[3] * np.sin(theta)
        + c[4] * np.sin(4.0 * theta)
    )
    if float(np.min(radial)) <= 0.28:
        raise ValueError("deformation coefficients make the radial graph unsafe")
    return np.column_stack((radius_x * radial * np.cos(theta), radius_y * radial * np.sin(theta)))
