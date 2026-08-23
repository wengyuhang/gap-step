"""Attitude-aware cuboid/gate-plane intersections and crossing intervals."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    FlatnessState,
    QuadrotorParameters,
    YawProfile,
    constant_yaw_profile,
    flatness_from_trajectory,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCDynamicWindow
from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap

from .body_model import CuboidBody
from .config import WholeBodySafetyConfig
from .gate_frame import GateFrame, frame_at


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class SectionVertex:
    """One cuboid-edge/gate-plane intersection in world and gate coordinates."""

    world: FloatArray
    local: FloatArray
    source_body_edge: tuple[int, int]
    alpha: float


@dataclass(frozen=True)
class PlaneSection:
    """Ordered convex section polygon at one instant."""

    time: float
    vertices: tuple[SectionVertex, ...]
    topology_key: tuple[tuple[int, int], ...]
    degenerate: bool

    @property
    def local_polygon(self) -> FloatArray:
        """Return ordered local vertices with shape ``(n,2)``."""

        if not self.vertices:
            return np.empty((0, 2), dtype=float)
        return np.stack([vertex.local for vertex in self.vertices])


@dataclass(frozen=True)
class CrossingInterval:
    """Connected cuboid/plane intersection component containing ``traversal_time``."""

    window_index: int
    traversal_time: float
    start: float
    end: float
    direction: int


@dataclass(frozen=True)
class TopologyInterval:
    """A time interval whose endpoint/midpoint section topology agrees."""

    start: float
    end: float
    topology_key: tuple[tuple[int, int], ...]


def cuboid_world_vertices(
    trajectory: MincoSnap,
    time: float,
    body: CuboidBody,
    *,
    yaw_profile: YawProfile | None = None,
    parameters: QuadrotorParameters | None = None,
) -> tuple[FloatArray, FlatnessState]:
    """Return world cuboid vertices and the flatness state at ``time``."""

    instant = float(time)
    total = float(np.real(trajectory.total_time))
    if not np.isfinite(instant) or instant < -1e-12 or instant > total + 1e-12:
        raise ValueError("time must lie on the trajectory")
    profile = constant_yaw_profile() if yaw_profile is None else yaw_profile
    yaw, yaw_rate, yaw_acceleration = profile(instant)
    state = flatness_from_trajectory(
        trajectory,
        instant,
        yaw=yaw,
        yaw_rate=yaw_rate,
        yaw_acceleration=yaw_acceleration,
        parameters=parameters,
    )
    rotation = np.asarray(np.real(state.rotation), dtype=float)
    position = np.asarray(np.real(trajectory.evaluate(instant)), dtype=float)
    vertices = position[None, :] + body.vertices_body @ rotation.T
    if not np.all(np.isfinite(vertices)):
        raise FloatingPointError("flatness produced non-finite cuboid vertices")
    return vertices, state


def gate_local_vertex_coordinates(vertices_world: ArrayLike, frame: GateFrame) -> FloatArray:
    """Return the eight vertices in the complete gate frame; column 2 is ``xi^3``."""

    vertices = np.asarray(vertices_world, dtype=float)
    if vertices.shape != (8, 3) or not np.all(np.isfinite(vertices)):
        raise ValueError("vertices_world must be finite with shape (8,3)")
    relative = vertices - frame.center[None, :]
    planar = relative @ frame.basis / frame.scale
    normal = relative @ frame.normal
    return np.column_stack((planar, normal))


def has_plane_section(
    xi3: ArrayLike,
    epsilon: float,
    *,
    include_contact: bool = True,
) -> bool:
    """Test whether scalar vertex coordinates straddle the plane.

    This deliberately uses the two extrema, not a multi-input XOR.
    """

    values = np.asarray(xi3, dtype=float)
    eps = float(epsilon)
    if values.shape != (8,) or not np.all(np.isfinite(values)):
        raise ValueError("xi3 must be finite with shape (8,)")
    if not np.isfinite(eps) or eps < 0.0:
        raise ValueError("epsilon must be finite and nonnegative")
    if include_contact:
        return bool(np.min(values) <= eps and np.max(values) >= -eps)
    return bool(np.any(values > eps) and np.any(values < -eps))


def plane_section_from_vertices(
    vertices_world: ArrayLike,
    frame: GateFrame,
    body: CuboidBody,
    *,
    time: float,
    plane_epsilon: float = 1.0e-9,
    dedup_epsilon: float = 1.0e-8,
) -> PlaneSection:
    """Construct the complete ordered plane/cuboid section polygon."""

    vertices = np.asarray(vertices_world, dtype=float)
    local3 = gate_local_vertex_coordinates(vertices, frame)
    xi = local3[:, 2]
    eps = float(plane_epsilon)
    dedup = float(dedup_epsilon)
    if eps < 0.0 or dedup <= 0.0 or not np.isfinite([eps, dedup]).all():
        raise ValueError("section tolerances must be finite and valid")
    candidates: list[SectionVertex] = []
    degenerate = False
    for edge in body.edges:
        a, b = edge
        xa, xb = float(xi[a]), float(xi[b])
        a_on, b_on = abs(xa) <= eps, abs(xb) <= eps
        if a_on and b_on:
            degenerate = True
            for index, alpha in ((a, 0.0), (b, 1.0)):
                candidates.append(SectionVertex(vertices[index].copy(), local3[index, :2].copy(), edge, alpha))
        elif a_on or b_on:
            index = a if a_on else b
            candidates.append(SectionVertex(vertices[index].copy(), local3[index, :2].copy(), edge, 0.0 if a_on else 1.0))
        elif xa * xb < 0.0:
            denominator = xa - xb
            if abs(denominator) <= eps:
                degenerate = True
                continue
            alpha = xa / denominator
            world = (1.0 - alpha) * vertices[a] + alpha * vertices[b]
            local = (frame.basis.T @ (world - frame.center)) / frame.scale
            candidates.append(SectionVertex(world, local, edge, float(alpha)))

    unique: list[SectionVertex] = []
    for candidate in candidates:
        match = next((i for i, item in enumerate(unique) if np.linalg.norm(item.local - candidate.local) <= dedup), None)
        if match is None:
            unique.append(candidate)
        elif candidate.source_body_edge < unique[match].source_body_edge:
            unique[match] = candidate
    if len(unique) >= 3:
        center = np.mean(np.stack([item.local for item in unique]), axis=0)
        unique.sort(key=lambda item: float(np.arctan2(item.local[1] - center[1], item.local[0] - center[0])))
    topology = tuple(sorted(item.source_body_edge for item in unique))
    if len(unique) < 3 or len(unique) > 6:
        degenerate = True
    return PlaneSection(float(time), tuple(unique), topology, degenerate)


def plane_section_at(
    trajectory: MincoSnap,
    time: float,
    window: SCDynamicWindow,
    body: CuboidBody,
    config: WholeBodySafetyConfig,
    *,
    yaw_profile: YawProfile | None = None,
    parameters: QuadrotorParameters | None = None,
) -> PlaneSection:
    """Evaluate the moving oriented cuboid section at one trajectory time."""

    vertices, _ = cuboid_world_vertices(
        trajectory, time, body, yaw_profile=yaw_profile, parameters=parameters
    )
    return plane_section_from_vertices(
        vertices,
        frame_at(window, time),
        body,
        time=time,
        plane_epsilon=config.plane_epsilon,
        dedup_epsilon=config.dedup_epsilon,
    )


def _bisect_transition(
    predicate: Callable[[float], bool],
    outside: float,
    inside: float,
    tolerance: float,
) -> float:
    left, right = sorted((float(outside), float(inside)))
    outside_is_left = outside < inside
    while right - left > tolerance:
        middle = 0.5 * (left + right)
        if predicate(middle):
            if outside_is_left:
                right = middle
            else:
                left = middle
        elif outside_is_left:
            left = middle
        else:
            right = middle
    return right if outside_is_left else left


def find_planned_crossing_interval(
    *,
    window_index: int,
    traversal_time: float,
    trajectory: MincoSnap,
    window: SCDynamicWindow,
    body: CuboidBody,
    config: WholeBodySafetyConfig,
    yaw_profile: YawProfile | None = None,
    parameters: QuadrotorParameters | None = None,
) -> CrossingInterval:
    """Find the plane-intersection component containing the planned crossing."""

    instant = float(traversal_time)
    total = float(np.real(trajectory.total_time))
    if window_index < 0 or not np.isfinite(instant) or not 0.0 <= instant <= total:
        raise ValueError("window_index and traversal_time are invalid")

    def predicate(time: float) -> bool:
        vertices, _ = cuboid_world_vertices(
            trajectory, time, body, yaw_profile=yaw_profile, parameters=parameters
        )
        xi = gate_local_vertex_coordinates(vertices, frame_at(window, time))[:, 2]
        return has_plane_section(xi, config.plane_epsilon, include_contact=True)

    if not predicate(instant):
        raise ValueError("planned traversal time has no cuboid/gate-plane section")
    durations = np.asarray(np.real(trajectory.durations), dtype=float)
    cumulative = np.concatenate(([0.0], np.cumsum(durations)))
    segment = min(int(np.searchsorted(cumulative[1:], instant, side="right")), len(durations) - 1)
    candidate_left = cumulative[max(0, segment - 1)]
    candidate_right = cumulative[min(len(durations), segment + 2)]

    def bracket(direction: int, initial_limit: float) -> tuple[float, float]:
        inside = instant
        limit = initial_limit
        while True:
            grid = np.linspace(inside, limit, config.interval_scan_steps + 1)[1:]
            for value in grid:
                if not predicate(float(value)):
                    return float(value), inside
                inside = float(value)
            if limit == (0.0 if direction < 0 else total):
                raise ValueError("crossing component reaches a trajectory boundary")
            index = int(np.searchsorted(cumulative, limit, side="left"))
            limit = float(cumulative[max(0, index - 1)] if direction < 0 else cumulative[min(len(durations), index + 1)])

    left_outside, left_inside = bracket(-1, float(candidate_left))
    right_outside, right_inside = bracket(+1, float(candidate_right))
    start = _bisect_transition(predicate, left_outside, left_inside, config.time_tolerance)
    end = _bisect_transition(predicate, right_outside, right_inside, config.time_tolerance)
    delta = min(max(config.time_tolerance, 1e-6), 0.25 * min(instant - start, end - instant))
    profile = constant_yaw_profile() if yaw_profile is None else yaw_profile

    def eta(time: float) -> float:
        position = np.asarray(trajectory.evaluate(time), dtype=float)
        gate = frame_at(window, time)
        return float(gate.normal @ (position - gate.center))

    slope = eta(min(total, instant + delta)) - eta(max(0.0, instant - delta))
    direction = 1 if slope >= 0.0 else -1
    _ = profile  # documents that crossing attitude uses the same fixed-yaw profile
    return CrossingInterval(int(window_index), instant, start, end, direction)


def split_into_topology_stable_intervals(
    crossing: CrossingInterval,
    *,
    section_at: Callable[[float], PlaneSection],
    time_tolerance: float,
    max_depth: int = 24,
) -> list[TopologyInterval]:
    """Adaptively partition a crossing into numerically topology-stable cells."""

    if time_tolerance <= 0.0 or max_depth < 1:
        raise ValueError("time_tolerance and max_depth must be positive")
    width = crossing.end - crossing.start
    inset = min(0.25 * width, max(time_tolerance * 0.25, 1e-10))
    pending = [(crossing.start + inset, crossing.end - inset, 0)]
    stable: list[TopologyInterval] = []
    while pending:
        left, right, depth = pending.pop()
        middle = 0.5 * (left + right)
        sections = (section_at(left), section_at(middle), section_at(right))
        keys = {section.topology_key for section in sections}
        if len(keys) == 1 and all(not section.degenerate for section in sections):
            stable.append(TopologyInterval(left, right, sections[1].topology_key))
        elif right - left <= time_tolerance or depth >= max_depth:
            continue
        else:
            pending.append((middle, right, depth + 1))
            pending.append((left, middle, depth + 1))
    stable.sort(key=lambda item: item.start)
    return stable


__all__ = [
    "CrossingInterval", "PlaneSection", "SectionVertex", "TopologyInterval",
    "cuboid_world_vertices", "find_planned_crossing_interval",
    "gate_local_vertex_coordinates", "has_plane_section", "plane_section_at",
    "plane_section_from_vertices", "split_into_topology_stable_intervals",
]
