"""Exact cuboid/plane sections and non-convex polygon intersection areas."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np
from numpy.typing import ArrayLike, NDArray
from shapely.geometry import GeometryCollection, MultiPolygon, Polygon


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class Cuboid:
    """Attitude-aware UAV box; body x is forward and body z is vertical."""

    half_extents: FloatArray

    def __post_init__(self) -> None:
        values = np.asarray(self.half_extents, dtype=float)
        if values.shape != (3,) or np.any(values <= 0.0) or not np.all(np.isfinite(values)):
            raise ValueError("half_extents must be a finite positive three-vector")
        object.__setattr__(self, "half_extents", values.copy())

    @classmethod
    def repository_default(cls) -> "Cuboid":
        # A quadrotor is a flat box: square footprint in the rotor plane and a
        # much shorter vertical extent.  Body x marks the nose direction.
        return cls(0.5 * np.array([0.5300801927129876, 0.5300801927129876, 0.11779559838066389]))

    @property
    def vertices_body(self) -> FloatArray:
        h = self.half_extents
        return np.asarray(
            [[sx * h[0], sy * h[1], sz * h[2]] for sx in (-1.0, 1.0)
             for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)],
            dtype=float,
        )

    @property
    def edges(self) -> tuple[tuple[int, int], ...]:
        return tuple(
            (left, right)
            for left in range(8)
            for right in range(left + 1, 8)
            if (left ^ right) in (1, 2, 4)
        )


@dataclass(frozen=True)
class GateFrame:
    """A metric, orthonormal gate-plane frame.

    ``boundary_world_2d`` is therefore ``scale * boundary_local``.  Areas are
    measured in square metres; they are never computed in scale-normalized SC
    coordinates.
    """

    center: FloatArray
    basis: FloatArray
    normal: FloatArray
    scale: float

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=float)
        basis = np.asarray(self.basis, dtype=float)
        normal = np.asarray(self.normal, dtype=float)
        if center.shape != (3,) or basis.shape != (3, 2) or normal.shape != (3,):
            raise ValueError("invalid gate-frame shapes")
        if not np.all(np.isfinite(center)) or not np.all(np.isfinite(basis)) or not np.all(np.isfinite(normal)):
            raise ValueError("gate frame must be finite")
        if not np.allclose(basis.T @ basis, np.eye(2), atol=1.0e-9):
            raise ValueError("gate basis must be orthonormal")
        if abs(float(np.linalg.norm(normal)) - 1.0) > 1.0e-9 or np.linalg.norm(basis.T @ normal) > 1.0e-9:
            raise ValueError("gate normal must be unit and orthogonal to the basis")
        if not np.isfinite(self.scale) or self.scale <= 0.0:
            raise ValueError("gate scale must be finite and positive")
        object.__setattr__(self, "center", center.copy())
        object.__setattr__(self, "basis", basis.copy())
        object.__setattr__(self, "normal", normal.copy())

    def world_to_plane(self, points: ArrayLike) -> FloatArray:
        values = np.asarray(points, dtype=float)
        return (values - self.center) @ self.basis

    def plane_to_world(self, points: ArrayLike) -> FloatArray:
        values = np.asarray(points, dtype=float)
        return self.center + values @ self.basis.T

    def boundary_world_2d(self, boundary_local: ArrayLike) -> FloatArray:
        return self.scale * np.asarray(boundary_local, dtype=float)


@dataclass(frozen=True)
class PlaneSection:
    vertices_world: FloatArray
    vertices_2d: FloatArray
    area: float
    degenerate_contact: bool


@dataclass(frozen=True)
class IntersectionMetrics:
    section_area: float
    intersection_area: float
    outside_area: float
    intersection_ratio: float
    penalty: float
    intersection_components: tuple[FloatArray, ...]
    outside_components: tuple[FloatArray, ...]
    whole_body_collision: bool
    degenerate_contact: bool


def cuboid_world_vertices(cuboid: Cuboid, center: ArrayLike, rotation: ArrayLike) -> FloatArray:
    position = np.asarray(center, dtype=float)
    attitude = np.asarray(rotation, dtype=float)
    if position.shape != (3,) or attitude.shape != (3, 3):
        raise ValueError("center and rotation must have shapes (3,) and (3,3)")
    if not np.allclose(attitude.T @ attitude, np.eye(3), atol=1.0e-8) or np.linalg.det(attitude) < 0.0:
        raise ValueError("rotation must be a proper orthogonal matrix")
    return position + cuboid.vertices_body @ attitude.T


def _deduplicate(points: Iterable[FloatArray], tolerance: float) -> FloatArray:
    unique: list[FloatArray] = []
    for point in points:
        if not any(np.linalg.norm(point - previous) <= tolerance for previous in unique):
            unique.append(np.asarray(point, dtype=float))
    return np.asarray(unique, dtype=float) if unique else np.empty((0, 3), dtype=float)


def plane_section(
    cuboid: Cuboid,
    body_center: ArrayLike,
    body_rotation: ArrayLike,
    frame: GateFrame,
    *,
    plane_epsilon: float = 1.0e-10,
    dedup_epsilon: float = 1.0e-9,
) -> PlaneSection:
    """Return the complete ordered 3--6 vertex cuboid/plane section."""

    vertices = cuboid_world_vertices(cuboid, body_center, body_rotation)
    signed = (vertices - frame.center) @ frame.normal
    candidates: list[FloatArray] = []
    degenerate = False
    for left, right in cuboid.edges:
        sl, sr = float(signed[left]), float(signed[right])
        left_on, right_on = abs(sl) <= plane_epsilon, abs(sr) <= plane_epsilon
        if left_on and right_on:
            degenerate = True
            candidates.extend((vertices[left], vertices[right]))
        elif left_on:
            candidates.append(vertices[left])
        elif right_on:
            candidates.append(vertices[right])
        elif sl * sr < 0.0:
            alpha = sl / (sl - sr)
            candidates.append((1.0 - alpha) * vertices[left] + alpha * vertices[right])
    world = _deduplicate(candidates, dedup_epsilon)
    if len(world) < 3:
        return PlaneSection(world, frame.world_to_plane(world), 0.0, bool(len(world) or degenerate))
    planar = frame.world_to_plane(world)
    centroid = np.mean(planar, axis=0)
    order = np.argsort(np.arctan2(planar[:, 1] - centroid[1], planar[:, 0] - centroid[0]))
    planar = planar[order]
    world = world[order]
    area = float(Polygon(planar).area)
    return PlaneSection(world, planar, area, degenerate or area <= plane_epsilon)


def _polygon_components(geometry: object, area_epsilon: float) -> tuple[FloatArray, ...]:
    polygons: list[Polygon] = []
    if isinstance(geometry, Polygon):
        polygons = [geometry]
    elif isinstance(geometry, MultiPolygon):
        polygons = list(geometry.geoms)
    elif isinstance(geometry, GeometryCollection):
        polygons = [item for item in geometry.geoms if isinstance(item, Polygon)]
    result = []
    for polygon in polygons:
        if polygon.area > area_epsilon:
            result.append(np.asarray(polygon.exterior.coords[:-1], dtype=float))
    result.sort(key=lambda item: (float(np.mean(item[:, 0])), float(np.mean(item[:, 1]))))
    return tuple(result)


def exact_intersection_metrics(
    section: PlaneSection,
    gate_boundary_world_2d: ArrayLike,
    *,
    area_epsilon: float = 1.0e-10,
) -> IntersectionMetrics:
    """Compute exact A/C/E and preserve every connected intersection component."""

    gate = Polygon(np.asarray(gate_boundary_world_2d, dtype=float))
    if not gate.is_valid or gate.is_empty or len(gate.interiors):
        raise ValueError("gate boundary must be a valid, hole-free polygon")
    if section.area <= area_epsilon or len(section.vertices_2d) < 3:
        return IntersectionMetrics(
            0.0, 0.0, 0.0, 0.0, 0.0, (), (), False,
            section.degenerate_contact or len(section.vertices_2d) > 0,
        )
    body = Polygon(section.vertices_2d)
    intersection = body.intersection(gate)
    outside = body.difference(gate)
    area_a = float(body.area)
    area_c = float(intersection.area)
    area_e = max(0.0, area_a - area_c)
    ratio = float(np.clip(area_c / area_a, 0.0, 1.0))
    penalty = float(area_a * (ratio * (1.0 - ratio)) ** 3)
    collision = area_c > area_epsilon and area_e > area_epsilon
    return IntersectionMetrics(
        area_a,
        area_c,
        area_e,
        ratio,
        penalty,
        _polygon_components(intersection, area_epsilon),
        _polygon_components(outside, area_epsilon),
        collision,
        section.degenerate_contact,
    )


__all__ = [
    "Cuboid",
    "GateFrame",
    "IntersectionMetrics",
    "PlaneSection",
    "cuboid_world_vertices",
    "exact_intersection_metrics",
    "plane_section",
]
