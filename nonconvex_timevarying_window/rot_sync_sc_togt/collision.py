"""Oriented cuboid collision checks against rotating finite-thickness frames."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray
from shapely.geometry import LineString, MultiPoint

from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    QuadrotorParameters,
    sample_flatness,
)

from .geometry import RotatingWindow
from .scenarios import RotSyncScenario


FloatArray = NDArray[np.float64]
_SIGNS = np.asarray(
    [
        (-1.0, -1.0, -1.0),
        (-1.0, -1.0, 1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, 1.0, 1.0),
        (1.0, -1.0, -1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, -1.0),
        (1.0, 1.0, 1.0),
    ],
    dtype=float,
)
_EDGES = tuple(
    (left, right)
    for left in range(8)
    for right in range(left + 1, 8)
    if np.count_nonzero(_SIGNS[left] != _SIGNS[right]) == 1
)


def cuboid_vertices(
    center: ArrayLike,
    rotation: ArrayLike,
    body: CuboidBody,
) -> FloatArray:
    """Return the eight world vertices of the closed oriented body cuboid."""

    origin = np.asarray(center, dtype=float)
    attitude = np.asarray(rotation, dtype=float)
    if origin.shape != (3,) or attitude.shape != (3, 3):
        raise ValueError("center and rotation must have shapes (3,) and (3,3)")
    local = _SIGNS * np.asarray(body.half_extents, dtype=float)
    return origin[None, :] + (attitude @ local.T).T


def body_rotations(
    trajectory,
    times: ArrayLike,
    *,
    parameters: QuadrotorParameters | None = None,
) -> NDArray[np.float64]:
    """Evaluate the same flatness attitude used by the dynamics penalties."""

    return np.asarray(
        sample_flatness(trajectory, times, parameters=parameters).rotation,
        dtype=float,
    )


def _slab_cross_section(
    window: RotatingWindow,
    absolute_time: float,
    body_center: ArrayLike,
    body_rotation: ArrayLike,
    body: CuboidBody,
    *,
    tolerance: float,
):
    """Project ``cuboid intersect gate-thickness slab`` into gate coordinates."""

    vertices = cuboid_vertices(body_center, body_rotation, body)
    frame = np.column_stack((window.rotated_basis(absolute_time), window.normal))
    local = (frame.T @ (vertices - window.center).T).T
    half_thickness = 0.5 * window.thickness
    points = [point for point in local if abs(float(point[2])) <= half_thickness + tolerance]
    for left, right in _EDGES:
        a, b = local[left], local[right]
        for plane in (-half_thickness, half_thickness):
            denominator = float(b[2] - a[2])
            if abs(denominator) <= tolerance:
                continue
            fraction = float((plane - a[2]) / denominator)
            if -tolerance <= fraction <= 1.0 + tolerance:
                points.append(a + np.clip(fraction, 0.0, 1.0) * (b - a))
    if not points:
        return None
    planar = np.asarray(points, dtype=float)[:, :2]
    rounded = np.round(planar / max(tolerance, 1.0e-12)).astype(np.int64)
    _, unique_indices = np.unique(rounded, axis=0, return_index=True)
    return MultiPoint(planar[np.sort(unique_indices)]).convex_hull


def cuboid_window_collision(
    window: RotatingWindow,
    absolute_time: float,
    body_center: ArrayLike,
    body_rotation: ArrayLike,
    body: CuboidBody,
    *,
    tolerance: float = 1.0e-9,
) -> tuple[bool, float]:
    """Check intersection with the rotating boundary curtain.

    The physical frame is the aperture boundary extruded through the declared
    thickness.  The cuboid/slab intersection is convex; its exact projected
    hull collides iff it reaches the non-convex polygon boundary.
    """

    if tolerance <= 0.0 or not np.isfinite(tolerance):
        raise ValueError("tolerance must be finite and positive")
    section = _slab_cross_section(
        window,
        float(absolute_time),
        body_center,
        body_rotation,
        body,
        tolerance=tolerance,
    )
    if section is None or section.is_empty:
        return False, float("inf")
    closed_boundary = np.vstack((window.physical_polygon, window.physical_polygon[0]))
    frame_boundary = LineString(closed_boundary)
    clearance = float(section.distance(frame_boundary))
    return bool(clearance <= tolerance), clearance


@dataclass(frozen=True)
class CollisionReport:
    sample_count: int
    colliding_sample_count: int
    sampled_collision_rate: float
    any_collision: bool
    first_collision_time: float | None
    minimum_frame_clearance: float
    per_window: tuple[dict[str, Any], ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "body_model": "oriented_square_bottom_cuboid",
            "sample_count": self.sample_count,
            "colliding_sample_count": self.colliding_sample_count,
            "sampled_collision_rate": self.sampled_collision_rate,
            "any_collision": self.any_collision,
            "first_collision_time": self.first_collision_time,
            "minimum_frame_clearance": self.minimum_frame_clearance,
            "per_window": list(self.per_window),
        }


def sample_collision_report(
    scenario: RotSyncScenario,
    trajectory,
    *,
    samples: int = 2001,
    parameters: QuadrotorParameters | None = None,
) -> CollisionReport:
    """Densely sample whole-body collisions and report trajectory-time rate."""

    if samples < 2:
        raise ValueError("collision audit needs at least two samples")
    grid = np.linspace(0.0, trajectory.total_time, int(samples))
    positions = np.asarray(trajectory.evaluate(grid), dtype=float)
    rotations = body_rotations(trajectory, grid, parameters=parameters)
    per_window_masks = []
    per_window_clearances = []
    per_window_rows = []
    for window in scenario.windows:
        mask = np.zeros(len(grid), dtype=bool)
        clearance = np.full(len(grid), np.inf)
        for index, instant in enumerate(grid):
            mask[index], clearance[index] = cuboid_window_collision(
                window,
                float(instant),
                positions[index],
                rotations[index],
                scenario.body,
            )
        per_window_masks.append(mask)
        per_window_clearances.append(clearance)
        finite = clearance[np.isfinite(clearance)]
        per_window_rows.append(
            {
                "window": window.name,
                "colliding_sample_count": int(np.count_nonzero(mask)),
                "sampled_collision_rate": float(np.mean(mask)),
                "minimum_frame_clearance": float(np.min(finite)) if finite.size else None,
            }
        )
    combined = np.any(np.stack(per_window_masks), axis=0)
    all_clearance = np.concatenate(per_window_clearances)
    finite = all_clearance[np.isfinite(all_clearance)]
    collision_indices = np.flatnonzero(combined)
    return CollisionReport(
        sample_count=len(grid),
        colliding_sample_count=int(len(collision_indices)),
        sampled_collision_rate=float(np.mean(combined)),
        any_collision=bool(np.any(combined)),
        first_collision_time=(float(grid[collision_indices[0]]) if len(collision_indices) else None),
        minimum_frame_clearance=float(np.min(finite)) if finite.size else float("inf"),
        per_window=tuple(per_window_rows),
    )


__all__ = [
    "CollisionReport",
    "body_rotations",
    "cuboid_vertices",
    "cuboid_window_collision",
    "sample_collision_report",
]
