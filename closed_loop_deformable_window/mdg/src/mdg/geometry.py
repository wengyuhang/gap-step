"""Geometry helpers that preserve true non-convex topology."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from shapely import contains_xy, distance, points
from shapely.geometry import GeometryCollection, MultiPolygon, Point, Polygon
from shapely.geometry.base import BaseGeometry


def polygonal_parts(geometry: BaseGeometry) -> tuple[Polygon, ...]:
    if geometry.is_empty:
        return ()
    if isinstance(geometry, Polygon):
        return (geometry,)
    if isinstance(geometry, MultiPolygon):
        return tuple(geometry.geoms)
    if isinstance(geometry, GeometryCollection):
        return tuple(item for item in geometry.geoms if isinstance(item, Polygon))
    return ()


def validate_simple_polygon(polygon: Polygon) -> None:
    if polygon.is_empty or not polygon.is_valid or polygon.area <= 0.0:
        raise ValueError("physical gate polygon must be nonempty and valid")
    if len(polygon.interiors) != 0:
        raise ValueError("physical gate polygon must not contain holes")


def rotation_and_derivative(
    rpy: np.ndarray, rpy_rate: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    roll, pitch, yaw = (float(x) for x in rpy)
    roll_dot, pitch_dot, yaw_dot = (float(x) for x in rpy_rate)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    drz = np.array(((-sy, -cy, 0.0), (cy, -sy, 0.0), (0.0, 0.0, 0.0)))
    dry = np.array(((-sp, 0.0, cp), (0.0, 0.0, 0.0), (-cp, 0.0, -sp)))
    drx = np.array(((0.0, 0.0, 0.0), (0.0, -sr, -cr), (0.0, cr, -sr)))
    rotation = rz @ ry @ rx
    derivative = (
        yaw_dot * drz @ ry @ rx
        + pitch_dot * rz @ dry @ rx
        + roll_dot * rz @ ry @ drx
    )
    return rotation, derivative


def interior_grid(geometry: BaseGeometry, resolution: float) -> tuple[np.ndarray, np.ndarray]:
    if geometry.is_empty:
        return np.empty((0, 2)), np.empty(0)
    min_x, min_y, max_x, max_y = geometry.bounds
    xs = np.arange(min_x, max_x + 0.5 * resolution, resolution)
    ys = np.arange(min_y, max_y + 0.5 * resolution, resolution)
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    flat_x, flat_y = xx.ravel(), yy.ravel()
    mask = contains_xy(geometry, flat_x, flat_y)
    coordinates = np.column_stack((flat_x[mask], flat_y[mask]))
    if not len(coordinates):
        return coordinates, np.empty(0)
    distances = np.asarray(distance(geometry.boundary, points(coordinates)), dtype=float)
    return coordinates, distances


def circle_covered(
    geometry: BaseGeometry,
    center: np.ndarray,
    radius: float,
    *,
    tolerance: float = 1.0e-9,
) -> bool:
    if radius <= 0.0 or geometry.is_empty:
        return False
    circle = Point(float(center[0]), float(center[1])).buffer(
        float(radius), quad_segs=32
    )
    return bool(geometry.buffer(tolerance).covers(circle))


def signed_clearance(geometry: BaseGeometry, center: np.ndarray) -> float:
    point = Point(float(center[0]), float(center[1]))
    if geometry.is_empty or not geometry.covers(point):
        return -float(point.distance(geometry))
    return float(point.distance(geometry.boundary))


def json_array(values: Iterable[float] | np.ndarray) -> list:
    return np.asarray(values).tolist()


__all__ = [
    "circle_covered",
    "interior_grid",
    "json_array",
    "polygonal_parts",
    "rotation_and_derivative",
    "signed_clearance",
    "validate_simple_polygon",
]
