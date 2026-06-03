from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

Point = np.ndarray


def rot2(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.asarray([[c, -s], [s, c]], dtype=np.float64)


def regular_polygon(vertices: int, radius: float = 1.0) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * math.pi, vertices, endpoint=False) + math.pi / 2.0
    return np.stack([radius * np.cos(angles), radius * np.sin(angles)], axis=1).astype(np.float64)


def rectangle(width: float, height: float) -> np.ndarray:
    return np.asarray(
        [
            [-0.5 * width, -0.5 * height],
            [0.5 * width, -0.5 * height],
            [0.5 * width, 0.5 * height],
            [-0.5 * width, 0.5 * height],
        ],
        dtype=np.float64,
    )


def transform_polygon(local: np.ndarray, center: Point, yaw: float, scale: Point) -> np.ndarray:
    scaled = local * np.asarray(scale, dtype=np.float64)[None, :]
    return scaled @ rot2(yaw).T + np.asarray(center, dtype=np.float64)[None, :]


def point_in_convex_polygon(point: Point, polygon: np.ndarray, margin: float = 0.0) -> bool:
    p = np.asarray(point, dtype=np.float64)
    poly = np.asarray(polygon, dtype=np.float64)
    sign = 0.0
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        edge = b - a
        rel = p - a
        cross = float(edge[0] * rel[1] - edge[1] * rel[0])
        if abs(cross) <= margin:
            continue
        if sign == 0.0:
            sign = 1.0 if cross > 0.0 else -1.0
        elif sign * cross < -margin:
            return False
    return True


def sample_convex_polygon(polygon: np.ndarray, count_per_axis: int, shrink: float = 0.82) -> np.ndarray:
    poly = np.asarray(polygon, dtype=np.float64)
    center = np.mean(poly, axis=0)
    candidates = [center]
    for vertex in poly:
        candidates.append(center + shrink * (vertex - center))
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        candidates.append(center + shrink * (0.5 * (a + b) - center))
    if count_per_axis > 1:
        lo = np.min(poly, axis=0)
        hi = np.max(poly, axis=0)
        xs = np.linspace(lo[0], hi[0], count_per_axis)
        ys = np.linspace(lo[1], hi[1], count_per_axis)
        for x in xs:
            for y in ys:
                point = np.asarray([x, y], dtype=np.float64)
                if point_in_convex_polygon(point, poly):
                    candidates.append(center + shrink * (point - center))
    unique: list[np.ndarray] = []
    for point in candidates:
        if not any(float(np.linalg.norm(point - old)) < 1e-6 for old in unique):
            unique.append(np.asarray(point, dtype=np.float64))
    return np.asarray(unique, dtype=np.float64)


def path_length(points: list[Point]) -> float:
    if len(points) < 2:
        return 0.0
    return float(sum(np.linalg.norm(np.asarray(b) - np.asarray(a)) for a, b in zip(points[:-1], points[1:])))


@dataclass(frozen=True)
class TimedPoint:
    t: float
    p: Point
