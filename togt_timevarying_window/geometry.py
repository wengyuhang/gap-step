from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

ShapeKind = Literal["rectangle", "circle", "triangle", "pentagon", "hexagon", "slanted_quadrilateral"]


def rot2(theta: float) -> np.ndarray:
    c = math.cos(theta)
    s = math.sin(theta)
    return np.asarray([[c, -s], [s, c]], dtype=np.float64)


def rotation_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class Shape2D:
    kind: ShapeKind
    size: tuple[float, float] = (1.6, 1.1)
    radius: float = 0.82

    def polygon(self, resolution: int = 32) -> np.ndarray:
        if self.kind == "rectangle":
            w, h = self.size
            return np.asarray([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]], dtype=np.float64)
        if self.kind == "slanted_quadrilateral":
            w, h = self.size
            return np.asarray([[-0.55 * w, -0.50 * h], [0.36 * w, -0.50 * h], [0.58 * w, 0.50 * h], [-0.36 * w, 0.50 * h]], dtype=np.float64)
        vertices = {"triangle": 3, "pentagon": 5, "hexagon": 6}.get(self.kind, max(16, resolution))
        angles = np.linspace(0.0, 2.0 * math.pi, vertices, endpoint=False) + math.pi / 2.0
        return np.stack([self.radius * np.cos(angles), self.radius * np.sin(angles)], axis=1).astype(np.float64)


def local_from_unconstrained(z: np.ndarray, polygon: np.ndarray, shrink: float = 0.72) -> np.ndarray:
    """Map two unconstrained variables into the convex window interior."""

    poly = np.asarray(polygon, dtype=np.float64)
    center = poly.mean(axis=0)
    direction = np.tanh(np.asarray(z, dtype=np.float64))
    lo = poly.min(axis=0)
    hi = poly.max(axis=0)
    candidate = center + 0.5 * shrink * direction * (hi - lo)
    if point_in_convex_polygon(candidate, poly, margin=1e-9):
        return candidate
    return center + shrink * (candidate - center) * max_scale_inside(center, candidate, poly)


def max_scale_inside(center: np.ndarray, point: np.ndarray, polygon: np.ndarray) -> float:
    lo = 0.0
    hi = 1.0
    for _ in range(32):
        mid = 0.5 * (lo + hi)
        p = center + mid * (point - center)
        if point_in_convex_polygon(p, polygon, margin=1e-9):
            lo = mid
        else:
            hi = mid
    return lo


def point_in_convex_polygon(point: np.ndarray, polygon: np.ndarray, margin: float = 0.0) -> bool:
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


def convex_margin(point: np.ndarray, polygon: np.ndarray) -> float:
    p = np.asarray(point, dtype=np.float64)
    poly = np.asarray(polygon, dtype=np.float64)
    signs = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        edge = b - a
        rel = p - a
        cross = float(edge[0] * rel[1] - edge[1] * rel[0])
        signs.append(cross / max(np.linalg.norm(edge), 1e-9))
    signs_arr = np.asarray(signs)
    if np.mean(signs_arr) < 0.0:
        signs_arr = -signs_arr
    return float(np.min(signs_arr))


def sample_polygon(polygon: np.ndarray, samples_per_axis: int = 3, shrink: float = 0.88) -> np.ndarray:
    poly = np.asarray(polygon, dtype=np.float64)
    center = poly.mean(axis=0)
    candidates = [center]
    for v in poly:
        candidates.append(center + shrink * (v - center))
    lo = poly.min(axis=0)
    hi = poly.max(axis=0)
    for x in np.linspace(lo[0], hi[0], samples_per_axis):
        for y in np.linspace(lo[1], hi[1], samples_per_axis):
            p = center + shrink * (np.asarray([x, y]) - center)
            if point_in_convex_polygon(p, poly, margin=1e-8):
                candidates.append(p)
    unique: list[np.ndarray] = []
    for p in candidates:
        if not any(np.linalg.norm(p - old) < 1e-7 for old in unique):
            unique.append(p)
    return np.asarray(unique, dtype=np.float64)


def path_length(points: np.ndarray | list[np.ndarray]) -> float:
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
