"""Independent non-convex, time-varying gate geometry used by AVS-PPO."""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import cached_property

import numpy as np

Array = np.ndarray


def distance_to_segment(point: Array, a: Array, b: Array) -> float:
    ab = b - a
    denominator = float(ab @ ab)
    if denominator <= 1.0e-14:
        return float(np.linalg.norm(point - a))
    fraction = float(np.clip((point - a) @ ab / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (a + fraction * ab)))


def point_in_polygon(point: Array, vertices: Array) -> bool:
    """Odd-even test; boundary points count as inside and margin handles clearance."""
    if min(distance_to_segment(point, a, b) for a, b in zip(vertices, np.roll(vertices, -1, axis=0))) <= 1.0e-10:
        return True
    x, y = map(float, point)
    inside = False
    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
        if (a[1] > y) != (b[1] > y):
            crossing_x = (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
            if x < crossing_x:
                inside = not inside
    return inside


def signed_margin(point: Array, vertices: Array) -> float:
    distance = min(distance_to_segment(point, a, b) for a, b in zip(vertices, np.roll(vertices, -1, axis=0)))
    return float(distance if point_in_polygon(point, vertices) else -distance)


def polygon_centroid(vertices: Array) -> Array:
    """Area centroid, followed by a robust interior fallback for strongly concave shapes."""
    p = np.asarray(vertices, dtype=float)
    cross = p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]
    area6 = 3.0 * float(np.sum(cross))
    if abs(area6) > 1.0e-12:
        centroid = np.array([
            np.sum((p[:, 0] + np.roll(p[:, 0], -1)) * cross) / area6,
            np.sum((p[:, 1] + np.roll(p[:, 1], -1)) * cross) / area6,
        ])
        if signed_margin(centroid, p) > 0.0:
            return centroid
    lo, hi = p.min(axis=0), p.max(axis=0)
    candidates = [
        np.array([u, v])
        for u in np.linspace(lo[0], hi[0], 31)
        for v in np.linspace(lo[1], hi[1], 31)
    ]
    return max(candidates, key=lambda q: signed_margin(q, p))


def make_shape(kind: str, samples: int = 64) -> Array:
    """Simple, hole-free polygons with genuine concavities."""
    if kind == "star":
        angles = np.linspace(0.0, 2.0 * math.pi, 10, endpoint=False)
        radii = np.where(np.arange(10) % 2 == 0, 1.28, 0.72)
        return np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))
    if kind == "u_notch":
        return np.array([
            [-1.20, -1.05], [1.20, -1.05], [1.20, 1.05], [0.48, 1.05],
            [0.48, -0.18], [-0.48, -0.18], [-0.48, 1.05], [-1.20, 1.05],
        ])
    if kind == "wavy":
        angles = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
        radius = 1.02 + 0.25 * np.cos(3.0 * angles) + 0.12 * np.sin(5.0 * angles)
        return np.column_stack((1.12 * radius * np.cos(angles), 0.92 * radius * np.sin(angles)))
    raise ValueError(f"unknown shape: {kind}")


@dataclass(frozen=True)
class DynamicGate:
    name: str
    x: float
    base_vertices: Array
    phase: float
    motion_scale: float = 1.0

    @cached_property
    def base_anchor(self) -> Array:
        return polygon_centroid(self.base_vertices)

    def state(self, time: float) -> tuple[Array, float, Array, Array]:
        """Return centre(y,z), in-plane angle, anisotropic scale, world-plane polygon."""
        w = 2.0 * math.pi / 5.8
        centre = np.array([
            0.34 * self.motion_scale * math.sin(w * time + self.phase),
            1.55 + 0.27 * self.motion_scale * math.cos(0.83 * w * time + 1.3 * self.phase),
        ])
        angle = 0.30 * self.motion_scale * math.sin(0.91 * w * time + 0.7 * self.phase)
        scale = np.array([
            1.0 + 0.13 * self.motion_scale * math.sin(0.77 * w * time + self.phase),
            1.0 + 0.11 * self.motion_scale * math.cos(1.07 * w * time + 0.4 * self.phase),
        ])
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        polygon = centre + (self.base_vertices * scale) @ rotation.T
        return centre, angle, scale, polygon

    def anchor(self, time: float) -> Array:
        centre, angle, scale, _ = self.state(time)
        # Polygon centroids commute with this diagonal affine scaling.
        local_anchor = self.base_anchor * scale
        rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
        return centre + rotation @ local_anchor

    def margin(self, yz: Array, time: float) -> float:
        return signed_margin(np.asarray(yz, dtype=float), self.state(time)[3])


def crossing_time(x0: float, velocity_x: float, acceleration_x: float, plane_x: float, dt: float) -> float | None:
    """Exact forward plane crossing time for constant acceleration on [0, dt]."""
    if x0 >= plane_x:
        return None
    offset = x0 - plane_x
    if abs(acceleration_x) <= 1.0e-12:
        if velocity_x <= 0.0:
            return None
        root = -offset / velocity_x
        return float(root) if 0.0 < root <= dt + 1.0e-12 else None
    discriminant = velocity_x * velocity_x - 2.0 * acceleration_x * offset
    if discriminant < 0.0:
        return None
    sqrt_discriminant = math.sqrt(discriminant)
    roots = [(-velocity_x - sqrt_discriminant) / acceleration_x, (-velocity_x + sqrt_discriminant) / acceleration_x]
    valid = [root for root in roots if 0.0 < root <= dt + 1.0e-12]
    return float(min(valid)) if valid else None
