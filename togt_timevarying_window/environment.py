from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .geometry import Shape2D, ShapeKind, convex_margin, local_from_unconstrained, rotation_matrix, sample_polygon

DEFAULT_ORDER = (0, 5, 2, 1, 4, 3)


@dataclass(frozen=True)
class MotionProfile:
    translation_amp: np.ndarray
    rotation_amp: np.ndarray
    scale_amp: np.ndarray
    period: float = 7.0
    phase: float = 0.0
    enabled_translation: bool = True
    enabled_rotation: bool = True
    enabled_scale: bool = True

    def translation(self, t: float) -> np.ndarray:
        if not self.enabled_translation:
            return np.zeros(3, dtype=np.float64)
        return self.translation_amp * math.sin(2.0 * math.pi * t / self.period + self.phase)

    def rotation(self, t: float) -> np.ndarray:
        if not self.enabled_rotation:
            return np.zeros(3, dtype=np.float64)
        angle = 2.0 * math.pi * t / (self.period * 0.91) + self.phase
        return self.rotation_amp * np.asarray([math.sin(angle), math.cos(angle * 0.83), math.sin(angle * 1.21)], dtype=np.float64)

    def scale(self, t: float) -> np.ndarray:
        if not self.enabled_scale:
            return np.ones(2, dtype=np.float64)
        angle = 2.0 * math.pi * t / (self.period * 1.13) + self.phase
        return np.asarray([1.0 + self.scale_amp[0] * math.sin(angle), 1.0 + self.scale_amp[1] * math.cos(angle)], dtype=np.float64)


@dataclass
class DynamicWindow:
    name: str
    shape: Shape2D
    center0: np.ndarray
    yaw0: float
    pitch0: float
    roll0: float
    motion: MotionProfile

    def center_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        return self.center0 + (self.motion.translation(t) if dynamic else 0.0)

    def angles_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        base = np.asarray([self.yaw0, self.pitch0, self.roll0], dtype=np.float64)
        return base + (self.motion.rotation(t) if dynamic else 0.0)

    def scale_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        return self.motion.scale(t) if dynamic else np.ones(2, dtype=np.float64)

    def local_polygon_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        return self.shape.polygon() * self.scale_at(t, dynamic=dynamic)[None, :]

    def basis_at(self, t: float, dynamic: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        yaw, pitch, roll = self.angles_at(t, dynamic=dynamic)
        rot = rotation_matrix(float(yaw), float(pitch), float(roll))
        normal = rot @ np.asarray([1.0, 0.0, 0.0])
        u_axis = rot @ np.asarray([0.0, 1.0, 0.0])
        v_axis = rot @ np.asarray([0.0, 0.0, 1.0])
        return normal, u_axis, v_axis

    def point_from_local(self, local: np.ndarray, t: float, dynamic: bool = True) -> np.ndarray:
        _, u_axis, v_axis = self.basis_at(t, dynamic=dynamic)
        return self.center_at(t, dynamic=dynamic) + local[0] * u_axis + local[1] * v_axis

    def point_from_unconstrained(self, z: np.ndarray, t: float, dynamic: bool = True) -> tuple[np.ndarray, np.ndarray]:
        poly = self.local_polygon_at(t, dynamic=dynamic)
        local = local_from_unconstrained(z, poly)
        return self.point_from_local(local, t, dynamic=dynamic), local

    def polygon_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        local = self.local_polygon_at(t, dynamic=dynamic)
        return np.asarray([self.point_from_local(p, t, dynamic=dynamic) for p in local], dtype=np.float64)

    def candidates(self, t: float, samples_per_axis: int = 3, dynamic: bool = True) -> list[tuple[np.ndarray, np.ndarray]]:
        poly = self.local_polygon_at(t, dynamic=dynamic)
        locals_ = sample_polygon(poly, samples_per_axis=samples_per_axis)
        return [(self.point_from_local(local, t, dynamic=dynamic), local) for local in locals_]

    def contains(self, point: np.ndarray, t: float, dynamic: bool = True, plane_tol: float = 1e-4) -> bool:
        local, plane_error = self.point_to_local(point, t, dynamic=dynamic)
        if plane_error > plane_tol:
            return False
        return self.local_margin(local, t, dynamic=dynamic) >= -1e-7

    def point_to_local(self, point: np.ndarray, t: float, dynamic: bool = True) -> tuple[np.ndarray, float]:
        center = self.center_at(t, dynamic=dynamic)
        normal, u_axis, v_axis = self.basis_at(t, dynamic=dynamic)
        rel = np.asarray(point, dtype=np.float64) - center
        local = np.asarray([np.dot(rel, u_axis), np.dot(rel, v_axis)], dtype=np.float64)
        return local, abs(float(np.dot(rel, normal)))

    def local_margin(self, local: np.ndarray, t: float, dynamic: bool = True) -> float:
        return convex_margin(local, self.local_polygon_at(t, dynamic=dynamic))


@dataclass
class WindowTrack:
    name: str
    start: np.ndarray
    goal: np.ndarray
    windows: list[DynamicWindow]
    order: tuple[int, ...] = DEFAULT_ORDER

    def ordered_windows(self, order: Iterable[int] | None = None) -> list[DynamicWindow]:
        idxs = self.order if order is None else tuple(order)
        return [self.windows[i] for i in idxs]


def make_window(idx: int, center: tuple[float, float, float], kind: ShapeKind, motion_scale: float = 1.0, motion_flags: tuple[bool, bool, bool] = (True, True, True)) -> DynamicWindow:
    amp_t = motion_scale * np.asarray([0.22 * math.sin(idx + 0.7), 0.45 + 0.06 * (idx % 3), 0.28 + 0.04 * (idx % 2)], dtype=np.float64)
    amp_r = motion_scale * np.asarray([0.28, 0.18, 0.34], dtype=np.float64)
    amp_s = motion_scale * np.asarray([0.18, -0.16], dtype=np.float64)
    return DynamicWindow(
        name=f"G{idx + 1}",
        shape=Shape2D(kind=kind, size=(1.65, 1.12), radius=0.86),
        center0=np.asarray(center, dtype=np.float64),
        yaw0=0.28 * math.sin(idx * 0.9),
        pitch0=0.10 * math.cos(idx * 0.6),
        roll0=0.16 * math.sin(idx * 0.5),
        motion=MotionProfile(
            translation_amp=amp_t,
            rotation_amp=amp_r,
            scale_amp=amp_s,
            period=6.0 + 0.45 * idx,
            phase=0.55 * idx,
            enabled_translation=motion_flags[0],
            enabled_rotation=motion_flags[1],
            enabled_scale=motion_flags[2],
        ),
    )


def canonical_track(motion_scale: float = 1.45, motion_flags: tuple[bool, bool, bool] = (True, True, True), name: str = "canonical_6") -> WindowTrack:
    centers = [
        (2.0, -0.3, 1.3),
        (8.4, 3.3, 2.7),
        (6.0, -3.0, 1.1),
        (12.4, -2.2, 3.2),
        (10.3, 2.0, 1.0),
        (4.4, 3.6, 2.5),
    ]
    kinds: list[ShapeKind] = ["rectangle", "circle", "pentagon", "slanted_quadrilateral", "hexagon", "triangle"]
    windows = [make_window(i, c, k, motion_scale=motion_scale, motion_flags=motion_flags) for i, (c, k) in enumerate(zip(centers, kinds))]
    return WindowTrack(name=name, start=np.asarray([0.0, -1.4, 1.2]), goal=np.asarray([14.4, 1.4, 1.9]), windows=windows)


def random_track(seed: int, count: int = 6) -> WindowTrack:
    rng = np.random.default_rng(seed)
    kinds: list[ShapeKind] = ["rectangle", "circle", "triangle", "pentagon", "hexagon", "slanted_quadrilateral"]
    centers = []
    for i in range(count):
        centers.append((2.0 + 2.0 * i + rng.uniform(-0.6, 0.6), rng.uniform(-3.5, 3.5), rng.uniform(0.9, 3.2)))
    windows = [make_window(i, centers[i], kinds[i % len(kinds)], motion_scale=float(rng.uniform(0.6, 1.3))) for i in range(count)]
    order = tuple(rng.permutation(count).tolist())
    return WindowTrack(name=f"random_{seed}", start=np.asarray([0.0, -1.0, 1.2]), goal=np.asarray([2.0 + 2.0 * count, 0.8, 1.8]), windows=windows, order=order)


def make_scenario(name: str) -> WindowTrack:
    if name == "canonical":
        return canonical_track()
    if name == "translation_only":
        return canonical_track(motion_flags=(True, False, False), name=name)
    if name == "rotation_only":
        return canonical_track(motion_flags=(False, True, False), name=name)
    if name == "scale_only":
        return canonical_track(motion_flags=(False, False, True), name=name)
    if name == "slow_dynamic":
        return canonical_track(motion_scale=0.45, name=name)
    if name == "fast_dynamic":
        return canonical_track(motion_scale=1.55, name=name)
    if name.startswith("random_"):
        return random_track(int(name.split("_", 1)[1]))
    raise ValueError(f"unknown scenario: {name}")
