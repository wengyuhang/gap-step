from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Literal

import numpy as np

from .geometry import rectangle, regular_polygon, sample_convex_polygon, point_in_convex_polygon

MotionFn = Callable[[float], np.ndarray]
ScalarFn = Callable[[float], float]
VectorFn = Callable[[float], np.ndarray]
ShapeKind = Literal["rectangle", "triangle", "pentagon", "hexagon", "ball"]


@dataclass(frozen=True)
class GateShape:
    kind: ShapeKind
    size: tuple[float, float] = (2.4, 1.5)
    radius: float = 1.0

    def local_polygon(self, resolution: int = 20) -> np.ndarray:
        if self.kind == "rectangle":
            return rectangle(self.size[0], self.size[1])
        if self.kind == "triangle":
            return regular_polygon(3, self.radius)
        if self.kind == "pentagon":
            return regular_polygon(5, self.radius)
        if self.kind == "hexagon":
            return regular_polygon(6, self.radius)
        if self.kind == "ball":
            return regular_polygon(max(12, resolution), self.radius)
        raise ValueError(f"Unsupported gate shape: {self.kind}")


@dataclass
class DynamicGate:
    name: str
    shape: GateShape
    center0: np.ndarray
    yaw0: float = 0.0
    pitch0: float = 0.0
    roll0: float = 0.0
    scale0: np.ndarray | None = None
    center_motion: MotionFn | None = None
    yaw_motion: ScalarFn | None = None
    pitch_motion: ScalarFn | None = None
    roll_motion: ScalarFn | None = None
    scale_motion: VectorFn | None = None

    def center_at(self, t: float) -> np.ndarray:
        base = np.asarray(self.center0, dtype=np.float64)
        return base if self.center_motion is None else base + np.asarray(self.center_motion(t), dtype=np.float64)

    def yaw_at(self, t: float) -> float:
        return float(self.yaw0 if self.yaw_motion is None else self.yaw0 + self.yaw_motion(t))

    def pitch_at(self, t: float) -> float:
        return float(self.pitch0 if self.pitch_motion is None else self.pitch0 + self.pitch_motion(t))

    def roll_at(self, t: float) -> float:
        return float(self.roll0 if self.roll_motion is None else self.roll0 + self.roll_motion(t))

    def scale_at(self, t: float) -> np.ndarray:
        base = np.ones(2, dtype=np.float64) if self.scale0 is None else np.asarray(self.scale0, dtype=np.float64)
        return base if self.scale_motion is None else base * np.asarray(self.scale_motion(t), dtype=np.float64)

    def basis_at(self, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        # Gate normal points roughly along the race direction; u/v span the gate opening plane.
        yaw = self.yaw_at(t)
        pitch = self.pitch_at(t)
        roll = self.roll_at(t)
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        rot = rz @ ry @ rx
        normal = rot @ np.asarray([1.0, 0.0, 0.0])
        u_axis = rot @ np.asarray([0.0, 1.0, 0.0])
        v_axis = rot @ np.asarray([0.0, 0.0, 1.0])
        return normal, u_axis, v_axis

    def polygon_at(self, t: float) -> np.ndarray:
        local = self.shape.local_polygon() * self.scale_at(t)[None, :]
        center = self.center_at(t)
        _, u_axis, v_axis = self.basis_at(t)
        return center[None, :] + local[:, 0:1] * u_axis[None, :] + local[:, 1:2] * v_axis[None, :]

    def contains(self, point: np.ndarray, t: float, plane_tol: float = 1e-5) -> bool:
        center = self.center_at(t)
        normal, u_axis, v_axis = self.basis_at(t)
        rel = np.asarray(point, dtype=np.float64) - center
        if abs(float(np.dot(rel, normal))) > plane_tol:
            return False
        local = np.asarray([np.dot(rel, u_axis), np.dot(rel, v_axis)], dtype=np.float64)
        scaled_poly = self.shape.local_polygon() * self.scale_at(t)[None, :]
        return point_in_convex_polygon(local, scaled_poly, margin=1e-7)

    def candidates(self, t: float, samples_per_axis: int = 3) -> np.ndarray:
        center = self.center_at(t)
        if samples_per_axis <= 1:
            return center[None, :]
        local_candidates = sample_convex_polygon(self.shape.local_polygon() * self.scale_at(t)[None, :], samples_per_axis)
        _, u_axis, v_axis = self.basis_at(t)
        return center[None, :] + local_candidates[:, 0:1] * u_axis[None, :] + local_candidates[:, 1:2] * v_axis[None, :]


@dataclass
class RaceTrack:
    start: np.ndarray
    goal: np.ndarray
    gates: list[DynamicGate]
    name: str = "dynamic_3d_gate_track"

    def validate_gate_order(self, timed_points: list[tuple[int, float, np.ndarray]]) -> bool:
        for gate_idx, t, point in timed_points:
            if gate_idx < 0 or gate_idx >= len(self.gates):
                return False
            if not self.gates[gate_idx].contains(point, t):
                return False
        return True


def sinusoid(amplitude: tuple[float, float, float], period: float, phase: float = 0.0) -> MotionFn:
    amp = np.asarray(amplitude, dtype=np.float64)

    def fn(t: float) -> np.ndarray:
        return amp * math.sin(2.0 * math.pi * t / period + phase)

    return fn


def scale_wave(u_amp: float, v_amp: float, period: float, phase: float = 0.0) -> VectorFn:
    def fn(t: float) -> np.ndarray:
        angle = 2.0 * math.pi * t / period + phase
        return np.asarray([1.0 + u_amp * math.sin(angle), 1.0 + v_amp * math.cos(angle)], dtype=np.float64)

    return fn


def angle_wave(amplitude: float, period: float, phase: float = 0.0) -> ScalarFn:
    def fn(t: float) -> float:
        return amplitude * math.sin(2.0 * math.pi * t / period + phase)

    return fn


def demo_track(dynamic: bool = True) -> RaceTrack:
    centers = [
        (2.0, 0.0, 1.2),
        (4.2, 2.8, 2.4),
        (6.5, -1.8, 1.0),
        (8.8, 3.2, 3.0),
        (11.5, -3.0, 1.4),
        (14.5, 2.6, 2.8),
        (17.0, -2.4, 1.1),
        (19.8, 3.4, 3.2),
        (22.4, -1.2, 1.6),
        (25.2, 2.2, 2.7),
        (27.8, -2.8, 1.3),
        (30.5, 0.6, 2.1),
    ]
    kinds: list[ShapeKind] = [
        "triangle",
        "rectangle",
        "pentagon",
        "hexagon",
        "rectangle",
        "ball",
        "pentagon",
        "rectangle",
        "hexagon",
        "triangle",
        "rectangle",
        "ball",
    ]
    gates: list[DynamicGate] = []
    for idx, (center, kind) in enumerate(zip(centers, kinds)):
        motion = (
            sinusoid(
                (0.12 * math.sin(idx), 0.65 + 0.04 * (idx % 4), 0.46 + 0.03 * (idx % 3)),
                period=8.0 + 0.45 * idx,
                phase=0.55 * idx,
            )
            if dynamic
            else None
        )
        scale = scale_wave(0.24, -0.22, period=9.0 + 0.35 * idx, phase=0.8 * idx) if dynamic else None
        yaw = angle_wave(0.42, period=8.5 + 0.2 * idx, phase=0.4 * idx) if dynamic else None
        pitch = angle_wave(0.32, period=7.5 + 0.25 * idx, phase=0.35 * idx) if dynamic else None
        roll = angle_wave(0.55, period=6.8 + 0.3 * idx, phase=0.65 * idx) if dynamic else None
        shape = GateShape(kind=kind, size=(1.85, 1.25), radius=0.98)
        gates.append(
            DynamicGate(
                f"G{idx + 1}",
                shape,
                np.asarray(center, dtype=np.float64),
                yaw0=0.08 * math.sin(idx * 0.7),
                pitch0=0.05 * math.cos(idx * 0.5),
                roll0=0.04 * math.sin(idx * 0.4),
                center_motion=motion,
                yaw_motion=yaw,
                pitch_motion=pitch,
                roll_motion=roll,
                scale_motion=scale,
            )
        )
    return RaceTrack(
        start=np.asarray([0.0, -0.8, 1.2], dtype=np.float64),
        goal=np.asarray([33.0, 0.0, 1.8], dtype=np.float64),
        gates=gates,
        name="paper_style_complex_dynamic_3d_gates",
    )
