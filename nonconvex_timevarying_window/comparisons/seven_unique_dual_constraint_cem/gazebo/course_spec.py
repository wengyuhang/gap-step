#!/usr/bin/env python3
"""Algorithm-independent geometric specification of the seven-gate course."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

import numpy as np


MESH_CHORD_TOLERANCE_M = 0.002
START = (-16.0, 4.0, 3.2)


@dataclass(frozen=True)
class GateSpec:
    name: str
    shape: str
    center: np.ndarray
    base_rpy: np.ndarray
    theta0: float
    omega: float
    boundary: np.ndarray

def _point_to_chord(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    delta = end - start
    denominator = float(delta @ delta)
    if denominator == 0.0:
        return float(np.linalg.norm(point - start))
    fraction = float(np.clip(((point - start) @ delta) / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + fraction * delta)))


def _adaptive_interval(
    evaluator: Callable[[float], np.ndarray],
    u0: float,
    u1: float,
    p0: np.ndarray,
    p1: np.ndarray,
    tolerance: float,
    depth: int = 0,
) -> list[np.ndarray]:
    probes = np.asarray([
        evaluator(u0 + (u1 - u0) * 0.25),
        evaluator(u0 + (u1 - u0) * 0.50),
        evaluator(u0 + (u1 - u0) * 0.75),
    ])
    error = max(_point_to_chord(point, p0, p1) for point in probes)
    if error <= tolerance or depth >= 14:
        return [p0]
    middle_u = 0.5 * (u0 + u1)
    middle = probes[1]
    return (
        _adaptive_interval(evaluator, u0, middle_u, p0, middle, tolerance, depth + 1)
        + _adaptive_interval(evaluator, middle_u, u1, middle, p1, tolerance, depth + 1)
    )


def _closed_curve(
    evaluator: Callable[[float], np.ndarray],
    *,
    seed_segments: int,
    tolerance: float = MESH_CHORD_TOLERANCE_M,
) -> np.ndarray:
    points: list[np.ndarray] = []
    for index in range(seed_segments):
        u0, u1 = index / seed_segments, (index + 1) / seed_segments
        points.extend(_adaptive_interval(
            evaluator, u0, u1, evaluator(u0), evaluator(u1), tolerance
        ))
    return np.asarray(points, dtype=float)


def _limacon() -> np.ndarray:
    def evaluate(u: float) -> np.ndarray:
        theta = 2.0 * math.pi * u
        radius = 2.1 + 0.72 * math.cos(theta)
        return np.asarray((radius * math.cos(theta), radius * math.sin(theta)))

    return _closed_curve(evaluate, seed_segments=32)


def _wavy() -> np.ndarray:
    def evaluate(u: float) -> np.ndarray:
        theta = 2.0 * math.pi * u
        radius = 2.15 + 0.35 * math.cos(5.0 * theta)
        return np.asarray((radius * math.cos(theta), radius * math.sin(theta)))

    return _closed_curve(evaluate, seed_segments=40)


def _bezier(control: np.ndarray, u: float) -> np.ndarray:
    one_minus = 1.0 - u
    return (
        one_minus ** 3 * control[0]
        + 3.0 * one_minus ** 2 * u * control[1]
        + 3.0 * one_minus * u ** 2 * control[2]
        + u ** 3 * control[3]
    )


def _line_bezier() -> np.ndarray:
    bottom_left = np.asarray((-2.25, -1.70))
    bottom_right = np.asarray((2.25, -1.70))
    top_right = np.asarray((2.25, 1.65))
    notch_right = np.asarray((0.55, 0.30))
    notch_left = np.asarray((-2.25, 1.65))
    first = np.asarray((top_right, (1.75, 1.65), (1.35, 0.20), notch_right))
    second = np.asarray((notch_right, (-0.15, 0.38), (-0.75, 1.65), notch_left))
    points = [bottom_left, bottom_right, top_right]
    for control in (first, second):
        curve = _closed_curve(
            lambda u, c=control: _bezier(c, u),
            seed_segments=4,
        )
        points.extend(curve[1:] if np.allclose(points[-1], curve[0]) else curve)
    if not np.allclose(points[-1], notch_left):
        points.append(notch_left)
    return np.asarray(points, dtype=float)


def _star() -> np.ndarray:
    angles = -0.5 * math.pi + np.arange(10) * math.pi / 5.0
    radii = np.where(np.arange(10) % 2 == 0, 2.5, 1.15)
    return np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))


BOUNDARIES = {
    "L": np.asarray((
        (-2.0, -2.0), (2.0, -2.0), (2.0, -0.5),
        (0.5, -0.5), (0.5, 2.0), (-2.0, 2.0),
    )),
    "U": np.asarray((
        (-2.5, -2.0), (2.5, -2.0), (2.5, 2.0), (0.8, 2.0),
        (0.8, 1.0), (-0.8, 1.0), (-0.8, 2.0), (-2.5, 2.0),
    )),
    "star": _star(),
    "limacon": _limacon(),
    "wavy": _wavy(),
    "line_bezier": _line_bezier(),
    "balanced_U": np.asarray((
        (-1.599139106, -2.238794748), (1.599139106, -2.238794748),
        (1.599139106, 0.959483463), (0.319827821, 0.959483463),
        (0.319827821, -0.959483463), (-0.319827821, -0.959483463),
        (-0.319827821, 0.959483463), (-1.599139106, 0.959483463),
    )),
}

SHAPES = ("L", "U", "star", "limacon", "wavy", "line_bezier", "balanced_U")
CENTERS = (
    (-2.42, -3.52, 6.48), (20.24, 14.52, 1.80), (20.24, -8.80, 2.16),
    (-9.90, -13.20, 6.30), (10.45, -1.98, 2.16), (-6.16, 14.96, 2.16),
    (2.00, 8.00, 5.50),
)
BASE_RPY = (
    (0.0, -math.pi / 2.0, 0.0),
    (0.0, -math.pi / 2.0, -math.radians(20.0)),
    (0.0, -math.pi / 2.0, -math.radians(130.0)),
    (0.0, -math.pi / 2.0, math.pi),
    (0.0, -math.pi / 2.0, math.radians(70.0)),
    (0.0, -math.pi / 2.0, math.radians(200.0)),
    (0.0, -math.pi / 2.0, math.radians(145.0)),
)
PHASES = (0.40, -0.70, 0.55, -0.35, 0.90, -1.10, 1.25)
OMEGAS = (5.0, -4.0, 4.5, -5.0, 5.5, -15.0, 18.0)

GATES = tuple(
    GateSpec(
        name=f"W{index + 1}_{shape}",
        shape=shape,
        center=np.asarray(center, dtype=float),
        base_rpy=np.asarray(base_rpy, dtype=float),
        theta0=float(theta0),
        omega=float(omega),
        boundary=BOUNDARIES[shape].copy(),
    )
    for index, (shape, center, base_rpy, theta0, omega) in enumerate(
        zip(SHAPES, CENTERS, BASE_RPY, PHASES, OMEGAS)
    )
)


def rotation_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = map(float, rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray(((1, 0, 0), (0, cr, -sr), (0, sr, cr)))
    ry = np.asarray(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)))
    rz = np.asarray(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)))
    return rz @ ry @ rx


def boundary_at(gate: GateSpec, instant: float) -> np.ndarray:
    angle = gate.theta0 + gate.omega * float(instant)
    spin = np.asarray(((math.cos(angle), -math.sin(angle)),
                       (math.sin(angle), math.cos(angle))))
    return gate.center + (rotation_matrix(gate.base_rpy)[:, :2] @ spin @ gate.boundary.T).T
