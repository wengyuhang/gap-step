"""Dynamic three-dimensional gates backed by a Schwarz--Christoffel map.

The local safe polygon and its SC map are immutable.  Time dependence is
introduced only through the centre ``c(t)``, the orthonormal plane basis
``E(t)``, and a uniform scale ``s(t)`` as required by the experiment plan.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .sc_mapping import SCDiskMap, B_with_jacobian


def _rotation_parts(roll: float, pitch: float, yaw: float) -> tuple[np.ndarray, ...]:
    """Return Euler rotation factors and their angle derivatives.

    The convention is ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.  It is the
    same convention used by the existing TOGT window experiments.
    """

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    drz = np.array([[-sy, -cy, 0.0], [cy, -sy, 0.0], [0.0, 0.0, 0.0]])
    dry = np.array([[-sp, 0.0, cp], [0.0, 0.0, 0.0], [-cp, 0.0, -sp]])
    drx = np.array([[0.0, 0.0, 0.0], [0.0, -sr, -cr], [0.0, cr, -sr]])
    return rz, ry, rx, drz, dry, drx


def rotation_and_derivative(
    angles: Iterable[float], angle_rates: Iterable[float]
) -> tuple[np.ndarray, np.ndarray]:
    """Compute ``R`` and the exact time derivative ``R_dot``."""

    # Match the bundled TOGT reproduction: the public vector is RPY while
    # the matrix is Rz(yaw) @ Ry(pitch) @ Rx(roll).
    roll, pitch, yaw = (float(x) for x in angles)
    roll_dot, pitch_dot, yaw_dot = (float(x) for x in angle_rates)
    rz, ry, rx, drz, dry, drx = _rotation_parts(roll, pitch, yaw)
    rotation = rz @ ry @ rx
    derivative = (
        yaw_dot * (drz @ ry @ rx)
        + pitch_dot * (rz @ dry @ rx)
        + roll_dot * (rz @ ry @ drx)
    )
    return rotation, derivative


@dataclass(frozen=True)
class MotionProfile:
    """Analytic translation, Euler rotation, and uniform scale profile.

    Independent phases avoid an accidental synchronisation of all motion
    components while retaining closed-form first derivatives.
    """

    translation_amplitude: np.ndarray
    rotation_amplitude: np.ndarray
    scale_amplitude: float
    translation_period: float = 7.0
    rotation_period: float = 8.0
    scale_period: float = 9.0
    phase: float = 0.0
    translation_enabled: bool = True
    rotation_enabled: bool = True
    scale_enabled: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "translation_amplitude", np.asarray(self.translation_amplitude, dtype=float)
        )
        object.__setattr__(
            self, "rotation_amplitude", np.asarray(self.rotation_amplitude, dtype=float)
        )
        if self.translation_amplitude.shape != (3,) or self.rotation_amplitude.shape != (3,):
            raise ValueError("translation_amplitude and rotation_amplitude must have shape (3,)")
        if not np.all(np.isfinite(self.translation_amplitude)) or not np.all(np.isfinite(self.rotation_amplitude)):
            raise ValueError("motion amplitudes must be finite")
        if not np.isfinite(self.scale_amplitude) or not np.isfinite(self.phase):
            raise ValueError("scale amplitude and phase must be finite")
        if not np.all(np.isfinite([self.translation_period, self.rotation_period, self.scale_period])):
            raise ValueError("motion periods must be finite")
        if min(self.translation_period, self.rotation_period, self.scale_period) <= 0.0:
            raise ValueError("motion periods must be positive")
        if abs(self.scale_amplitude) >= 1.0:
            raise ValueError("uniform scale amplitude must keep s(t) positive")

    @staticmethod
    def static() -> "MotionProfile":
        return MotionProfile(np.zeros(3), np.zeros(3), 0.0)

    def translation(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        if not self.translation_enabled:
            return np.zeros(3), np.zeros(3)
        omega = 2.0 * math.pi / self.translation_period
        phases = self.phase + np.array([0.0, 0.7, 1.4])
        angle = omega * float(t) + phases
        return self.translation_amplitude * np.sin(angle), self.translation_amplitude * omega * np.cos(angle)

    def rotation(self, t: float) -> tuple[np.ndarray, np.ndarray]:
        if not self.rotation_enabled:
            return np.zeros(3), np.zeros(3)
        omega = 2.0 * math.pi / self.rotation_period
        phases = self.phase + np.array([0.0, 0.9, 1.8])
        angle = omega * float(t) + phases
        return self.rotation_amplitude * np.sin(angle), self.rotation_amplitude * omega * np.cos(angle)

    def scale(self, t: float) -> tuple[float, float]:
        if not self.scale_enabled:
            return 1.0, 0.0
        omega = 2.0 * math.pi / self.scale_period
        angle = omega * float(t) + self.phase
        return 1.0 + self.scale_amplitude * math.sin(angle), self.scale_amplitude * omega * math.cos(angle)


def _xy(value: object) -> np.ndarray:
    """Normalize either a complex SC value or a length-two vector."""

    if np.iscomplexobj(value) or isinstance(value, complex):
        z = complex(value)  # type: ignore[arg-type]
        return np.array([z.real, z.imag], dtype=float)
    out = np.asarray(value, dtype=float)
    if out.shape != (2,):
        raise ValueError(f"expected a planar point, got shape {out.shape}")
    return out


def _point_in_polygon(point: np.ndarray, polygon: np.ndarray, tol: float = 1.0e-9) -> bool:
    """Boundary-inclusive ray test used only for final physical validation."""

    p = np.asarray(point, dtype=float)
    vertices = np.asarray(polygon, dtype=float)
    a = vertices
    b = np.roll(vertices, -1, axis=0)
    ab = b - a
    den = np.einsum("ij,ij->i", ab, ab)
    u = np.divide(np.einsum("ij,ij->i", p - a, ab), den, out=np.zeros_like(den), where=den > 0)
    closest = a + np.clip(u, 0.0, 1.0)[:, None] * ab
    if float(np.linalg.norm(closest - p, axis=1).min()) <= tol:
        return True
    crossings = (a[:, 1] > p[1]) != (b[:, 1] > p[1])
    x_cross = np.divide(
        (b[:, 0] - a[:, 0]) * (p[1] - a[:, 1]),
        b[:, 1] - a[:, 1],
        out=np.zeros(len(vertices)),
        where=np.abs(b[:, 1] - a[:, 1]) > 1.0e-15,
    ) + a[:, 0]
    return bool(np.count_nonzero(crossings & (p[0] < x_cross)) % 2)


@dataclass
class SCDynamicWindow:
    """A non-convex dynamic window with two unconstrained spatial variables."""

    name: str
    sc_map: SCDiskMap
    safe_polygon: np.ndarray
    center0: np.ndarray
    angles0: np.ndarray
    motion: MotionProfile

    def __post_init__(self) -> None:
        self.safe_polygon = np.asarray(self.safe_polygon, dtype=float)
        self.center0 = np.asarray(self.center0, dtype=float)
        self.angles0 = np.asarray(self.angles0, dtype=float)
        if self.safe_polygon.ndim != 2 or self.safe_polygon.shape[1] != 2:
            raise ValueError("safe_polygon must have shape (n, 2)")
        if self.center0.shape != (3,) or self.angles0.shape != (3,):
            raise ValueError("center0 and angles0 must have shape (3,)")
        if not np.all(np.isfinite(self.safe_polygon)) or not np.all(np.isfinite(self.center0)) or not np.all(np.isfinite(self.angles0)):
            raise ValueError("window geometry and pose must be finite")
        if self.safe_polygon.shape != self.sc_map.vertices.shape or not np.allclose(
            self.safe_polygon, self.sc_map.vertices, rtol=0.0, atol=1.0e-12
        ):
            raise ValueError("safe_polygon must exactly match the polygon used by sc_map")

    def state_at(self, t: float) -> tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, float]:
        translation, translation_dot = self.motion.translation(t)
        angle_delta, angle_dot = self.motion.rotation(t)
        scale, scale_dot = self.motion.scale(t)
        rotation, rotation_dot = rotation_and_derivative(self.angles0 + angle_delta, angle_dot)
        # TOGT's Rectangle/Polygon vertices live in the local x-y plane.
        plane_axes = rotation[:, :2]
        plane_axes_dot = rotation_dot[:, :2]
        return (
            self.center0 + translation,
            plane_axes,
            scale,
            translation_dot,
            plane_axes_dot,
            scale_dot,
        )

    def local_point_and_jacobian(self, d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        z, jac_b = B_with_jacobian(np.asarray(d, dtype=float))
        value = _xy(self.sc_map.evaluate(complex(float(z[0]), float(z[1]))))
        jac_sc = np.asarray(self.sc_map.jacobian(complex(float(z[0]), float(z[1]))), dtype=float)
        return value, jac_sc @ jac_b

    def point_and_jacobians(self, d: np.ndarray, t: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return ``p, q, dp/dd, dp/dt`` using the plan's exact chain rule."""

        q, jac_local = self.local_point_and_jacobian(d)
        center, basis, scale, center_dot, basis_dot, scale_dot = self.state_at(t)
        point = center + basis @ (scale * q)
        jac_d = basis @ (scale * jac_local)
        derivative_t = center_dot + basis_dot @ (scale * q) + basis @ (scale_dot * q)
        return point, q, jac_d, derivative_t

    def to_point(self, d: np.ndarray, traversal_time: float) -> np.ndarray:
        return self.point_and_jacobians(d, traversal_time)[0]

    def get_grad(
        self, d: np.ndarray, traversal_time: float, grad_point: np.ndarray
    ) -> tuple[np.ndarray, float]:
        _, _, jac_d, derivative_t = self.point_and_jacobians(d, traversal_time)
        upstream = np.asarray(grad_point, dtype=float)
        return jac_d.T @ upstream, float(upstream @ derivative_t)

    def polygon_at(self, t: float) -> np.ndarray:
        center, basis, scale, *_ = self.state_at(t)
        return center[None, :] + (basis @ (scale * self.safe_polygon).T).T

    def world_to_local(self, point: np.ndarray, t: float) -> tuple[np.ndarray, float]:
        center, basis, scale, *_ = self.state_at(t)
        rel = np.asarray(point, dtype=float) - center
        normal = np.cross(basis[:, 0], basis[:, 1])
        plane_error = abs(float(rel @ normal))
        return (basis.T @ rel) / scale, plane_error

    def contains(self, point: np.ndarray, t: float, plane_tolerance: float = 1.0e-7) -> bool:
        local, plane_error = self.world_to_local(point, t)
        return plane_error <= plane_tolerance and _point_in_polygon(local, self.safe_polygon, tol=1.0e-8)


@dataclass(frozen=True)
class SCWindowTrack:
    """Start/goal states and the prescribed one-pass gate order."""

    name: str
    start: np.ndarray
    goal: np.ndarray
    windows: tuple[SCDynamicWindow, ...]
    order: tuple[int, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "start", np.asarray(self.start, dtype=float))
        object.__setattr__(self, "goal", np.asarray(self.goal, dtype=float))
        object.__setattr__(self, "windows", tuple(self.windows))
        object.__setattr__(self, "order", tuple(int(value) for value in self.order))
        if self.start.shape != (3,) or self.goal.shape != (3,):
            raise ValueError("track start and goal must have shape (3,)")
        if not np.all(np.isfinite(self.start)) or not np.all(np.isfinite(self.goal)):
            raise ValueError("track endpoints must be finite")
        if not self.windows:
            raise ValueError("track must contain at least one window")
        if len(self.order) != len(self.windows) or set(self.order) != set(range(len(self.windows))):
            raise ValueError("the current problem requires each window exactly once in a fixed order")
