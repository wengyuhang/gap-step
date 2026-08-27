"""Fixed-plane, in-plane rotation and uniform-scale problem types."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import BoundarySegment
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    SCDynamicWindow,
    SCWindowTrack,
    rotation_and_derivative,
)
from nonconvex_timevarying_window.sip_dynatogt.intervals import exact_ball, iv_matmul
from nonconvex_timevarying_window.sip_dynatogt.model import (
    SUPPORTED_SEGMENTS,
    SIPConfig,
    SIPProblem,
    polygon_segments,
)


@dataclass(frozen=True)
class PlanarRSMotion:
    """Sinusoidal rotation about the fixed local normal and uniform scaling."""

    angle_amplitude: float = 0.0
    angle_period: float = 8.0
    scale_amplitude: float = 0.0
    scale_period: float = 9.0
    phase: float = 0.0

    def __post_init__(self) -> None:
        values = (
            self.angle_amplitude,
            self.angle_period,
            self.scale_amplitude,
            self.scale_period,
            self.phase,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("planar motion parameters must be finite")
        if self.angle_period <= 0.0 or self.scale_period <= 0.0:
            raise ValueError("motion periods must be positive")
        if abs(self.scale_amplitude) >= 1.0:
            raise ValueError("uniform scale must remain strictly positive")

    def angle(self, time: float) -> tuple[float, float]:
        omega = 2.0 * math.pi / self.angle_period
        value = omega * float(time) + self.phase
        return self.angle_amplitude * math.sin(
            value
        ), self.angle_amplitude * omega * math.cos(value)

    def scale(self, time: float) -> tuple[float, float]:
        omega = 2.0 * math.pi / self.scale_period
        value = omega * float(time) + self.phase
        return 1.0 + self.scale_amplitude * math.sin(
            value
        ), self.scale_amplitude * omega * math.cos(value)

    @property
    def minimum_scale(self) -> float:
        return 1.0 - abs(self.scale_amplitude)


@dataclass
class PlanarRSDynamicWindow:
    """SC-compatible window whose center and plane are exactly fixed."""

    base: SCDynamicWindow
    planar_motion: PlanarRSMotion

    def __post_init__(self) -> None:
        if self.base.required_world_clearance is not None:
            guaranteed = self.planar_motion.minimum_scale * float(
                self.base.reference_local_clearance
            )
            if guaranteed + 1e-12 < float(self.base.required_world_clearance):
                raise ValueError(
                    "minimum planar scale does not preserve the configured SC inset"
                )

    def __getattr__(self, name):
        return getattr(self.base, name)

    @property
    def fixed_rotation(self) -> np.ndarray:
        return rotation_and_derivative(
            np.asarray(self.base.angles0, dtype=float), np.zeros(3)
        )[0]

    @property
    def fixed_normal(self) -> np.ndarray:
        return self.fixed_rotation[:, 2].copy()

    def state_at(self, time: float):
        angle, angle_dot = self.planar_motion.angle(time)
        scale, scale_dot = self.planar_motion.scale(time)
        c, s = math.cos(angle), math.sin(angle)
        rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        drz = angle_dot * np.array([[-s, -c, 0.0], [c, -s, 0.0], [0.0, 0.0, 0.0]])
        rotation = self.fixed_rotation @ rz
        rotation_dot = self.fixed_rotation @ drz
        return (
            self.base.center0.copy(),
            rotation[:, :2],
            scale,
            np.zeros(3),
            rotation_dot[:, :2],
            scale_dot,
        )

    def point_and_jacobians(self, d: np.ndarray, time: float):
        q, jac_local = self.base.local_point_and_jacobian(d)
        center, basis, scale, _, basis_dot, scale_dot = self.state_at(time)
        point = center + basis @ (scale * q)
        jac_d = basis @ (scale * jac_local)
        derivative_t = basis_dot @ (scale * q) + basis @ (scale_dot * q)
        return point, q, jac_d, derivative_t

    def to_point(self, d: np.ndarray, traversal_time: float) -> np.ndarray:
        return self.point_and_jacobians(d, traversal_time)[0]


@dataclass(frozen=True)
class PlanarRSSIPWindow:
    name: str
    center0: np.ndarray
    fixed_rotation: np.ndarray
    motion: PlanarRSMotion
    boundary: tuple[BoundarySegment, ...]

    def __post_init__(self) -> None:
        center = np.asarray(self.center0, dtype=float)
        rotation = np.asarray(self.fixed_rotation, dtype=float)
        if (
            center.shape != (3,)
            or rotation.shape != (3, 3)
            or not np.all(np.isfinite(center))
            or not np.all(np.isfinite(rotation))
        ):
            raise ValueError("fixed window pose must be finite")
        if (
            not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0.0, atol=1e-10)
            or np.linalg.det(rotation) <= 0.0
        ):
            raise ValueError("fixed_rotation must be a proper orthogonal matrix")
        boundary = tuple(self.boundary)
        if not boundary or any(
            not isinstance(segment, SUPPORTED_SEGMENTS) for segment in boundary
        ):
            raise TypeError(
                "only Line, CircularArc, Bezier and non-rational BSpline are supported"
            )
        for index, segment in enumerate(boundary):
            end = np.asarray(segment.evaluate(1.0), dtype=float)
            start = np.asarray(
                boundary[(index + 1) % len(boundary)].evaluate(0.0), dtype=float
            )
            if not np.allclose(end, start, rtol=0.0, atol=1e-8):
                raise ValueError(f"boundary is not closed at segment {index}")
        object.__setattr__(self, "center0", center.copy())
        object.__setattr__(self, "fixed_rotation", rotation.copy())
        object.__setattr__(self, "boundary", boundary)

    @property
    def fixed_normal(self) -> np.ndarray:
        return self.fixed_rotation[:, 2].copy()

    def state_at(self, time: float):
        angle, _ = self.motion.angle(time)
        scale, _ = self.motion.scale(time)
        c, s = math.cos(angle), math.sin(angle)
        rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return self.center0.copy(), self.fixed_rotation @ rz, float(scale)

    def state_interval(self, time):
        from flint import arb

        two_pi = exact_ball(2.0) * arb.pi()
        phase = exact_ball(self.motion.phase)
        angle = (
            exact_ball(self.motion.angle_amplitude)
            * (two_pi / exact_ball(self.motion.angle_period) * time + phase).sin()
        )
        c, s = angle.cos(), angle.sin()
        z, o = exact_ball(0.0), exact_ball(1.0)
        rz = [[c, -s, z], [s, c, z], [z, z, o]]
        r0 = [[exact_ball(float(v)) for v in row] for row in self.fixed_rotation]
        rotation = iv_matmul(r0, rz)
        scale = (
            o
            + exact_ball(self.motion.scale_amplitude)
            * (two_pi / exact_ball(self.motion.scale_period) * time + phase).sin()
        )
        return [exact_ball(float(v)) for v in self.center0], rotation, scale


@dataclass(frozen=True)
class PlanarRSConfig:
    sip: SIPConfig = field(default_factory=SIPConfig)
    plane_prune_max_depth: int = 12
    plane_prune_min_time_width: float = 1e-4

    def __post_init__(self) -> None:
        if self.plane_prune_max_depth < 0 or self.plane_prune_min_time_width <= 0.0:
            raise ValueError("plane pruning settings must be positive")


def make_planar_problem(
    track: SCWindowTrack,
    motions: Sequence[PlanarRSMotion],
    boundaries: Sequence[Sequence[BoundarySegment]] | None = None,
) -> SIPProblem:
    if len(motions) != len(track.windows) or (
        boundaries is not None and len(boundaries) != len(track.windows)
    ):
        raise ValueError(
            "one planar motion and boundary collection are required per window"
        )
    dynamic = tuple(
        PlanarRSDynamicWindow(window, motions[i])
        for i, window in enumerate(track.windows)
    )
    planar_track = SCWindowTrack(
        track.name, track.start, track.goal, dynamic, track.order
    )
    sip_windows = []
    for i, window in enumerate(dynamic):
        boundary = (
            tuple(boundaries[i])
            if boundaries is not None
            else polygon_segments(window.base.physical_boundary)
        )
        sip_windows.append(
            PlanarRSSIPWindow(
                window.name,
                window.base.center0,
                window.fixed_rotation,
                motions[i],
                boundary,
            )
        )
    return SIPProblem(track.name, tuple(sip_windows), track.order, planar_track)


__all__ = [
    "PlanarRSConfig",
    "PlanarRSDynamicWindow",
    "PlanarRSMotion",
    "PlanarRSSIPWindow",
    "make_planar_problem",
]
