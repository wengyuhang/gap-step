"""Public types for compact, certified SIP-DynaTOGT."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import factorial
from typing import Any, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.boundary import (
    BSpline, Bezier, BoundarySegment, CircularArc, Line,
)
from nonconvex_timevarying_window.sc_dynatogt.dynamics import DynamicLimits, QuadrotorParameters
from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile, SCDynamicWindow, SCWindowTrack, rotation_and_derivative,
)

FloatArray = NDArray[np.float64]
SUPPORTED_SEGMENTS = (Line, CircularArc, Bezier, BSpline)


class CertificateStatus(str, Enum):
    CERTIFIED_FEASIBLE = "CERTIFIED_FEASIBLE"
    VIOLATED = "VIOLATED"
    UNRESOLVED = "UNRESOLVED"
    NUMERICAL_FAILURE = "NUMERICAL_FAILURE"


@dataclass(frozen=True)
class SIPConfig:
    body: CuboidBody = field(default_factory=CuboidBody)
    clearance: float = 0.015
    planning_clearance_buffer: float = 0.001
    flatness_floor: float = 1e-6
    dynamic_guard_fraction: float = 1e-3
    dynamic_limits: DynamicLimits = field(default_factory=DynamicLimits)
    quadrotor: QuadrotorParameters = field(default_factory=QuadrotorParameters)
    initial_speed: float = 1.0
    minimum_initial_duration: float = 0.20
    initial_nodes: tuple[float, ...] = (0.0, 0.5, 1.0)
    separator_grid_size: int = 3
    max_exchange_iterations: int = 12
    max_witnesses_per_iteration: int = 8
    slsqp_max_iterations: int = 250
    slsqp_ftol: float = 1e-9
    precision_bits: tuple[int, ...] = (128, 256)
    max_cells: int = 200_000
    max_depth: int = 24
    min_time_width: float = 1e-7
    min_boundary_width: float = 1e-7
    violation_tolerance: float = 1e-9

    def __post_init__(self) -> None:
        positives = (
            self.clearance, self.flatness_floor, self.initial_speed,
            self.minimum_initial_duration, self.slsqp_ftol,
            self.min_time_width, self.min_boundary_width,
        )
        if any(not np.isfinite(v) or v <= 0 for v in positives):
            raise ValueError("positive SIP settings must be finite and positive")
        if self.planning_clearance_buffer < 0 or not np.isfinite(self.planning_clearance_buffer):
            raise ValueError("planning clearance buffer must be finite and nonnegative")
        if not 0 <= self.dynamic_guard_fraction < 0.1:
            raise ValueError("dynamic_guard_fraction must lie in [0, 0.1)")
        nodes = tuple(sorted(set(float(v) for v in self.initial_nodes)))
        if not nodes or any(v < 0 or v > 1 for v in nodes):
            raise ValueError("initial_nodes must be a nonempty subset of [0,1]")
        object.__setattr__(self, "initial_nodes", nodes)
        if self.separator_grid_size < 3 or self.separator_grid_size % 2 == 0:
            raise ValueError("separator_grid_size must be an odd integer >= 3")
        if not self.precision_bits or any(int(v) < 64 for v in self.precision_bits):
            raise ValueError("precision_bits must be >= 64")
        budgets = (
            self.max_exchange_iterations, self.max_witnesses_per_iteration,
            self.slsqp_max_iterations, self.max_cells, self.max_depth,
        )
        if any(v < 1 for v in budgets):
            raise ValueError("all iteration and subdivision budgets must be positive")
        if self.violation_tolerance < 0 or not np.isfinite(self.violation_tolerance):
            raise ValueError("violation_tolerance must be finite and nonnegative")

    @property
    def planning_clearance(self) -> float:
        return self.clearance + self.planning_clearance_buffer

    def to_dict(self) -> dict[str, Any]:
        return {
            "body": {"half_extents": list(self.body.half_extents)},
            "clearance": self.clearance,
            "planning_clearance_buffer": self.planning_clearance_buffer,
            "flatness_floor": self.flatness_floor,
            "dynamic_guard_fraction": self.dynamic_guard_fraction,
            "dynamic_limits": asdict(self.dynamic_limits),
            "quadrotor": {
                "mass": self.quadrotor.mass,
                "gravity": self.quadrotor.gravity,
                "inertia": np.asarray(self.quadrotor.inertia).tolist(),
                "arm_length": self.quadrotor.arm_length,
                "yaw_moment_coefficient": self.quadrotor.yaw_moment_coefficient,
                "mixing_matrix": np.asarray(self.quadrotor.mixing_matrix).tolist(),
                "singularity_epsilon": self.quadrotor.singularity_epsilon,
            },
            "separator_grid_size": self.separator_grid_size,
            "initial_speed": self.initial_speed,
            "minimum_initial_duration": self.minimum_initial_duration,
            "initial_nodes": list(self.initial_nodes),
            "max_exchange_iterations": self.max_exchange_iterations,
            "max_witnesses_per_iteration": self.max_witnesses_per_iteration,
            "slsqp_max_iterations": self.slsqp_max_iterations,
            "slsqp_ftol": self.slsqp_ftol,
            "precision_bits": list(self.precision_bits),
            "max_cells": self.max_cells,
            "max_depth": self.max_depth,
            "min_time_width": self.min_time_width,
            "min_boundary_width": self.min_boundary_width,
            "violation_tolerance": self.violation_tolerance,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SIPConfig":
        d = dict(data)
        d["body"] = CuboidBody(tuple(d["body"]["half_extents"]))
        d["dynamic_limits"] = DynamicLimits(**d["dynamic_limits"])
        q = dict(d["quadrotor"])
        q["inertia"] = np.asarray(q["inertia"], dtype=float)
        q["mixing_matrix"] = np.asarray(q["mixing_matrix"], dtype=float)
        d["quadrotor"] = QuadrotorParameters(**q)
        d["initial_nodes"] = tuple(d["initial_nodes"])
        d["precision_bits"] = tuple(int(v) for v in d["precision_bits"])
        return cls(**d)


def polygon_segments(vertices: ArrayLike) -> tuple[Line, ...]:
    p = np.asarray(vertices, dtype=float)
    if p.ndim != 2 or p.shape[1:] != (2,) or len(p) < 3 or not np.all(np.isfinite(p)):
        raise ValueError("physical polygon must have finite shape (n,2), n>=3")
    return tuple(Line(p[i], p[(i + 1) % len(p)]) for i in range(len(p)))


@dataclass(frozen=True)
class SIPWindow:
    name: str
    center0: FloatArray
    angles0: FloatArray
    motion: MotionProfile
    boundary: tuple[BoundarySegment, ...]

    def __post_init__(self) -> None:
        c, a = np.asarray(self.center0, dtype=float), np.asarray(self.angles0, dtype=float)
        segments = tuple(self.boundary)
        if c.shape != (3,) or a.shape != (3,) or not np.all(np.isfinite(c)) or not np.all(np.isfinite(a)):
            raise ValueError("center0 and angles0 must be finite three-vectors")
        if not segments or any(not isinstance(s, SUPPORTED_SEGMENTS) for s in segments):
            raise TypeError("only Line, CircularArc, Bezier and non-rational BSpline are supported")
        for i, segment in enumerate(segments):
            end = np.asarray(segment.evaluate(1.0), dtype=float)
            start = np.asarray(segments[(i + 1) % len(segments)].evaluate(0.0), dtype=float)
            if not np.allclose(end, start, rtol=0, atol=1e-8):
                raise ValueError(f"boundary is not closed at segment {i}")
        object.__setattr__(self, "center0", c.copy())
        object.__setattr__(self, "angles0", a.copy())
        object.__setattr__(self, "boundary", segments)

    @classmethod
    def from_sc_window(cls, window: SCDynamicWindow, boundary: Sequence[BoundarySegment] | None = None) -> "SIPWindow":
        if boundary is None:
            if window.physical_boundary is None:
                raise ValueError(f"window {window.name!r} has no physical boundary")
            boundary = polygon_segments(window.physical_boundary)
        return cls(window.name, window.center0, window.angles0, window.motion, tuple(boundary))

    def state_at(self, time: float) -> tuple[FloatArray, FloatArray, float]:
        translation, _ = self.motion.translation(float(time))
        delta, rates = self.motion.rotation(float(time))
        scale, _ = self.motion.scale(float(time))
        rotation, _ = rotation_and_derivative(self.angles0 + delta, rates)
        return self.center0 + translation, rotation, float(scale)


@dataclass(frozen=True)
class SIPProblem:
    name: str
    windows: tuple[SIPWindow, ...]
    order: tuple[int, ...]
    track: SCWindowTrack | None = field(default=None, compare=False, repr=False)

    def __post_init__(self) -> None:
        windows, order = tuple(self.windows), tuple(int(v) for v in self.order)
        if not windows or len(order) != len(windows) or set(order) != set(range(len(windows))):
            raise ValueError("the supported problem visits every window exactly once")
        if self.track is not None and self.track.order != order:
            raise ValueError("SIP and SC orders disagree")
        object.__setattr__(self, "windows", windows)
        object.__setattr__(self, "order", order)

    @classmethod
    def from_track(cls, track: SCWindowTrack, boundaries: Sequence[Sequence[BoundarySegment]] | None = None) -> "SIPProblem":
        if boundaries is not None and len(boundaries) != len(track.windows):
            raise ValueError("boundaries must have one entry per window")
        windows = tuple(
            SIPWindow.from_sc_window(w, None if boundaries is None else boundaries[i])
            for i, w in enumerate(track.windows)
        )
        return cls(track.name, windows, track.order, track)


@dataclass(frozen=True)
class PolynomialTrajectory:
    durations: FloatArray
    coefficients: FloatArray

    def __post_init__(self) -> None:
        t, c = np.asarray(self.durations, dtype=float), np.asarray(self.coefficients, dtype=float)
        if t.ndim != 1 or np.any(t <= 0) or not np.all(np.isfinite(t)):
            raise ValueError("durations must be a finite positive vector")
        if c.shape != (len(t), 8, 3) or not np.all(np.isfinite(c)):
            raise ValueError("coefficients must have shape (segments,8,3)")
        object.__setattr__(self, "durations", t.copy())
        object.__setattr__(self, "coefficients", c.copy())

    @classmethod
    def from_minco(cls, trajectory: Any) -> "PolynomialTrajectory":
        return cls(np.asarray(trajectory.durations, dtype=float), np.asarray(trajectory.coefficients, dtype=float))

    @property
    def num_segments(self) -> int:
        return len(self.durations)

    @property
    def total_time(self) -> float:
        return float(np.sum(self.durations))

    def evaluate_segment(self, segment: int, local_time: float, derivative: int = 0) -> FloatArray:
        if not 0 <= segment < self.num_segments or not 0 <= derivative <= 7:
            raise ValueError("invalid segment or derivative")
        t = float(local_time)
        if t < -1e-12 or t > self.durations[segment] + 1e-12:
            raise ValueError("local time outside segment")
        out = np.zeros(3)
        for power in range(derivative, 8):
            out += (factorial(power) / factorial(power - derivative)) * self.coefficients[segment, power] * t ** (power - derivative)
        return out

    def evaluate(self, time: float, derivative: int = 0) -> FloatArray:
        cumulative = np.concatenate(([0.0], np.cumsum(self.durations)))
        segment = min(int(np.searchsorted(cumulative[1:], time, side="right")), self.num_segments - 1)
        return self.evaluate_segment(segment, float(time) - cumulative[segment], derivative)


@dataclass(frozen=True)
class Witness:
    kind: str
    trajectory_segment: int
    normalized_time: float
    residual: float
    window_index: int | None = None
    boundary_segment: int | None = None
    boundary_parameter: float | None = None
    source: str = "interval"

    def key(self) -> tuple[Any, ...]:
        return (self.kind, self.trajectory_segment, round(self.normalized_time, 12), self.window_index, self.boundary_segment, None if self.boundary_parameter is None else round(self.boundary_parameter, 12))

    def to_dict(self) -> dict[str, Any]: return asdict(self)


@dataclass(frozen=True)
class CertificateResult:
    status: CertificateStatus
    reason: str
    precision_bits: int
    checked_cells: int
    maximum_depth: int
    minimum_safety_squared_margin: float | None
    minimum_dynamic_margin: float | None
    witnesses: tuple[Witness, ...] = ()

    @property
    def certified(self) -> bool: return self.status is CertificateStatus.CERTIFIED_FEASIBLE

    def to_dict(self) -> dict[str, Any]:
        return {"status": self.status.value, "reason": self.reason, "precision_bits": self.precision_bits, "checked_cells": self.checked_cells, "maximum_depth": self.maximum_depth, "minimum_safety_squared_margin": self.minimum_safety_squared_margin, "minimum_dynamic_margin": self.minimum_dynamic_margin, "witnesses": [w.to_dict() for w in self.witnesses]}


@dataclass(frozen=True)
class ExchangeRecord:
    iteration: int
    optimizer_success: bool
    total_time: float
    active_witnesses: int
    certificate_status: CertificateStatus
    certificate_cells: int

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self); d["certificate_status"] = self.certificate_status.value; return d


@dataclass(frozen=True)
class SIPResult:
    status: CertificateStatus
    message: str
    x: FloatArray
    trajectory: PolynomialTrajectory
    durations: FloatArray
    traversal_times: FloatArray
    waypoints: FloatArray
    certificate: CertificateResult
    history: tuple[ExchangeRecord, ...]
    optimizer_success: bool
    optimizer_iterations: int
    active_witnesses: tuple[Witness, ...]

    @property
    def success(self) -> bool: return self.status is CertificateStatus.CERTIFIED_FEASIBLE
    @property
    def total_time(self) -> float: return float(np.sum(self.durations))
    def to_dict(self) -> dict[str, Any]:
        return {"status": self.status.value, "message": self.message, "success": self.success, "x": self.x.tolist(), "durations": self.durations.tolist(), "traversal_times": self.traversal_times.tolist(), "waypoints": self.waypoints.tolist(), "total_time": self.total_time, "certificate": self.certificate.to_dict(), "history": [h.to_dict() for h in self.history], "optimizer_success": self.optimizer_success, "optimizer_iterations": self.optimizer_iterations, "active_witnesses": [w.to_dict() for w in self.active_witnesses]}


def segment_to_dict(segment: BoundarySegment) -> dict[str, Any]:
    if isinstance(segment, Line): return {"type": "line", "start": segment.start.tolist(), "end": segment.end.tolist()}
    if isinstance(segment, CircularArc): return {"type": "circular_arc", "center": segment.center.tolist(), "radius": segment.radius, "start_angle": segment.start_angle, "end_angle": segment.end_angle, "ccw": segment.ccw}
    if isinstance(segment, Bezier): return {"type": "bezier", "control_points": segment.control_points.tolist()}
    if isinstance(segment, BSpline): return {"type": "bspline", "control_points": segment.control_points.tolist(), "degree": segment.degree, "knots": np.asarray(segment.knots).tolist()}
    raise TypeError(type(segment).__name__)


def segment_from_dict(d: dict[str, Any]) -> BoundarySegment:
    if d["type"] == "line": return Line(d["start"], d["end"])
    if d["type"] == "circular_arc": return CircularArc(d["center"], d["radius"], d["start_angle"], d["end_angle"], ccw=d["ccw"])
    if d["type"] == "bezier": return Bezier(d["control_points"])
    if d["type"] == "bspline": return BSpline(d["control_points"], degree=d["degree"], knots=d["knots"])
    raise ValueError(f"unknown segment type {d['type']!r}")


def problem_to_dict(problem: SIPProblem) -> dict[str, Any]:
    windows = []
    for w in problem.windows:
        m = w.motion
        windows.append({"name": w.name, "center0": w.center0.tolist(), "angles0": w.angles0.tolist(), "motion": {"translation_amplitude": m.translation_amplitude.tolist(), "rotation_amplitude": m.rotation_amplitude.tolist(), "scale_amplitude": m.scale_amplitude, "translation_period": m.translation_period, "rotation_period": m.rotation_period, "scale_period": m.scale_period, "phase": m.phase, "translation_enabled": m.translation_enabled, "rotation_enabled": m.rotation_enabled, "scale_enabled": m.scale_enabled}, "boundary": [segment_to_dict(s) for s in w.boundary]})
    return {"name": problem.name, "order": list(problem.order), "windows": windows}


def problem_from_dict(data: dict[str, Any]) -> SIPProblem:
    windows = tuple(SIPWindow(w["name"], np.asarray(w["center0"]), np.asarray(w["angles0"]), MotionProfile(**w["motion"]), tuple(segment_from_dict(s) for s in w["boundary"])) for w in data["windows"])
    return SIPProblem(data["name"], windows, tuple(data["order"]))


__all__ = ["CertificateResult", "CertificateStatus", "CuboidBody", "ExchangeRecord", "PolynomialTrajectory", "SIPConfig", "SIPProblem", "SIPResult", "SIPWindow", "Witness", "polygon_segments", "problem_from_dict", "problem_to_dict"]
