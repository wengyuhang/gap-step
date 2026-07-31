"""Continuous deforming-gate interfaces and scenario serialization."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, ClassVar

import numpy as np
from scipy.interpolate import PchipInterpolator
from shapely.geometry import Polygon
from shapely.geometry.base import BaseGeometry

from .geometry import rotation_and_derivative, validate_simple_polygon


@dataclass
class SplineSeries:
    """PCHIP control series with deterministic value and derivative queries."""

    times: np.ndarray
    values: np.ndarray

    def __post_init__(self) -> None:
        self.times = np.asarray(self.times, dtype=float)
        self.values = np.asarray(self.values, dtype=float)
        if self.times.ndim != 1 or len(self.times) < 2:
            raise ValueError("spline times must contain at least two entries")
        if self.values.shape[0] != len(self.times):
            raise ValueError("spline values must use the same leading dimension as times")
        if np.any(np.diff(self.times) <= 0.0):
            raise ValueError("spline times must be strictly increasing")
        if not np.all(np.isfinite(self.values)):
            raise ValueError("spline values must be finite")
        self._curve = PchipInterpolator(
            self.times, self.values, axis=0, extrapolate=False
        )
        self._derivative = self._curve.derivative()

    def value(self, time: float):
        value = np.asarray(self._curve(float(time)))
        return float(value) if value.ndim == 0 else value.astype(float)

    def value_and_derivative(self, time: float):
        value = np.asarray(self._curve(float(time)))
        derivative = np.asarray(self._derivative(float(time)))
        if value.ndim == 0:
            return float(value), float(derivative)
        return value.astype(float), derivative.astype(float)

    def to_dict(self) -> dict[str, Any]:
        return {"times": self.times.tolist(), "values": self.values.tolist()}

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SplineSeries":
        return cls(np.asarray(data["times"], dtype=float), np.asarray(data["values"], dtype=float))


@dataclass
class DynamicGate:
    """Base interface for a planar, continuously deforming physical opening."""

    gate_id: int
    name: str
    center_profile: SplineSeries
    rpy_profile: SplineSeries
    scale_profile: SplineSeries
    deformation_profile: SplineSeries
    boundary_samples: int = 256
    shape_kind: ClassVar[str] = "abstract"

    def __post_init__(self) -> None:
        if self.center_profile.values.shape[1:] != (3,):
            raise ValueError("center profile must contain three-vectors")
        if self.rpy_profile.values.shape[1:] != (3,):
            raise ValueError("RPY profile must contain three-vectors")
        if self.boundary_samples < 32:
            raise ValueError("boundary_samples must be at least 32")

    def _unit_polygon(self, deformation: float) -> Polygon:
        raise NotImplementedError

    def pose(self, time: float) -> tuple[np.ndarray, np.ndarray]:
        center, _ = self.center_profile.value_and_derivative(time)
        rpy, _ = self.rpy_profile.value_and_derivative(time)
        rotation, _ = rotation_and_derivative(rpy, np.zeros(3))
        return center, rotation

    def pose_with_derivative(
        self, time: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        center, center_dot = self.center_profile.value_and_derivative(time)
        rpy, rpy_dot = self.rpy_profile.value_and_derivative(time)
        rotation, rotation_dot = rotation_and_derivative(rpy, rpy_dot)
        return center, rotation, center_dot, rotation_dot

    def local_polygon(self, time: float) -> Polygon:
        scale = float(self.scale_profile.value(time))
        deformation = float(self.deformation_profile.value(time))
        if not np.isfinite(scale) or scale <= 0.0:
            raise ValueError("gate scale must remain positive")
        base = self._unit_polygon(deformation)
        coordinates = np.asarray(base.exterior.coords[:-1], dtype=float) * scale
        polygon = Polygon(coordinates)
        validate_simple_polygon(polygon)
        return polygon

    def safe_polygon(self, time: float, safety_radius: float) -> BaseGeometry:
        if safety_radius < 0.0:
            raise ValueError("safety radius must be nonnegative")
        safe = self.local_polygon(time).buffer(-float(safety_radius))
        if safe.is_empty:
            return safe
        if not safe.is_valid:
            raise ValueError("safe polygon offset is invalid")
        return safe

    def local_to_world(self, point: np.ndarray, time: float) -> np.ndarray:
        local = np.asarray(point, dtype=float)
        if local.shape[-1] != 2:
            raise ValueError("local gate points must have final dimension two")
        center, rotation = self.pose(time)
        basis = rotation[:, :2]
        if local.ndim == 1:
            return center + basis @ local
        return center[None, :] + local @ basis.T

    def world_to_local(
        self, point: np.ndarray, time: float
    ) -> tuple[np.ndarray, float]:
        world = np.asarray(point, dtype=float)
        center, rotation = self.pose(time)
        relative = world - center
        normal = rotation[:, 2]
        return rotation[:, :2].T @ relative, abs(float(relative @ normal))

    def boundary_world(self, time: float) -> np.ndarray:
        coordinates = np.asarray(self.local_polygon(time).exterior.coords[:-1])
        return self.local_to_world(coordinates, time)

    def validate_over(self, times: np.ndarray) -> None:
        for time in np.asarray(times, dtype=float):
            validate_simple_polygon(self.local_polygon(float(time)))

    def to_dict(self) -> dict[str, Any]:
        return {
            "gate_id": self.gate_id,
            "name": self.name,
            "shape_kind": self.shape_kind,
            "center_profile": self.center_profile.to_dict(),
            "rpy_profile": self.rpy_profile.to_dict(),
            "scale_profile": self.scale_profile.to_dict(),
            "deformation_profile": self.deformation_profile.to_dict(),
            "boundary_samples": self.boundary_samples,
        }


@dataclass(frozen=True)
class EndpointState:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    yaw: float = 0.0

    def __post_init__(self) -> None:
        for name in ("position", "velocity", "acceleration", "jerk"):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != (3,) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must be a finite three-vector")
            object.__setattr__(self, name, value)

    @property
    def pvaj(self) -> np.ndarray:
        return np.stack(
            (self.position, self.velocity, self.acceleration, self.jerk), axis=0
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "position": self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "acceleration": self.acceleration.tolist(),
            "jerk": self.jerk.tolist(),
            "yaw": self.yaw,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "EndpointState":
        return cls(
            np.asarray(data["position"], dtype=float),
            np.asarray(data["velocity"], dtype=float),
            np.asarray(data["acceleration"], dtype=float),
            np.asarray(data["jerk"], dtype=float),
            float(data.get("yaw", 0.0)),
        )


@dataclass
class Scenario:
    name: str
    seed: int
    horizon: float
    gates: tuple[DynamicGate, ...]
    order: tuple[int, ...]
    start: EndpointState
    difficulty: str
    closed_ratio: float

    def __post_init__(self) -> None:
        self.gates = tuple(self.gates)
        self.order = tuple(int(value) for value in self.order)
        if self.order != tuple(range(len(self.gates))):
            raise ValueError("MDG scenarios require each gate once in fixed order")
        if not self.gates or self.horizon <= 0.0:
            raise ValueError("scenario requires gates and a positive horizon")

    @property
    def goal(self) -> EndpointState:
        return self.start

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "seed": self.seed,
            "horizon": self.horizon,
            "gates": [gate.to_dict() for gate in self.gates],
            "order": list(self.order),
            "start": self.start.to_dict(),
            "difficulty": self.difficulty,
            "closed_ratio": self.closed_ratio,
        }

    def save(self, path: str | Path) -> Path:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        return target

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Scenario":
        from .gate_shapes import gate_from_dict

        return cls(
            name=str(data["name"]),
            seed=int(data["seed"]),
            horizon=float(data["horizon"]),
            gates=tuple(gate_from_dict(item) for item in data["gates"]),
            order=tuple(data["order"]),
            start=EndpointState.from_dict(data["start"]),
            difficulty=str(data["difficulty"]),
            closed_ratio=float(data["closed_ratio"]),
        )

    @classmethod
    def load(cls, path: str | Path) -> "Scenario":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


__all__ = [
    "DynamicGate",
    "EndpointState",
    "Scenario",
    "SplineSeries",
]

