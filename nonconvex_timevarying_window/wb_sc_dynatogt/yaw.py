"""A scalar degree-seven minimum-snap yaw trajectory."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap


FloatArray = NDArray[np.float64]


def yaw_from_unconstrained(values: ArrayLike) -> FloatArray:
    """Map unconstrained values smoothly to unwrapped yaw in ``(-pi, pi)``."""

    array = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(array)):
        raise ValueError("yaw variables must be finite")
    return 2.0 * np.arctan(array)


def yaw_to_unconstrained(yaw: ArrayLike) -> FloatArray:
    """Inverse of :func:`yaw_from_unconstrained` away from ``+-pi``."""

    array = np.asarray(yaw, dtype=float)
    if np.any(np.abs(array) >= np.pi):
        raise ValueError("yaw values must lie strictly inside (-pi, pi)")
    return np.tan(0.5 * array)


@dataclass(frozen=True)
class YawTrajectory:
    """Continuous yaw/yaw-rate/yaw-acceleration using the MINCO solve."""

    waypoints: FloatArray
    durations: FloatArray
    start_yaw: float = 0.0
    end_yaw: float = 0.0

    def __post_init__(self) -> None:
        values = np.asarray(self.waypoints, dtype=float)
        times = np.asarray(self.durations, dtype=float)
        if values.ndim != 1 or times.ndim != 1 or len(times) != len(values) + 1:
            raise ValueError("yaw waypoints must have one fewer entry than durations")
        if not np.all(np.isfinite(values)) or not np.all(np.isfinite(times)):
            raise ValueError("yaw trajectory values must be finite")
        if np.any(times <= 0.0):
            raise ValueError("yaw trajectory durations must be positive")
        points = np.zeros((len(values), 3), dtype=float)
        points[:, 0] = values
        start = BoundaryState(np.array([float(self.start_yaw), 0.0, 0.0]))
        finish = BoundaryState(np.array([float(self.end_yaw), 0.0, 0.0]))
        trajectory = MincoSnap(start, finish, points, times)
        object.__setattr__(self, "waypoints", values.copy())
        object.__setattr__(self, "durations", times.copy())
        object.__setattr__(self, "_trajectory", trajectory)

    @property
    def coefficients(self) -> FloatArray:
        return np.asarray(self._trajectory.coefficients[:, :, 0], dtype=float)

    @property
    def minco(self) -> MincoSnap:
        return self._trajectory

    @property
    def total_time(self) -> float:
        return float(np.sum(self.durations))

    def evaluate(self, time: ArrayLike, derivative: int = 0):
        values = self._trajectory.evaluate(time, derivative)
        result = np.asarray(values)[..., 0]
        return result.item() if result.ndim == 0 else result

    def evaluate_segment(self, segment: int, time: ArrayLike, derivative: int = 0):
        values = self._trajectory.evaluate_segment(segment, time, derivative)
        result = np.asarray(values)[..., 0]
        return result.item() if result.ndim == 0 else result

    def to_dict(self) -> dict[str, object]:
        return {
            "waypoints": self.waypoints.tolist(),
            "durations": self.durations.tolist(),
            "start_yaw": float(self.start_yaw),
            "end_yaw": float(self.end_yaw),
            "coefficients": self.coefficients.tolist(),
        }


__all__ = ["YawTrajectory", "yaw_from_unconstrained", "yaw_to_unconstrained"]
