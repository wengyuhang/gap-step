"""Analytic Sync segments and C3 MINCO/Sync composition."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.minco import (
    BoundaryState,
    MincoSnap,
    TrajectorySamples,
)

from .geometry import RotatingWindow, rotation_2d


FloatArray = NDArray[np.float64]
_J = np.asarray(((0.0, -1.0), (1.0, 0.0)))


@dataclass(frozen=True)
class RotationSyncSegment:
    """Exact helical crossing with a fixed planar coordinate in the gate frame."""

    window: RotatingWindow
    local_point: ArrayLike
    entry_time: float
    duration: float

    def __post_init__(self) -> None:
        q = np.asarray(self.local_point, dtype=float)
        if q.shape != (2,) or not np.all(np.isfinite(q)):
            raise ValueError("local_point must be a finite two-vector")
        if not np.isfinite(self.entry_time) or not np.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError("entry_time must be finite and duration positive")
        object.__setattr__(self, "local_point", q)

    @property
    def total_time(self) -> float:
        return float(self.duration)

    def evaluate(self, local_time: ArrayLike, derivative: int = 0) -> NDArray[np.float64]:
        """Return an analytic derivative; P/V/A/J follow equations (5), (11)-(13)."""

        if derivative < 0:
            raise ValueError("derivative must be nonnegative")
        tau = np.asarray(local_time, dtype=float)
        tolerance = 1.0e-10 * max(1.0, self.duration)
        if np.any(tau < -tolerance) or np.any(tau > self.duration + tolerance):
            raise ValueError("local_time lies outside the Sync segment")
        theta = self.window.theta0 + self.window.omega * (self.entry_time + tau)
        flat_theta = theta.reshape(-1)
        rotated = np.stack(
            [rotation_2d(value) @ np.linalg.matrix_power(_J, derivative) @ self.local_point for value in flat_theta]
        ).reshape(theta.shape + (2,))
        planar = np.einsum("ij,...j->...i", self.window.plane_basis, rotated)
        planar = planar * self.window.omega**derivative
        if derivative == 0:
            distance = self.window.clearance_distance
            z = -distance + 2.0 * distance * tau / self.duration
            planar = planar + self.window.center + z[..., None] * self.window.normal
        elif derivative == 1:
            planar = planar + self.window.normal * (2.0 * self.window.clearance_distance / self.duration)
        return planar

    def state(self, local_time: float) -> BoundaryState:
        return BoundaryState(
            self.evaluate(local_time, 0),
            self.evaluate(local_time, 1),
            self.evaluate(local_time, 2),
            self.evaluate(local_time, 3),
        )

    @property
    def entry_state(self) -> BoundaryState:
        return self.state(0.0)

    @property
    def exit_state(self) -> BoundaryState:
        return self.state(self.duration)

    def snap_energy(self) -> float:
        # ||E R q omega^4|| is constant because E and R are orthonormal.
        return float(self.duration * np.dot(self.local_point, self.local_point) * self.window.omega**8)


class CompositeTrajectory:
    """Alternating one-piece degree-7 MINCO and analytic Sync segments."""

    degree = 7
    derivative_order = 4

    def __init__(
        self,
        free_segments: Sequence[MincoSnap],
        sync_segments: Sequence[RotationSyncSegment],
    ) -> None:
        self.free_segments = tuple(free_segments)
        self.sync_segments = tuple(sync_segments)
        if len(self.free_segments) != len(self.sync_segments) + 1:
            raise ValueError("N Sync segments require N+1 free MINCO segments")
        components: list[MincoSnap | RotationSyncSegment] = []
        kinds: list[str] = []
        for index, free in enumerate(self.free_segments):
            if free.num_segments != 1:
                raise ValueError("each free MINCO segment must contain exactly one degree-7 piece")
            components.append(free)
            kinds.append("minco")
            if index < len(self.sync_segments):
                components.append(self.sync_segments[index])
                kinds.append("sync")
        self.components = tuple(components)
        self.segment_kinds = tuple(kinds)
        self._durations = np.asarray([float(item.total_time) for item in self.components])

    @property
    def durations(self) -> FloatArray:
        return self._durations.copy()

    @property
    def coefficients(self) -> FloatArray:
        # Only the dtype is inspected by the reused dynamics quadrature.
        return np.zeros((self.num_segments, 1, 3), dtype=float)

    @property
    def num_segments(self) -> int:
        return len(self.components)

    @property
    def piece_num(self) -> int:
        return self.num_segments

    @property
    def total_time(self) -> float:
        return float(np.sum(self._durations))

    def evaluate_segment(self, segment: int, local_time: ArrayLike, derivative: int = 0):
        if segment < 0 or segment >= self.num_segments:
            raise IndexError("segment index is out of range")
        component = self.components[segment]
        if isinstance(component, MincoSnap):
            return component.evaluate_segment(0, local_time, derivative)
        return component.evaluate(local_time, derivative)

    def evaluate(self, time: ArrayLike, derivative: int = 0):
        query = np.asarray(time, dtype=float)
        flat = query.reshape(-1)
        cumulative = np.r_[0.0, np.cumsum(self._durations)]
        tolerance = 1.0e-10 * max(1.0, self.total_time)
        if np.any(flat < -tolerance) or np.any(flat > self.total_time + tolerance):
            raise ValueError("time lies outside the composite trajectory")
        output = np.empty((flat.size, 3), dtype=float)
        for index, instant in enumerate(flat):
            segment = min(int(np.searchsorted(cumulative[1:], instant, side="right")), self.num_segments - 1)
            output[index] = self.evaluate_segment(segment, instant - cumulative[segment], derivative)
        return output.reshape(query.shape + (3,))

    __call__ = evaluate

    def sample(
        self,
        *,
        times: ArrayLike | None = None,
        num_samples: int | None = None,
        samples_per_segment: int | None = None,
    ) -> TrajectorySamples:
        if sum(value is not None for value in (times, num_samples, samples_per_segment)) > 1:
            raise ValueError("specify only one sampling mode")
        if times is not None:
            grid = np.asarray(times, dtype=float)
        elif samples_per_segment is not None:
            if samples_per_segment < 2:
                raise ValueError("samples_per_segment must be at least two")
            chunks = []
            elapsed = 0.0
            for index, duration in enumerate(self._durations):
                local = np.linspace(0.0, duration, samples_per_segment)
                chunks.append(elapsed + (local if index == 0 else local[1:]))
                elapsed += duration
            grid = np.concatenate(chunks)
        else:
            count = 101 if num_samples is None else int(num_samples)
            if count < 2:
                raise ValueError("num_samples must be at least two")
            grid = np.linspace(0.0, self.total_time, count)
        return TrajectorySamples(
            time=grid,
            position=self.evaluate(grid, 0),
            velocity=self.evaluate(grid, 1),
            acceleration=self.evaluate(grid, 2),
            jerk=self.evaluate(grid, 3),
            snap=self.evaluate(grid, 4),
            crackle=self.evaluate(grid, 5),
        )

    def snap_energy(self) -> float:
        free = sum(float(segment.snap_energy()) for segment in self.free_segments)
        sync = sum(segment.snap_energy() for segment in self.sync_segments)
        return free + sync

    energy = snap_energy

    def interface_residuals(self) -> FloatArray:
        """Return absolute P/V/A/J jumps at every MINCO/Sync interface."""

        residuals = []
        for index, sync in enumerate(self.sync_segments):
            before = self.free_segments[index]
            after = self.free_segments[index + 1]
            residuals.append(
                [np.linalg.norm(before.evaluate(before.total_time, order) - sync.evaluate(0.0, order)) for order in range(4)]
            )
            residuals.append(
                [np.linalg.norm(sync.evaluate(sync.duration, order) - after.evaluate(0.0, order)) for order in range(4)]
            )
        return np.asarray(residuals, dtype=float)


__all__ = ["CompositeTrajectory", "RotationSyncSegment"]
