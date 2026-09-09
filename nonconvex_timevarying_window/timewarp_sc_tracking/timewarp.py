"""Compactly supported time warps for multi-window reference tracking.

Each patch changes progress only inside a finite interval.  Position through
jerk exactly match the nominal trajectory at both ends, and the time map is
the identity outside all patches.  A correction around gate i therefore does
not shift any later gate whose crossing lies after the patch recovery time.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.sc_dynatogt.minco import TrajectorySamples


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class TimeWarpPatch:
    start_time: float
    recovery_time: float
    peak_shift: float

    def __post_init__(self) -> None:
        if not np.all(
            np.isfinite((self.start_time, self.recovery_time, self.peak_shift))
        ):
            raise ValueError("time-warp patch parameters must be finite")
        if self.start_time < 0.0 or self.recovery_time <= self.start_time:
            raise ValueError("time-warp patch must have positive duration")

    @property
    def duration(self) -> float:
        return self.recovery_time - self.start_time


# b(s) = 256 s^4 (1-s)^4, stored in ascending powers.  It is one at s=1/2
# and has derivatives 0..3 equal to zero at both endpoints.
_BUMP = np.asarray((0, 0, 0, 0, 256, -1024, 1536, -1024, 256), dtype=float)


def _poly_derivative_value(coefficients: FloatArray, value: float, order: int) -> float:
    derived = np.asarray(coefficients, dtype=float)
    for _ in range(order):
        derived = np.arange(1, len(derived), dtype=float) * derived[1:]
        if not len(derived):
            return 0.0
    return float(np.polynomial.polynomial.polyval(value, derived))


def _jet_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    order = min(len(left), len(right)) - 1
    output = np.zeros(order + 1, dtype=float)
    for degree in range(order + 1):
        output[degree] = sum(
            left[index] * right[degree - index] for index in range(degree + 1)
        )
    return output


class LocalTimeWarpTrajectory:
    """Follow the nominal spatial curve with local, schedule-neutral timing."""

    def __init__(self, nominal, patches: tuple[TimeWarpPatch, ...]) -> None:
        self.nominal = nominal
        self.patches = tuple(sorted(patches, key=lambda patch: patch.start_time))
        previous = 0.0
        for patch in self.patches:
            if patch.start_time < previous - 1.0e-12:
                raise ValueError("time-warp patches must not overlap")
            if patch.recovery_time > float(nominal.total_time) + 1.0e-12:
                raise ValueError("time-warp patch extends beyond the trajectory")
            previous = patch.recovery_time
        probe = np.linspace(0.0, float(nominal.total_time), 4001)
        if np.min(self.time_derivative(probe)) <= 0.05:
            raise ValueError("time warp must preserve strictly increasing progress")

    @property
    def total_time(self) -> float:
        return float(self.nominal.total_time)

    @property
    def durations(self) -> FloatArray:
        nominal_knots = np.r_[0.0, np.cumsum(np.asarray(self.nominal.durations, dtype=float))]
        patch_knots = np.asarray(
            [value for patch in self.patches for value in (patch.start_time, patch.recovery_time)],
            dtype=float,
        )
        knots = np.unique(np.r_[nominal_knots, patch_knots])
        return np.diff(knots)

    def _active_patch(self, time: float) -> TimeWarpPatch | None:
        for patch in self.patches:
            if patch.start_time < time < patch.recovery_time:
                return patch
        return None

    def _time_derivative_scalar(self, time: float, order: int) -> float:
        patch = self._active_patch(float(time))
        if patch is None:
            return float(time) if order == 0 else (1.0 if order == 1 else 0.0)
        progress = (float(time) - patch.start_time) / patch.duration
        if order == 0:
            return float(time) + patch.peak_shift * _poly_derivative_value(
                _BUMP, progress, 0
            )
        base = 1.0 if order == 1 else 0.0
        return base + patch.peak_shift * _poly_derivative_value(
            _BUMP, progress, order
        ) / patch.duration**order

    def warp_time(self, time: ArrayLike):
        query = np.asarray(time, dtype=float)
        output = np.asarray(
            [self._time_derivative_scalar(value, 0) for value in query.reshape(-1)]
        ).reshape(query.shape)
        return float(output) if output.ndim == 0 else output

    def time_derivative(self, time: ArrayLike):
        query = np.asarray(time, dtype=float)
        output = np.asarray(
            [self._time_derivative_scalar(value, 1) for value in query.reshape(-1)]
        ).reshape(query.shape)
        return float(output) if output.ndim == 0 else output

    def _evaluate_scalar(self, time: float, derivative: int) -> FloatArray:
        patch = self._active_patch(time)
        if patch is None:
            return np.asarray(self.nominal.evaluate(time, derivative), dtype=float)
        order = int(derivative)
        warped = self._time_derivative_scalar(time, 0)
        delta = np.zeros(order + 1, dtype=float)
        for degree in range(1, order + 1):
            delta[degree] = self._time_derivative_scalar(time, degree) / math.factorial(degree)
        result = np.zeros((order + 1, 3), dtype=float)
        power = np.zeros(order + 1, dtype=float)
        power[0] = 1.0
        for nominal_order in range(order + 1):
            coefficient = np.asarray(
                self.nominal.evaluate(warped, nominal_order), dtype=float
            ) / math.factorial(nominal_order)
            result += power[:, None] * coefficient[None, :]
            power = _jet_multiply(power, delta)
        return result[order] * math.factorial(order)

    def evaluate(self, time: ArrayLike, derivative: int = 0):
        if derivative < 0:
            raise ValueError("derivative order must be nonnegative")
        query = np.asarray(time, dtype=float)
        if np.any(query < -1.0e-12) or np.any(query > self.total_time + 1.0e-12):
            raise ValueError("query lies outside the trajectory")
        scalar = query.ndim == 0
        output = np.stack(
            [self._evaluate_scalar(float(value), derivative) for value in query.reshape(-1)]
        ).reshape(query.shape + (3,))
        return output[()] if scalar else output

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
            chunks = []
            elapsed = 0.0
            for index, duration in enumerate(self.durations):
                local = np.linspace(0.0, duration, int(samples_per_segment))
                if index:
                    local = local[1:]
                chunks.append(elapsed + local)
                elapsed += duration
            grid = np.concatenate(chunks)
        else:
            grid = np.linspace(0.0, self.total_time, 101 if num_samples is None else int(num_samples))
        return TrajectorySamples(
            time=grid,
            position=self.evaluate(grid, 0),
            velocity=self.evaluate(grid, 1),
            acceleration=self.evaluate(grid, 2),
            jerk=self.evaluate(grid, 3),
            snap=self.evaluate(grid, 4),
            crackle=self.evaluate(grid, 5),
        )

    def interface_residuals(self) -> FloatArray:
        # b and its first three derivatives vanish at both patch boundaries.
        # Retain the nominal trajectory's own interface residuals rather than
        # claiming that an imperfect nominal reference became continuous.
        residuals = np.zeros(8 * len(self.patches), dtype=float)
        if hasattr(self.nominal, "interface_residuals"):
            residuals = np.r_[
                residuals,
                np.ravel(np.asarray(self.nominal.interface_residuals(), dtype=float)),
            ]
        elif hasattr(self.nominal, "evaluate_segment"):
            nominal_durations = np.asarray(self.nominal.durations, dtype=float)
            nominal_residuals = []
            for index in range(len(nominal_durations) - 1):
                for derivative in range(4):
                    left = self.nominal.evaluate_segment(
                        index, nominal_durations[index], derivative
                    )
                    right = self.nominal.evaluate_segment(
                        index + 1, 0.0, derivative
                    )
                    nominal_residuals.append(np.linalg.norm(left - right))
            residuals = np.r_[residuals, nominal_residuals]
        return residuals


__all__ = ["LocalTimeWarpTrajectory", "TimeWarpPatch"]
