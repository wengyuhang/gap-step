"""SC-input curve analytic crossing segments."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import RotatingWindow
from nonconvex_timevarying_window.rot_sync_sc_togt.trajectory import CompositeTrajectory
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState


FloatArray = NDArray[np.float64]
_BEZIER_DEGREE = 7
_BEZIER_CONTROL_COUNT = _BEZIER_DEGREE + 1
_NORMAL_SHAPE_DIMENSION = _BEZIER_DEGREE - 1


def _jet_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Multiply derivative/factorial Taylor coefficients."""

    order = min(len(left), len(right)) - 1
    return np.asarray(
        [sum(left[k] * right[n - k] for k in range(n + 1)) for n in range(order + 1)],
        dtype=np.complex128,
    )


def _jet_reciprocal(values: np.ndarray) -> np.ndarray:
    if abs(values[0]) <= np.finfo(float).tiny:
        raise FloatingPointError("Taylor reciprocal has a zero constant term")
    output = np.zeros_like(values, dtype=np.complex128)
    output[0] = 1.0 / values[0]
    for n in range(1, len(values)):
        output[n] = -sum(
            values[k] * output[n - k] for k in range(1, n + 1)
        ) / values[0]
    return output


def _jet_exp(values: np.ndarray) -> np.ndarray:
    output = np.zeros_like(values, dtype=np.complex128)
    output[0] = np.exp(values[0])
    for n in range(1, len(values)):
        output[n] = sum(
            k * values[k] * output[n - k] for k in range(1, n + 1)
        ) / n
    return output


def _jet_log(values: np.ndarray) -> np.ndarray:
    if abs(values[0]) <= np.finfo(float).tiny:
        raise FloatingPointError("Taylor logarithm has a zero constant term")
    output = np.zeros_like(values, dtype=np.complex128)
    output[0] = np.log(values[0])
    derivative = np.zeros_like(values, dtype=np.complex128)
    derivative[:-1] = np.arange(1, len(values)) * values[1:]
    quotient = _jet_multiply(derivative, _jet_reciprocal(values))
    for n in range(1, len(values)):
        output[n] = quotient[n - 1] / n
    return output


def _jet_power(values: np.ndarray, exponent: float) -> np.ndarray:
    return _jet_exp(float(exponent) * _jet_log(values))


def _bezier_value(control_points: np.ndarray, progress: ArrayLike) -> np.ndarray:
    """Evaluate a Bezier curve for scalar or array-valued progress."""

    controls = np.asarray(control_points, dtype=float)
    if controls.shape[0] != _BEZIER_CONTROL_COUNT:
        raise ValueError(
            f"degree-7 Bezier curves require {_BEZIER_CONTROL_COUNT} control points"
        )
    s = np.asarray(progress, dtype=float)
    weights = np.stack(
        [
            math.comb(_BEZIER_DEGREE, index)
            * (1.0 - s) ** (_BEZIER_DEGREE - index)
            * s**index
            for index in range(_BEZIER_CONTROL_COUNT)
        ],
        axis=-1,
    )
    return np.tensordot(weights, controls, axes=([-1], [0]))


def _bezier_jet(
    control_points: np.ndarray,
    local_time: float,
    duration: float,
    order: int,
) -> np.ndarray:
    """Return derivative/factorial Taylor coefficients of a degree-7 Bezier."""

    controls = np.asarray(control_points, dtype=float)
    if controls.shape[0] != _BEZIER_CONTROL_COUNT:
        raise ValueError(
            f"degree-7 Bezier curves require {_BEZIER_CONTROL_COUNT} control points"
        )
    progress = float(local_time / duration)
    jet = np.zeros((order + 1,) + controls.shape[1:], dtype=float)
    for derivative in range(min(order, _BEZIER_DEGREE) + 1):
        differences = np.diff(controls, n=derivative, axis=0)
        reduced_degree = _BEZIER_DEGREE - derivative
        weights = np.asarray(
            [
                math.comb(reduced_degree, index)
                * (1.0 - progress) ** (reduced_degree - index)
                * progress**index
                for index in range(reduced_degree + 1)
            ],
            dtype=float,
        )
        jet[derivative] = (
            math.comb(_BEZIER_DEGREE, derivative)
            * np.tensordot(weights, differences, axes=(0, 0))
            / duration**derivative
        )
    return jet


def _mapped_latent_sc_jet(
    window: RotatingWindow,
    latent_jet: np.ndarray,
) -> np.ndarray:
    """Map an unconstrained planar Taylor jet through the disk and SC maps."""

    values = np.asarray(latent_jet, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError("latent Taylor jet must have shape (order + 1, 2)")
    size = values.shape[0]
    x = np.asarray(values[:, 0], dtype=np.complex128)
    y = np.asarray(values[:, 1], dtype=np.complex128)

    norm_squared = _jet_multiply(x, x) + _jet_multiply(y, y)
    norm_squared[0] += 1.0
    inverse_norm = _jet_power(norm_squared, -0.5)
    disk = _jet_multiply(x + 1j * y, inverse_norm)
    if abs(disk[0]) >= 1.0:
        raise ValueError("interpolated unconstrained input left the open disk")

    sc_map = window.gate.sc_map
    normalization = complex(sc_map.normalization)
    denominator = np.zeros(size, dtype=np.complex128)
    denominator[0] = 1.0
    denominator += np.conj(normalization) * disk
    numerator = disk.copy()
    numerator[0] += normalization
    transformed = _jet_multiply(numerator, _jet_reciprocal(denominator))

    factor = np.zeros(size, dtype=np.complex128)
    factor[0] = 1.0
    for prevertex, exponent in zip(sc_map.prevertices, sc_map.exponents):
        term = -transformed / prevertex
        term[0] += 1.0
        factor = _jet_multiply(factor, _jet_power(term, float(exponent)))
    automorphism_derivative = (1.0 - abs(normalization) ** 2) * _jet_power(
        denominator, -2.0
    )
    psi_prime = complex(sc_map.C) * _jet_multiply(
        factor, automorphism_derivative
    )

    disk_derivative = np.zeros(size, dtype=np.complex128)
    disk_derivative[:-1] = np.arange(1, size) * disk[1:]
    composed_derivative = _jet_multiply(psi_prime, disk_derivative)
    mapped = np.zeros(size, dtype=np.complex128)
    mapped[0] = sc_map.map_complex(complex(disk[0]))
    for n in range(1, size):
        mapped[n] = composed_derivative[n - 1] / n
    return mapped


def _linear_latent_sc_jet(
    window: RotatingWindow,
    latent_entry: np.ndarray,
    latent_exit: np.ndarray,
    local_time: float,
    duration: float,
    order: int,
) -> np.ndarray:
    """Taylor jet of ``Psi(B(d_entry+s(d_exit-d_entry)))``."""

    latent = np.zeros((order + 1, 2), dtype=float)
    progress = float(local_time / duration)
    delta = np.asarray(latent_exit - latent_entry, dtype=float)
    latent[0] = latent_entry + progress * delta
    if order:
        latent[1] = delta / duration
    return _mapped_latent_sc_jet(window, latent)


def _normal_control_points(distance: float, shape: np.ndarray) -> np.ndarray:
    """Create ordered Bezier controls from six free log-increment ratios."""

    parameters = np.asarray(shape, dtype=float)
    if parameters.shape != (_NORMAL_SHAPE_DIMENSION,):
        raise ValueError(
            f"normal shape must have shape ({_NORMAL_SHAPE_DIMENSION},)"
        )
    logits = np.concatenate((np.zeros(1), parameters))
    shifted = logits - float(np.max(logits))
    weights = np.exp(shifted)
    weights /= np.sum(weights)
    increments = 2.0 * float(distance) * weights
    return np.concatenate(((-float(distance),), -float(distance) + np.cumsum(increments)))


@dataclass(frozen=True)
class SCInputInterpolatedSyncSegment:
    """Crossing formed by interpolation before the disk and SC maps."""

    window: RotatingWindow
    latent_entry: ArrayLike
    latent_exit: ArrayLike
    entry_time: float
    duration: float

    def __post_init__(self) -> None:
        start = np.asarray(self.latent_entry, dtype=float)
        finish = np.asarray(self.latent_exit, dtype=float)
        if start.shape != (2,) or finish.shape != (2,):
            raise ValueError("entry and exit SC inputs must have shape (2,)")
        if not np.all(np.isfinite(start)) or not np.all(np.isfinite(finish)):
            raise ValueError("entry and exit SC inputs must be finite")
        if (
            not np.isfinite(self.entry_time)
            or not np.isfinite(self.duration)
            or self.duration <= 0.0
        ):
            raise ValueError("entry_time must be finite and duration positive")
        object.__setattr__(self, "latent_entry", start)
        object.__setattr__(self, "latent_exit", finish)

    @property
    def total_time(self) -> float:
        return float(self.duration)

    def latent_at(self, local_time: ArrayLike) -> FloatArray:
        tau = np.asarray(local_time, dtype=float)
        tolerance = 1.0e-10 * max(1.0, self.duration)
        if np.any(tau < -tolerance) or np.any(tau > self.duration + tolerance):
            raise ValueError("local_time lies outside the interpolated Sync segment")
        return self.latent_entry + (tau[..., None] / self.duration) * (
            self.latent_exit - self.latent_entry
        )

    def local_point_at(self, local_time: ArrayLike) -> FloatArray:
        latent = self.latent_at(local_time)
        flat = latent.reshape(-1, 2)
        mapped = np.stack([self.window.local_point(value) for value in flat])
        return mapped.reshape(latent.shape)

    def normal_at(self, local_time: ArrayLike, derivative: int = 0) -> FloatArray:
        if derivative < 0:
            raise ValueError("derivative must be nonnegative")
        tau = np.asarray(local_time, dtype=float)
        if derivative == 0:
            distance = self.window.clearance_distance
            return -distance + 2.0 * distance * tau / self.duration
        if derivative == 1:
            return np.full_like(tau, 2.0 * self.window.clearance_distance / self.duration)
        return np.zeros_like(tau)

    @property
    def local_entry_point(self) -> FloatArray:
        return np.asarray(self.local_point_at(0.0), dtype=float)

    @property
    def local_exit_point(self) -> FloatArray:
        return np.asarray(self.local_point_at(self.duration), dtype=float)

    @property
    def local_point(self) -> FloatArray:
        """Midpoint compatibility view for shared visualization code."""

        return np.asarray(self.local_point_at(0.5 * self.duration), dtype=float)

    @property
    def plane_crossing_time(self) -> float:
        return 0.5 * float(self.duration)

    def _evaluate_scalar(self, local_time: float, derivative: int) -> FloatArray:
        order = int(derivative)
        local = _linear_latent_sc_jet(
            self.window,
            self.latent_entry,
            self.latent_exit,
            float(local_time),
            self.duration,
            order,
        )
        theta = np.zeros(order + 1, dtype=np.complex128)
        theta[0] = self.window.theta0 + self.window.omega * (
            self.entry_time + local_time
        )
        if order:
            theta[1] = self.window.omega
        rotated = _jet_multiply(_jet_exp(1j * theta), local)
        planar = math.factorial(order) * rotated[order]
        output = self.window.plane_basis @ np.asarray((planar.real, planar.imag))
        distance = self.window.clearance_distance
        if order == 0:
            z = -distance + 2.0 * distance * local_time / self.duration
            output = output + self.window.center + self.window.normal * z
        elif order == 1:
            output = output + self.window.normal * (2.0 * distance / self.duration)
        return np.asarray(output, dtype=float)

    def evaluate(self, local_time: ArrayLike, derivative: int = 0) -> FloatArray:
        if derivative < 0:
            raise ValueError("derivative must be nonnegative")
        tau = np.asarray(local_time, dtype=float)
        tolerance = 1.0e-10 * max(1.0, self.duration)
        if np.any(tau < -tolerance) or np.any(tau > self.duration + tolerance):
            raise ValueError("local_time lies outside the interpolated Sync segment")
        values = np.stack(
            [self._evaluate_scalar(float(value), derivative) for value in tau.reshape(-1)]
        )
        return values.reshape(tau.shape + (3,))

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
        nodes, weights = np.polynomial.legendre.leggauss(16)
        times = 0.5 * self.duration * (nodes + 1.0)
        snap = self.evaluate(times, 4)
        return float(
            0.5
            * self.duration
            * np.sum(weights * np.einsum("ij,ij->i", snap, snap))
        )


@dataclass(frozen=True)
class SCInputSplineSyncSegment:
    """Degree-7 latent-space and monotone-normal Bezier crossing."""

    window: RotatingWindow
    latent_control_points: ArrayLike
    normal_shape: ArrayLike
    entry_time: float
    duration: float

    def __post_init__(self) -> None:
        controls = np.asarray(self.latent_control_points, dtype=float)
        shape = np.asarray(self.normal_shape, dtype=float)
        if controls.shape != (_BEZIER_CONTROL_COUNT, 2):
            raise ValueError(
                f"latent_control_points must have shape ({_BEZIER_CONTROL_COUNT}, 2)"
            )
        if shape.shape != (_NORMAL_SHAPE_DIMENSION,):
            raise ValueError(
                f"normal_shape must have shape ({_NORMAL_SHAPE_DIMENSION},)"
            )
        if not np.all(np.isfinite(controls)) or not np.all(np.isfinite(shape)):
            raise ValueError("Bezier parameters must be finite")
        if (
            not np.isfinite(self.entry_time)
            or not np.isfinite(self.duration)
            or self.duration <= 0.0
        ):
            raise ValueError("entry_time must be finite and duration positive")
        object.__setattr__(self, "latent_control_points", controls)
        object.__setattr__(self, "normal_shape", shape)

    @property
    def total_time(self) -> float:
        return float(self.duration)

    @property
    def normal_control_points(self) -> FloatArray:
        return _normal_control_points(
            self.window.clearance_distance, np.asarray(self.normal_shape)
        )

    def _check_time(self, local_time: ArrayLike) -> np.ndarray:
        tau = np.asarray(local_time, dtype=float)
        tolerance = 1.0e-10 * max(1.0, self.duration)
        if np.any(tau < -tolerance) or np.any(tau > self.duration + tolerance):
            raise ValueError("local_time lies outside the spline Sync segment")
        return tau

    def latent_at(self, local_time: ArrayLike) -> FloatArray:
        tau = self._check_time(local_time)
        return np.asarray(
            _bezier_value(self.latent_control_points, tau / self.duration),
            dtype=float,
        )

    def local_point_at(self, local_time: ArrayLike) -> FloatArray:
        latent = self.latent_at(local_time)
        flat = latent.reshape(-1, 2)
        mapped = np.stack([self.window.local_point(value) for value in flat])
        return mapped.reshape(latent.shape)

    def normal_at(self, local_time: ArrayLike, derivative: int = 0) -> FloatArray:
        if derivative < 0:
            raise ValueError("derivative must be nonnegative")
        tau = self._check_time(local_time)
        flat = tau.reshape(-1)
        values = []
        for instant in flat:
            jet = _bezier_jet(
                self.normal_control_points,
                float(instant),
                self.duration,
                derivative,
            )
            values.append(math.factorial(derivative) * jet[derivative])
        return np.asarray(values, dtype=float).reshape(tau.shape)

    @property
    def local_entry_point(self) -> FloatArray:
        return np.asarray(self.local_point_at(0.0), dtype=float)

    @property
    def local_exit_point(self) -> FloatArray:
        return np.asarray(self.local_point_at(self.duration), dtype=float)

    @property
    def local_point(self) -> FloatArray:
        return np.asarray(self.local_point_at(self.plane_crossing_time), dtype=float)

    @property
    def plane_crossing_time(self) -> float:
        """Return the unique local time at which the normal coordinate is zero."""

        left, right = 0.0, float(self.duration)
        for _ in range(60):
            middle = 0.5 * (left + right)
            if float(self.normal_at(middle)) < 0.0:
                left = middle
            else:
                right = middle
        return 0.5 * (left + right)

    def _evaluate_scalar(self, local_time: float, derivative: int) -> FloatArray:
        order = int(derivative)
        latent_jet = _bezier_jet(
            self.latent_control_points,
            local_time,
            self.duration,
            order,
        )
        local = _mapped_latent_sc_jet(self.window, latent_jet)
        theta = np.zeros(order + 1, dtype=np.complex128)
        theta[0] = self.window.theta0 + self.window.omega * (
            self.entry_time + local_time
        )
        if order:
            theta[1] = self.window.omega
        rotated = _jet_multiply(_jet_exp(1j * theta), local)
        planar = math.factorial(order) * rotated[order]
        output = self.window.plane_basis @ np.asarray((planar.real, planar.imag))
        normal = float(self.normal_at(local_time, order))
        output = output + self.window.normal * normal
        if order == 0:
            output = output + self.window.center
        return np.asarray(output, dtype=float)

    def evaluate(self, local_time: ArrayLike, derivative: int = 0) -> FloatArray:
        if derivative < 0:
            raise ValueError("derivative must be nonnegative")
        tau = self._check_time(local_time)
        values = np.stack(
            [self._evaluate_scalar(float(value), derivative) for value in tau.reshape(-1)]
        )
        return values.reshape(tau.shape + (3,))

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
        nodes, weights = np.polynomial.legendre.leggauss(16)
        times = 0.5 * self.duration * (nodes + 1.0)
        snap = self.evaluate(times, 4)
        return float(
            0.5
            * self.duration
            * np.sum(weights * np.einsum("ij,ij->i", snap, snap))
        )


__all__ = [
    "CompositeTrajectory",
    "SCInputInterpolatedSyncSegment",
    "SCInputSplineSyncSegment",
]
