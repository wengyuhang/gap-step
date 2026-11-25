"""Hand-written convex-frame safety integral and MINCO adjoint gradient."""

from __future__ import annotations

from dataclasses import dataclass
from math import factorial

import numpy as np

from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore


@dataclass(frozen=True)
class ConvexSafetyConfig:
    body_radius: float
    extra_optimization_margin: float = 0.030
    smoothmax_temperature_m: float = 0.01
    check_time_sec: float = 0.02
    min_num_check: int = 16
    max_num_check: int = 64
    smoothing_epsilon_m2: float = 0.01
    objective_weight: float = 100000.0

    def __post_init__(self):
        if self.body_radius <= 0 or self.extra_optimization_margin < 0:
            raise ValueError("invalid planning radius")
        if self.smoothmax_temperature_m <= 0 or self.check_time_sec <= 0:
            raise ValueError("invalid smooth-max or quadrature setting")
        if not 1 <= self.min_num_check <= self.max_num_check:
            raise ValueError("invalid quadrature bounds")
        if self.smoothing_epsilon_m2 <= 0 or self.objective_weight < 0:
            raise ValueError("invalid penalty setting")

    @property
    def planning_radius(self):
        return self.body_radius + self.extra_optimization_margin


@dataclass(frozen=True)
class PolygonHalfspaces:
    normals: np.ndarray
    offsets: np.ndarray


@dataclass(frozen=True)
class SafetyIntegralEvaluation:
    value: float
    waypoint_gradient: np.ndarray
    duration_gradient: np.ndarray
    per_window: tuple[float, ...]
    nodes: int


def polygon_halfspaces(vertices) -> PolygonHalfspaces:
    """Convert a convex ordered polygon to unit outward halfspaces."""
    vertices = np.asarray(vertices, dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3:
        raise ValueError("polygon vertices must have shape (n>=3, 2)")
    signed_twice_area = np.sum(
        vertices[:, 0] * np.roll(vertices[:, 1], -1)
        - vertices[:, 1] * np.roll(vertices[:, 0], -1)
    )
    if abs(signed_twice_area) <= 1e-12:
        raise ValueError("degenerate polygon")
    edges = np.roll(vertices, -1, axis=0) - vertices
    lengths = np.linalg.norm(edges, axis=1)
    if np.any(lengths <= 1e-12):
        raise ValueError("polygon has a zero-length edge")
    if signed_twice_area > 0:
        normals = np.column_stack((edges[:, 1], -edges[:, 0])) / lengths[:, None]
    else:
        normals = np.column_stack((-edges[:, 1], edges[:, 0])) / lengths[:, None]
    offsets = np.einsum("ij,ij->i", normals, vertices)
    if np.any(vertices @ normals.T - offsets > 1e-8):
        raise ValueError("vertices do not define a convex ordered polygon")
    return PolygonHalfspaces(normals, offsets)


def _smoothed_l1(value: float, mu: float):
    if value < 0.0:
        return 0.0, 0.0
    if value > mu:
        return value - 0.5 * mu, 1.0
    ratio = value / mu
    cost = (mu - 0.5 * value) * ratio**3
    derivative = ratio**2 * (-0.5 * ratio + 3.0 * (mu - 0.5 * value) / mu)
    return cost, derivative


def _basis(time: float, derivative: int):
    result = np.zeros(8)
    for power in range(derivative, 8):
        result[power] = factorial(power) / factorial(power - derivative) * time ** (power - derivative)
    return result


class HandwrittenConvexSafetyIntegral:
    """Evaluate the prior frame-distance residual with analytic convex fields.

    For each moving frame, ``g = r_plan^2 - z^2 - h(q)^2``.  Polygon ``h``
    is the requested smooth maximum of unit halfspace residuals.  Circle ``h``
    is the requested squared implicit function divided by ``2a`` so it has
    distance units without changing its zero set or sign.
    """

    def __init__(self, windows, head_pvaj, tail_pvaj, config: ConvexSafetyConfig,
                 core: AnalyticTOGTCore | None = None):
        self.windows = tuple(windows)
        self.head = np.asarray(head_pvaj, dtype=float)
        self.tail = np.asarray(tail_pvaj, dtype=float)
        self.config = config
        self.core = AnalyticTOGTCore() if core is None else core
        self.geometry = []
        for window in self.windows:
            aperture = window.aperture
            if aperture.kind == "polygon":
                self.geometry.append(polygon_halfspaces(aperture.physical_vertices))
            elif aperture.kind == "circle":
                self.geometry.append(float(aperture.radius))
            else:
                raise ValueError(f"unsupported aperture kind: {aperture.kind}")

    def _field(self, index: int, q: np.ndarray):
        aperture = self.windows[index].aperture
        prepared = self.geometry[index]
        if aperture.kind == "circle":
            radius = float(prepared)
            value = (float(q @ q) - radius * radius) / (2.0 * radius)
            return value, q / radius
        residuals = prepared.normals @ q - prepared.offsets
        scaled = residuals / self.config.smoothmax_temperature_m
        shifted = scaled - np.max(scaled)
        exponential = np.exp(shifted)
        probabilities = exponential / np.sum(exponential)
        value = self.config.smoothmax_temperature_m * (
            np.max(scaled) + np.log(np.sum(exponential))
        )
        gradient = probabilities @ prepared.normals
        return float(value), gradient

    def _node(self, position: np.ndarray, absolute_time: float, window_index: int):
        window = self.windows[window_index]
        center, rotation, center_rate, rotation_rate = window.state_at(absolute_time)
        relative = position - center
        local = rotation.T @ relative
        field, field_gradient = self._field(window_index, local[:2])
        residual = self.config.planning_radius**2 - local[2]**2 - field**2
        penalty, penalty_derivative = _smoothed_l1(
            float(residual), self.config.smoothing_epsilon_m2
        )
        local_residual_gradient = np.r_[
            -2.0 * field * field_gradient,
            -2.0 * local[2],
        ]
        local_penalty_gradient = penalty_derivative * local_residual_gradient
        position_gradient = rotation @ local_penalty_gradient
        local_time_rate_at_fixed_position = (
            rotation_rate.T @ relative - rotation.T @ center_rate
        )
        explicit_time_gradient = float(local_penalty_gradient @ local_time_rate_at_fixed_position)
        return penalty, position_gradient, explicit_time_gradient

    def evaluate(self, trajectory, *, with_gradient: bool = True):
        durations = np.asarray(trajectory.durations, dtype=float)
        coefficients = np.asarray(trajectory.coefficients, dtype=float)
        piece_count = len(durations)
        coefficient_gradient = np.zeros_like(coefficients)
        direct_duration_gradient = np.zeros(piece_count)
        segment_start_gradient = np.zeros(piece_count)
        per_window = np.zeros(len(self.windows))
        elapsed = 0.0
        node_count = 0
        for segment, duration in enumerate(durations):
            count = int(duration / self.config.check_time_sec)
            count = min(max(count, self.config.min_num_check), self.config.max_num_check)
            fraction = 1.0 / count
            step = duration * fraction
            for node in range(count + 1):
                alpha = node * fraction
                local_time = alpha * duration
                beta0 = _basis(local_time, 0)
                position = coefficients[segment].T @ beta0
                velocity = coefficients[segment].T @ _basis(local_time, 1)
                weight = 0.5 if node in (0, count) else 1.0
                absolute_time = elapsed + local_time
                for window_index in range(len(self.windows)):
                    penalty, position_gradient, explicit_time_gradient = self._node(
                        position, absolute_time, window_index
                    )
                    contribution = weight * step * penalty
                    per_window[window_index] += contribution
                    if with_gradient:
                        coefficient_gradient[segment] += (
                            weight * step * np.outer(beta0, position_gradient)
                        )
                        direct_duration_gradient[segment] += weight * (
                            fraction * penalty
                            + step * alpha * (
                                float(position_gradient @ velocity) + explicit_time_gradient
                            )
                        )
                        segment_start_gradient[segment] += weight * step * explicit_time_gradient
                node_count += 1
            elapsed += duration

        total = float(np.sum(per_window))
        if with_gradient:
            waypoint_gradient, duration_gradient = self.core.propagate_minco_gradient(
                self.head, self.tail, trajectory.intermediate_points, durations,
                coefficient_gradient, direct_duration_gradient,
            )
            # S_i=sum_{k<i}T_k: each segment's explicit start-time derivative
            # contributes to every preceding physical duration.
            for segment, gradient in enumerate(segment_start_gradient):
                duration_gradient[:segment] += gradient
        else:
            waypoint_gradient = np.zeros_like(trajectory.intermediate_points)
            duration_gradient = np.zeros_like(durations)
        return SafetyIntegralEvaluation(
            value=total,
            waypoint_gradient=waypoint_gradient,
            duration_gradient=duration_gradient,
            per_window=tuple(float(value) for value in per_window),
            nodes=node_count,
        )
__all__ = [
    "ConvexSafetyConfig", "HandwrittenConvexSafetyIntegral",
    "PolygonHalfspaces", "SafetyIntegralEvaluation", "polygon_halfspaces",
]
