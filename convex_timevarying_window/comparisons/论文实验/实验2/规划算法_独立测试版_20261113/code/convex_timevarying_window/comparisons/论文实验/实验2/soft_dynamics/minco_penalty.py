"""Configured TOGT soft integral with the existing native MINCO adjoint."""
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.native_backend import AnalyticTOGTCore
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from .kernels import PolynomialPenalty


class ConfiguredMINCOCore(AnalyticTOGTCore):
    def __init__(self, limit_fraction):
        super().__init__()
        if not np.isfinite(limit_fraction) or not (0.0225 < limit_fraction <= 1.):
            raise ValueError('Dynamic limit fraction must be in (0.0225, 1]')
        self.penalty = PolynomialPenalty('togt', limit_fraction)

    def value_and_gradient(self, head_pvaj, tail_pvaj, points, durations):
        times = np.asarray(durations, dtype=float)
        curve = MincoSnap(BoundaryState.from_array(head_pvaj),
            BoundaryState.from_array(tail_pvaj), points, times)
        powers = np.arange(8)[None, :, None]
        normalization = times[:, None, None]**powers
        coefficients = curve.coefficients*normalization
        cost, gradient, direct_times = self.penalty.evaluate(coefficients, times)
        partial_coefficients = gradient*normalization
        partial_times = direct_times+np.sum(powers*coefficients*gradient, axis=(1, 2))/times
        grad_points, grad_times = self.propagate_minco_gradient(
            head_pvaj, tail_pvaj, points, times,
            partial_coefficients.reshape(-1, 3), partial_times)
        return float(times.sum()+cost), grad_points, grad_times+1.
