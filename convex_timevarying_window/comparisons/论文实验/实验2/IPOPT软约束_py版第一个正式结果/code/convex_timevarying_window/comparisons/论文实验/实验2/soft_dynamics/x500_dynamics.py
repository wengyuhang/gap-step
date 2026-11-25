"""x500 aerodynamic soft penalty with the common execution reference model."""
from functools import lru_cache

import casadi as ca
import numpy as np

from convex_timevarying_window.togt_style_mpc.flatness import maps
from convex_timevarying_window.togt_style_mpc.model import FMIN, FMAX, RATE_MAX
from .symbolic_penalty import positive_penalty


RATE_LIMITS = np.array([RATE_MAX[0]**2, RATE_MAX[2]**2])
CENTER = (FMIN+FMAX)/2.
HALF_RANGE = (FMAX-FMIN)/2.
RESIDUAL_SCALE = np.r_[RATE_LIMITS, np.full(4, HALF_RANGE**2)]


@lru_cache(maxsize=8)
def instantaneous(limit_fraction=1.):
    states = [ca.SX.sym(name, 3) for name in ('velocity', 'acceleration', 'jerk', 'snap')]
    _, _, omega, _, rotors = maps().expand()(*states)
    center = (FMIN+FMAX*limit_fraction)/2.
    half_range = (FMAX*limit_fraction-FMIN)/2.
    residuals = ca.vertcat(ca.sumsqr(omega[:2])-RATE_LIMITS[0]*limit_fraction**2,
        omega[2]**2-RATE_LIMITS[1]*limit_fraction**2,
        *[(rotors[index]-center)**2-half_range**2 for index in range(4)])
    cost = 1000.*sum(positive_penalty(value) for value in ca.vertsplit(residuals))
    return ca.Function('x500_aerodynamic_penalty', states, [cost, omega, rotors, residuals],
        {'cse': True})


class Penalty:
    def __init__(self):
        from . import kernels
        values = ca.MX.sym('segment_coefficients', 25)
        residuals = kernels.dynamic_residuals('x500')(values)
        cost = kernels.slack_integral('x500')(ca.fmax(residuals, 0.), values[24])
        self.segment = ca.Function('x500_segment_penalty', [values],
            [cost, ca.gradient(cost, values)], {'cse': True})

    def evaluate(self, coefficients, durations):
        coefficients = np.asarray(coefficients, dtype=float)
        durations = np.asarray(durations, dtype=float)
        if coefficients.shape != (len(durations), 8, 3):
            raise ValueError('Expected normalized degree-seven coefficients')
        values = np.vstack([coefficients.transpose(0, 2, 1).reshape(len(durations), 24).T, durations])
        costs, derivative = self.segment.map(len(durations))(values)
        derivative = np.asarray(derivative).T
        gradient = derivative[:, :24].reshape(len(durations), 3, 8).transpose(0, 2, 1)
        return float(np.asarray(costs).sum()), gradient, derivative[:, 24]
