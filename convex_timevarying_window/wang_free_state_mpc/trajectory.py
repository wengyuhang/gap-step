"""Wang's two-piece minimum-jerk interpolation, with arrival jerk coordinates.

For fixed endpoint PVA and duration, the free middle position and arrival
jerk are related by an invertible affine map. This changes coordinates, not
the degree-five trajectory family. Gate joins are C2, as in the reproduction;
left and right jerk need not agree. Window motion is never a decision variable.
"""
from dataclasses import dataclass
import math
import casadi as ca
import numpy as np
from 复现.polynomial_dynamic_racing_2023.minco import normalized_inverse, basis, coefficients
from convex_timevarying_window.togt_minco_mpc.trajectory import Plan as BasePlan


INVERSE = normalized_inverse(2)
MIDDLE_RESPONSE = float(basis(1., 3) @ (INVERSE[6:, 3] + INVERSE[6:, 4]))
assert abs(MIDDLE_RESPONSE) > 1e-8


def from_arrival_jerk(head, tail, arrival_jerk, duration):
    """Symbolic endpoint PVA (3x3) and incoming end jerk -> 12x3 coefficients."""
    h = duration / 2
    zero = ca.DM.zeros(3)
    rhs = ca.horzcat(head[:, 0], h*head[:, 1], h*h*head[:, 2],
                     *[zero for _ in range(6)],
                     tail[:, 0], h*tail[:, 1], h*h*tail[:, 2]).T
    base = ca.DM(INVERSE) @ rhs
    free_middle = (h**3*arrival_jerk - base[6:, :].T @ ca.DM(basis(1., 3))) / MIDDLE_RESPONSE
    rhs[3, :], rhs[4, :] = free_middle.T, free_middle.T
    return ca.DM(INVERSE) @ rhs


def arrival_jerk_for_middle(head, tail, middle, duration):
    C = coefficients(np.asarray(head), np.asarray(tail), [middle], duration)
    return basis(1., 3) @ C[-1] / (duration/2)**3


@dataclass
class Plan(BasePlan):
    """Same clock/IPC contract, but preserve six coefficients per piece."""
    def sample_many(self, times, order=0):
        times = np.asarray(times, dtype=float)
        elapsed = np.maximum(0., times-self.made_at)
        ends = np.cumsum(self.durations)
        ids = np.minimum(np.searchsorted(ends, elapsed, side='right'), len(ends)-1)
        result = np.empty((len(times), 3))
        for i, T in enumerate(self.durations):
            mask = ids == i
            s = np.clip((elapsed[mask]-(ends[i-1] if i else 0.))/T, 0., 1.)
            if np.any(mask):
                B = np.column_stack([math.factorial(k)/math.factorial(k-order)*s**(k-order)/T**order
                                     if k >= order else np.zeros_like(s) for k in range(6)])
                result[mask] = B @ self.coefficients[i]
        return result

    def sample(self, t, order=0):
        elapsed = max(0., float(t)-self.made_at)
        ends = np.cumsum(self.durations)
        i = min(int(np.searchsorted(ends, elapsed, side='right')), len(ends)-1)
        T = self.durations[i]
        s = np.clip((elapsed-(ends[i-1] if i else 0.))/T, 0., 1.)
        return basis(float(s), order) @ self.coefficients[i] / T**order
