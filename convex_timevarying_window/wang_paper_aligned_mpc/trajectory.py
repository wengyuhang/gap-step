"""Time-uniform MINCO-s3, arbitrary M; published race configuration M=4."""
from dataclasses import dataclass
import casadi as ca
import numpy as np
from 复现.polynomial_dynamic_racing_2023.minco import normalized_inverse
from convex_timevarying_window.wang_center_seed_mpc.trajectory import Plan as BasePlan


def coefficients(head, tail, middle, duration, pieces=4):
    h = duration/pieces
    columns = [head[:, 0], h*head[:, 1], h*h*head[:, 2]]
    for j in range(pieces-1):
        columns.extend([middle[:, j], middle[:, j], *[ca.DM.zeros(3) for _ in range(4)]])
    columns.extend([tail[:, 0], h*tail[:, 1], h*h*tail[:, 2]])
    return ca.DM(normalized_inverse(pieces))@ca.horzcat(*columns).T


@dataclass
class Plan(BasePlan):
    pieces_per_leg: int = 4

    @property
    def crossing_times(self):
        M = self.pieces_per_leg
        return self.made_at+np.cumsum(self.durations)[M-1:M*self.gate_count:M]
