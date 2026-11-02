"""The existing P/V-constrained minimum-snap curve, with any knot count.

Eliminate the free internal A/J using the snap quadratic. This is the same
C5 minimizer as the full 8N coefficient system, not an independent Hermite fit.
"""
import math
import casadi as ca
import numpy as np
from convex_timevarying_window.togt_minco_mpc.trajectory import basis

HERMITE = np.vstack([np.asarray(basis(u, d)).ravel() for u in (0., 1.) for d in range(4)])
INVERSE = np.linalg.inv(HERMITE)
SNAP = np.zeros((8, 8))
for i in range(4, 8):
    for j in range(4, 8):
        SNAP[i, j] = math.factorial(i)/math.factorial(i-4)*math.factorial(j)/math.factorial(j-4)/(i+j-7)


def full_state_coefficients(head, tail, points, velocities, accelerations, jerks, durations):
    """Septic pieces with shared internal PVAJ, hence C3 (not imposed C5).

    Each piece is the unique minimum-snap polynomial for its endpoint PVAJ.
    The outer problem now optimizes A/J instead of eliminating them by snap.
    """
    n = int(durations.numel())
    nodes = [head]+[ca.horzcat(points[:, k], velocities[:, k], accelerations[:, k], jerks[:, k])
                   for k in range(n-1)]+[tail]
    return ca.vertcat(*[ca.DM(INVERSE)@ca.vertcat(
        *[nodes[k][:, d].T*durations[k]**d for d in range(4)],
        *[nodes[k+1][:, d].T*durations[k]**d for d in range(4)]) for k in range(n)])


def coefficients(head, tail, points, velocities, durations):
    n = int(durations.numel())
    if n == 1:
        values = ca.vertcat(*[head[:, d].T*durations[0]**d for d in range(4)],
                            *[tail[:, d].T*durations[0]**d for d in range(4)])
        return ca.DM(INVERSE)@values
    dimension = 2*(n-1)
    matrix = ca.MX(ca.Sparsity(dimension, dimension))
    rhs = ca.MX(ca.Sparsity(dimension, 3))
    constants, maps = [], []
    for k in range(n):
        T = durations[k]
        known = ca.MX(ca.Sparsity(8, 3))
        free = ca.MX(ca.Sparsity(8, dimension))
        for side, node in enumerate((k, k+1)):
            row = 4*side
            if node == 0 or node == n:
                endpoint = head if node == 0 else tail
                for d in range(4):
                    known[row+d, :] = endpoint[:, d].T*T**d
            else:
                known[row, :] = points[:, node-1].T
                known[row+1, :] = velocities[:, node-1].T*T
                free[row+2, 2*(node-1)] = T**2
                free[row+3, 2*(node-1)+1] = T**3
        C0, E = ca.DM(INVERSE)@known, ca.DM(INVERSE)@free
        Q = ca.DM(SNAP)/T**7
        matrix += E.T@Q@E
        rhs -= E.T@Q@C0
        constants.append(C0)
        maps.append(E)
    internal = ca.solve(matrix, rhs, 'qr')
    return ca.vertcat(*[C0+E@internal for C0, E in zip(constants, maps)])
