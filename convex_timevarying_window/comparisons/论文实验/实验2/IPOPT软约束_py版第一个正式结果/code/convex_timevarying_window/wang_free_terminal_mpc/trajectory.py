"""Invertible last-middle-position <-> terminal-jerk coordinates for MINCO-s3."""
from functools import lru_cache
import casadi as ca
import numpy as np
from 复现.polynomial_dynamic_racing_2023.minco import normalized_inverse, basis
from convex_timevarying_window.wang_paper_aligned_mpc.trajectory import coefficients


@lru_cache(None)
def jerk_response(pieces):
    if pieces < 2:
        raise ValueError('At least two pieces are needed for independent terminal jerk')
    inverse = normalized_inverse(pieces)
    row = 3+6*(pieces-2)
    response = float(basis(1., 3)@(inverse[-6:, row]+inverse[-6:, row+1]))
    if abs(response) < 1e-8:
        raise ValueError('Singular terminal-jerk parameterization')
    return response


def from_terminal_jerk(head, tail, middle_prefix, jerk, duration, pieces=4):
    """PVA endpoints, M-2 free internal points, and independent end jerk."""
    h = duration/pieces
    middle = ca.horzcat(middle_prefix, ca.DM.zeros(3))
    base = coefficients(head, tail, middle, duration, pieces)
    last = (h**3*jerk-base[-6:, :].T@ca.DM(basis(1., 3)))/jerk_response(pieces)
    return coefficients(head, tail, ca.horzcat(middle_prefix, last), duration, pieces)

