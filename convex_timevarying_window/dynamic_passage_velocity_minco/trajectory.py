"""Minimum snap with prescribed interior position AND velocity, two pieces.

The position-only TOGT construction leaves the interior velocity implicit. An
interior velocity constraint replaces continuity of derivative six; continuity
through derivative five remains. This is not an unconstrained Hermite fit and
does not add a geometric knot. Endpoint PVAJ is unchanged.
"""
import casadi as ca
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.togt_point_only_mpc.trajectory import Plan


def minco_coefficients(head, tail, point, velocity, durations, acceleration=None):
    if int(durations.numel()) != 2:
        raise ValueError('The single-window validation has exactly two pieces')
    A = ca.MX(ca.Sparsity(16, 16))
    B = ca.MX(ca.Sparsity(16, 3))
    for derivative in range(4):
        A[derivative, :8] = (basis(0., derivative)/durations[0]**derivative).T
        B[derivative, :] = head[:, derivative].T
        A[12+derivative, 8:] = (basis(1., derivative)/durations[1]**derivative).T
        B[12+derivative, :] = tail[:, derivative].T
    continuous = ((4, 4), (5, 5)) if acceleration is None else ((4, 4),)
    for row, derivative in continuous:
        A[row, :8] = (basis(1., derivative)/durations[0]**derivative).T
        A[row, 8:] = -(basis(0., derivative)/durations[1]**derivative).T
    if acceleration is not None:
        A[5, :8] = (basis(1., 2)/durations[0]**2).T
        B[5, :] = acceleration.T
    A[6, :8] = (basis(1., 1)/durations[0]).T
    B[6, :] = velocity.T
    A[7, :8] = basis(1.).T
    B[7, :] = point.T
    for derivative in range(4):
        A[8+derivative, :8] = (basis(1., derivative)/durations[0]**derivative).T
        A[8+derivative, 8:] = -(basis(0., derivative)/durations[1]**derivative).T
    return ca.solve(A, B, 'qr')


__all__ = ['Plan', 'minco_coefficients']
