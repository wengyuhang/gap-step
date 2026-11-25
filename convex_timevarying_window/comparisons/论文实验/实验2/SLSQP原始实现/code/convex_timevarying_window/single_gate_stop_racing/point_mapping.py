"""TOGT 2024 shape maps, adapted to the gate's local two-dimensional plane.

Source: FSC-Lab/TOGT-Planner, 6707d3a1d06e4bd33d4aaa5e6f9fa02497ee90c5,
src/shape/{ball,polyhedron}.cpp. The ball formula is used in 2D for a disk,
not as a 3D ball that could move a waypoint away from the gate plane.
"""
import casadi as ca
import numpy as np


def dimension(aperture):
    return 2 if aperture.kind=='circle' else len(aperture.physical_vertices)


def initial(aperture):
    if aperture.kind=='circle':return np.zeros(2)
    return np.full(dimension(aperture),1/np.sqrt(dimension(aperture)))


def symbolic(aperture,d):
    if aperture.kind=='circle':return 2*aperture.radius*d/(1+ca.sumsqr(d))
    v=aperture.physical_vertices
    # Same normalized-square barycentric map. A tiny denominator guard makes
    # the all-zero vector a defined vertex-zero point as well; every weight
    # remains nonnegative and their sum remains exactly one mathematically.
    weights=d[:-1]**2/(ca.sumsqr(d)+1e-24)
    return ca.DM(v[0])+ca.DM((v[1:]-v[0]).T)@weights


def numeric(aperture,d):
    return np.asarray(symbolic(aperture,ca.DM(d))).ravel()
