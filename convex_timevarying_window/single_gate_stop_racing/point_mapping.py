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


def inverse(aperture, point):
    """Interior inverse using disk algebra or positive mean-value coordinates."""
    point = np.asarray(point, float)
    if aperture.kind == 'circle':
        return point/(aperture.radius+np.sqrt(aperture.radius**2-point@point))
    vectors = aperture.physical_vertices-point
    distances = np.linalg.norm(vectors, axis=1)
    following = np.roll(vectors, -1, axis=0)
    cross = vectors[:, 0]*following[:, 1]-vectors[:, 1]*following[:, 0]
    tangents = cross/(distances*np.roll(distances, -1)+np.sum(vectors*following, axis=1))
    weights = (np.roll(tangents, 1)+tangents)/distances
    weights /= weights.sum()
    if not np.isfinite(weights).all() or np.any(weights < 0.):
        raise ValueError('Mean-value inverse requires a point strictly inside a convex polygon')
    return np.sqrt(np.r_[weights[1:], weights[:1]])
