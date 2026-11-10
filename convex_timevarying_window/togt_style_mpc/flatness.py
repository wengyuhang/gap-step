"""Shared x500 drag/torque reference used by planning and tracking."""
from functools import lru_cache

import casadi as ca
import numpy as np

from .model import MASS, G, KF, MIXER, X500_INERTIA, ROTOR_DRAG, aerodynamics
from convex_timevarying_window.tilt_heading_minco.dynamics import rotation_from_direction


@lru_cache(maxsize=1)
def maps():
    velocity, acceleration, jerk, snap = [ca.MX.sym(name, 3)
        for name in ('velocity', 'acceleration', 'jerk', 'snap')]
    force = acceleration+ca.DM([0., 0., G])
    for _ in range(3):
        direction = force/ca.sqrt(ca.sumsqr(force)+1e-12)
        speed = ca.sqrt(MASS*ca.sqrt(ca.sumsqr(force)+1e-12)/(4*KF))
        drag = 4*ROTOR_DRAG*speed/MASS
        force = acceleration+ca.DM([0., 0., G])+drag*(velocity-ca.dot(velocity, direction)*direction)
    force *= MASS
    collective = ca.sqrt(ca.sumsqr(force)+1e-12)
    rotation = rotation_from_direction(force/collective)
    derivative = ca.reshape(ca.jtimes(ca.vec(rotation), ca.vertcat(velocity, acceleration),
        ca.vertcat(acceleration, jerk)), 3, 3)
    skew = rotation.T@derivative
    omega = ca.vertcat(skew[2, 1]-skew[1, 2], skew[0, 2]-skew[2, 0],
        skew[1, 0]-skew[0, 1])/2.
    alpha = ca.jtimes(omega, ca.vertcat(velocity, acceleration, jerk),
        ca.vertcat(acceleration, jerk, snap))
    _, aero_torque = aerodynamics(rotation, velocity, omega, ca.repmat(collective/4., 4, 1))
    torque = ca.DM(X500_INERTIA)*alpha+ca.cross(omega, ca.DM(X500_INERTIA)*omega)-aero_torque
    rotors = ca.DM(np.linalg.inv(MIXER))@ca.vertcat(collective, torque)
    return ca.Function('x500_reference_flatness', [velocity, acceleration, jerk, snap],
        [rotation, collective, omega, alpha, rotors], {'cse': True})
