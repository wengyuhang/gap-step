"""Differentiable expression of the same gravity-only x500 TOGT penalty."""
import casadi as ca
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500_INERTIA, X500_MASS
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import MIXER
from convex_timevarying_window.togt_minco_mpc.trajectory import basis


def positive_penalty(value):
    mu = .01
    transition = (mu-.5*value)*(value/mu)**3
    return ca.if_else(value < 0., 0., ca.if_else(value > mu, value-.5*mu, transition))


def instantaneous(*, inline=False, limit_fraction=1.):
    a, j, s = [ca.SX.sym(name, 3) for name in ('acceleration', 'jerk', 'snap')]
    alpha = a+ca.DM([0., 0., 9.8066])
    magnitude = ca.sqrt(ca.sumsqr(alpha))
    direction = alpha/magnitude
    projection = ca.dot(alpha, j)
    direction_rate = j/magnitude-alpha*projection/magnitude**3
    direction_acceleration = (s-direction*ca.dot(direction, s))/magnitude
    direction_acceleration -= (2.*j*projection+alpha*ca.sumsqr(j))/magnitude**3
    direction_acceleration += 3.*alpha*projection**2/magnitude**5
    x, y, z = ca.vertsplit(direction)
    dx, dy, dz = ca.vertsplit(direction_rate)
    ddx, ddy, ddz = ca.vertsplit(direction_acceleration)
    denominator = 1.+z
    twist = y*dx-x*dy
    omega = ca.vertcat(-dy+y*dz/denominator,
        dx-x*dz/denominator, twist/denominator)
    angular_acceleration = ca.vertcat(
        -ddy+y*ddz/denominator+dy*dz/denominator-y*dz**2/denominator**2,
        ddx-x*ddz/denominator-dx*dz/denominator+x*dz**2/denominator**2,
        (y*ddx-x*ddy)/denominator-twist*dz/denominator**2)
    inertia = ca.DM(X500_INERTIA)
    torque = inertia*angular_acceleration+ca.cross(omega, inertia*omega)
    thrusts = ca.DM(np.linalg.inv(MIXER))@ca.vertcat(X500_MASS*magnitude, torque)
    rate_xy, rate_z = 14.74348422*limit_fraction**2, 12.18469679*limit_fraction**2
    rotor_midpoint, collective_midpoint = 4.27429*limit_fraction, 17.09716*limit_fraction
    regular = ca.vertcat(ca.sumsqr(omega[:2])-rate_xy,
        omega[2]**2-rate_z, *[(thrusts[k]-rotor_midpoint)**2-rotor_midpoint**2 for k in range(4)])
    fallback = ca.vertcat(ca.sumsqr(j[:2])/magnitude**2-rate_xy,
        -1., (X500_MASS*magnitude-collective_midpoint)**2-collective_midpoint**2, -1., -1., -1.)
    residuals = ca.if_else(ca.fabs(1.+direction[2]) > .001, regular, fallback)
    cost = 1000.*sum(positive_penalty(value) for value in ca.vertsplit(residuals))
    return ca.Function('native_equivalent_instantaneous', [a, j, s], [cost, omega, thrusts, residuals],
        {'cse': True, 'never_inline': not inline})


def objective(planner, x):
    if planner.dynamics_model == 'x500':
        from . import kernels
        times = ca.exp(x[:planner.n])
        coefficients = planner.curve_values(x)
        value = ca.sum1(times)
        for piece in range(planner.n):
            residuals = kernels.dynamic_residuals('x500', planner.dynamic_limit_fraction)(ca.vertcat(
                coefficients[24*piece:24*(piece+1)], times[piece]))
            value += kernels.slack_integral('x500', planner.dynamic_limit_fraction)(ca.fmax(residuals, 0.), times[piece])
        return value
    times = ca.exp(x[:planner.n])
    coefficients = planner.curve_values(x)
    states = ca.SX.sym('instantaneous_states', 9)
    penalty = ca.Function('instantaneous_cost_only', [states],
        [instantaneous(limit_fraction=planner.dynamic_limit_fraction)(states[:3], states[3:6], states[6:])[0]],
        {'cse': True, 'never_inline': True})
    value = ca.sum1(times)
    for piece in range(planner.n):
        duration = times[piece]
        count = 32
        curve = ca.reshape(coefficients[24*piece:24*(piece+1)], 8, 3)
        for node in range(33):
            u = node/count
            states = [curve.T@basis(u, order)/duration**order for order in (2, 3, 4)]
            cost = penalty(ca.vertcat(*states))
            weight = .5 if node in (0, count) else 1.
            value += duration/count*weight*cost
    return value
