"""13-state x500 rigid body, four physical rotor thrust inputs (ENU / FLU).

No PX4 PID, allocator or motor lag states in the optimization. Rotor drag and
its moments use the existing Gazebo x500 physical parameters, not gate data.
This is an independent model, NOT licensed Agilicious source.
"""
import casadi as ca
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS, KF, MIXER
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500_INERTIA

G = 9.8066
FMIN, FMAX = KF*150.**2, KF*1000.**2
RATE_MAX = np.array([3.8, 3.8, 3.49])
POSITIONS = np.array([[.174, -.174, .06], [-.174, .174, .06],
                      [.174, .174, .06], [-.174, -.174, .06]])
COM = np.array([0., 0., 4*.016076923076923075*.06/MASS])
ROTOR_DRAG = 8.06428e-5
ROLLING_MOMENT = 1e-6


def rotation(q):
    w, a, b, c = ca.vertsplit(q)
    return ca.vertcat(ca.horzcat(1-2*(b*b+c*c), 2*(a*b-w*c), 2*(a*c+w*b)),
                     ca.horzcat(2*(a*b+w*c), 1-2*(a*a+c*c), 2*(b*c-w*a)),
                     ca.horzcat(2*(a*c-w*b), 2*(b*c+w*a), 1-2*(a*a+b*b)))


def quaternion_error(q, reference):
    # Vector part of conjugate(reference) * q. Squared cost is sign invariant.
    return reference[0]*q[1:]-q[0]*reference[1:]-ca.cross(reference[1:], q[1:])


def aerodynamics(rot, velocity, omega, rotor_forces):
    force = ca.vertcat(0., 0., ca.sum1(rotor_forces))
    torque = ca.DM.zeros(3)
    body_v = rot.T@velocity
    for i in range(4):
        velocity = body_v+ca.cross(omega, ca.DM(POSITIONS[i]))
        planar = ca.vertcat(velocity[:2], 0.)
        speed = ca.sqrt(ca.fmax(rotor_forces[i], 1e-8)/KF)
        drag = -ROTOR_DRAG*speed*planar
        force += drag
        torque += ca.cross(ca.DM(POSITIONS[i]-COM), drag)-ROLLING_MOMENT*speed*planar
    return force, torque


def expressions():
    x, u = ca.SX.sym('x', 13), ca.SX.sym('u', 4)
    rot, omega = rotation(x[6:10]), x[10:13]
    force, aero_torque = aerodynamics(rot, x[3:6], omega, u)
    inertia = ca.DM(X500_INERTIA)
    torque = ca.DM(MIXER[1:])@u+aero_torque
    acc = rot@force/MASS-ca.vertcat(0., 0., G)
    angular_acc = (torque-ca.cross(omega, inertia*omega))/inertia
    qw, qv = x[6], x[7:10]
    qdot = .5*ca.vertcat(-ca.dot(qv, omega), qw*omega+ca.cross(qv, omega))
    return x, u, ca.vertcat(x[3:6], acc, qdot, angular_acc), aero_torque


def functions():
    x, u, rhs, aero = expressions()
    f = ca.Function('rotor_rhs', [x, u], [rhs])
    h = ca.SX.sym('h')
    k1 = f(x,u); k2 = f(x+h*k1/2,u); k3 = f(x+h*k2/2,u); k4 = f(x+h*k3,u)
    y = x+h*(k1+2*k2+2*k3+k4)/6
    y = ca.vertcat(y[:6], y[6:10]/ca.norm_2(y[6:10]), y[10:])
    return f, ca.Function('rotor_step', [x,u,h], [y]), ca.Function('aero_torque', [x,u], [aero])
