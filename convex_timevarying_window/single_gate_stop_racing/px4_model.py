"""PX4 83c4f4e + Gazebo x500 CTBR prediction, ENU/FLU.

Model, not a new flight policy. See PX4_MODEL_SOURCES.md for parameters,
source attribution and approximation boundaries. State[:17] is the existing
physical telemetry; the remainder is the rate controller's internal memory.
"""
import casadi as ca
import numpy as np
import hashlib
from pathlib import Path
import subprocess
from scipy.signal import butter
from convex_timevarying_window.checked_soft_mpc_racing.mpc import MASS, G, KF
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MIXER
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500_INERTIA

NX = 33
DT = .004
KP = np.array([.15, .15, .2])
KI = np.array([.2, .2, .1])
KD = np.array([.003, .003, 0.])
BODY = np.array([1., -1., -1.])
POSITIONS = np.array([[.174, -.174, .06], [-.174, .174, .06],
                      [.174, .174, .06], [-.174, -.174, .06]])
COM = np.array([0., 0., 4*.016076923076923075*.06/MASS])


def normalized_mixer():
    """Actual CA_ROTOR geometry and PX4 pseudo-inverse normalization (FRD)."""
    positions = np.array([[.13, .22, 0.], [-.13, -.2, 0.],
                          [.13, -.22, 0.], [-.13, .2, 0.]])
    km = np.array([.05, .05, -.05, -.05])
    axes = np.tile([0., 0., -1.], (4, 1))
    effectiveness = np.vstack(((6.5*np.cross(positions, axes)
                                - 6.5*km[:, None]*axes).T, (6.5*axes).T))
    mix = np.linalg.pinv(effectiveness)
    rp = max(np.linalg.norm(mix[:, 0])/np.sqrt(2),
             np.linalg.norm(mix[:, 1])/np.sqrt(2))
    mix[:, :2] /= rp
    mix[:, 2] /= np.max(mix[:, 2])
    mix[:, 3:] /= np.mean(np.abs(mix[:, 5]))
    mix[np.abs(mix) < 1e-3] = 0.
    return np.column_stack((mix[:, :3], -mix[:, 5]))


NORMALIZED_MIXER = normalized_mixer()


def _gain(a, direction, high):
    low_gains, high_gains = [0.], [0.]
    for i in range(4):
        if abs(direction[i]) < .2:
            continue
        value = ca.if_else(a[i] < 0., -a[i]/direction[i],
                          ca.if_else(a[i] > high, (high-a[i])/direction[i], 0.))
        low_gains.append(value); high_gains.append(value)
    return ca.mmin(ca.vertcat(*low_gains))+ca.mmax(ca.vertcat(*high_gains))


def _desaturate(a, direction, high=1., reduce_thrust_only=False):
    gain = _gain(a, direction, high)
    first = a+gain*ca.DM(direction)
    second = first+.5*_gain(first, direction, high)*ca.DM(direction)
    # PX4 thrust direction is negative. A negative gain would INCREASE thrust.
    return ca.if_else(gain < 0., a, second) if reduce_thrust_only else second


def allocate(throttle, torque_flu):
    """MC_AIRMODE=0 sequential desaturation, normalized controls -> rad/s."""
    m = NORMALIZED_MIXER
    torque = ca.DM(BODY)*torque_flu
    thrust_direction = -m[:, 3]
    a = ca.DM(m[:, :2])@torque[:2]+throttle*ca.DM(m[:, 3])
    a = _desaturate(a, thrust_direction, reduce_thrust_only=True)
    a = _desaturate(a, m[:, 0]); a = _desaturate(a, m[:, 1])
    a += ca.DM(m[:, 2])*torque[2]
    a = _desaturate(a, m[:, 2], high=1.15)
    a = _desaturate(a, thrust_direction, reduce_thrust_only=True)
    normalized = ca.fmax(0., ca.fmin(1., a))
    achieved = ca.DM(np.linalg.inv(m))@normalized
    unallocated_flu = ca.DM(BODY)*(torque-achieved[:3])
    return 150.+850.*normalized, unallocated_flu


def initial_state(physical):
    physical = np.asarray(physical, float)
    b, a = butter(2, 40., fs=1/DT)
    delay = physical[10:13]/np.sum(b)
    # I[3], gyro biquad delays[6], previous FILTERED rate[3], D filter[3], yaw filter[1].
    return np.r_[physical, np.zeros(3), delay, delay, physical[10:13], np.zeros(4)]


def dynamics():
    x, u = ca.SX.sym('px4_x', NX), ca.SX.sym('px4_u', 4)
    h = ca.SX.sym('px4_h')
    w, a, b, c = ca.vertsplit(x[6:10])
    rotation = ca.vertcat(ca.horzcat(1-2*(b*b+c*c), 2*(a*b-w*c), 2*(a*c+w*b)),
                         ca.horzcat(2*(a*b+w*c), 1-2*(a*a+c*c), 2*(b*c-w*a)),
                         ca.horzcat(2*(a*c-w*b), 2*(b*c+w*a), 1-2*(a*a+b*b)))
    omega, motors = x[10:13], x[13:17]
    inertia = ca.DM(X500_INERTIA)
    force = ca.vertcat(0., 0., KF*ca.sumsqr(motors))
    torque = (ca.DM(MIXER)@(KF*motors**2))[1:4]
    body_v = rotation.T@x[3:6]
    for i in range(4):
        arm = ca.DM(POSITIONS[i]-COM)
        velocity = body_v+ca.cross(omega, ca.DM(POSITIONS[i]))
        planar = ca.vertcat(velocity[:2], 0.)
        drag = -8.06428e-5*motors[i]*planar
        force += drag
        torque += ca.cross(arm, drag)-1e-6*motors[i]*planar
    angular_acc = (torque-ca.cross(omega, inertia*omega))/inertia
    acceleration = rotation@force/MASS-ca.vertcat(0., 0., G)
    # PX4 FilterAngularVelocity mutates its input array BEFORE the same array
    # is passed to FilterAngularAcceleration (83c4f4e, lines 827-829).
    bb, aa = butter(2, 40., fs=1/DT)
    delay0 = omega-aa[1]*x[20:23]-aa[2]*x[23:26]
    filtered = bb[0]*delay0+bb[1]*x[20:23]+bb[2]*x[23:26]
    alpha_d = h/(h+1/(2*np.pi*20.))
    derivative = x[29:32]+alpha_d*((filtered-x[26:29])/h-x[29:32])
    error = u[1:4]-filtered
    requested = ca.DM(KP)*error+x[17:20]-ca.DM(KD)*derivative
    yaw = x[32]+h/(h+1/(2*np.pi*2.))*(requested[2]-x[32])
    requested = ca.vertcat(requested[:2], yaw)
    throttle = (ca.sqrt(ca.fmax(4*KF*150**2, u[0])/(4*KF))-150.)/850.
    target, residual = allocate(throttle, requested)
    integrating_error = ca.if_else(residual > 1e-7, ca.fmin(error, 0.),
                          ca.if_else(residual < -1e-7, ca.fmax(error, 0.), error))
    factor = ca.fmax(0., 1.-(integrating_error/np.radians(400.))**2)
    integral = ca.fmax(-.3, ca.fmin(.3, x[17:20]+h*ca.DM(KI)*factor*integrating_error))
    motor_next = target+(motors-target)*ca.exp(-h/ca.if_else(target > motors, .0125, .025))
    mid_omega = omega+.5*h*angular_acc
    p, q, r = ca.vertsplit(mid_omega)
    quaternion_dot = .5*ca.vertcat(-a*p-b*q-c*r, w*p+b*r-c*q,
                                  w*q+c*p-a*r, w*r+a*q-b*p)
    quat = x[6:10]+h*quaternion_dot
    physical = ca.vertcat(x[:3]+h*x[3:6]+.5*h*h*acceleration,
                         x[3:6]+h*acceleration, quat/ca.norm_2(quat),
                         omega+h*angular_acc, motor_next)
    y = ca.vertcat(physical, integral, delay0, x[20:23], filtered, derivative, yaw)
    tick = ca.Function('px4_tick', [x, u, h], [y])
    # Ten simulator-sized stages per original 40 ms prediction node. Keep the
    # function calls shared, avoiding an enormous expanded symbolic graph.
    xx, uu, hh = ca.MX.sym('x', NX), ca.MX.sym('u', 4), ca.MX.sym('h')
    yy = xx
    for _ in range(10):
        yy = tick(yy, uu, hh/10)
    step = ca.Function('px4_step', [xx, uu, hh], [yy])
    linear = ca.Function('px4_linear', [xx, uu, hh], [yy, ca.jacobian(yy, xx), ca.jacobian(yy, uu)])
    allocation = ca.Function('px4_allocate', [u], [allocate(u[0], u[1:4])[0]])
    return tick, step, linear, allocation


def native_dynamics(extra=(), suffix=''):
    """Compile the identical CasADi expressions, not a simpler prediction."""
    source = Path(__file__).resolve()
    digest = hashlib.sha256(source.read_bytes()+suffix.encode()).hexdigest()[:16]
    directory = source.parent/'native'/'build'/digest
    library = directory/'px4_model.so'
    if not library.exists():
        directory.mkdir(parents=True, exist_ok=True)
        generator = ca.CodeGenerator('px4_model.c')
        for function in (*dynamics(), *extra): generator.add(function)
        generator.generate(str(directory)+'/')
        subprocess.run(['gcc', '-O3', '-fPIC', '-shared', str(directory/'px4_model.c'),
                        '-o', str(library), '-lm'], check=True)
    names = ('px4_tick', 'px4_step', 'px4_linear', 'px4_allocate')+tuple(f.name() for f in extra)
    return tuple(ca.external(name, str(library), {'enable_fd': False}) for name in names)
