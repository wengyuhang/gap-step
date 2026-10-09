"""Real-time-iteration nonlinear MPC, ENU/FLU, direct collective/body rates.

No gate identities/poses, safety filter, path retiming, or planner modifications.
The rate-loop model is empirical; this is NOT a robust safety certificate.
"""
from dataclasses import dataclass
import time

import casadi as ca
import numpy as np
import osqp
from scipy import sparse
from scipy.spatial.transform import Rotation

MASS = 2.0643076923076924
G = 9.8066
KF = 8.54858e-6
RATE_TAU = np.array([.09, .06, .45])
LOWER = np.array([4*KF*150**2, -3.8, -3.8, -3.49])
UPPER = np.array([4*KF*1000**2, 3.8, 3.8, 3.49])
NX, NU = 17, 4


@dataclass(frozen=True)
class Config:
    steps: int = 25
    dt: float = .04
    update_period: float = .02
    state_prediction_s: float = .015
    position_weight: float = 2000.
    velocity_weight: float = 30.
    attitude_weight: float = 20.
    terminal_multiplier: float = 4.
    tilt_limit_deg: float = 45.
    qp_iterations: int = 1200
    iterations: int = 2
    trust_thrust_n: float = 4.
    trust_rate_rps: float = .8


def dynamics():
    x, u = ca.SX.sym('x', NX), ca.SX.sym('u', NU)
    w, a, b, c = ca.vertsplit(x[6:10])
    z = ca.vertcat(2*(a*c+w*b), 2*(b*c-w*a), 1-2*(a*a+b*b))
    motors = ca.fmin(1000., ca.fmax(0., x[13:17]))
    target = ca.sqrt(ca.fmin(UPPER[0], ca.fmax(LOWER[0], u[0]))/(4*KF))
    motor_dot = (target-motors)/ca.if_else(target > motors, .0125, .025)
    thrust = KF*ca.sumsqr(motors)
    drag = 8.06428e-5*ca.sum1(motors)/MASS
    acceleration = z*thrust/MASS-ca.vertcat(0, 0, G)-drag*(x[3:6]-ca.dot(x[3:6], z)*z)
    p, q, r = ca.vertsplit(x[10:13])
    qdot = .5*ca.vertcat(-a*p-b*q-c*r, w*p+b*r-c*q,
                        w*q+c*p-a*r, w*r+a*q-b*p)
    f = ca.Function('derivative', [x, u], [ca.vertcat(x[3:6], acceleration, qdot,
                        (u[1:4]-x[10:13])/ca.DM(RATE_TAU), motor_dot)])
    h = ca.SX.sym('h')
    # Same stable 10 ms RK4 substeps as the existing CTBR nominal model.
    y = x
    for _ in range(4):
        hh = h/4
        k1 = f(y, u); k2 = f(y+hh*k1/2, u)
        k3 = f(y+hh*k2/2, u); k4 = f(y+hh*k3, u)
        y = y+hh*(k1+2*k2+2*k3+k4)/6
        y = ca.vertcat(y[:6], y[6:10]/ca.norm_2(y[6:10]), y[10:])
    step = ca.Function('step', [x, u, h], [y])
    linear = ca.Function('linear', [x, u, h], [y, ca.jacobian(y, x), ca.jacobian(y, u)])
    return f, step, linear


def hover_state(position=(0., 0., 3.), yaw=np.pi/2):
    return np.r_[position, np.zeros(3), np.cos(yaw/2), 0., 0., np.sin(yaw/2),
                 np.zeros(3), np.full(4, np.sqrt(MASS*G/(4*KF)))]


def reference_states(pva, dt, initial_quaternion):
    """Flat reference with rotor drag; same constant NED yaw=0 as old PVA."""
    count = len(pva)
    rotations, thrusts = [], []
    heading = np.array([0., 1., 0.])
    for p, v, a in pva.reshape(count, 3, 3):
        force = a+np.array([0., 0., G])
        for _ in range(3):
            z = force/max(np.linalg.norm(force), 1e-6)
            motor = np.sqrt(MASS*np.linalg.norm(force)/(4*KF))
            drag = 4*8.06428e-5*motor/MASS
            force = a+np.array([0., 0., G])+drag*(v-v.dot(z)*z)
        z = force/np.linalg.norm(force)
        y = np.cross(z, heading); y /= max(np.linalg.norm(y), 1e-9)
        rotations.append(np.column_stack((np.cross(y, z), y, z)))
        thrusts.append(MASS*np.linalg.norm(force))
    rotations = np.array(rotations)
    quats = Rotation.from_matrix(rotations).as_quat()[:, [3, 0, 1, 2]]
    for i in range(count):
        prev = initial_quaternion if i == 0 else quats[i-1]
        if quats[i].dot(prev) < 0: quats[i] *= -1
    rates = Rotation.from_matrix(np.transpose(rotations[:-1], (0, 2, 1))@rotations[1:]).as_rotvec()/dt
    rates = np.vstack((rates, rates[-1]))
    refs = np.c_[pva[:, :6], quats, rates,
                 np.repeat(np.sqrt(np.array(thrusts)/(4*KF))[:, None], 4, axis=1)]
    controls = np.c_[thrusts[:-1], rates[:-1]]
    return refs, np.clip(controls, LOWER, UPPER)


class MPC:
    def __init__(self, cfg=Config()):
        self.cfg = cfg
        self.derivative, self.step, linear = dynamics()
        self.linear = linear.map(cfg.steps)
        self.controls = None; self.previous = np.array([MASS*G, 0., 0., 0.])
        self.last_time = None; self.logs = []
        n = cfg.steps; self.xsize = (n+1)*NX; self.size = self.xsize+n*NU
        self.Q = np.r_[np.full(3, cfg.position_weight), np.full(3, cfg.velocity_weight),
                      np.full(4, cfg.attitude_weight), np.full(3, .3), np.full(4, 1e-5)]
        self.R = np.array([.02, .05, .05, .05])
        self.S = np.array([.02, .6, .6, .6])
        self.xscale = np.r_[np.ones(13), np.full(4, 1000.)]
        self.uscale = np.array([20., 1., 1., 1.])
        self.scale = np.r_[np.tile(self.xscale, n+1), np.tile(self.uscale, n)]
        self.qdiag = np.tile(self.Q, n+1); self.qdiag[-NX:] *= cfg.terminal_multiplier
        D = sparse.diags([np.ones(n), -np.ones(n-1)], [0, -1], shape=(n, n), format='csc')
        self.D = sparse.kron(D, sparse.eye(NU), format='csc')
        self.SD = sparse.diags(np.tile(self.S, n))
        pu = sparse.diags(np.tile(self.R, n))+self.D.T@self.SD@self.D
        scaling = sparse.diags(self.scale)
        self.P = sparse.triu(2*scaling@sparse.block_diag((sparse.diags(self.qdiag), pu))@scaling, format='csc')
        # Fixed dense block pattern, including numerical zeros: OSQP updates
        # must NEVER silently change matrix sparsity between linearizations.
        rows, cols = [], []
        for k in range(n+1):
            rows.extend(k*NX+np.arange(NX)); cols.extend(k*NX+np.arange(NX))
        for k in range(n):
            rows.extend(((k+1)*NX+np.arange(NX)[:, None]+np.zeros((NX, NX), int)).ravel())
            cols.extend((k*NX+np.arange(NX)[None, :]+np.zeros((NX, NX), int)).ravel())
            rows.extend(((k+1)*NX+np.arange(NX)[:, None]+np.zeros((NX, NU), int)).ravel())
            cols.extend((self.xsize+k*NU+np.arange(NU)[None, :]+np.zeros((NX, NU), int)).ravel())
        rows.extend(self.xsize+np.arange(n*NU)); cols.extend(self.xsize+np.arange(n*NU))
        for k in range(1, n+1):
            rows.extend([self.size+k-1]*2); cols.extend([k*NX+7, k*NX+8])
        self.order = np.lexsort((rows, cols))
        self.rows, self.cols = np.asarray(rows)[self.order], np.asarray(cols)[self.order]
        self.solver = None

    def rollout(self, x, controls):
        states = [np.asarray(x).copy()]
        for u in controls:
            states.append(np.array(self.step(states[-1], u, self.cfg.dt)).ravel())
        return np.array(states)

    def merit(self, states, controls, refs, ff):
        error = (states-refs).ravel()
        delta = np.diff(np.vstack((self.previous, controls)), axis=0)
        tilt_violation = np.maximum(0., np.cos(np.radians(self.cfg.tilt_limit_deg))
                                    -(1-2*np.sum(states[:, 7:9]**2, axis=1)))
        return float(np.dot(self.qdiag, error**2)+np.sum(self.R*(controls-ff)**2)
                     +np.sum(self.S*delta**2)+1e6*np.sum(tilt_violation**2))

    def command(self, state, now, reference):
        """reference(times)->(N,9) physical PVA, NOT legacy preview PVA."""
        began = time.perf_counter(); cfg = self.cfg; n = cfg.steps
        if not np.isfinite(state).all(): raise ValueError('nonfinite MPC state')
        x = np.array(self.step(state, self.previous, cfg.state_prediction_s)).ravel()
        at = now+cfg.state_prediction_s
        pva = reference(at+np.arange(n+1)*cfg.dt)
        refs, ff = reference_states(pva, cfg.dt, x[6:10])
        if self.controls is None:
            controls = ff.copy()
        else:
            offset = max(0., (now-self.last_time)/cfg.dt)
            knots = np.arange(n)
            controls = np.column_stack([np.interp(knots+offset, knots, self.controls[:, j]) for j in range(NU)])
        status = None; qp_iters = 0; inexact = 0; worst_qp_residual = 0.
        for _ in range(cfg.iterations):
            states = self.rollout(x, controls)
            f, aa, bb = self.linear(states[:-1].T, controls.T, cfg.dt)
            aa = np.asarray(aa).reshape(NX, n, NX).transpose(1, 0, 2)
            bb = np.asarray(bb).reshape(NX, n, NU).transpose(1, 0, 2)
            defect = np.asarray(f).T-np.einsum('kij,kj->ki', aa, states[:-1])-np.einsum('kij,kj->ki', bb, controls)
            blocks = [np.ones(self.xsize)]
            for A, B in zip(aa, bb):
                blocks.extend(((-A*self.xscale[None, :]/self.xscale[:, None]).ravel(),
                               (-B*self.uscale[None, :]/self.xscale[:, None]).ravel()))
            blocks.append(np.ones(n*NU))
            blocks.append((-4*states[1:, 7:9]).ravel())
            av = np.concatenate(blocks)[self.order]
            A = sparse.csc_matrix((av, (self.rows, self.cols)), shape=(self.size+n, self.size))
            equality = np.r_[x/self.xscale, (defect/self.xscale).ravel()]
            tilt_lower = np.cos(np.radians(cfg.tilt_limit_deg))-1-2*np.sum(states[1:, 7:9]**2, axis=1)
            trust = np.array([cfg.trust_thrust_n]+[cfg.trust_rate_rps]*3)
            lower = np.r_[equality, (np.maximum(LOWER, controls-trust)/self.uscale).ravel(), tilt_lower]
            upper = np.r_[equality, (np.minimum(UPPER, controls+trust)/self.uscale).ravel(), np.full(n, np.inf)]
            d0 = np.zeros(n*NU); d0[:NU] = self.previous
            q = -2*np.r_[self.qdiag*refs.ravel(), np.tile(self.R, n)*ff.ravel()+self.D.T@(self.SD@d0)]
            q *= self.scale
            if self.solver is None:
                self.solver = osqp.OSQP()
                self.solver.setup(P=self.P, q=q, A=A, l=lower, u=upper, verbose=False,
                                  eps_abs=1e-5, eps_rel=1e-5, max_iter=cfg.qp_iterations,
                                  polish=False, warm_start=True, time_limit=.025)
            else: self.solver.update(q=q, Ax=A.data, l=lower, u=upper)
            self.solver.warm_start(x=np.r_[states.ravel(), controls.ravel()]/self.scale)
            solution = self.solver.solve(); status = solution.info.status
            qp_iters += solution.info.iter
            # RTI does not require the QP's optimality flag. A time-limited
            # iterate may be usable, but only with an explicit primal residual
            # bound AND nonlinear descent below. Infeasible/invalid solutions
            # are never admitted this way. 0.002 is a NUMERICAL QP tolerance,
            # not a changed frame margin or a claimed safety certificate.
            residual = np.inf
            if solution.x is not None and np.isfinite(solution.x).all():
                predicted = A@solution.x
                residual = float(max(np.max(lower-predicted), np.max(predicted-upper), 0.))
            partial_ok = status in ('maximum iterations reached', 'run time limit reached') and residual <= .002
            if solution.x is None or (solution.info.status_val not in (1, 2) and not partial_ok):
                raise RuntimeError(f'MPC QP {status}; primal={solution.info.pri_res}')
            inexact += int(partial_ok); worst_qp_residual = max(worst_qp_residual, residual)
            proposal = np.clip(solution.x[self.xsize:].reshape(n, NU)*self.uscale, LOWER, UPPER)
            best_cost = self.merit(states, controls, refs, ff)
            selected = controls
            # Globalize RTI: a good LINEAR prediction can be a bad nonlinear
            # rollout. Never accept a large QP step without checking its merit.
            for alpha in (1., .5, .25, .125):
                trial = controls+alpha*(proposal-controls)
                cost = self.merit(self.rollout(x, trial), trial, refs, ff)
                if cost < best_cost:
                    selected = trial; best_cost = cost
            controls = selected
        self.controls = controls; self.last_time = now; self.previous = controls[0].copy()
        elapsed = (time.perf_counter()-began)*1000
        self.logs.append(dict(time=float(now), cycle_ms=elapsed, qp_status=status, qp_iterations=qp_iters,
                              inexact_qp_steps=inexact, max_normalized_qp_primal_residual=worst_qp_residual,
                              position_error_m=float(np.linalg.norm(state[:3]-reference(np.array([now]))[0, :3]))))
        if elapsed >= 100.: raise RuntimeError(f'MPC deadline {elapsed:.3f} ms')
        return self.previous.copy()
