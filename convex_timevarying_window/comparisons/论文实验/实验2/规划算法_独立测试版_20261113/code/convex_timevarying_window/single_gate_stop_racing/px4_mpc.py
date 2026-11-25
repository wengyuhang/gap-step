"""Trajectory-only NMPC, with the actual PX4/x500 prediction chain."""
from dataclasses import dataclass
import time
import casadi as ca
import numpy as np
from scipy.optimize import minimize
from .px4_model import native_dynamics, dynamics, initial_state, NX, DT
from convex_timevarying_window.simple_two_gate_soft_only_mpc.tracking_mpc import Config as BaseConfig
from convex_timevarying_window.checked_soft_mpc_racing.mpc import LOWER, UPPER, NU, MASS, G, reference_states


@dataclass(frozen=True)
class Config(BaseConfig):
    state_prediction_s: float = .03
    iterations: int = 1
    box_iterations: int = 40
    max_solve_ms: float = 85.


class SolveBudgetReached(Exception):
    pass


class MPC:
    def __init__(self, cfg=None):
        # 30 ms was measured from state timestamp to received rate setpoint in
        # the previous flight. Refine by measured compute latency, not by gates.
        cfg = Config() if cfg is None else cfg
        if cfg.dt != .04:
            raise ValueError('PX4 model is validated at 40 ms nodes / 4 ms internal clock')
        self.cfg = cfg; self.controls = None; self.last_time = None; self.logs = []
        self.previous = np.array([MASS*G, 0., 0., 0.])
        self.tick, self.step, linear, self.allocation = native_dynamics()
        self._accumulate = self.step.mapaccum(cfg.steps)
        self.Q = np.r_[np.full(3, cfg.position_weight), np.full(3, cfg.velocity_weight),
                      np.full(4, cfg.attitude_weight), np.full(3, .3), np.full(4, 1e-5), np.zeros(NX-17)]
        self.R = np.array([.02, .05, .05, .05]); self.S = np.array([.02, .6, .6, .6])
        self.uscale = np.array([20., 1., 1., 1.])
        self.observer = None; self.observer_time = None; self.observer_physical = None
        self.deliveries = []; self.delivery_delay = cfg.state_prediction_s
        # Direct single shooting uses the SAME nonlinear tracking objective.
        # With the real saturating allocator, a linearized QP can propose a
        # direction which its nonlinear line search rejects. L-BFGS-B instead
        # differentiates the actual prediction, including all internal states.
        _, symbolic_step, _, _ = dynamics()
        controls = ca.MX.sym('controls', NU, cfg.steps)
        initial = ca.MX.sym('initial', NX)
        reference = ca.MX.sym('reference', 17, cfg.steps+1)
        ff = ca.MX.sym('feedforward', NU, cfg.steps)
        previous = ca.MX.sym('previous', NU)
        predicted = ca.horzcat(initial, symbolic_step.mapaccum(cfg.steps)(initial, controls, cfg.dt))
        cost = 0.
        for k in range(cfg.steps+1):
            error = predicted[:17, k]-reference[:, k]
            cost += (cfg.terminal_multiplier if k == cfg.steps else 1.)*ca.dot(ca.DM(self.Q[:17]), error**2)
        for k in range(cfg.steps):
            du = controls[:, k]-(previous if k == 0 else controls[:, k-1])
            cost += ca.dot(ca.DM(self.R), (controls[:, k]-ff[:, k])**2)+ca.dot(ca.DM(self.S), du**2)
        function = ca.Function('tracking_cost_gradient', [controls, initial, reference, ff, previous],
                               [cost, ca.gradient(cost, controls)])
        import hashlib
        from pathlib import Path
        signature = hashlib.sha256(Path(__file__).read_bytes()+repr(cfg).encode()).hexdigest()
        *_, self.objective = native_dynamics((function,), signature)

    def advance(self, state, command, duration):
        # Biquad memory is clocked at simulator frequency, not at MPC frequency.
        x = np.asarray(state).copy()
        count = max(1, int(round(duration/DT)))
        for _ in range(count):
            x = np.asarray(self.tick(x, command, duration/count)).ravel()
        return x

    def observe(self, state, now):
        if self.observer is None or now < self.observer_time:
            self.observer = initial_state(state)
            self.deliveries.clear()
        elif now > self.observer_time:
            begin = self.observer_time; duration = now-begin
            count = max(1, int(round(duration/DT)))
            old = self.observer_physical
            for k in range(count):
                t = begin+k*duration/count
                eligible = [v for stamp, v in self.deliveries if stamp <= t]
                u = eligible[-1] if eligible else np.array([MASS*G, 0., 0., 0.])
                self.observer = np.asarray(self.tick(self.observer, u, duration/count)).ravel()
                fraction = (k+1)/count
                self.observer[:17] = old+fraction*(state-old)
                self.observer[6:10] /= np.linalg.norm(self.observer[6:10])
        self.observer[:17] = state
        self.observer_time = now; self.observer_physical = state.copy()
        self.deliveries = [(t, u) for t, u in self.deliveries if t > now-.2]
        return self.observer.copy()

    def rollout(self, state, controls):
        return np.vstack((state, np.asarray(self._accumulate(state, controls.T, self.cfg.dt)).T))

    def command(self, state, now, reference):
        began = time.perf_counter(); cfg = self.cfg; n = cfg.steps
        state = np.asarray(state, float)
        if state.shape != (17,) or not np.isfinite(state).all():
            raise ValueError('expected finite physical state[17]')
        x = self.observe(state, now)
        delay = self.delivery_delay
        x = self.advance(x, self.previous, delay)
        refs17, ff = reference_states(reference(now+delay+np.arange(n+1)*cfg.dt), cfg.dt, x[6:10])
        # Hidden controller states are predicted but never penalized toward zero.
        if self.controls is None:
            controls = ff.copy()
        else:
            offset = max(0., (now-self.last_time)/cfg.dt); knots = np.arange(n)
            controls = np.column_stack([np.interp(knots+offset, knots, self.controls[:, j]) for j in range(NU)])
        controls = np.clip(controls, LOWER, UPPER)
        best_controls = controls.copy(); best_value = np.inf
        evaluations = 0
        def objective(vector):
            nonlocal best_controls, best_value, evaluations
            if np.isfinite(best_value) and 1000*(time.perf_counter()-began) >= cfg.max_solve_ms:
                raise SolveBudgetReached
            physical_controls = vector.reshape(n, NU)*self.uscale
            value, gradient = self.objective(physical_controls.T, x, refs17.T, ff.T, self.previous)
            evaluations += 1
            value = float(value)
            gradient = np.asarray(gradient).T*self.uscale
            if np.isfinite(value) and value < best_value:
                best_value, best_controls = value, physical_controls.copy()
            return value, gradient.ravel()
        initial_cost = objective((controls/self.uscale).ravel())[0]
        try:
            result = minimize(objective, (controls/self.uscale).ravel(), jac=True, method='L-BFGS-B',
                bounds=list(zip(np.tile(LOWER/self.uscale, n), np.tile(UPPER/self.uscale, n))),
                options=dict(maxiter=cfg.box_iterations, maxls=12, maxcor=12, ftol=cfg.box_ftol, gtol=cfg.box_gtol))
            status, converged, iterations = str(result.message), bool(result.success), int(result.nit)
        except SolveBudgetReached:
            status, converged, iterations = 'online_solve_budget_best_evaluated_iterate', False, None
        controls = np.clip(best_controls, LOWER, UPPER)
        stats = [dict(status=status, converged=converged, iterations=iterations,
                      evaluations=evaluations, initial_value=initial_cost, final_value=best_value)]
        steps = [float(best_value < initial_cost)]
        self.controls = controls; self.previous = controls[0].copy(); self.last_time = now
        elapsed = 1000*(time.perf_counter()-began)
        # Include the measured computing time plus the 8 ms telemetry/transport
        # offset seen in recorded setpoint receipt times. No path retiming.
        self.delivery_delay = .8*self.delivery_delay+.2*(elapsed*.001+.008)
        self.deliveries.append((now+elapsed*.001+.008, self.previous.copy()))
        self.logs.append(dict(time=float(now), cycle_ms=elapsed,
            controller='px4_pid_normalized_allocation_x500_tracking_lbfgs',
            qp_status='tracking_box', qp_iterations=iterations,
            box_solves=stats, nonlinear_steps=steps, command=self.previous.tolist(),
            state_prediction_s=float(delay), estimated_rate_integral=self.observer[17:20].tolist(),
            position_error_m=float(np.linalg.norm(state[:3]-reference(np.array([now]))[0, :3]))))
        return self.previous.copy()
