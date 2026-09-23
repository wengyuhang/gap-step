"""Budgeted dynamic TOGT references; no full-trajectory safety inspection."""
from dataclasses import dataclass
import multiprocessing as mp
import time
import numpy as np
from convex_timevarying_window.togt.native_objective import DynamicRightTailObjective, NativeJointTOGTObjective
from convex_timevarying_window.togt.dynamic_rolling_pvaj import _ClockShiftWindow
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs
from .native_backend import Core


class PolynomialReference:
    """Vectorized evaluation of unchanged MINCO coefficients, for online use.

    The shared MINCO solver/evaluator is untouched. The original evaluator
    loops in Python over query times; a control cycle queries many times.
    """
    def __init__(self, trajectory):
        self.durations=trajectory.durations
        self.coefficients=trajectory.coefficients
        self.ends=np.cumsum(self.durations)
        self.starts=np.r_[0.,self.ends[:-1]]
        self.total_time=float(self.ends[-1])
        self.derivatives=[self.coefficients]
        for d in range(1,8):
            prior=self.derivatives[-1]
            self.derivatives.append(prior[:,1:]*np.arange(1,prior.shape[1])[None,:,None])

    def evaluate(self,time,derivative=0):
        if not 0<=derivative<=7:raise ValueError('derivative must be in [0,7]')
        t=np.asarray(time,dtype=float)
        tolerance=1e-10*max(1.,self.total_time)
        if np.any(t < -tolerance) or np.any(t>self.total_time+tolerance):
            raise ValueError('time outside reference interval')
        ids=np.minimum(np.searchsorted(self.ends,t,side='right'),len(self.ends)-1)
        local=t-self.starts[ids]
        coeff=self.derivatives[derivative][ids]
        result=coeff[...,-1,:].copy()
        for i in range(coeff.shape[-2]-2,-1,-1):result=result*local[...,None]+coeff[...,i,:]
        return result


@dataclass
class Reference:
    trajectory: object
    made_at: float
    first: int
    crossing_times: np.ndarray
    local_points: np.ndarray
    solve_ms: float
    converged: bool
    evaluations: int

    def __post_init__(self):
        if isinstance(self.trajectory,MincoSnap):
            self.trajectory=PolynomialReference(self.trajectory)

    def sample(self, times, indices):
        relative = np.asarray(times)-self.made_at
        query = np.clip(relative, 0., self.trajectory.total_time)
        p, v = (np.asarray(self.trajectory.evaluate(query, d)) for d in (0, 1))
        active = ((relative >= 0) & (relative <= self.trajectory.total_time))[None]
        # Actual full-body progress belongs to the safety FSM, not this cost.
        active = np.broadcast_to(active, np.shape(indices))
        return p[None], v[None], active


class _StopOptimization(Exception):
    pass


class Planner:
    def __init__(self, track, optimization_config):
        self.track, self.optimization_config = track, optimization_config
        self.core = Core()  # Build/load before entering timed online solves.

    def solve(self, head, now, first, budget=.1):
        started = time.perf_counter()
        self.last_stats = {}
        if first>=len(self.track.order):
            # No windows remain: an ordinary PVAJ-to-goal MINCO segment,
            # rather than constructing an invalid zero-window TOGT problem.
            distance=float(np.linalg.norm(self.track.goal-head.position))
            duration=max(1.,np.sqrt(6.*distance/8.),np.linalg.norm(head.velocity)/4.)
            trajectory=MincoSnap(head,BoundaryState(self.track.goal),np.empty((0,3)),np.array([duration]))
            trajectory=PolynomialReference(trajectory)
            return Reference(trajectory,now,first,np.empty(0),np.zeros((len(self.track.order),2)),
                             (time.perf_counter()-started)*1000,False,0)
        windows = [self.track.windows[i] for i in self.track.order[first:first+3]]
        terminal = first+3 >= len(self.track.order)
        if terminal:
            shifted = tuple(_ClockShiftWindow(w, now) for w in windows)
            local = SCWindowTrack("v3_terminal", head.position, self.track.goal,
                                  shifted, tuple(range(len(shifted))))
            objective = NativeJointTOGTObjective(local, self.optimization_config)
            objective.head = head.matrix.T.copy()
        else:
            seed = np.zeros((3, 4))
            c, _, *_ = windows[-1].state_at(now)
            previous, _, *_ = windows[-2].state_at(now)
            direction = c-previous
            seed[:, 0] = c
            seed[:, 1] = 5.*direction/max(np.linalg.norm(direction), 1e-9)
            objective = DynamicRightTailObjective(windows, head, now, self.optimization_config,
                                                  seed, tail_prior_weight=0.)
        objective.core = self.core
        initial = objective.initial_guess()
        best = [np.inf, None]
        evaluations = 0

        def evaluate(x):
            nonlocal evaluations
            # Reserve time for MINCO construction and IPC. This is an anytime
            # reference optimizer, not a claim of optimizer convergence.
            if time.perf_counter()-started >= .75*budget:
                raise _StopOptimization()
            value, gradient = objective.scipy_value_and_gradient(x)
            evaluations += 1
            if np.isfinite(value) and value < best[0]:
                best[:] = [value, x.copy()]
            return value, gradient

        converged = False
        try:
            result = _minimize_togt_lbfgs(evaluate, initial, self.optimization_config)
            converged = bool(result.success)
        except _StopOptimization:
            pass
        self.last_stats = {"best_objective":float(best[0]), "evaluations":evaluations,
                           "elapsed_ms":(time.perf_counter()-started)*1000}
        if best[1] is None or best[0] >= self.optimization_config.invalid_trial_cost:
            return None
        forward = objective.forward(best[1])
        if terminal:
            # NativeJoint.forward uses a resting head; rebuild with the actual PVAJ.
            trajectory = MincoSnap(head, BoundaryState(self.track.goal),
                                   forward.waypoints, forward.durations)
            crossings = now+np.cumsum(forward.durations)[:-1]
        else:
            trajectory = forward.trajectory
            crossings = now+np.cumsum(forward.durations)
        local_points = np.zeros((len(self.track.order), 2))
        local_points[first:first+len(windows)] = forward.local_points
        trajectory=PolynomialReference(trajectory)
        elapsed = time.perf_counter()-started
        if elapsed >= budget:
            self.last_stats["elapsed_ms"] = elapsed*1000
            return None
        return Reference(trajectory, now, first, crossings, local_points,
                         elapsed*1000, converged, evaluations)


def _worker(connection, track, config):
    planner = Planner(track, config)
    connection.send(("ready", None))
    while True:
        request = connection.recv()
        if request is None: break
        identifier, head, anchor, first = request
        try:
            result = planner.solve(head, anchor, first)
            if result is None:
                result = {"error":"reference_not_returned", "details":planner.last_stats}
            connection.send((identifier, result))
        except Exception as error:
            connection.send((identifier, {"error":repr(error)}))


class AsyncPlanner:
    """Single isolated worker: caller keeps executing the validated control cache."""
    def __init__(self, track, config):
        self.context = mp.get_context("spawn")
        self.track, self.config = track, config
        self.retired = []
        self._start_worker()

    def _start_worker(self):
        self.connection, child = self.context.Pipe()
        self.process = self.context.Process(target=_worker, args=(child, self.track, self.config), daemon=True)
        self.process.start()
        self.ready = False
        self.running = None
        self.started = None

    def poll(self):
        if self.running is not None and time.perf_counter()-self.started >= .1:
            identifier = self.running
            self.process.terminate()
            self.retired.append(self.process)
            self.connection.close()
            self._start_worker()
            return identifier, {"error":"worker_deadline_exceeded"}
        if self.connection.poll():
            identifier, result = self.connection.recv()
            if identifier == "ready": self.ready = True; return None
            self.running = None
            return identifier, result
        return None

    def submit(self, identifier, head, anchor, first):
        if not self.ready or self.running is not None: return False
        self.started = time.perf_counter()
        self.running = identifier
        self.connection.send((identifier, head, anchor, first))
        return True

    def close(self):
        if self.process.is_alive():
            self.process.terminate(); self.process.join(timeout=1.)
        self.connection.close()
        for process in self.retired: process.join(timeout=.1)
