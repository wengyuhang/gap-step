"""A fixed-application-time MPC worker and a clock-indexed command sender."""
from collections import deque
from dataclasses import asdict
import json
import multiprocessing as mp
from queue import Empty
import time
import traceback

import numpy as np

from convex_timevarying_window.checked_soft_mpc_racing.mpc import MASS, G, hover_state
from convex_timevarying_window.togt_style_mpc.aligned_controller import next_control_tick
from convex_timevarying_window.togt_style_mpc.shooting_controller import Config
from ..independent_repair.run import frozen_plan
from .reference import MPC as TrackingMPC, Reference
from ..no_tilt_comparison.flight import RecordedSession


class ScheduledSession(RecordedSession):
    command_period_s = .004


def scheduled_control(plans, when, default, dt):
    """Only a sequence whose application time has arrived can supply a command."""
    for plan in reversed(plans):
        elapsed = when - plan['start']
        if elapsed < -1e-9:
            continue
        index = int(np.floor((elapsed + 1e-9) / dt))
        if index >= len(plan['controls']):
            raise RuntimeError('scheduled_mpc_control_horizon_expired')
        return np.asarray(plan['controls'][index]).copy()
    return np.asarray(default).copy()


class PredictionMPC(TrackingMPC):
    def applied_at(self, when):
        # Observer memory uses delivered commands. The future rollout uses the
        # sequences already accepted by the sender, including pending switches.
        if not self.deliveries or when > self.deliveries[-1][0] + 1e-9:
            return scheduled_control(self.scheduled, when, self.previous_input, self.cfg.dt)
        return super().applied_at(when)


def solve_worker(requests, results, stopping, config, start, raw_plan):
    try:
        controller = PredictionMPC(Config(**config))
        controller.scheduled = []
        controller.previous_input = np.array([MASS * G, 0., 0., 0.])
        reference = Reference(start, frozen_plan(raw_plan))
        controller.command(hover_state(start), -1., reference.sample)
        controller.controls = None
        controller.last_time = None
        controller.observer = None
        controller.observer_time = None
        controller.observer_physical = None
        results.put(dict(ready=True, metadata=controller.metadata))
        while not stopping.is_set():
            try:
                request = requests.get(timeout=.1)
            except Empty:
                continue
            controller.deliveries = request['deliveries']
            controller.scheduled = request['scheduled']
            controller.previous_input = request['previous_input']
            reference.advance(request['clock'])
            controller.command(request['state'], request['state_time'], reference.sample,
                               execution_time=request['application_time'])
            results.put(dict(sequence=request['sequence'], start=request['application_time'],
                             controls=controller.controls.copy(), log=controller.logs[-1].copy()))
    except BaseException:
        results.put(dict(error=traceback.format_exc()))


class MPC:
    """Preserve the tracking OCP and run finite solves outside the sender."""
    uses_simulation_schedule = True

    def __init__(self, *, solve_period_s=.04, application_lead_s=.08):
        self.cfg = Config(max_solve_ms=None)
        self.solve_period = solve_period_s
        self.application_lead = application_lead_s
        self.previous = np.array([MASS * G, 0., 0., 0.])
        self.controls = None
        self.last_time = None
        self.logs = []
        self.metadata = {}
        self.session = None
        self.worker = None
        self.plans = []
        self.computed = []
        self.deliveries = deque()
        self.busy = False
        self.last_request = -np.inf
        self.sequence = 0
        self.deadline_misses = 0
        self.dispatches = []
        self.startup = None
        self.worker_errors = []
        self.closed = False
        self.context = mp.get_context('spawn')
        self.requests = self.context.Queue(maxsize=1)
        self.results = self.context.Queue(maxsize=2)
        self.stopping = self.context.Event()

    def _start(self, reference):
        timeline = reference.__self__
        plan = timeline.frozen_plan
        raw = dict(durations=plan.durations.tolist(), coefficients=plan.coefficients.tolist(), variables=[])
        self.worker = self.context.Process(target=solve_worker,
            args=(self.requests, self.results, self.stopping, asdict(self.cfg), timeline.start, raw),
            daemon=True)
        self.worker.start()
        try:
            ready = self.results.get(timeout=180.)
            if 'error' in ready:
                raise RuntimeError(ready['error'])
        except BaseException:
            self.close()
            raise
        self.metadata = dict(ready['metadata'],
            implementation='scheduled_process_single_shooting_ctbr_mpc',
            solve_period_s=self.solve_period, application_lead_s=self.application_lead,
            sender_period_s=.004, optimization_budget='fixed max_iterations; application deadline checked by sender',
            late_plan_handling='discard candidate and continue the accepted time-indexed sequence')
        self.startup = raw

    def bind_session(self, session):
        self.session = session
        session.command_sent = self.command_sent

    def _receive(self, now, closing=False):
        while True:
            try:
                packet = self.results.get_nowait()
            except Empty:
                break
            if 'error' in packet:
                self.worker_errors.append(packet['error'])
                if not closing:
                    raise RuntimeError(packet['error'])
                continue
            if packet.get('ready'):
                continue
            self.busy = False
            accepted = not closing and now <= packet['start'] - .004 + 1e-9
            log = packet['log']
            log.update(sequence=packet['sequence'], scheduled_application_time_s=packet['start'],
                       result_received_sim_s=now, accepted=accepted, closed_before_application=closing)
            self.logs.append(log)
            self.computed.append(packet)
            if accepted:
                self.plans.append(packet)
            elif not closing:
                self.deadline_misses += 1

    def command(self, state, now, reference):
        began = time.perf_counter()
        state = np.asarray(state, float)
        if state.shape != (17,) or not np.isfinite(state).all() or np.linalg.norm(state[6:10]) < .5:
            raise ValueError('Expected finite measured physical state')
        if self.worker is None:
            self._start(reference)
            self.logs.append(dict(cycle_ms=1000 * (time.perf_counter() - began), startup=True))
            return self.previous.copy()
        clock = float(self.session.clock.value) - 30.
        self._receive(clock)
        if self.worker.exitcode is not None:
            raise RuntimeError(f'scheduled_mpc_worker_exit_{self.worker.exitcode}')
        if not self.busy and clock - self.last_request >= self.solve_period - 1e-9:
            application = next_control_tick(clock + self.application_lead)
            self.requests.put_nowait(dict(sequence=self.sequence, state=state.copy(), state_time=now,
                clock=clock, application_time=application, deliveries=list(self.deliveries),
                scheduled=self.plans.copy(), previous_input=self.previous.copy()))
            self.sequence += 1
            self.last_request = clock
            self.busy = True
        applicable = [i for i, plan in enumerate(self.plans) if plan['start'] <= clock]
        if applicable:
            self.plans = self.plans[applicable[-1]:]
        if clock >= 0. and not any(plan['start'] <= next_control_tick(clock) for plan in self.plans):
            raise RuntimeError('scheduled_mpc_missing_race_control')
        self.previous = scheduled_control(self.plans, next_control_tick(clock), self.previous, self.cfg.dt)
        self.last_time = now
        self.dispatches.append([clock, now, 1000 * (time.perf_counter() - began), self.sequence - 1])
        return self.previous.copy()

    def command_sent(self, command, sim_time):
        self.deliveries.append((next_control_tick(sim_time), np.asarray(command).copy()))
        while len(self.deliveries) > 1 and self.deliveries[1][0] < sim_time - 1.2:
            self.deliveries.popleft()

    def close(self):
        if self.closed:
            return
        self.stopping.set()
        terminated = False
        now = float(self.session.clock.value) - 30. if self.session is not None else 0.
        if self.worker is not None:
            deadline = time.monotonic() + 5.
            while self.worker.is_alive() and time.monotonic() < deadline:
                self._receive(now, closing=True)
                self.worker.join(.02)
            terminated = self.worker.is_alive()
            if terminated:
                self.worker.terminate()
                self.worker.join(5.)
        self._receive(now, closing=True)
        if self.session is not None:
            np.savez_compressed(self.session.out / 'tracking_schedule.npz',
                dispatches=np.asarray(self.dispatches),
                starts=np.asarray([packet['start'] for packet in self.computed]),
                controls=np.asarray([packet['controls'] for packet in self.computed]))
            summary = dict(computed_plans=len(self.computed),
                accepted_plans=sum(log.get('accepted', False) for log in self.logs),
                deadline_misses=self.deadline_misses, command_dispatches=len(self.dispatches),
                sender_selection_max_ms=max((row[2] for row in self.dispatches), default=0.),
                worker_terminated_on_close=terminated, worker_errors=self.worker_errors,
                metadata=self.metadata)
            (self.session.out / 'tracking_schedule.json').write_text(json.dumps(summary, indent=2) + '\n')
        self.requests.close()
        self.results.close()
        self.closed = True
