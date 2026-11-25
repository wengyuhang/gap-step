"""One persistent spawned planner process, one request in flight.

The controller never waits for a solve. Admission uses submit-to-receive wall
time including IPC, not just IPOPT's process CPU counter. Geometry, objective,
solver budget and reference splice are unchanged.
"""
import multiprocessing as mp
import os
import queue
import time
from types import SimpleNamespace

import numpy as np

from convex_timevarying_window.checked_soft_racing.planner import Planner
from convex_timevarying_window.receding_gate_racing.planner import Plan
from convex_timevarying_window.continuation_racing.guard import GuardedPlan


def pack_plan(plan):
    if plan is None:
        return None
    if isinstance(plan, GuardedPlan):
        return dict(nominal=pack_plan(plan.nominal), brake=pack_plan(plan.brake), clearance=plan.clearance)
    return {name: getattr(plan, name) for name in
            ('made_at', 'first_gate', 'durations', 'coefficients', 'x', 'solve_ms', 'violation', 'status')}


def unpack_plan(value):
    # Explicit wire format avoids GuardedPlan.__getattr__ during unpickling.
    if value is None:
        return None
    if 'nominal' in value:
        return GuardedPlan(unpack_plan(value['nominal']), unpack_plan(value['brake']), value['clearance'])
    return Plan(**value)


def _serve(track, cfg, requests, results, ready):
    try:
        began = time.perf_counter()
        planner = Planner(track, cfg)
        ready.put(dict(pid=os.getpid(), build_s=time.perf_counter()-began, signs=planner.signs))
    except Exception as exc:
        ready.put(dict(failure=repr(exc)))
        return
    while True:
        request = requests.get()
        if request is None:
            return
        state, at, gate, admitted, submitted, reset = request
        if reset:
            planner.search = None
            planner.previous = None
            planner.dual_cache.clear()
            planner.collocation_cache = {}
        planner.commit(unpack_plan(admitted))
        began, cpu, thread = time.perf_counter(), time.process_time(), time.thread_time()
        plan, clearance, error = None, None, None
        try:
            plan = planner.solve(state, at, gate)
            clearance = planner.prefix_clearance(plan, at)
        except Exception as exc:
            error = repr(exc)
        metrics = dict(worker_wall_ms=1000*(time.perf_counter()-began),
                       worker_process_cpu_ms=1000*(time.process_time()-cpu),
                       worker_thread_cpu_ms=1000*(time.thread_time()-thread),
                       queue_ms=1000*(began-submitted), worker_pid=os.getpid())
        if plan is not None:
            metrics['iterations'] = planner.solvers[gate][1].stats().get('iter_count')
        results.put((pack_plan(plan), clearance, error, submitted, metrics))


class Worker:
    def __init__(self, track, cfg):
        context = mp.get_context('spawn')
        self.requests = context.Queue(maxsize=1)
        self.results = context.Queue(maxsize=1)
        self.ready = context.Queue(maxsize=1)
        self.process = context.Process(target=_serve, args=(track, cfg, self.requests, self.results, self.ready), daemon=True)
        self.admitted = None
        self.busy = False
        self.metrics = []
        self.process.start()
        try:
            self.startup = self.ready.get(timeout=60.)
            if 'failure' in self.startup:
                raise RuntimeError(self.startup['failure'])
        except Exception:
            self.close()
            raise
        self.geometry = SimpleNamespace(cfg=cfg, windows=[track.windows[i] for i in track.order], signs=self.startup['signs'])

    def commit(self, plan):
        self.admitted = pack_plan(plan)

    def submit(self, state, at, gate, *, reset=False):
        if self.busy:
            raise RuntimeError('Planner request already in flight')
        submitted = time.perf_counter()
        self.requests.put_nowait((np.asarray(state).copy(), at, gate, self.admitted, submitted, reset))
        self.busy = True

    def poll(self):
        try:
            payload, clearance, error, submitted, metrics = self.results.get_nowait()
        except queue.Empty:
            if not self.process.is_alive():
                raise RuntimeError(f'Planner process exited ({self.process.exitcode})')
            return None
        plan = unpack_plan(payload)
        elapsed = 1000*(time.perf_counter()-submitted)
        self.busy = False
        metrics['end_to_end_ms'] = elapsed
        self.metrics.append(metrics)
        return plan, clearance, elapsed, error

    def close(self):
        if self.process.is_alive():
            try:
                self.requests.put_nowait(None)
            except queue.Full:
                pass
            self.process.join(2.)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(2.)
        for channel in (self.requests, self.results, self.ready):
            channel.close()
            channel.cancel_join_thread()
