import os
import time
from dataclasses import replace

import numpy as np
import pytest

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.checked_soft_racing.planner import Config
from convex_timevarying_window.continuation_racing.guard import GuardedPlan
from convex_timevarying_window.receding_gate_racing.planner import Plan
from convex_timevarying_window.checked_soft_mpc_isolated.worker import Worker, pack_plan, unpack_plan
from convex_timevarying_window.checked_soft_mpc_isolated.px4 import Reference
from convex_timevarying_window.checked_soft_mpc_racing.px4 import Reference as OldReference


def test_guarded_plan_round_trip():
    a = Plan(.2, 0, np.array([1., 1.]), np.arange(36.).reshape(2,6,3), np.zeros(18), 5., 0., 'fixture')
    b = Plan(1.2, 0, a.durations[-1:], a.coefficients[-1:], np.zeros(9), 0., 0., 'brake')
    p = GuardedPlan(a, b, .4)
    import pickle
    q = unpack_plan(pickle.loads(pickle.dumps(pack_plan(p))))
    assert q.end_time == p.end_time and q.clearance == p.clearance
    for t in (-.1, .4, 1.5, 3.):
        for d in range(3): np.testing.assert_equal(q.sample(t,d), p.sample(t,d))
    assert unpack_plan(pack_plan(None)) is None


def test_reference_semantics_unchanged():
    p = Plan(.2, 0, np.array([1.]), np.arange(18.).reshape(1,6,3), np.zeros(9), 5., 0., 'fixture')
    old, new = OldReference([0,0,0]), Reference([0,0,0])
    for ref in (old,new): ref.install(p)
    times = np.linspace(0.,2.,51)
    np.testing.assert_equal(old.sample(times),new.sample(times))


def test_worker_is_separate_process_and_includes_delivery_delay():
    track,_ = build_track()
    cfg = replace(Config(), horizon=2)
    worker = Worker(track,cfg)
    try:
        assert worker.startup['pid'] != os.getpid()
        assert len(worker.geometry.windows)==7
        worker.submit(np.r_[track.start,np.zeros(6)],.15,0)
        with pytest.raises(RuntimeError,match='in flight'):
            worker.submit(np.zeros(9),.2,0)
        time.sleep(.15)
        end = time.monotonic()+5
        response = worker.poll()
        while response is None and time.monotonic()<end:
            time.sleep(.001); response = worker.poll()
        assert response is not None and response[0] is not None, response
        assert response[2]>=150  # The deliberate receiver delay is not hidden.
        assert worker.metrics[-1]['worker_pid']!=os.getpid()
        assert response[2]>=worker.metrics[-1]['worker_wall_ms']
        assert not worker.busy
    finally:
        worker.close()
    assert not worker.process.is_alive()
