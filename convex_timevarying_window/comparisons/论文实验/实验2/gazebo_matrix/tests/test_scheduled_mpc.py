from queue import Queue
from types import SimpleNamespace
import time

import numpy as np
import pytest

from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state, MASS, G
from convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix.scheduled_mpc import MPC, scheduled_control
from convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix.reference import Reference
from convex_timevarying_window.comparisons.论文实验.实验2.independent_repair.run import frozen_plan


def packet(start, values, sequence=0):
    return dict(start=start, controls=np.asarray(values, float), sequence=sequence, log=dict(cycle_ms=1.))


def test_sequence_uses_current_clock_and_future_switch():
    plans = [packet(0., [[1, 0, 0, 0], [2, 0, 0, 0], [3, 0, 0, 0]]),
             packet(.08, [[8, 0, 0, 0], [9, 0, 0, 0]])]
    np.testing.assert_array_equal(scheduled_control(plans, .06, np.zeros(4), .04), [2, 0, 0, 0])
    np.testing.assert_array_equal(scheduled_control(plans, .08, np.zeros(4), .04), [8, 0, 0, 0])
    with pytest.raises(RuntimeError, match='horizon_expired'):
        scheduled_control(plans, .16, np.zeros(4), .04)


def test_expired_solution_is_discarded():
    controller = MPC()
    original = packet(0., np.zeros((25, 4)))
    controller.plans = [original]
    controller.results.close()
    controller.results = Queue()
    controller.results.put(packet(.08, np.ones((25, 4)), 1))
    controller._receive(.09)
    assert controller.plans == [original]
    assert controller.deadline_misses == 1
    assert controller.logs[-1]['accepted'] is False
    controller.requests.close()


def test_solution_waits_for_its_application_time():
    controller = MPC()
    controller.results.close()
    controller.results = Queue()
    controller.results.put(packet(.08, np.ones((25, 4))))
    controller._receive(.04)
    np.testing.assert_array_equal(scheduled_control(controller.plans, .04, np.zeros(4), .04), np.zeros(4))
    np.testing.assert_array_equal(scheduled_control(controller.plans, .08, np.zeros(4), .04), np.ones(4))
    controller.requests.close()


def test_worker_process_tracks_hover_and_closes(tmp_path):
    coefficients = np.zeros((1, 8, 3))
    coefficients[0, 0] = [0, 0, 3]
    plan = frozen_plan(dict(durations=[2.], coefficients=coefficients.tolist(), variables=[]))
    reference = Reference([0, 0, 3], plan)
    controller = MPC()
    state = hover_state([0, 0, 3])
    controller.command(state, -1., reference.sample)
    session = SimpleNamespace(clock=SimpleNamespace(value=29.), out=tmp_path)
    controller.bind_session(session)
    try:
        controller.command(state, -1., reference.sample)
        deadline = time.monotonic() + 10.
        while controller.busy and time.monotonic() < deadline:
            time.sleep(.01)
            controller.command(state, -1., reference.sample)
        assert controller.plans
        session.clock.value = 29.084
        command = controller.command(state, -.916, reference.sample)
        np.testing.assert_allclose(command, [MASS * G, 0, 0, 0], atol=1e-5)
        assert controller.worker.pid != __import__('os').getpid()
    finally:
        controller.close()
    assert not controller.worker.is_alive()
    assert (tmp_path / 'tracking_schedule.json').is_file()


def test_worker_error_is_preserved_during_shutdown():
    controller = MPC()
    controller.results.close()
    controller.results = Queue()
    controller.results.put(dict(error='worker failed during shutdown'))
    controller._receive(0., closing=True)
    assert controller.worker_errors == ['worker failed during shutdown']
    controller.requests.close()


def test_current_seven_gate_entry_uses_scheduled_tracker(monkeypatch):
    from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import flight
    from convex_timevarying_window.comparisons.论文实验.实验2.radius_ablation import execution
    from convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix.scheduled_mpc import ScheduledSession
    for name in ('MPC', 'RecordedSession', 'scenario'):
        monkeypatch.setattr(execution, name, getattr(execution, name))
    monkeypatch.setattr(execution.world, 'launch', execution.world.launch)
    flight.configure()
    assert execution.MPC is MPC
    assert execution.RecordedSession is ScheduledSession
    assert ScheduledSession.command_period_s == .004
