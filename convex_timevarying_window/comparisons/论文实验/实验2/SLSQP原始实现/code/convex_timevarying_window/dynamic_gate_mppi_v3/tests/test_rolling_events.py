import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.scheduler import Scheduler


def observe(s,t,gate,margin=1.,inside=False):
    return s.observe(t,t+np.arange(101)*.01,np.full(100,margin),inside,gate)


def test_full_exit_requests_next_three_without_avoidance_or_three_cycle_wait():
    s=Scheduler();s.gate_passed(1)
    assert s.rolling_pending and not s.pending and not s.avoiding
    r=observe(s,0.,1)
    assert r is not None and r.reason=='gate_exit' and r.first_gate==1
    assert s.running is not None and not s.avoiding
    assert s.finish(r,.08,1,0.,0.,0.)
    assert not s.rolling_pending and not s.pending and not s.avoiding


def test_risk_during_rolling_is_independent_of_result_acceptance():
    s=Scheduler();s.gate_passed(1);r=observe(s,0.,1)
    s.set_risk(True);assert s.avoiding
    assert s.finish(r,.08,1,0.,0.,0.)
    assert s.avoiding  # a new reference is not a safety authorization
    s.set_risk(False);assert not s.avoiding


def test_old_generation_rejected_and_latest_gate_queued_without_avoidance():
    s=Scheduler();s.gate_passed(1);r=observe(s,0.,1)
    s.gate_passed(2)
    assert not s.finish(r,.08,1,0.,0.,0.)
    assert not s.avoiding and s.rolling_pending
    new=observe(s,.08,2)
    assert new is not None and new.first_gate==2 and new.generation==2


def test_gate_event_never_bypasses_execution_window_or_inside_guard():
    s=Scheduler();s.gate_passed(1)
    assert observe(s,0.,1,margin=.01) is None
    assert observe(s,.02,1,inside=True) is None
    assert observe(s,.04,1) is not None


def test_failed_normal_replan_does_not_turn_safe_flight_into_avoidance():
    s=Scheduler();s.gate_passed(1);r=observe(s,0.,1);s.failed(r)
    assert s.rolling_pending and not s.pending and not s.avoiding
    s.gate_passed(1);assert s.generation==1  # no double count from feedback


def test_risk_request_needs_safe_prediction_not_three_cycles():
    s=Scheduler();s.set_risk(True)
    assert observe(s,0.,0,margin=-.1) is None
    s.set_risk(False)
    assert s.pending and not s.avoiding
    assert observe(s,.02,0).reason=='risk'
    assert not s.avoiding


def test_gate_exit_can_replan_while_avoiding_if_current_controls_are_safe():
    s=Scheduler();s.set_risk(True);s.gate_passed(1)
    r=observe(s,0.,1)
    assert r is not None and r.reason=='gate_exit'
    assert s.avoiding  # starting planning never switches flight mode
