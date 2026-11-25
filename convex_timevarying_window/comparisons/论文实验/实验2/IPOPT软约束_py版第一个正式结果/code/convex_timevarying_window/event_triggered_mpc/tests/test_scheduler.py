from convex_timevarying_window.event_triggered_mpc.scheduler import Scheduler


def accept(s, t):
    s.submitted(t)
    return s.resolved(t+.06, True, None)


def test_no_periodic_work_after_initial_acceptance():
    s=Scheduler();s.observe(0.,0,False,0.)
    assert accept(s,0.)['reason']=='initial'
    for t in (.2,1.,20.,60.):
        s.observe(t,0,False,.1)
        assert not s.due(t)


def test_body_exit_triggers_once_including_final_exit():
    s=Scheduler();s.observe(0.,0,False,0.);accept(s,0.)
    for gate in range(1,8):
        s.observe(gate,gate,True,0.)
        assert accept(s,gate)['reason']=='body_exit'
        s.observe(gate+.1,gate,False,0.)
        assert not s.due(gate+.1)


def test_tracking_hysteresis_no_continuous_replanning():
    s=Scheduler();s.observe(0.,0,False,0.);accept(s,0.)
    s.observe(1.,0,False,.36);assert accept(s,1.)['reason']=='tracking_error'
    for t in (2.,3.):
        s.observe(t,0,False,.5);assert not s.due(t)
    s.observe(4.,0,False,.17);s.observe(5.,0,False,.36)
    assert s.due(5.)


def test_failed_candidate_retries_pending_event_only():
    s=Scheduler();s.observe(0.,0,False,0.);s.submitted(0.)
    s.resolved(.06,False,'optimizer_residual')
    assert not s.due(.25) and s.due(.27)
    r=accept(s,.27);assert r['attempts']==2 and r['event_id']==1
    assert not s.due(100.)


def test_new_exit_survives_old_inflight_response():
    s=Scheduler();s.observe(0.,0,False,0.);s.submitted(0.)
    s.observe(.03,1,True,.1)
    s.resolved(.06,False,'stale_gate')
    assert s.due(.06) and s.pending['gate']==1
    assert accept(s,.07)['reason']=='body_exit'


def test_blocked_event_is_not_consumed_or_rescheduled():
    s=Scheduler();s.observe(0.,0,False,0.);accept(s,0.)
    s.observe(1.,0,False,.36)
    for t in (1.1,1.2,2.):
        s.observe(t,0,False,.4)
        assert s.due(t) and s.pending['attempts']==0
    assert accept(s,3.)['reason']=='tracking_error'
