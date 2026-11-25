from convex_timevarying_window.simple_two_gate_soft_only_mpc.scheduler import Scheduler


def ready():
    s = Scheduler()
    s.observe(0., 0, False, 0.)
    s.submitted(0.)
    s.resolved(.02, True, None)
    return s


def test_error_immediately_triggers_without_exit_or_timer():
    s = ready()
    s.observe(.021, 0, False, .36)
    assert s.due(.021) and s.pending['reason'] == 'tracking_error'
    s.submitted(.021)
    s.observe(.022, 0, False, 2.)
    assert not s.due(.022)
    s.resolved(.04, True, None)
    s.observe(.041, 0, False, .5)
    assert s.due(.041)


def test_no_periodic_work_when_tracking_is_good():
    s = ready()
    for t in (1., 2., 60.):
        s.observe(t, 0, False, .1)
        assert not s.due(t)


def test_exit_during_recovery_survives_stale_response():
    s = ready()
    s.observe(1., 0, False, .5)
    s.submitted(1.)
    s.observe(1.01, 1, True, .5)
    s.resolved(1.02, False, 'stale_gate')
    assert s.due(1.02) and s.pending['gate'] == 1
    assert s.pending['reason'] == 'body_exit'
