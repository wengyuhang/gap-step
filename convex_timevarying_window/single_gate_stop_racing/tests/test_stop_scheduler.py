from convex_timevarying_window.single_gate_stop_racing.scheduler import Scheduler


def test_tracking_error_never_creates_request():
    scheduler=Scheduler()
    scheduler.observe(0.,0,False,100.)
    assert scheduler.pending['reason']=='initial'
    scheduler.submitted(0.);scheduler.resolved(.05,True,None)
    for at,error in ((.1,10.),(.2,.0),(.3,1000.)):
        scheduler.observe(at,0,False,error)
        assert scheduler.pending is None
    assert [event['reason'] for event in scheduler.log if event['action']=='trigger']==['initial']


def test_body_exit_and_existing_request_retry_remain():
    scheduler=Scheduler()
    scheduler.observe(0.,0,False,0.);scheduler.submitted(0.)
    scheduler.resolved(.05,False,'budget')
    assert not scheduler.due(.1)
    assert scheduler.due(.26)
    scheduler.submitted(.26);scheduler.resolved(.3,True,None)
    scheduler.observe(1.,1,True,100.)
    assert scheduler.pending['reason']=='body_exit'
    scheduler.submitted(1.)
    scheduler.observe(1.01,1,False,100.)
    assert scheduler.pending['reason']=='body_exit'
    scheduler.resolved(1.05,True,None)
    assert scheduler.pending is None
