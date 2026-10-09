from types import SimpleNamespace
from dataclasses import replace
import numpy as np
import pytest
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.geometry import ConvexAperture
from convex_timevarying_window.wang_free_state_mpc.events import (
    Progress, local_coordinates, plane_events, plan_event_check, planning_block_reason)
from convex_timevarying_window.wang_free_state_mpc.reference import Reference
from convex_timevarying_window.wang_free_state_mpc.passage_audit import audit_positions


def window():
    track, _ = build_track(); w = track.windows[0]
    w.center0 = np.zeros(3); w.angles0_rpy = np.zeros(3)
    w.motion = replace(w.motion, translation_enabled=False, rotation_enabled=False)
    w.aperture = ConvexAperture('circle', radius=2.)
    return w


def plan(z=lambda t:1.-t, crossing=1., end=2., first=0, made=0.):
    def samples(t): return np.column_stack((np.zeros_like(t), np.zeros_like(t), z(np.asarray(t))))
    def sample(t, d=0):
        return samples([t])[0] if d==0 else np.array([0.,0.,-1.]) if d==1 else np.zeros(3)
    return SimpleNamespace(made_at=made, end_time=end, first_gate=first, gate_count=1,
                           crossing_times=np.array([crossing]), sample_many=samples, sample=sample)


def test_vector_pose_matches_time_queries():
    track, _ = build_track(); w = track.windows[0]
    times = np.linspace(0., 9., 27); positions = np.arange(81).reshape(27,3)/7
    expected = np.array([np.r_[q,z] for q,z in (w.world_to_local(p,t) for p,t in zip(positions,times))])
    np.testing.assert_allclose(local_coordinates(w,positions,times), expected, atol=1e-12)


def test_center_crossing_latches_until_entire_body_exits():
    progress = Progress([window()], [1.], .4, .08)
    assert not progress.update([0,0,.1], [0,0,-.1], 0., .1)
    assert progress.gate==0 and progress.phase=='exit' and len(progress.crossings)==1
    assert progress.update([0,0,-.1], [0,0,-.49], .1, .2)
    assert progress.gate==1 and progress.phase=='approach' and len(progress.exits)==1
    assert not progress.update([0,0,-.49], [0,0,.1], .2, .3)
    assert progress.gate==1  # Never decrement the observed progress.


def test_reversing_before_exit_is_not_a_second_attempt():
    progress = Progress([window()], [1.], .4, .08)
    progress.update([0,0,.1], [0,0,-.1], 0., .1)
    progress.update([0,0,-.1], [0,0,.1], .1, .2)
    assert progress.violation=='reversed_before_body_exit' and progress.gate==0


def test_progress_uses_fresh_measurement_time_not_polling_clock():
    queried = []
    class MovingWindow:
        aperture = ConvexAperture('circle',radius=2.)
        def world_to_local(self,p,t):
            queried.append(t)
            return np.asarray(p[:2]), p[2]+10*t
    progress = Progress([MovingWindow()],[1.],.4,.08)
    progress.observe([0,0,1.],0.)
    progress.observe([0,0,-10.1],1.)  # -0.1 m: center crossed, body still in slab.
    assert progress.phase=='exit'
    # Re-reading an old snapshot must not advance its window query time.
    for _ in range(10): assert not progress.observe([0,0,-10.1],1.)
    assert not progress.observe([0,0,1.],0.)  # Also ignore out-of-order samples.
    assert queried==[0.,1.] and progress.violation is None
    assert progress.observe([0,0,-11.1],1.05)
    assert progress.gate==1 and progress.violation is None


def test_outside_opening_is_not_success():
    progress = Progress([window()], [1.], .4, .08)
    progress.update([3,0,.1], [3,0,-.1], 0., .1)
    assert progress.violation=='missed_current_opening' and not progress.crossings


def test_plan_contract_rejects_early_crossing_loop_and_expired_head():
    w = window(); cfg = SimpleNamespace(radius=.4, frame_thickness=.08)
    assert plan_event_check(plan(), [w], [1.], cfg)[0]
    looping = plan(lambda t:-(t-1)*(t-2)*(t-3), crossing=3., end=4.)
    assert plan_event_check(looping, [w], [1.], cfg)==(False,'non_single_passage:0')
    assert plan_event_check(plan(crossing=1.5),[w],[1.],cfg)==(False,'early_or_late_passage:0')
    assert plan_event_check(plan(made=1.1),[w],[1.],cfg)==(False,'head_already_beyond_target:0')
    assert plan_event_check(plan(first=1),[w],[1.],cfg)==(False,'unplanned_gate_event:0')


def test_roundoff_zero_does_not_duplicate_crossing():
    times = np.array([0.,.5,1.,1.5,2.]); positions = np.zeros((5,3))
    positions[:,2] = [1.,.1,-1e-14,-.1,-1.]
    events, _ = plane_events(window(),1.,positions,times,gate=0)
    assert len(events)==1 and events[0]['time']==pytest.approx(1.)


def test_later_gate_infinite_plane_does_not_constrain_distant_head():
    first, later = window(), window()
    later.center0 = np.array([10.,0.,-2.]); later.angles0_rpy = np.array([np.pi,0.,0.])
    def samples(t):
        return np.column_stack((np.interp(t,[0.,2.,3.,4.],[0.,0.,10.,10.]),
            np.zeros_like(t),np.interp(t,[0.,1.,2.,3.,4.],[1.,0.,-3.,-2.,-1.])))
    p = SimpleNamespace(made_at=0., end_time=4., first_gate=0, gate_count=2,
                        crossing_times=np.array([1.,3.]), sample_many=samples)
    cfg = SimpleNamespace(radius=.4,frame_thickness=.08)
    assert plan_event_check(p,[first,later],[1.,1.],cfg)==(True,'single_passage')


def test_truth_audit_detects_repeats_that_first_crossing_counter_misses():
    times = np.linspace(0.,4.,401); w = window()
    audit = audit_positions([w],[1.],plan(lambda t:-(t-1)*(t-2)*(t-3)).sample_many(times),times,.4,.08)
    assert not audit['passed'] and len(audit['aperture_events'])==3
    assert 'returned_near_gate:0' in audit['violations']
    # Plane roots far away from the finite gate are not another gate passage.
    positions = plan().sample_many(times); positions[times>2.,0] = 20.
    positions[times>2.,2] = times[times>2.]-3.
    assert audit_positions([w],[1.],positions,times,.4,.08)['passed']


def test_reference_future_head_and_mpc_use_same_pending_plan():
    ref = Reference([0,0,1]); ref.active = plan()
    future = plan(z=lambda t:2.-t,made=.4)
    assert ref.install(future)
    assert ref.plan_at(.5) is future
    np.testing.assert_equal(ref.state_at(.5,3)[0],ref.sample([.5])[0])
    ref.advance(.5,1,'approach')
    assert ref.pending is None and ref.active.first_gate==0
    assert ref.discarded[-1]['reason']=='passage_stage_changed'


def test_pending_is_discarded_at_center_passage_before_activation():
    ref = Reference([0,0,1]); ref.active = plan(); old = ref.active
    assert ref.install(plan(made=1.1))
    ref.advance(1.02,0,'exit')
    assert ref.pending is None and ref.active is old
    assert not ref.install(plan(made=1.2))


def test_committed_crossing_blocks_replanning_not_flight_reference():
    ref = Reference([0,0,1]); ref.active = plan()
    progress = Progress([window()], [1.], .1, .02)
    assert planning_block_reason(progress,ref,.9,.12)=='committed_passage'
    assert ref.state_at(1.1)[0][2]<0  # Keep flying out; no hover command.
    progress.crossed=True
    assert planning_block_reason(progress,ref,.4,.12)=='body_exiting'


def test_replacement_must_not_regress_or_change_its_head():
    ref = Reference([0,0,1]); ref.active = plan()
    assert ref.candidate_reason(plan(made=.3),0,'approach') is None
    assert ref.candidate_reason(plan(made=.3,end=3.),0,'approach')=='slower_same_horizon'
    assert ref.candidate_reason(plan(made=.3,crossing=1.5),0,'approach')=='delayed_current_passage'
    assert ref.candidate_reason(plan(made=.3,z=lambda t:2.-t),0,'approach')=='head_timeline_changed'
    assert ref.candidate_reason(plan(first=0),1,'approach')=='stale_gate_stage'
