"""Passage task constraints; no Gazebo/performance claim from these tests."""
from dataclasses import replace
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import GateProgress
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import Rollout, hover_state, MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.controller import Controller


def single_gate():
    track, _ = build_track()
    w = track.windows[0]
    w.center0 = np.array([0.,0.,4.])
    w.angles0_rpy = np.array([0.,np.pi/2,0.])
    w.motion = replace(w.motion, translation_enabled=False, rotation_enabled=False)
    return replace(track, start=np.array([-3.,0.,4.]), goal=np.array([3.,0.,4.]),
                   windows=(w,), order=(0,))

def prediction(position, speed, phase=0):
    r = Rollout(single_gate()); s = hover_state(position); s[3] = speed
    return r(s, np.full((1,50,4),MASS*9.8066/4),r.prepare(0.),crossing=phase)


def test_enters_when_body_front_reaches_slab_before_centre():
    p = GateProgress(single_gate())
    assert not p.update(np.array([-1.,0.,4.]),0.,np.array([-.7,0.,4.]),.01,True)
    assert p.crossing == 1 and p.first == 0
    assert not p.update(np.array([-.7,0.,4.]),.01,np.array([.2,0.,4.]),.02,True)
    assert p.crossing == 2
    assert p.update(np.array([.2,0.,4.]),.02,np.array([1.,0.,4.]),.03,True)


def test_outside_crossing_is_rejected_even_without_frame_collision():
    b = prediction([-1.,2.4,4.],4.)
    assert b.clearances.min()>0  # collision-free is NOT task-feasible
    assert not b.valid[0]
    p=GateProgress(single_gate())
    p.update(np.array([-1.,2.4,4.]),0.,np.array([1.,2.4,4.]),.01,True)
    assert p.first==0 and p.violation=='illegal_gate_slab_crossing'


def test_distant_bypass_is_collision_free_but_violates_ordered_task_guard():
    b=prediction([-1.,8.,4.],4.)
    assert b.clearances.min()>0 and not b.valid[0]
    p=GateProgress(single_gate())
    p.update(np.array([-1.,8.,4.]),0.,np.array([1.,8.,4.]),.01,True)
    assert p.violation=='illegal_gate_slab_crossing' and p.first==0
    assert not p.computation_allowed(np.array([1.,8.,4.]),.01)


def test_entry_requires_complete_forward_exit_within_horizon():
    assert prediction([-1.,0.,4.],4.).valid[0]
    partial=prediction([-1.,0.,4.],.5)
    assert partial.clearances.min()>0
    assert partial.phases[0,-1]==1 and not partial.valid[0]


def test_already_inside_can_exit_forward_or_retreat_but_not_linger():
    forward=prediction([-.2,0.,4.],3.,1)
    assert forward.valid[0] and forward.indices[0,-1]==1
    back=prediction([.2,0.,4.],-3.,2)
    assert back.valid[0] and back.indices[0,-1]==0 and back.phases[0,-1]==0
    assert not prediction([-.2,0.,4.],0.,1).valid[0]


def test_retreat_does_not_complete_or_forget_gate():
    p=GateProgress(single_gate(),crossing=2)
    assert not p.update(np.array([.2,0.,4.]),0.,np.array([-1.,0.,4.]),.01,True)
    assert p.first==0 and p.crossing==0 and p.violation is None


def test_avoidance_never_queries_reference_points_or_cost():
    class ForbiddenReference:
        def __getattribute__(self,name):
            raise AssertionError('Avoidance accessed old reference: '+name)
    track=single_gate(); a=Controller(track,seed=7); b=Controller(track,seed=7)
    x=hover_state(track.start)
    sa=a.select(x,0.,avoid=True,reference=ForbiddenReference())
    sb=b.select(x,0.,avoid=True,reference=None)
    assert sa is not None and sb is not None
    np.testing.assert_array_equal(sa.controls,sb.controls)
    assert a.last_diagnostics['reference_weight']==0.


def test_cache_keeps_complete_exit_witness():
    c=Controller(single_gate()); x=hover_state([-1.,0.,4.]);x[3]=4.
    # Deterministic safe candidate is retained, not just an entry prefix.
    c.initial_controls=np.full((50,4),MASS*9.8066/4)
    selection=c.select(x,0.,avoid=True)
    assert selection is not None
    assert selection.indices[-1]==1 and selection.phases[-1]==0
    assert c.publish(selection,x,0.)
    assert c.cache.expires_at==1.


def test_passage_deadline_revalidates_remaining_cache_and_releases_unsafe_one():
    c=Controller(single_gate());x=hover_state([-1.,0.,4.]);x[3]=4.
    c.initial_controls=np.full((50,4),MASS*9.8066/4)
    selection=c.select(x,0.,avoid=True)
    assert selection is not None and c.publish(selection,x,0.)
    deadline=c.passage_deadline(x,0.,0,0,c.rollout.prepare(0.))
    assert deadline is not None and 0<deadline<=1.
    # Prefix revalidation must preserve the original absolute exit time.
    y=selection.states[2].copy()
    assert abs(c.passage_deadline(y,.02,0,0,c.rollout.prepare(.02))-deadline)<1e-8
    # Position is still in the cache tube, but changed velocity makes the
    # remaining controls unsafe. An old commitment must not override safety.
    y[3:6]=[0.,20.,0.]
    assert c.cached_action(y,.02) is not None
    assert c.passage_deadline(y,.02,0,0,c.rollout.prepare(.02)) is None


def test_distant_invalid_tail_does_not_itself_trigger_immediate_avoidance():
    from types import SimpleNamespace
    c=Controller(single_gate());states=np.tile(hover_state(single_gate().start),(1,101,1))
    clear=np.ones((1,100));clear[0,70]=-1.
    prediction=SimpleNamespace(states=states,clearances=clear,valid=np.array([False]))
    assert not c.imminent_risk(prediction)
    clear[0,5]=.01
    assert c.imminent_risk(prediction)
