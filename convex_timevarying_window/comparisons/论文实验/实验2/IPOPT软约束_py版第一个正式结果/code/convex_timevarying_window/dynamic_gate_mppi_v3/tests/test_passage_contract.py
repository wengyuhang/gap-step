from dataclasses import replace
from types import SimpleNamespace
import numpy as np

from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import CTBRRollout
from convex_timevarying_window.dynamic_gate_mppi_v3.commitment_gpu import CommitmentController
from convex_timevarying_window.dynamic_gate_mppi_v3.passage_contract import candidate_pool,entry_merit
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS,hover_state
from convex_timevarying_window.dynamic_gate_mppi_v3.scheduler import Scheduler
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate


def test_event_endpoint_ignores_unexecuted_tail_but_never_exports_it():
    cfg=replace(Config(),horizon=2.5,passage_event_tail=True)
    r=CTBRRollout(single_gate(),cfg);x=hover_state([-1.,0.,4.]);x[3]=4.
    u=np.tile([MASS*9.8066,0,0,0],(1,125,1));u[0,100:,1]=5.
    b=r(x,u,r.prepare(0.))
    assert b.valid[0] and b.indices[0,-1]==1
    exit_step=np.flatnonzero(b.indices[0]>0)[0]
    assert 30<=b.end_steps[0]-exit_step<=31
    s=CommitmentController.selection_from_batch(0.,u[0],b,1)
    assert len(s.controls)*2==b.end_steps[0]
    assert len(s.states)==len(s.clearances)+1==len(s.controls)*2+1
    full=CTBRRollout(single_gate(),replace(cfg,passage_event_tail=False))
    assert not full(x,u,full.prepare(0.)).valid[0]


def test_event_tail_still_rejects_unsafe_executed_part():
    r=CTBRRollout(single_gate(),replace(Config(),horizon=2.5,passage_event_tail=True))
    x=hover_state([-1.,0.,4.]);x[3]=4.
    u=np.tile([MASS*9.8066,0,0,0],(1,125,1))
    nominal=r(x,u,r.prepare(0.));exit_step=np.flatnonzero(nominal.indices[0]>0)[0]
    u[0,(exit_step+1)//2,1]=5.
    b=r(x,u,r.prepare(0.))
    assert not b.valid[0] and b.rejection_bits[0]&64


def test_near_control_safety_is_not_full_horizon_feasibility():
    r=CTBRRollout(single_gate(),replace(Config(),horizon=2.5))
    u=np.tile([MASS*9.8066,0,0,0],(1,125,1));u[0,80:,1]=5.
    b=r(hover_state(single_gate().start),u,r.prepare(0.))
    assert b.near_valid[0] and not b.valid[0]
    u[0,0,1]=5.
    assert not r(hover_state(single_gate().start),u,r.prepare(0.)).near_valid[0]


def test_recovery_update_must_improve_and_complete_passage_wins():
    ids=np.zeros((3,251),int);phases=ids.copy();valid=np.ones(3,bool)
    improving=np.array([False,True,False])
    pool=candidate_pool(valid,ids,phases,0,0,True,0.,None,improving)
    np.testing.assert_array_equal(pool,[False,True,False])
    ids[2,100:]=1
    pool=candidate_pool(valid,ids,phases,0,0,True,0.,None,improving)
    np.testing.assert_array_equal(pool,[False,False,True])
    assert not candidate_pool(valid,ids,phases,0,2,True,0.,.5,improving).any()


def test_recovery_merit_accounts_for_alignment_velocity_not_just_position():
    cfg=Config();r=CTBRRollout(single_gate(),cfg);pose=r.prepare(0.)[0][-1,0]
    x=hover_state([-3.,2.,4.]);aligned=hover_state([-3.,0.,4.]);fast=aligned.copy();fast[4]=12.
    a=entry_merit(np.array([x,aligned,fast]),pose,pose,r.signs[0],cfg)
    assert a[1]<a[0] and a[2]>a[1]


def test_far_tracking_failure_requests_replan_without_switching_mode():
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=64))
    class Path:
        total_time=10.
        def evaluate(self,t,d):
            return np.broadcast_to(single_gate().start if d==0 else np.zeros(3),np.shape(t)+(3,)).copy()
    ref=SimpleNamespace(made_at=0.,trajectory=Path())
    evaluate=c.gpu.evaluate
    def far_invalid(*args,**kwargs):
        b=evaluate(*args,**kwargs)
        if len(args[1])==366:b.valid[1:6]=False
        return b
    c.gpu.evaluate=far_invalid
    c.select(hover_state(single_gate().start),0.,reference=ref)
    assert c.last_diagnostics['tracking_near_safe']
    assert c.last_diagnostics['replan_requested']
    assert not c.last_diagnostics['avoidance']
    scheduler=Scheduler();scheduler.flag_prediction_warning()
    assert scheduler.pending and not scheduler.avoiding


def test_valid_witness_survives_an_ordinary_nonpassage_publish():
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=64))
    x=hover_state([-1.,0.,4.]);x[3]=4.
    u=np.tile([MASS*9.8066,0,0,0],(125,1));b=c.rollout(x,u[None],c.rollout.prepare(0.))
    s=c.selection_from_batch(0.,u,b,1);s.passage_target=.01*np.flatnonzero(s.indices>0)[0]
    assert c.publish(s,x,0.)
    witness=c.passage
    ordinary=replace(s,indices=np.zeros_like(s.indices))
    assert c.publish(ordinary,x,0.) and c.passage is witness
    shifted=c.rollout.integrate(x,u[0],.02)
    selected=c.select(shifted,.02,avoid=True)
    assert selected is not None and c.last_diagnostics['witness_valid']
    assert selected.indices[-1]==1
    assert selected.passage_target<=s.passage_target+1e-9


def test_event_gpu_cpu_and_trimmed_endpoint_agree():
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=64))
    x=hover_state([-1.,0.,4.]);x[3]=4.
    u=np.tile([MASS*9.8066,0,0,0],(2,125,1));u[1,100:,1]=5.
    p=c.rollout.prepare(0.);a=c.rollout(x,u,p);b=c.gpu.evaluate(x,u,c.gpu.prepare_device(p))
    np.testing.assert_array_equal(a.end_steps,c.cp.asnumpy(b.end_steps))
    np.testing.assert_array_equal(a.valid,c.cp.asnumpy(b.valid))
    np.testing.assert_array_equal(a.near_valid,c.cp.asnumpy(b.near_valid))
    np.testing.assert_allclose(a.states,c.cp.asnumpy(b.states),atol=5e-4,rtol=5e-5)


def test_actual_gate_advance_still_checks_old_safe_exit_tail():
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=64))
    x=hover_state([-1.,0.,4.]);x[3]=4.
    u=np.tile([MASS*9.8066,0,0,0],(125,1));b=c.rollout(x,u[None],c.rollout.prepare(0.))
    s=c.selection_from_batch(0.,u,b,1);assert c.publish(s,x,0.)
    exit_step=np.flatnonzero(s.indices>0)[0];now=.01*(exit_step+1)
    actual=s.states[exit_step+1]
    c.select(actual,now,first=1,avoid=True)
    assert c.last_diagnostics['continuation_valid']
    assert not c.last_diagnostics['witness_valid']


def test_imminent_entry_without_complete_tracking_witness_triggers_capture():
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=64))
    class Path:
        total_time=10.
        def evaluate(self,t,d):
            return np.broadcast_to(single_gate().start if d==0 else np.zeros(3),np.shape(t)+(3,)).copy()
    evaluate=c.gpu.evaluate
    def missing_exit(*args,**kwargs):
        b=evaluate(*args,**kwargs)
        if len(args[1])==366:
            b.valid[1:6]=False
            b.near_valid[1:6]=True;b.clearances[1:6,:10]=1.
            b.phases[1:6,15]=1  # entry after 100 ms, before braking time
        return b
    c.gpu.evaluate=missing_exit
    x=hover_state(single_gate().start);x[3]=4.
    c.select(x,0.,reference=SimpleNamespace(made_at=0.,trajectory=Path()))
    assert c.last_diagnostics['tracking_near_safe']
    assert c.last_diagnostics['capture_risk'] and c.last_diagnostics['avoidance']
