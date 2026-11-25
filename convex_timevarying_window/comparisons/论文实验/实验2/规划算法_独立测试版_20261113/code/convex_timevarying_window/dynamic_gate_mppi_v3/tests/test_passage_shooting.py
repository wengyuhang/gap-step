import numpy as np
from dataclasses import replace
from types import SimpleNamespace
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate,prediction
from convex_timevarying_window.dynamic_gate_mppi_v3.controller import Controller
from convex_timevarying_window.dynamic_gate_mppi_v3.reference import Planner,PolynomialReference
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,Rollout,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState,MincoSnap


class Passage:
    total_time=2.
    def __init__(self,reverse):self.reverse=reverse
    def evaluate(self,t,d):
        t=np.asarray(t);p=np.zeros(t.shape+(3,))
        if d==0:
            p[...,0]=-1+4*t-4*t*t if self.reverse else -1+3*t
            p[...,2]=4.
        elif d==1:p[...,0]=4-8*t if self.reverse else 3.
        return p


def test_reference_touch_and_reverse_is_task_risk_despite_clear_frame():
    c=Controller(single_gate());x=hover_state([-1.,0.,4.])
    for reverse in (True,False):
        tr=Passage(reverse);ref=SimpleNamespace(trajectory=tr,made_at=0.)
        t=np.arange(101)*.01
        assert c.rollout.geometry.interval_clearances(tr.evaluate(t,0),tr.evaluate(t,1),t).min()>.05
        assert c.reference_risk(x,0.,0,ref)==reverse


def test_terminal_braking_proxy_rejects_clear_but_irrecoverable_entry():
    fast=prediction([-8.,0.,4.],6.)
    assert fast.clearances.min()>0 and not fast.valid[0]
    assert prediction([-8.,0.,4.],3.).valid[0]


def test_after_all_gates_planner_returns_goal_segment_not_empty_track_error():
    track,opt=build_track();planner=Planner(track,opt)
    head=BoundaryState(track.goal+np.array([1.,0.,0.]),np.array([1.,0.,0.]))
    ref=planner.solve(head,5.,len(track.order))
    assert ref is not None and len(ref.crossing_times)==0
    np.testing.assert_allclose(ref.trajectory.evaluate(0.,0),head.position,atol=1e-8)
    np.testing.assert_allclose(ref.trajectory.evaluate(ref.trajectory.total_time,0),track.goal,atol=1e-8)


def test_fresh_passage_shots_do_not_all_postpone_first_action():
    rollout=Rollout(single_gate());x=hover_state([-2.,0.,4.]);u=np.full(4,MASS*9.8066/4)
    shots=rollout.guided(x,rollout.prepare(0.),0,0,u,count=360)
    assert np.max(np.abs(shots[:,0]-u))>.01


def test_vectorized_reference_matches_original_minco_at_knots_and_scalars():
    tr=MincoSnap(BoundaryState(np.array([0.,0.,2.])),BoundaryState(np.array([9.,1.,3.])),
                np.array([[3.,2.,4.],[6.,-1.,2.]]),np.array([1.1,1.7,1.3]))
    fast=PolynomialReference(tr)
    t=np.r_[np.linspace(0.,tr.total_time,101),np.cumsum(tr.durations)]
    for d in range(8):
        np.testing.assert_allclose(fast.evaluate(t,d),tr.evaluate(t,d),rtol=1e-10,atol=1e-8)
        np.testing.assert_allclose(fast.evaluate(1.1,d),tr.evaluate(1.1,d),rtol=1e-10,atol=1e-8)


def test_long_horizon_gpu_matches_cpu_and_keeps_shifted_seed_in_pilot_bank():
    from convex_timevarying_window.dynamic_gate_mppi_v3.gpu import GPUController
    tr=single_gate();cfg=replace(Config(),samples=64,horizon=2.5)
    c=GPUController(tr,cfg);x=hover_state([-8.,0.,4.]);prepared=c.rollout.prepare(0.)
    u=np.full((2,125,4),MASS*9.8066/4)
    cpu=c.rollout(x,u,prepared);gpu=c.gpu.evaluate(x,u,c.gpu.prepare_device(prepared))
    np.testing.assert_allclose(c.cp.asnumpy(gpu.states),cpu.states,atol=5e-4,rtol=5e-5)
    np.testing.assert_array_equal(c.cp.asnumpy(gpu.valid),cpu.valid)
    original=c.gpu.evaluate;captured=[]
    def capture(state,controls,*args):
        if not captured:captured.append(c.cp.asnumpy(controls[0]))
        return original(state,controls,*args)
    c.gpu.evaluate=capture
    baseline=c.baseline(0.)
    c.select(x,0.,avoid=True)
    np.testing.assert_allclose(captured[0],baseline,atol=1e-6)


def test_return_goal_cost_is_terminal_not_paid_per_post_passage_step():
    tr=single_gate();r=Rollout(tr);x=hover_state([-1.,0.,4.]);x[3]=4.
    prepared=r.prepare(0.);u=np.full((1,50,4),MASS*9.8066/4)
    b=r(x,u,prepared)
    assert b.valid[0] and b.indices[0,-1]==1
    score=0.
    for k in range(100):
        target=prepared[1][k+1,b.indices[0,k]]
        score+=np.linalg.norm(b.states[0,k+1,:3]-target)-np.linalg.norm(b.states[0,k,:3]-target)
        if b.indices[0,k+1]>b.indices[0,k]:score-=5*(1-(k+1)/100)
    score+=np.sum(b.states[0,-1,3:6]**2)/(2*r.config.acceleration)
    np.testing.assert_allclose(b.race[0],score,atol=1e-8)
