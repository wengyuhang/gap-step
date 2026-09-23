from dataclasses import replace
from types import SimpleNamespace
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.reference import Reference
from convex_timevarying_window.dynamic_gate_mppi_v3.scheduler import Scheduler
from convex_timevarying_window.dynamic_gate_mppi_v3.controller import Controller
from convex_timevarying_window.dynamic_gate_mppi_v3.diagnostic import pvaj_from_state
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state, MASS


class ConstantTrajectory:
    total_time=10.
    def __init__(self,x):self.x=x
    def evaluate(self,t,d):
        return np.broadcast_to([self.x,0.,4.] if d==0 else [0.,0.,0.],np.shape(t)+(3,))


def test_tracking_is_independent_of_reference_and_actual_gate_mismatch():
    ref=Reference(ConstantTrajectory(-2.),0.,0,np.array([.5]),np.zeros((1,2)),0.,True,1)
    _,_,active=ref.sample(np.array([1.,1.1]),np.zeros((3,2),int))
    assert active.all()


def test_tracking_request_uses_now_and_first_safe_100ms_only():
    s=Scheduler();s.flag_tracking_error()
    clear=np.ones(100);clear[10:]=-.1
    r=s.observe(2.,2.+np.arange(101)*.01,clear,False,2)
    assert r.reason=='tracking_error' and r.anchor==2. and r.first_gate==2
    assert not s.avoiding
    s.flag_tracking_error()  # coalesce, do not cancel current computation
    assert s.running==r
    assert s.finish(r,2.08,2,0.,0.,0.)


def test_danger_within_100ms_prevents_start():
    s=Scheduler();s.gate_passed(1)
    clear=np.ones(100);clear[9]=-.1
    assert s.observe(0.,np.arange(101)*.01,clear,False,1) is None


def test_current_pvaj_keeps_measured_position_and_velocity():
    c=Controller(single_gate(),replace(Config(),samples=64))
    x=hover_state([-2.,0.,4.]);x[3]=3.
    head=pvaj_from_state(c.rollout,x,np.full(4,MASS*9.8066/4))
    np.testing.assert_array_equal(head.position,x[:3])
    np.testing.assert_array_equal(head.velocity,x[3:6])
    assert np.isfinite(head.matrix).all()


def test_gpu_cost_responds_to_reference_even_after_scheduled_gate_time():
    from convex_timevarying_window.dynamic_gate_mppi_v3.gpu import GPUController
    c=GPUController(single_gate(),replace(Config(),samples=64));cp=c.cp
    x=hover_state([-2.,0.,4.]);prepared=c.gpu.prepare_device(c.rollout.prepare(1.))
    u=cp.full((64,50,4),MASS*9.8066/4,dtype=cp.float32)
    batch=c.gpu.evaluate(x,u,prepared)
    def cost(target):
        ref=SimpleNamespace(trajectory=ConstantTrajectory(target),made_at=0.,first=0,crossing_times=np.array([.5]))
        return cp.asnumpy(c.scores(batch,u,u[0,0],prepared,ref,1.,False)[0])
    assert np.min(cost(10.)-cost(-2.))>100.
