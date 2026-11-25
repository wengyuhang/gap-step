from dataclasses import replace
import numpy as np
import pytest
cp=pytest.importorskip('cupy')
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.gpu import GPURollout,GPUController
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import Rollout,hover_state,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate


@pytest.mark.parametrize('passage',[False,True])
def test_same_equations_gpu_float32_matches_cpu_float64(passage):
    track=single_gate() if passage else build_track()[0];cfg=replace(Config(),samples=64)
    cpu=Rollout(track,cfg);gpu=GPURollout(track,cfg)
    x=hover_state([-1.,0.,4.] if passage else track.start)
    if passage:x[3]=4.
    u=np.full((64,50,4),MASS*9.8066/4)
    u+=np.random.default_rng(1).normal(0,.01,u.shape)
    u=u.astype(np.float32).astype(np.float64)
    prepared=cpu.prepare(.4)
    a=cpu(x,u,prepared);b=gpu.evaluate(x,u,gpu.prepare_device(prepared))
    np.testing.assert_allclose(cp.asnumpy(b.states),a.states,rtol=5e-5,atol=4e-4)
    np.testing.assert_allclose(cp.asnumpy(b.clearances),a.clearances,rtol=1e-4,atol=4e-4)
    np.testing.assert_allclose(cp.asnumpy(b.race),a.race,rtol=1e-4,atol=4e-4)
    np.testing.assert_array_equal(cp.asnumpy(b.valid),a.valid)
    np.testing.assert_array_equal(cp.asnumpy(b.indices),a.indices)
    np.testing.assert_array_equal(cp.asnumpy(b.phases),a.phases)


def test_gpu_recovery_scan_matches_cpu_all_ten_intervals():
    track=single_gate();c=GPUController(track,replace(Config(),samples=64));x=hover_state(track.start)
    prepared=c.rollout.prepare(0.);device=c.gpu.prepare_device(prepared)
    u=cp.full((64,50,4),MASS*9.8066/4,dtype=cp.float32)
    batch=c.gpu.evaluate(x,u,device)
    clear=np.ones((64,100),np.float32)
    for i in range(64):clear[i,i%10::10]=-.01
    clear[-1]=1.;batch.clearances=cp.asarray(clear)
    _,recovered=c.scores(batch,u,u[0,0],device,None,0.,True)
    np.testing.assert_array_equal(cp.asnumpy(recovered),np.r_[np.zeros(63,dtype=bool),True])


def test_gpu_avoidance_does_not_read_reference_and_winner_is_fp64_checked():
    class Forbidden:
        def __getattribute__(self,name):raise AssertionError(name)
    track=single_gate();c=GPUController(track,replace(Config(),samples=128));x=hover_state(track.start)
    s=c.select(x,0.,avoid=True,reference=Forbidden())
    assert s is not None and s.states.dtype==np.float64
    check=c.rollout(x,s.controls[None],c.rollout.prepare(0.))
    assert check.valid[0]
    np.testing.assert_array_equal(check.states[0],s.states)
    assert c.last_diagnostics['backend']=='cuda'


def test_gpu_race_and_reference_cost_match_cpu_formula():
    class Trajectory:
        total_time=10.
        def evaluate(self,t,d):
            return np.broadcast_to(np.array([-2.,0.,4.]) if d==0 else np.zeros(3),np.asarray(t).shape+(3,))
    from types import SimpleNamespace
    ref=SimpleNamespace(trajectory=Trajectory(),made_at=0.,first=0,crossing_times=np.array([9.]))
    track=single_gate();c=GPUController(track,replace(Config(),samples=64));x=hover_state(track.start)
    prepared=c.gpu.prepare_device(c.rollout.prepare(0.));u=cp.full((64,50,4),MASS*9.8066/4,dtype=cp.float32)
    batch=c.gpu.evaluate(x,u,prepared);costs,_=c.scores(batch,u,u[0,0],prepared,ref,0.,False)
    states=cp.asnumpy(batch.states);p=ref.trajectory.evaluate(np.arange(101)*.01,0)
    expected=.01*np.sum(np.sum(((states[...,:3]-p)/.5)**2,axis=-1)
                                +np.sum((states[...,3:6]/2)**2,axis=-1),axis=1)
    np.testing.assert_allclose(cp.asnumpy(costs),expected,rtol=2e-5,atol=2e-5)
