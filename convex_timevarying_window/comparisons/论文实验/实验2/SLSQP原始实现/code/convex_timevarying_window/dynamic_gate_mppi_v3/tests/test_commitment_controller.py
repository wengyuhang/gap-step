from dataclasses import replace
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.commitment_gpu import CommitmentController
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS,hover_state


def test_ctbr_gpu_matches_cpu():
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=64))
    x=hover_state(single_gate().start);u=np.tile([MASS*9.8066,0.,0.,0.],(2,125,1));u[1,:,1]=.03
    prepared=c.rollout.prepare(0.);a=c.rollout(x,u,prepared)
    b=c.gpu.evaluate(x,u,c.gpu.prepare_device(prepared))
    np.testing.assert_allclose(a.states,c.cp.asnumpy(b.states),rtol=5e-5,atol=5e-4)
    np.testing.assert_array_equal(a.valid,c.cp.asnumpy(b.valid))
    np.testing.assert_array_equal(a.rejection_bits,c.cp.asnumpy(b.rejection_bits))


def test_inside_post_center_never_reads_next_reference_or_retreats():
    class Forbidden:
        def __getattribute__(self,name):raise AssertionError(name)
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=128))
    x=hover_state([.1,0,4]);x[3]=3.
    s=c.select(x,0,0,2,False,Forbidden())
    assert s is not None and s.indices[-1]==1
    assert not np.any((s.phases==0)&(s.indices==0))


def test_collision_free_reference_beyond_unpassed_gate_is_task_risk():
    from types import SimpleNamespace
    class Path:
        total_time=5.
        def evaluate(self,t,d):
            return np.broadcast_to([3.,3.,4.] if d==0 else [0.,0.,0.],(len(t),3)).copy()
    c=CommitmentController(single_gate(),replace(Config(),horizon=2.5,samples=64))
    ref=SimpleNamespace(made_at=0.,trajectory=Path())
    assert c.reference_risk(hover_state(single_gate().start),0.,0,ref)
