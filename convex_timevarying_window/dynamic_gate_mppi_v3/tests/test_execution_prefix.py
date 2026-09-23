import threading
import time
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr_probe import Session
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import CTBRRollout
from convex_timevarying_window.dynamic_gate_mppi_v3.execution_prefix import prefix_violation
from convex_timevarying_window.dynamic_gate_mppi_v3.execution_prefix import feedback_prefix,prefix_task_violation
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate


def test_snapshot_age_keeps_growing_without_telemetry():
    s=object.__new__(Session);s.lock=threading.Lock()
    s.snapshot=(1.,np.zeros(17),.02,time.monotonic()-.15)
    assert s.read()[2]>=.17


def test_prefix_has_independent_physical_checks():
    r=CTBRRollout(single_gate());x=np.tile(hover_state(single_gate().start),(21,1));ts=np.arange(21)*.005
    assert prefix_violation(r,x,ts) is None
    x[:,10]=4.
    assert prefix_violation(r,x,ts)=='body_rate'
    x[:,10]=0.;x[-1,3]=1.
    assert prefix_violation(r,x,ts)=='acceleration'


def test_feedback_revalidation_does_not_depend_on_gpu_result():
    from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS
    class Hover:
        def command(self,x,t):return np.array([MASS*9.8066,0,0,0])
    r=CTBRRollout(single_gate());x=hover_state(single_gate().start)
    states,times=feedback_prefix(r,Hover(),x,2.)
    assert times[-1]==2.1 and len(states)==11
    assert prefix_violation(r,states,times) is None
