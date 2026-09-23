from dataclasses import replace
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import GateProgress
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import CTBRRollout
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS,hover_state
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate


def test_feedback_tightening_never_moves_gate_event_surfaces():
    t=single_gate();base=Config();tight=replace(base,feedback_geometry_reserve=.1)
    a=GateProgress(t,base);b=GateProgress(t,tight)
    a.update(np.array([-1.,0,4]),0.,np.array([-.78,0,4]),.02,True)
    b.update(np.array([-1.,0,4]),0.,np.array([-.78,0,4]),.02,True)
    assert (a.first,a.crossing,a.violation)==(b.first,b.crossing,b.violation)
    r=CTBRRollout(t,base);s=CTBRRollout(t,tight)
    x=hover_state(t.start);u=np.tile([MASS*9.8066,0,0,0],(1,50,1))
    p=r(x,u,r.prepare(0));q=s(x,u,s.prepare(0))
    np.testing.assert_allclose(p.states,q.states)
    assert np.all(q.clearances<=p.clearances)


def test_off_aperture_slab_skimming_needs_entry_reserve():
    t=single_gate();base=Config();tight=replace(base,feedback_geometry_reserve=.2)
    x=hover_state([-.9,3.,4.]);u=np.tile([MASS*9.8066,0,0,0],(1,50,1))
    a=CTBRRollout(t,base);b=CTBRRollout(t,tight)
    raw=a(x,u,a.prepare(0.));reserved=b(x,u,b.prepare(0.))
    assert raw.valid[0]
    assert reserved.clearances.min()>0
    assert not reserved.valid[0] and reserved.rejection_bits[0]&2
