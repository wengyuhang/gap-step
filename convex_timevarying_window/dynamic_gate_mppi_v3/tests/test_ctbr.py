import numpy as np
import pytest
from dataclasses import replace
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import encode_ctbr, CTBRRollout
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS, hover_state
from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate


def test_ctbr_units_and_axes():
    r,t=encode_ctbr(np.array([MASS*9.8066,.1,.2,.3]))
    np.testing.assert_allclose(r,[.1,-.2,-.3])
    np.testing.assert_allclose(4*8.54858e-6*(150+850*t)**2,MASS*9.8066)
    with pytest.raises(ValueError):encode_ctbr(np.array([np.nan,0,0,0]))


def test_ctbr_hover_and_feedback_attitude():
    r=CTBRRollout(single_gate(),replace(Config(),horizon=2.5))
    x=hover_state(single_gate().start);u=np.array([MASS*9.8066,0,0,0])
    np.testing.assert_allclose(r.integrate(x,u,.1),x,atol=1e-9)
    pd=x[:3].copy();a=r.feedback(x,pd,np.zeros(3),np.zeros(3))
    x[6:10]=[np.cos(.05),np.sin(.05),0,0]
    b=r.feedback(x,pd,np.zeros(3),np.zeros(3))
    assert a[1]==pytest.approx(0.) and b[1]<-.1


def test_ctbr_command_limits_are_not_rotor_limits():
    r=CTBRRollout(single_gate());x=hover_state(single_gate().start)
    u=np.tile([MASS*9.8066,0,0,0],(1,50,1))
    assert r(x,u,r.prepare(0.)).valid[0]
    u[:,:,1]=5.
    assert not r(x,u,r.prepare(0.)).valid[0]


def test_handover_does_not_force_arbitrary_zero_heading():
    r=CTBRRollout(single_gate());x=hover_state(single_gate().start)
    x[6:10]=[np.sqrt(.5),0,0,np.sqrt(.5)]
    u=r.feedback(x,x[:3],np.zeros(3),np.zeros(3))
    np.testing.assert_allclose(u[1:],0.,atol=1e-12)


def test_shooting_proposals_respect_rate_command_slew():
    r=CTBRRollout(single_gate());x=hover_state(single_gate().start)
    previous=np.array([MASS*9.8066,0.,0.,0.])
    u=r.guided(x,r.prepare(0.),0,0,previous,count=24)
    du=np.diff(np.concatenate([np.broadcast_to(previous,(len(u),1,4)),u],axis=1),axis=1)
    assert np.max(np.abs(du[:,:,1:]))<=.160001
    bad=np.tile(previous,(1,50,1));bad[0,2,1]=.3
    b=r(x,bad,r.prepare(0.))
    assert b.rejection_bits[0]&64
