import numpy as np
from types import SimpleNamespace
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import MASS,hover_state
from convex_timevarying_window.dynamic_gate_mppi_v3.feedback_maneuver import FeedbackManeuver


def make():
    x=hover_state([0.,0.,4.]);u=np.array([MASS*9.8066,0,0,0])
    s=SimpleNamespace(made_at=0.,expires_at=1.,states=np.tile(x,(101,1)),controls=np.tile(u,(50,1)))
    return x,u,FeedbackManeuver(s,0)


def test_nominal_is_exact_feedforward():
    x,u,m=make();np.testing.assert_allclose(m.command(x,.2),u,atol=1e-12)


def test_position_agreement_does_not_hide_tilt():
    x,u,m=make();x[6:10]=[np.cos(.05),np.sin(.05),0,0]
    assert m.command(x,.2)[1]<0
    x[6:10]=[np.cos(.4),np.sin(.4),0,0]
    assert not m.within_domain(x,.2)


def test_stale_policy_and_velocity_departure_rejected():
    x,u,m=make();assert not m.within_domain(x,1.)
    x[3]=1.01;assert not m.within_domain(x,.2)
