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


def test_actual_feedback_uses_same_command_slew_as_prediction():
    x,u,m=make();x[6:10]=[np.cos(.05),np.sin(.05),0,0]
    assert abs(m.command(x,.2)[1])>.08
    a=m.command(x,.2,u,.01)
    assert np.max(np.abs(a[1:]-u[1:]))<=.08000001
    b=m.command(x,.22,a,.02)
    assert np.max(np.abs(b[1:]-a[1:]))<=.16000001
    np.testing.assert_array_equal(m.command(x,.22,b,0.),b)


def test_vectorized_targets_match_scalar_including_quaternion_sign():
    x,u,m=make();m.selection.states[30:50,6:10]*=-1
    times=np.array([.001,.295,.305,.499,.71])
    states,controls=m.targets(times)
    for i,t in enumerate(times):
        a,b=m.target(t)
        np.testing.assert_allclose(states[i],a,atol=1e-12)
        np.testing.assert_array_equal(controls[i],b)


def test_native_feedback_replay_matches_python_with_identical_limits():
    from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import CTBRRollout
    from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate
    r=CTBRRollout(single_gate());x,u,m=make()
    x[0]+=.05;x[3]=.1;x[6:10]=[np.cos(.015),np.sin(.015),0.,0.]
    commands=r.replay_feedback(x,m,.003,30,u)
    assert commands is not None
    previous=u;actual=x.copy();expected=[]
    for k in range(30):
        v=m.command(actual,.003+.02*k,previous,.02);expected.append(v)
        actual=r.integrate(actual,v);previous=v
    np.testing.assert_allclose(commands,expected,rtol=1e-10,atol=1e-10)
    x[3]=1.01
    assert r.replay_feedback(x,m,.003,30,u) is None
