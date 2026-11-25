import casadi as ca
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators,KF
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_planner import Config,Planner
from convex_timevarying_window.comparisons.single_gate_translation.scenario import Case,build_case
from convex_timevarying_window.checked_soft_mpc_racing.mpc import MASS,G


def test_hover_rotors_and_rates():
    force,rates,rotors=flat_actuators(*[np.zeros(3)]*4)
    np.testing.assert_allclose(force,MASS*G,atol=1e-8)
    np.testing.assert_allclose(rates,0.,atol=1e-10)
    np.testing.assert_allclose(rotors,MASS*G/4,atol=1e-8)
    assert np.min(rotors)>KF*150**2 and np.max(rotors)<KF*1000**2


def test_only_time_objective_and_actuator_jacobian():
    track,_=build_case(Case(4.,4.,0.))
    p=Planner(track,Config(compile_objective=False,forward_tail=True))
    assert not p.cfg.forward_tail
    x=np.r_[np.log([2.9,3.6]),.2,-.6,12.,-3.,.4]
    par=np.r_[.162,track.start,np.zeros(9),np.tile(np.linspace(0.,.2,p.cfg.witness_slots),2)]
    out=p.function(x,par)
    np.testing.assert_allclose(float(out[0])*1000,np.exp(x[:2]).sum())
    np.testing.assert_allclose(np.array(out[1]).ravel()[2:],0.,atol=1e-12)
    for i in range(7):
        delta=np.eye(7)[i]*1e-6
        a,b=p.function(x+delta,par),p.function(x-delta,par)
        np.testing.assert_allclose(np.array(out[3])[:,i],np.array(a[2]-b[2]).ravel()/2e-6,
                                   rtol=1e-3,atol=5e-5)
