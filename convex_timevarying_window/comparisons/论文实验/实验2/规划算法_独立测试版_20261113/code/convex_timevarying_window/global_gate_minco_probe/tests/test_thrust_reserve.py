import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import constraints
from convex_timevarying_window.global_gate_minco_probe.planner import Planner
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case


def test_reserve_only_changes_upper_thrust_constraints_not_jacobian():
    x=ca.MX.sym('x',12)
    def state(piece,u,d=0):return x[3*(d-1):3*d] if d else ca.MX.zeros(3)
    old=ca.vertcat(*constraints(state,ca.DM([1.]),nodes=1))
    new=ca.vertcat(*constraints(state,ca.DM([1.]),nodes=1,thrust_fraction=.9))
    f=ca.Function('reserve_test',[x],[new-old,ca.jacobian(new-old,x)])
    delta,jac=f(np.linspace(.01,.12,12))
    np.testing.assert_allclose(np.asarray(delta).ravel(),np.tile(np.r_[np.zeros(7),np.full(4,-.1)],2),atol=1e-14)
    np.testing.assert_allclose(jac,0.,atol=1e-14)
    for value in [0.,1.1,float('nan')]:
        with pytest.raises(ValueError):constraints(state,ca.DM([1.]),thrust_fraction=value)


def test_global_planner_records_reserve_without_changing_seed_curve():
    track,_=build_case(Case(4.,4.,0.))
    nominal=Planner(track,full_state=True)
    reserved=Planner(track,full_state=True,planning_thrust_fraction=.9)
    assert nominal.cfg.physical_thrust_fraction==1.
    assert reserved.cfg.physical_thrust_fraction==.9
    for a,b in zip(nominal.seeds(),reserved.seeds()):np.testing.assert_array_equal(a,b)
