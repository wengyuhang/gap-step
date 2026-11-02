import casadi as ca
import numpy as np

from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import minco_coefficients
from convex_timevarying_window.single_gate_stop_racing.planner import Config
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.global_optimality.model import coefficients,bound_knot_velocity


def test_explicit_pv_minco_matches_original_qr():
    rng=np.random.default_rng(20261030)
    t=ca.MX.sym('t',2);p=ca.MX.sym('p',3);v=ca.MX.sym('v',3)
    head=ca.horzcat(ca.DM([0,-6,3.2]),ca.DM.zeros(3,3))
    tail=ca.horzcat(ca.DM([0,6,3.2]),ca.DM.zeros(3,3))
    a=coefficients(t,p,v,head[:,0],tail[:,0])
    b=minco_coefficients(head,tail,p,v,t)
    f=ca.Function('equivalent',[t,p,v],[a,b])
    for _ in range(12):
        aa,bb=f(rng.uniform(.7,2.4,2),rng.uniform(-2,2,3),rng.uniform(-4,10,3))
        np.testing.assert_allclose(aa,bb,atol=2e-8,rtol=2e-9)


def test_workspace_velocity_bound_contains_sample_polynomials():
    low,high=bound_knot_velocity(Config())
    assert np.all(low<0) and np.all(high>0)
    assert np.all(np.isfinite(low)) and np.all(np.isfinite(high))
