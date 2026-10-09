import pickle
import casadi as ca
import numpy as np
from convex_timevarying_window.togt_minco_mpc.trajectory import Plan,minco_coefficients
from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap,BoundaryState


def problem(n):
    T=ca.MX.sym('T',n);P=ca.MX.sym('P',3,n-1);H=ca.MX.sym('H',3,4);E=ca.MX.sym('E',3,4)
    C=minco_coefficients(H,E,P,T)
    return ca.Function('coefficient_test',[H,E,P,T],[C])


def test_matches_togt_minco_and_pvaj_c6():
    rng=np.random.default_rng(7);n=6;T=rng.uniform(.8,2.,n)
    head=rng.normal(size=(4,3));tail=rng.normal(size=(4,3));points=rng.normal(size=(n-1,3))
    reference=MincoSnap(BoundaryState.from_array(head),BoundaryState.from_array(tail),points,T)
    C=np.array(problem(n)(head.T,tail.T,points.T,T)).reshape(n,8,3)
    expected=reference.coefficients*T[:,None,None]**np.arange(8)[None,:,None]
    np.testing.assert_allclose(C,expected,atol=2e-7,rtol=2e-7)
    plan=Plan(3.,0,T,C,np.zeros(19),0.,0.,'test',2,.4)
    for d in range(7):
        tt=np.linspace(0,T.sum(),41)
        np.testing.assert_allclose(plan.sample_many(tt+3.,d),np.array([reference.evaluate(t,d) for t in tt]),atol=1e-6,rtol=1e-6)
    np.testing.assert_allclose([plan.sample(3.,d) for d in range(4)],head,atol=1e-7)
    np.testing.assert_allclose([plan.sample(plan.end_time,d) for d in range(4)],tail,atol=1e-7)


def test_ipc_keeps_eight_coefficients_and_gate_indices():
    plan=Plan(.3,2,np.ones(6),np.arange(144.).reshape(6,8,3),np.zeros(19),1.,0.,'test',2,.5)
    other=pickle.loads(pickle.dumps(plan))
    assert other.coefficients.shape==(6,8,3)
    np.testing.assert_equal(other.crossing_times,[2.3,4.3])
    for t in (.2,1.4,4.8,7.):
        for d in range(4):np.testing.assert_equal(plan.sample(t,d),other.sample(t,d))


def test_time_gradient_through_minco_system():
    n=4;T=ca.MX.sym('T',n);P=ca.DM([[1.,2.,4.],[1.,-1.,.5],[3.,4.,3.]])
    H=ca.DM.zeros(3,4);H[:,0]=[0,0,3];E=ca.DM.zeros(3,4);E[:,0]=[6,0,3]
    C=minco_coefficients(H,E,P,T);value=ca.sumsqr(C)/10000
    fun=ca.Function('gradient_test',[T],[value,ca.gradient(value,T)])
    x=np.array([1.,1.4,.9,1.3]);_,grad=fun(x)
    for i in range(n):
        dx=np.eye(n)[i]*1e-5;fd=float(fun(x+dx)[0]-fun(x-dx)[0])/2e-5
        np.testing.assert_allclose(float(grad[i]),fd,atol=2e-6,rtol=2e-5)
