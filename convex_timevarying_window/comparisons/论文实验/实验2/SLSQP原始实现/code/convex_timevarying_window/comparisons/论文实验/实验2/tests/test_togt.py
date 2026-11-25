import numpy as np
import pytest
from convex_timevarying_window.comparisons.论文实验.实验2.scenario import Case,build_case
from convex_timevarying_window.comparisons.论文实验.实验2.togt_dynamic import Objective
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap,BoundaryState

@pytest.mark.parametrize('center',[False,True])
def test_dynamic_TOGT_time_and_spatial_full_gradient(center):
    track,_=build_case(Case('slalom','medium'));o=Objective(track,center);x=o.seeds()[0]
    cost,grad=o.value_and_gradient(x)
    for i in ([0,3,7] if center else [0,3,7,8,12,16]):
        e=np.eye(len(x))[i]*2e-6;g=(o.value_and_gradient(x+e)[0]-o.value_and_gradient(x-e)[0])/(4e-6)
        np.testing.assert_allclose(grad[i],g,rtol=3e-3,atol=.03)
    T,P,_=o.geometry(x);T=np.array(T).ravel();P=np.array(P).T
    centres=np.array([w.state_at(t)[0] for w,t in zip(track.windows,np.cumsum(T)[:7])]);np.testing.assert_allclose(P,centres,atol=1e-10)
    assert o.dimension==(8 if center else 30)


def test_TOGT_coefficient_export_is_exact_and_C6():
    tr,_=build_case(Case('height','static'));o=Objective(tr);x=o.seeds()[0];T,P,_=o.geometry(x);T=np.array(T).ravel();P=np.array(P).T
    curve=MincoSnap(BoundaryState(tr.start),BoundaryState(tr.goal),P,T)
    C=curve.coefficients*T[:,None,None]**np.arange(8)[None,:,None]
    for k in range(7):
        for d in range(7):
            a=C[k].T@np.array(basis(1,d)).ravel()/T[k]**d;b=C[k+1].T@np.array(basis(0,d)).ravel()/T[k+1]**d
            np.testing.assert_allclose(a,b,atol=2e-7)
