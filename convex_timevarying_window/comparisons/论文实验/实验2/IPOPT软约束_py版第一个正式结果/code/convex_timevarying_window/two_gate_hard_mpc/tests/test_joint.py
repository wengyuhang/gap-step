import casadi as ca
import numpy as np
from convex_timevarying_window.two_gate_hard_mpc.joint_planner import coefficients,Planner,Config
from convex_timevarying_window.two_gate_hard_mpc.scenario import Case,build_case
from convex_timevarying_window.togt_minco_mpc.trajectory import basis


def test_route_velocity_initialization():
    from convex_timevarying_window.two_gate_hard_mpc.joint_planner import route_velocity_seeds
    v=route_velocity_seeds([0,0,0],[[10,0,0],[20,10,0]],[30,10,0],9.6).reshape(2,3)
    np.testing.assert_allclose(np.linalg.norm(v,axis=1),9.6)
    assert np.all(v[:,0]>0) and np.all(v[:,1]>0)
    straight=route_velocity_seeds([0,0,0],[[10,0,0],[20,0,0]],[30,0,0],9.6).reshape(2,3)
    np.testing.assert_allclose(straight,[[9.6,0,0],[9.6,0,0]])


def test_joint_three_piece_global_constraints():
    head=ca.DM.zeros(3,4);tail=ca.DM.zeros(3,4);tail[:,0]=ca.DM([20.,8.,3.])
    P=ca.DM([[5.,13.],[2.,-2.],[3.,3.]])
    V=ca.DM([[5.,5.],[1.,1.],[0.,0.]]);T=ca.DM([2.,2.5,2.])
    C=np.array(ca.Function('coeff',[],[coefficients(head,tail,P,V,T)])()['o0'])
    def sample(k,u,d):return C[8*k:8*k+8].T@np.array(basis(u,d)).ravel()/float(T[k])**d
    for k in range(2):
        np.testing.assert_allclose(sample(k,1.,0),np.array(P[:,k]).ravel(),atol=1e-7)
        np.testing.assert_allclose(sample(k,1.,1),np.array(V[:,k]).ravel(),atol=1e-7)
        for d in range(6):np.testing.assert_allclose(sample(k,1.,d),sample(k+1,0.,d),atol=1e-6)
    for d in range(4):
        np.testing.assert_allclose(sample(0,0.,d),np.array(head[:,d]).ravel(),atol=1e-7)
        np.testing.assert_allclose(sample(2,1.,d),np.array(tail[:,d]).ravel(),atol=1e-6)


def test_joint_objective_and_analytic_jacobian():
    track,_=build_case(Case(1.,2.,0.));p=Planner(track,Config(compile_objective=False))
    x=np.r_[np.log([3.,3.,2.]),[.1,-.1,.2,.1],[8.,2.,0.,8.,0.,0.]]
    par=np.r_[0.,track.start,np.zeros(9),np.zeros(4*p.cfg.witness_slots)]
    out=p.function(x,par)
    np.testing.assert_allclose(float(out[0])*1000,8.)
    for i in (0,1,3,6,8,12):
        delta=np.eye(13)[i]*1e-6;a=p.function(x+delta,par);b=p.function(x-delta,par)
        np.testing.assert_allclose(np.array(out[3])[:,i],np.array((a[2]-b[2])/(2e-6)).ravel(),rtol=2e-3,atol=3e-4)
