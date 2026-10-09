from dataclasses import replace
import numpy as np

from convex_timevarying_window.checked_soft_mpc_racing.mpc import (
    MPC as SparseMPC, Config, hover_state, reference_states, MASS, G)
from convex_timevarying_window.checked_soft_mpc_isolated.fast.mpc import MPC


def fixture():
    m=MPC();x=hover_state();x[3:6]=[1.,-.3,.1]
    u=np.tile([MASS*G,.05,-.04,.02],(m.cfg.steps,1))
    states=m.rollout(x,u)
    ref=np.tile(np.r_[[0.,0.,3.],np.zeros(6)],(m.cfg.steps+1,1))
    refs,ff=reference_states(ref,m.cfg.dt,x[6:10])
    return m,x,u,states,refs,ff


def test_accumulated_rollout_is_identical():
    m,x,u,states,_,_=fixture()
    old=SparseMPC.rollout(m,x,u)
    np.testing.assert_allclose(states,old,rtol=1e-12,atol=1e-12)


def test_elimination_exactly_satisfies_linearized_dynamics():
    m,x,u,states,_,_=fixture()
    c,D=m.affine_prediction(x,states,u)
    trial=u+np.random.default_rng(1).normal(0,.02,u.shape)
    prediction=(c+D@(trial/m.uscale).ravel()).reshape(-1,17)*m.xscale
    f,A,B=m.linear(states[:-1].T,u.T,m.cfg.dt)
    A=np.asarray(A).reshape(17,m.cfg.steps,17).transpose(1,0,2)
    B=np.asarray(B).reshape(17,m.cfg.steps,4).transpose(1,0,2)
    predicted=np.asarray(f).T+np.einsum('kij,kj->ki',A,prediction[:-1]-states[:-1])+np.einsum('kij,kj->ki',B,trial-u)
    np.testing.assert_allclose(prediction[0],x,atol=1e-12)
    np.testing.assert_allclose(prediction[1:],predicted,atol=1e-10)


def test_quadratic_cost_and_tilt_constraints_are_equivalent():
    m,x,u,states,refs,ff=fixture()
    c,D=m.affine_prediction(x,states,u)
    P,q,A,l,h=m.condense(x,states,u,refs,ff)
    P=P.toarray();P=P+P.T-np.diag(P.diagonal())
    values=[]
    for trial in (u,u+np.random.default_rng(4).normal(0,.015,u.shape)):
        z=(trial/m.uscale).ravel()
        predicted=(c+D@z).reshape(-1,17)*m.xscale
        err=(predicted-refs).ravel()
        delta=np.diff(np.vstack((m.previous,trial)),axis=0)
        full=np.dot(m.qdiag,err**2)+np.sum(m.R*(trial-ff)**2)+np.sum(m.S*delta**2)
        reduced=.5*z@P@z+q@z
        values.append(float(full-reduced))
        lhs=np.sum(-4*states[1:,7:9]*predicted[1:,7:9],axis=1)
        rhs=np.cos(np.radians(m.cfg.tilt_limit_deg))-1-2*np.sum(states[1:,7:9]**2,axis=1)
        np.testing.assert_allclose((A@z-l)[-m.cfg.steps:],lhs-rhs,atol=1e-10)
    np.testing.assert_allclose(values[0],values[1],atol=1e-8)


def test_first_command_matches_sparse_qp():
    cfg=replace(Config(),iterations=1)
    a,b=SparseMPC(cfg),MPC(cfg)
    x=hover_state();x[:3]+=[.1,-.05,.02]
    ref=lambda ts:np.tile(np.r_[[0.,0.,3.],np.zeros(6)],(len(ts),1))
    ua,ub=a.command(x,0.,ref),b.command(x,0.,ref)
    np.testing.assert_allclose(ua,ub,atol=.025,rtol=.01)


def test_condensed_mpc_recovers_displacement():
    m=MPC();x=hover_state();x[0]+=.3
    ref=lambda ts:np.tile(np.r_[[0.,0.,3.],np.zeros(6)],(len(ts),1))
    for k in range(100):
        u=m.command(x,k*.02,ref)
        x=np.asarray(m.step(x,u,.02)).ravel()
    assert np.linalg.norm(x[:3]-[0,0,3])<.01


def test_nonfinite_solver_vector_fails_explicitly(monkeypatch):
    import pytest
    from types import SimpleNamespace
    m=MPC();x=hover_state()
    ref=lambda ts:np.tile(np.r_[[0.,0.,3.],np.zeros(6)],(len(ts),1))
    m.command(x,0.,ref)
    invalid=SimpleNamespace(x=np.full(m.cfg.steps*4,None,dtype=object),
        info=SimpleNamespace(status='primal infeasible',status_val=-3,iter=1,pri_res=1.))
    monkeypatch.setattr(m.solver,'solve',lambda:invalid)
    with pytest.raises(RuntimeError,match='primal infeasible'):m.command(x,.02,ref)
