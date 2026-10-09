import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog
from convex_timevarying_window.simple_two_gate_soft_only_mpc.elastic_mpc import MPC, elastic_value_gradient
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state, LOWER, UPPER, reference_states


def test_elastic_gradient():
    rng=np.random.default_rng(7);B=rng.normal(size=(6,6));H=B.T@B+np.eye(6)
    u=rng.normal(size=6);q=rng.normal(size=6);T=rng.normal(size=(3,6));b=np.ones(3)*3
    f,g=elastic_value_gradient(u,H,q,T,b,30.)
    eps=1e-6
    numeric=np.array([(elastic_value_gradient(u+eps*np.eye(6)[i],H,q,T,b,30.)[0]-
                       elastic_value_gradient(u-eps*np.eye(6)[i],H,q,T,b,30.)[0])/(2*eps) for i in range(6)])
    np.testing.assert_allclose(g,numeric,rtol=1e-6,atol=1e-5)


def test_infeasible_hard_tilt_has_finite_elastic_solution():
    m=MPC();x=hover_state()
    x[6:10]=Rotation.from_euler('x',65,degrees=True).as_quat()[[3,0,1,2]]
    controls=np.tile(m.previous,(m.cfg.steps,1));states=m.rollout(x,controls)
    pva=np.tile(np.r_[x[:3],np.zeros(6)],(m.cfg.steps+1,1))
    refs,ff=reference_states(pva,m.cfg.dt,x[6:10])
    P,q,A,lo,hi=m.condense(x,states,controls,refs,ff);n=m.cfg.steps*4
    hard=linprog(np.zeros(n),A_ub=-A[n:].toarray(),b_ub=-lo[n:],bounds=list(zip(lo[:n],hi[:n])),method='highs')
    assert hard.status==2
    u,stats=m.solve_elastic(P,q,A,lo,hi,(controls/m.uscale).ravel())
    physical=u.reshape(-1,4)*m.uscale
    assert np.isfinite(physical).all()
    assert (physical>=LOWER-1e-9).all() and (physical<=UPPER+1e-9).all()
    assert stats['final_value']<stats['initial_value']
    command=m.command(x,0.,lambda t:np.tile(np.r_[x[:3],np.zeros(6)],(len(t),1)))
    assert np.isfinite(command).all()
    assert (command>=LOWER).all() and (command<=UPPER).all()
    assert np.linalg.norm(command-m.previous)<1e-12
    assert any(a>0 for a in m.logs[-1]['nonlinear_steps'])


def test_hover_and_time_limit_not_abort():
    m=MPC();x=hover_state()
    for t in (0.,.02,.04):
        u=m.command(x,t,lambda ts:np.tile(np.r_[x[:3],np.zeros(6)],(len(ts),1)))
        assert np.isfinite(u).all() and np.all(u>=LOWER) and np.all(u<=UPPER)
    assert all(r['controller']=='elastic_input_box_rti' for r in m.logs)


def test_encoder_boundary_roundoff_is_projected():
    from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import encode_ctbr
    rng=np.random.default_rng(0)
    raw_outside=0
    for c in rng.uniform(LOWER,UPPER,(1000,4)):
        for target in (LOWER,UPPER):
            raw=c+(target-c)
            raw_outside+=int(np.any(raw<LOWER)|np.any(raw>UPPER))
            encode_ctbr(np.clip(raw,LOWER,UPPER))
    assert raw_outside>0
