from types import SimpleNamespace
import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from convex_timevarying_window.event_free_terminal_mpc.planner import Planner,Config
from convex_timevarying_window.crossing_resolved_mpc.planner import Planner as Old
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan,basis
from convex_timevarying_window.event_free_terminal_mpc.reference import Reference


@pytest.fixture(scope='module')
def problem():
    install();track,_=build_case(Case(1.,2.,0.))
    cfg=Config(workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
    p=Planner(track,cfg,build_indices=[0,5]);old=Old(track,cfg,build_indices=[0,5])
    state=np.r_[track.start,np.zeros(9)]
    par=np.r_[.15,state,np.tile([.15,.35,.65,.85],6)]
    return p,old,state,par


def endpoint(p,x,par,first=0):
    _,C,_=p.solvers[first][1](x,par);C=np.asarray(C).reshape(6,6,3)
    T=np.exp(x[2])/2
    return np.concatenate([basis(1.,d)@C[-1]/T**d for d in range(4)])


def test_terminal_12_independent_coordinates(problem):
    p,_,state,par=problem;x=p.initial(state,.15,0)
    assert len(x)==37
    variables=ca.SX.sym('test_x',len(x));_,C,_=p.solvers[0][1](variables,par)
    T=ca.exp(variables[2])/2
    y=ca.vertcat(*[(C[-6:,:].T@ca.DM(basis(1.,d)))/T**d for d in range(4)])
    jac=np.asarray(ca.Function('tail_jac',[variables],[ca.jacobian(y,variables)])(x))
    cols=list(range(28,37))+list(range(21,24))
    assert np.linalg.matrix_rank(jac[:,cols])==12
    np.testing.assert_allclose(endpoint(p,x,par)[3:6],p.cfg.speed*x[31:34],atol=1e-9)
    assert np.linalg.norm(endpoint(p,x,par)[3:6])>1.


def test_old_stopped_trajectory_is_only_a_special_case(problem):
    p,old,state,par=problem;xold=old.initial(state,.15,0)
    x=np.r_[xold,old.terminal_position(0)/10,np.zeros(6)]
    for new_value,old_value in zip(p.solvers[0][1](x,par),old.solvers[0][1](xold,par)):
        np.testing.assert_allclose(new_value,old_value,rtol=1e-8,atol=1e-6)
    pp=np.r_[par,[1000.,1000.]]
    for i in (0,2):
        assert float(p.solvers[0][0].solver.fg(x,pp)[i])==pytest.approx(float(old.solvers[0][0].solver.fg(xold,pp)[i]),rel=1e-8)


def test_terminal_pvaj_warm_start(problem):
    p,_,state,par=problem;x=p.initial(state,.15,0)
    _,C,_=p.solvers[0][1](x,par)
    plan=Plan(.15,0,np.repeat(np.exp(x[:3])/2,2),np.asarray(C).reshape(6,6,3),x,0.,0.,'test',2)
    try:
        p.commit(plan);nextx=p.initial(state,.25,0)
        np.testing.assert_allclose(nextx[28:37],x[28:37],atol=1e-8)
        np.testing.assert_allclose(nextx[21:24],x[21:24],atol=1e-8)
    finally:p.commit(None)


def test_final_mission_boundary_unchanged(problem):
    p,old,state,par=problem
    assert not p.free_terminal(5)
    x=p.initial(state,.15,5)
    assert len(x)==28
    y=endpoint(p,x,par,5)
    np.testing.assert_allclose(y[:3],p.track.goal,atol=1e-8)
    np.testing.assert_allclose(y[3:9],0.,atol=1e-8)


def test_new_coordinate_gradients(problem):
    p,_,state,par=problem;x=p.initial(state,.15,0);pp=np.r_[par,[1000.,1000.]]
    fn=p.solvers[0][0].solver.fg;_,g,_=fn(x,pp);g=np.array(g).ravel()
    for i in range(28,37):
        d=np.zeros(len(x));d[i]=1e-6
        numeric=(float(fn(x+d,pp)[0])-float(fn(x-d,pp)[0]))/2e-6
        assert abs(numeric-g[i])<.005*max(1.,abs(numeric))


def test_reference_never_fabricates_local_hover():
    r=Reference(np.zeros(3),7)
    r.active=SimpleNamespace(first_gate=0,gate_count=2,made_at=0.,end_time=2.,sample=lambda t,d:np.ones(3)*(d+1))
    np.testing.assert_array_equal(r.state_at(2.,3)[0],np.repeat([1.,2.,3.],3))
    with pytest.raises(RuntimeError,match='reference_exhausted'):r.state_at(2.01)
    r.active.first_gate=5
    np.testing.assert_array_equal(r.state_at(3.,3)[0],np.r_[np.ones(3),np.zeros(6)])
