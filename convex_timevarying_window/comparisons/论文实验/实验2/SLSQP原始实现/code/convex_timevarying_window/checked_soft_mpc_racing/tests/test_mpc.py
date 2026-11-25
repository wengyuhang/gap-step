import numpy as np
import pytest
from types import SimpleNamespace
from convex_timevarying_window.checked_soft_mpc_racing.mpc import MPC, hover_state, MASS, G, LOWER, UPPER, dynamics
from convex_timevarying_window.checked_soft_mpc_racing.px4 import Reference


def test_hover_and_motor_response():
    f, step, _ = dynamics(); x = hover_state(); u = np.array([MASS*G, 0., 0., 0.])
    np.testing.assert_allclose(np.array(f(x, u)).ravel(), 0., atol=1e-10)
    np.testing.assert_allclose(np.array(step(x, u, .04)).ravel(), x, atol=1e-10)
    u[0] += 2
    out = np.array(step(x, u, .04)).ravel()
    assert out[5] > 0 and np.all(out[13:] > x[13:])


def test_body_rate_axis_and_quaternion_normalization():
    _, step, _ = dynamics(); x = hover_state(yaw=0)
    u = np.array([MASS*G, .5, 0, 0])
    y = np.array(step(x, u, .04)).ravel()
    assert y[7] > 0 and y[10] > 0 and y[4] < 0
    np.testing.assert_allclose(np.linalg.norm(y[6:10]), 1., atol=1e-12)


def test_jacobian_matches_finite_difference():
    _, step, linear = dynamics(); x = hover_state(); x[3:6] = [1, -2, .4]
    u = np.array([MASS*G+1, .2, -.1, .15]); _, A, B = linear(x, u, .04)
    for j in (0, 4, 7, 12, 13):
        e = np.zeros(17); e[j] = 1e-5
        fd = (np.array(step(x+e,u,.04))-np.array(step(x-e,u,.04))).ravel()/2e-5
        np.testing.assert_allclose(np.array(A)[:,j], fd, atol=1e-6)
    for j in range(4):
        e = np.zeros(4); e[j] = 1e-5
        fd = (np.array(step(x,u+e,.04))-np.array(step(x,u-e,.04))).ravel()/2e-5
        np.testing.assert_allclose(np.array(B)[:,j], fd, atol=1e-6)


def test_mpc_recovers_displacement_without_gate_information():
    m = MPC(); x = hover_state(); x[0] += .3
    ref = lambda t: np.tile(np.r_[[0.,0.,3.],np.zeros(6)], (len(t),1))
    for k in range(100):
        u = m.command(x,k*.02,ref)
        assert np.all(u >= LOWER) and np.all(u <= UPPER)
        x = np.array(m.step(x,u,.02)).ravel()
    assert np.linalg.norm(x[:3]-[0,0,3]) < .01


def test_reference_switches_at_physical_splice_not_preview():
    class Plan:
        made_at=1.; end_time=2.
        def sample(self,t,d=0): return np.full(3, 10+d)
    ref=Reference([0,0,0]); ref.install(Plan())
    values=ref.sample(np.array([.95,1.,2.1]))
    np.testing.assert_equal(values[0],np.zeros(9))
    np.testing.assert_equal(values[1],np.r_[np.full(3,10),np.full(3,11),np.full(3,12)])
    np.testing.assert_equal(values[2],np.r_[np.full(3,10),np.zeros(6)])
    assert ref.advance(.95) is None
    assert ref.advance(1.) is not None


def test_preview_start_with_one_update_transport_delay():
    from convex_timevarying_window.receding_gate_racing.planner import coefficients, poly
    duration=2.5; start=np.array([0.,0.,3.]); goal=np.array([6.,-4.,3.])
    C=coefficients(start,np.zeros(3),np.zeros(3),goal,np.zeros(3),np.zeros(3),duration)
    def ref(times):
        return np.array([np.concatenate([poly(C,np.clip((t-.15)/duration,0,1),duration,d) for d in range(3)]) for t in times])
    m=MPC();x=hover_state(start);previous=np.array([MASS*G,0,0,0]);errors=[]
    for k in range(160):
        t=k*.02;u=m.command(x,t,ref)
        x=np.array(m.step(x,previous,.02)).ravel();previous=u
        errors.append(np.linalg.norm(x[:3]-ref(np.array([t+.02]))[0,:3]))
    assert max(errors)<.25
    assert errors[-1]<.03


def test_inexact_qp_requires_explicit_primal_residual(monkeypatch):
    m=MPC();x=hover_state()
    ref=lambda t:np.tile(np.r_[[0.,0.,3.],np.zeros(6)],(len(t),1))
    m.command(x,0.,ref);original=m.solver.solve
    corrupt=[False]
    def limited():
        result=original();value=result.x.copy()
        if corrupt[0]:value[0]+=1.
        return SimpleNamespace(x=value,info=SimpleNamespace(status='run time limit reached',status_val=-6,
                                iter=result.info.iter,pri_res=.01))
    monkeypatch.setattr(m.solver,'solve',limited)
    m.command(x,.02,ref)
    assert m.logs[-1]['inexact_qp_steps']==m.cfg.iterations
    corrupt[0]=True
    with pytest.raises(RuntimeError,match='run time limit reached'):m.command(x,.04,ref)
