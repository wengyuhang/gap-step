from types import SimpleNamespace
import numpy as np
from convex_timevarying_window.simple_two_gate_soft_only_mpc import solver


def config():
    return SimpleNamespace(max_iterations=100,optimizer_ftol=1e-6,optimizer_gtol=1e-6)


def test_zero_step_false_success_recovers(monkeypatch):
    original=solver.minimize;calls=[]
    def stalled_once(fun,x,**kwargs):
        calls.append(1)
        if len(calls)>1:return original(fun,x,**kwargs)
        f,g=fun(x);kwargs['callback'](x.copy())
        return SimpleNamespace(x=x.copy(),fun=f,jac=g,nit=1,success=True,message='relative reduction')
    monkeypatch.setattr(solver,'minimize',stalled_once)
    fun=lambda x:(float((x[0]-.2)**2),2*(x-.2))
    x,stats=solver.guarded_minimize(fun,np.array([.9]),np.array([-1.]),np.array([1.]),config())
    assert len(stats['stagnation_repairs'])==1 and stats['converged']
    assert abs(x[0]-.2)<1e-5
    assert stats['stagnation_repairs'][0]['after']<stats['stagnation_repairs'][0]['before']


def test_stationary_bound_needs_no_repair():
    x,s=solver.guarded_minimize(lambda x:(float(x[0]),np.ones(1)),np.zeros(1),np.zeros(1),np.ones(1),config())
    assert s['converged'] and not s['stagnation_repairs']


def test_no_descent_cannot_be_marked_converged():
    x,s=solver.guarded_minimize(lambda x:(1.,np.ones(1)),np.zeros(1),-np.ones(1),np.ones(1),config())
    assert not s['converged'] and 'STAGNATION' in s['return_status']
