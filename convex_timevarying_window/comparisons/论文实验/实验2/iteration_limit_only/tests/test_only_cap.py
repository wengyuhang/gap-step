import numpy as np
import pytest
from scipy.optimize import OptimizeResult
from ..planner import Planner
from ... import planner as original
from ...basic_group.whole_course_difficulty import scenario
from convex_timevarying_window.global_gate_minco_probe import planner as core


@pytest.mark.parametrize('free',[True,False])
def test_original_model_curve_seeds_config_bounds_and_jacobian_identical(free):
    track,_=scenario.build_case(scenario.Case(level='a25_f3'))
    previous=original.pose;original.pose=scenario.pose
    try:baseline=original.Planner(track,free)
    finally:original.pose=previous
    changed=Planner(track,free,pose_function=scenario.pose)
    assert baseline.cfg==changed.cfg and baseline.nvar==changed.nvar
    np.testing.assert_array_equal(baseline.lower,changed.lower);np.testing.assert_array_equal(baseline.upper,changed.upper)
    assert len(changed.seeds())==4
    for x,y in zip(baseline.seeds(),changed.seeds()):
        np.testing.assert_array_equal(x,y)
        for a,b in zip(baseline.evaluate(x,baseline.witnesses()),changed.evaluate(y,changed.witnesses())):
            np.testing.assert_array_equal(a,b)


def test_wrapper_changes_only_iteration_option_and_restores_backend(monkeypatch):
    seen={}
    def backend(fun,x,*a,**kw):
        seen.update(kw);return OptimizeResult(x=x,success=False,message='test',nit=1)
    def original_solve(self,*args,**kwargs):
        origin=np.array([1.,2.]);scale=np.array([2.,3.])
        def fun(x):return float(np.sum(origin+scale*x)),scale
        core.minimize(fun,np.zeros(2),jac=True,method='SLSQP',constraints='unchanged',
            options=dict(maxiter=400,ftol=1e-7,disp=False))
        return None,dict(found=False)
    monkeypatch.setattr(core,'minimize',backend);monkeypatch.setattr(original.Planner,'solve_all',original_solve)
    p=Planner.__new__(Planner);p.max_iterations=2000
    _,result=p.solve_all()
    assert seen==dict(jac=True,method='SLSQP',constraints='unchanged',options=dict(maxiter=2000,ftol=1e-7,disp=False))
    assert core.minimize is backend and len(result['backend_calls'])==1


def test_backend_restored_after_exception(monkeypatch):
    backend=core.minimize
    def fail(*a,**kw):raise RuntimeError('test exception')
    monkeypatch.setattr(original.Planner,'solve_all',fail)
    p=Planner.__new__(Planner);p.max_iterations=2000
    with pytest.raises(RuntimeError,match='test exception'):p.solve_all()
    assert core.minimize is backend
