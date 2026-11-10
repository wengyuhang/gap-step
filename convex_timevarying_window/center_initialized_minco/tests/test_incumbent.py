from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from convex_timevarying_window.center_initialized_minco import planner as module
from convex_timevarying_window.dynamic_passage_minco.planner import Stats


def make_plan(time, point):
    x = np.array([np.log(time*.6), np.log(time*.4), point, point, 4., 1., 0.])
    return SimpleNamespace(x=x, durations=np.array([time*.6,time*.4]),
        coefficients=np.full((2,8,3), point), made_at=2.3, solve_ms=0.,
        optimizer_stats=dict(iter_count=3, final_objective=time, passage={'passed': True}))


class NumericalParent:
    def __init__(self, outcome='improve', budget=None, clock=None):
        self.cfg=module.Config(max_solve_ms=budget)
        self.nvar=7;self.lower=np.array([-2.,-2.,-1.,-1.,-np.inf,-np.inf,-np.inf])
        self.upper=np.array([3.,3.,1.,1.,np.inf,np.inf,np.inf])
        self.admitted=object();self.solvers=[(Stats(),)];self.calls=[];self.outcome=outcome;self.clock=clock
    def order_seeds(self,seeds,state,now):return list(reversed(seeds))
    def solve(self,state,now,first):
        self.calls.append(dict(free=self.cfg.free_points,budget=self.cfg.max_solve_ms,now=now,
            head=state.copy(),lower=self.lower.copy(),upper=self.upper.copy(),
            seeds=self.order_seeds([np.ones(7),np.full(7,2.)],state,now)))
        if self.clock is not None:self.clock[0]+=.01 if self.cfg.free_points else .02
        if not self.cfg.free_points:
            if self.outcome=='center_failure':raise RuntimeError('no_converged_safe_passage: center')
            plan=make_plan(6.,0.)
        else:
            if self.outcome=='unknown_error':raise RuntimeError('unexpected implementation error')
            if self.outcome=='failed':
                self.solvers[0][0].value={'iter_count':7,'converged':False}
                raise RuntimeError('no_converged_safe_passage: refinement')
            plan=make_plan({'improve':5.,'slower':7.,'equal':6.}[self.outcome],.3)
        self.solvers[0][0].value=plan.optimizer_stats
        return plan


class Planner(module.CenterInitialized,NumericalParent):pass


def test_same_request_and_complete_center_result_initialize_refinement():
    p=Planner();original_cfg=p.cfg;low,high,admitted=p.lower.copy(),p.upper.copy(),p.admitted
    state=np.arange(12,dtype=float);plan=p.solve(state,2.3,0)
    assert len(p.calls)==2
    a,b=p.calls
    assert not a['free'] and b['free']
    np.testing.assert_array_equal(a['lower'][2:4],[0.,0.]);np.testing.assert_array_equal(a['upper'][2:4],[0.,0.])
    np.testing.assert_array_equal(b['lower'],low);np.testing.assert_array_equal(b['upper'],high)
    assert a['now']==b['now']==2.3
    np.testing.assert_array_equal(a['head'],state);np.testing.assert_array_equal(b['head'],state)
    assert len(b['seeds'])==1
    np.testing.assert_array_equal(b['seeds'][0],p.last_center_plan.x)
    assert plan.optimizer_stats['selected_stage']=='free'
    assert plan.optimizer_stats['nominal_time_saved_s']==pytest.approx(1.)
    assert p.cfg is original_cfg and p.admitted is admitted
    np.testing.assert_array_equal(p.lower,low);np.testing.assert_array_equal(p.upper,high)


@pytest.mark.parametrize('outcome',['failed','slower','equal'])
def test_nonimprovement_returns_exact_original_center_trajectory(outcome):
    p=Planner(outcome);plan=p.solve(np.zeros(12),2.3,0)
    np.testing.assert_array_equal(plan.coefficients,p.last_center_plan.coefficients)
    np.testing.assert_array_equal(plan.x,p.last_center_plan.x)
    assert plan.optimizer_stats['selected_stage']=='center'
    assert plan.optimizer_stats['nominal_time_saved_s']==0.
    assert plan.optimizer_stats['passage']['passed']
    assert p.last_center_plan.optimizer_stats['final_objective']==6.


def test_center_failure_does_not_execute_a_free_cold_start():
    p=Planner('center_failure');cfg=p.cfg
    with pytest.raises(RuntimeError,match='center'):p.solve(np.zeros(12),2.3,0)
    assert len(p.calls)==1 and p.cfg is cfg
    np.testing.assert_array_equal(p.lower[2:4],[-1.,-1.])


def test_unknown_programming_error_is_not_hidden_by_center_fallback():
    p=Planner('unknown_error');cfg=p.cfg
    with pytest.raises(RuntimeError,match='implementation error'):p.solve(np.zeros(12),2.3,0)
    assert p.cfg is cfg and p._center_refinement_seed is None


def test_finite_budget_and_total_timing_are_shared(monkeypatch):
    clock=[0.];monkeypatch.setattr(module,'time',SimpleNamespace(perf_counter=lambda:clock[0]))
    p=Planner(budget=100.,clock=clock);plan=p.solve(np.zeros(12),2.3,0)
    assert p.calls[1]['budget']==pytest.approx(80.)
    assert plan.solve_ms==pytest.approx(30.)
    assert plan.optimizer_stats['center_stage_ms']==pytest.approx(20.)
    assert plan.optimizer_stats['free_stage_ms']==pytest.approx(10.)
    p=Planner(budget=10.,clock=clock);plan=p.solve(np.zeros(12),2.3,0)
    assert len(p.calls)==1 and plan.optimizer_stats['selected_stage']=='center'
    assert plan.optimizer_stats['free_stage_error']=='shared_wall_budget_exhausted_after_center'


def test_center_only_comparison_retains_original_single_stage():
    p=Planner();p.cfg=replace(p.cfg,free_points=False);plan=p.solve(np.zeros(12),2.3,0)
    assert len(p.calls)==1 and 'center_reference' not in plan.optimizer_stats
