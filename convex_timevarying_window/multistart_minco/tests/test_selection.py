from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from convex_timevarying_window.multistart_minco import planner as module
from convex_timevarying_window.dynamic_passage_minco.planner import Stats


class NumericalParent:
    def __init__(self,free=True,outcomes=(5.,3.,4.,6.),budget=None,clock=None):
        self.cfg=module.Config(free_points=free,max_solve_ms=budget);self.admitted=object();self.calls=[];self.outcomes=outcomes
        self.nvar=7;self.clock=clock;self.solvers=[(Stats(),)]
    def solve(self,state,now,first):
        seeds=[np.array([np.log(t),0.,0.,0.,4.,0.,0.]) for t in (1.,.8,1.2,1.5)]
        seed=self.order_seeds(seeds,state,now)[0];index=self._time_seed_index
        self.calls.append(dict(index=index,seed=seed.copy(),head=state.copy(),now=now,free=self.cfg.free_points,budget=self.cfg.max_solve_ms,admitted=self.admitted))
        if self.clock is not None:self.clock[0]+=.02
        result=self.outcomes[index];self.solvers[0][0].value=dict(iter_count=index+1)
        if isinstance(result,str):raise RuntimeError(result)
        plan=SimpleNamespace(durations=np.array([result*.6,result*.4]),x=seed.copy(),coefficients=np.full((2,8,3),index),made_at=now,
            optimizer_stats=dict(iter_count=index+1,passage={'passed':True},final_objective=result))
        self.solvers[0][0].value=plan.optimizer_stats
        return plan


class Planner(module.MultipleTimeStarts,NumericalParent):pass


@pytest.mark.parametrize('free',[True,False])
def test_all_four_center_point_starts_and_shortest_admitted_selection(free):
    p=Planner(free=free);cfg,admitted=p.cfg,p.admitted;head=np.arange(12.)
    plan=p.solve(head,1.23,0)
    assert len(p.calls)==4 and plan.durations.sum()==pytest.approx(3.)
    assert plan.optimizer_stats['selected_seed_index']==1
    assert plan.optimizer_stats['all_time_seeds_attempted']
    for i,c in enumerate(p.calls):
        assert c['index']==i and c['now']==1.23 and c['free']==free and c['admitted'] is None
        np.testing.assert_array_equal(c['seed'][2:4],[0.,0.]);np.testing.assert_array_equal(c['head'],head)
    assert p.cfg is cfg and p.admitted is admitted


def test_failed_candidate_is_retained_in_log_but_not_selected():
    p=Planner(outcomes=(5.,'no_converged_safe_passage: candidate',4.,6.));plan=p.solve(np.zeros(12),0.,0)
    assert plan.durations.sum()==pytest.approx(4.)
    assert not plan.optimizer_stats['temporal_candidates'][1]['admitted']
    assert len(plan.optimizer_stats['temporal_candidates'])==4


def test_no_accepted_candidate_is_a_failure_not_a_flight_plan():
    p=Planner(outcomes=('no_converged_safe_passage: candidate',)*4)
    with pytest.raises(RuntimeError,match='no_admitted_multistart_candidate'):p.solve(np.zeros(12),0.,0)
    assert len(p.calls)==4
    assert not p.solvers[0][0].stats()['converged']


def test_unknown_error_is_not_silently_masked():
    p=Planner(outcomes=(5.,'programming error',4.,6.));cfg=p.cfg
    with pytest.raises(RuntimeError,match='programming error'):p.solve(np.zeros(12),0.,0)
    assert p.cfg is cfg and p._time_seed_index is None


def test_finite_budget_is_shared_and_total_timing_contains_all_starts(monkeypatch):
    clock=[0.];monkeypatch.setattr(module,'time',SimpleNamespace(perf_counter=lambda:clock[0]))
    p=Planner(budget=30.,clock=clock);plan=p.solve(np.zeros(12),0.,0)
    assert len(p.calls)==2 and p.calls[1]['budget']==pytest.approx(10.)
    assert plan.solve_ms==pytest.approx(40.)
    assert not plan.optimizer_stats['all_time_seeds_attempted']
