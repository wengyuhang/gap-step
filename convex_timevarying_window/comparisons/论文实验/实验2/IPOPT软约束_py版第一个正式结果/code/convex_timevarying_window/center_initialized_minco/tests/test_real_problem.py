from dataclasses import replace
import numpy as np
from convex_timevarying_window.center_initialized_minco.planner import Planner,Config
from convex_timevarying_window.comparisons.single_gate_swing.scenario import Case,build_case
from convex_timevarying_window.comparisons.single_gate_swing.planner import Planner as Original


def test_real_graph_and_constraints_match_original_center_and_free_problems():
    track,_=build_case(Case(1.,0.,0.));cfg=Config(compile_objective=False)
    p=Planner(track,cfg);head=np.r_[track.start,np.zeros(9)];plan=p.solve(head,.15,0)
    center=p.last_center_plan
    assert plan.durations.sum()<=center.durations.sum()
    assert plan.made_at==center.made_at==.15
    assert center.optimizer_stats['passage']['passed'] and plan.optimizer_stats['passage']['passed']
    original=Original(track,replace(cfg,free_points=False))
    offsets=np.tile(np.r_[cfg.passage_seed_offsets,np.zeros(cfg.witness_slots-len(cfg.passage_seed_offsets))],2)
    par=np.r_[.15,head,offsets]
    for a,b in zip(p.function(center.x,par),original.function(center.x,par)):
        np.testing.assert_allclose(np.asarray(a),np.asarray(b),atol=1e-10,rtol=1e-10)
    s=plan.optimizer_stats
    assert s['both_stages_in_timing'] and s['optimization_ms']>=s['center_stage_ms']+s['free_stage_ms']
    assert len(s['free_stage']['attempts'])==1
    np.testing.assert_array_equal(s['free_stage']['attempts'][0]['seed'],center.x)


def test_circle_pendulum_uses_the_same_two_stage_mechanism():
    from convex_timevarying_window.center_initialized_minco.planner import CenterInitialized
    from convex_timevarying_window.comparisons.论文实验.实验1.pendulum.planner import Planner as Circle,Config as CircleConfig
    from convex_timevarying_window.comparisons.论文实验.实验1.pendulum.motion import build,Case as CircleCase
    class StagedCircle(CenterInitialized,Circle):pass
    track=build(CircleCase(0.));p=StagedCircle(track,CircleConfig(compile_objective=False))
    plan=p.solve(np.r_[track.start,np.zeros(9)],.15,0)
    assert plan.durations.sum()<=p.last_center_plan.durations.sum()
    assert plan.optimizer_stats['passage']['passed']
    assert len(plan.optimizer_stats['free_stage']['attempts'])==1
