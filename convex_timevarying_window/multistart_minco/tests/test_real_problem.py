import numpy as np
from convex_timevarying_window.multistart_minco.planner import Planner,Config
from convex_timevarying_window.comparisons.single_gate_swing.scenario import Case,build_case


def test_real_free_and_center_have_same_starts_and_select_best_safe_candidate():
    track,_=build_case(Case(1.,0.,0.));head=np.r_[track.start,np.zeros(9)];starts=[]
    for free in (True,False):
        p=Planner(track,Config(free_points=free,compile_objective=False));plan=p.solve(head,.15,0);s=plan.optimizer_stats
        assert len(s['temporal_candidates'])==4 and s['all_time_seeds_attempted']
        feasible=[r for r in s['temporal_candidates'] if r['admitted']]
        assert plan.durations.sum()==min(r['duration_s'] for r in feasible)
        assert s['passage']['passed'] and plan.made_at==.15
        assert all(r['optimizer_stats']['passage']['passed'] for r in feasible)
        assert all(np.array_equal(r['seed_variables'][2:4],[0.,0.]) for r in s['temporal_candidates'])
        starts.append([r['seed_variables'] for r in s['temporal_candidates']])
        if not free:np.testing.assert_array_equal(plan.x[2:4],[0.,0.])
    np.testing.assert_array_equal(starts[0],starts[1])
