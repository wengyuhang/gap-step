"""Fixed-head counterfactuals: initializer and stopping-tolerance mechanisms."""
import argparse,json,time
from pathlib import Path
from dataclasses import replace
import numpy as np
from convex_timevarying_window.third_gate_stop_mpc.planner import Planner,Config
from convex_timevarying_window.wang_free_state_mpc.trajectory import Plan,arrival_jerk_for_middle
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case

ROOT=Path(__file__).resolve().parents[2]
FLIGHT=ROOT/'convex_timevarying_window/third_gate_stop_mpc/results/hardest_v1_20261008/third_stop_ours_f1_a2_p000_attempt1'


from .initialization import seed


def unpack(s):
    gates=len(s['durations'])//2-1
    return Plan(s['made_at'],s['candidate_first_gate'],np.array(s['durations']),np.array(s['coefficients']),
                np.array(s['variables']),0.,s['violation'],s['status'],gates)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--outdir',type=Path,required=True);a=ap.parse_args()
    a.outdir.mkdir(parents=True,exist_ok=False);install();track,_=build_case(Case(1.,2.,0.))
    raw=json.loads((FLIGHT/'result.json').read_text());plans=[unpack(s) for s in raw['solves'] if s['accepted']]
    cfg=Config(compile_objective=False,workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
    p=Planner(track,cfg,build_indices=[0,1,2,3,4]);original=p.initial;rows=[]
    for index,current in enumerate(plans[:5]):
        state=np.concatenate([current.sample(current.made_at,d) for d in range(4)])
        old=plans[index-1] if index else None
        for name,travel,velocity,tol in [('original',None,None,1e-6),('strict',None,None,1e-8),
            ('time_seed',12.,5.4,1e-6),('velocity_seed',6.,9.,1e-6),('racing_seed',12.,9.,1e-6)]:
            p.search.clear();p.commit(old);p.solvers[index][0].cfg=replace(cfg,optimizer_ftol=tol)
            p.initial=original if travel is None else lambda state,now,first:seed(p,state,now,first,travel,velocity)
            plan=p.solve(state,current.made_at,index)
            row=dict(first=index,variant=name,converged=plan.optimizer_stats['converged'],
                event_valid=plan.event_valid,reason=plan.event_reason,residual=plan.violation,
                first_crossing=float(plan.crossing_times[0]),second_crossing=float(plan.crossing_times[1]),
                end=plan.end_time,objective=plan.optimizer_stats['final_objective'],
                solve_ms=plan.solve_ms,iterations=plan.optimizer_stats['iter_count'],variables=plan.x.tolist())
            rows.append(row);print(json.dumps({k:v for k,v in row.items() if k!='variables'}),flush=True)
            (a.outdir/'counterfactuals.json').write_text(json.dumps(rows,indent=2)+'\n')


if __name__=='__main__':main()
