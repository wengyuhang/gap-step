"""Compute one frozen global plan before launching the physical race."""
import argparse,json,time
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .scenario import Case,build_case,manifest
from .planner import Planner
from convex_timevarying_window.comparisons.single_gate_constant_velocity.run import save

def main():
    p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--level',required=True);p.add_argument('--method',choices=('ours','center'),required=True);p.add_argument('--outdir',type=Path,required=True);p.add_argument('--seconds-per-seed',type=float,default=180.);a=p.parse_args()
    out=a.outdir.resolve();out.mkdir(parents=True,exist_ok=False);case=Case(a.layout,a.level);track,_=build_case(case)
    save(out/'protocol.json',dict(case=asdict(case),method=a.method,scene=manifest(case),all_seven_joint=True,intermediate_stops=False,replanning=False,
        clock='offline solve at race t=0; exogenous known motion starts at race epoch; planning cost reported separately',seconds_per_seed=a.seconds_per_seed))
    began=time.perf_counter();planner=Planner(track,a.method=='ours');build_s=time.perf_counter()-began
    save(out/'build.json',dict(build_s=build_s,variables=planner.nvar,pieces=planner.n,config=asdict(planner.cfg)))
    print('BUILD',a.layout,a.level,a.method,build_s,planner.nvar,flush=True)
    with (out/'progress.jsonl').open('w') as f:
        def progress(r):f.write(json.dumps(r,ensure_ascii=False)+'\n');f.flush();print(json.dumps(r,ensure_ascii=False),flush=True)
        plan,result=planner.solve_all(a.seconds_per_seed,progress)
    result.update(build_s=build_s,case=asdict(case),method=a.method,config=asdict(planner.cfg))
    if plan is not None:
        result.update(durations=plan.durations.tolist(),coefficients=plan.coefficients.tolist(),variables=plan.x.tolist(),made_at=0.,nominal_time_s=float(plan.durations.sum()),
            crossing_times_s=plan.crossing_times.tolist(),crossing_points=np.array(planner.point_function(plan.x)).T.tolist())
        np.savez_compressed(out/'trajectory.npz',durations=plan.durations,coefficients=plan.coefficients,variables=plan.x)
    save(out/'result.json',result);print('PLAN_RESULT',result['found'],result.get('nominal_time_s'),result['solve_s'],flush=True)

if __name__=='__main__':main()
