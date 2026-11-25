"""Normal-convergence cold-start measurements, not simulated flight results."""
import argparse,json,time
from pathlib import Path
from dataclasses import replace
import numpy as np
from convex_timevarying_window.comparisons.original_fixed_plane_deadline.adapter import install
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from .planner import Planner,Config


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--outdir',type=Path,required=True);a=ap.parse_args()
    a.outdir.mkdir(parents=True,exist_ok=False);install();track,_=build_case(Case(1.,2.,0.))
    rows=[]
    for variant in ('ours','center','no_value'):
        cfg=Config(free_points=variant!='center',use_third_gate=variant!='no_value',
            workspace_lower=(-28.,-30.,.65),workspace_upper=(28.,22.,12.))
        began=time.perf_counter();planner=Planner(track,cfg,build_indices=[0]);build=time.perf_counter()-began
        plan=planner.solve(np.r_[track.start,np.zeros(9)],.15,0)
        row=dict(variant=variant,build_s=build,solve_ms=plan.solve_ms,violation=plan.violation,
            event_valid=plan.event_valid,event_reason=plan.event_reason,status=plan.optimizer_stats,
            third_time_s=plan.estimated_third_time_s,local_time_s=plan.end_time-plan.made_at,
            crossing_times=plan.crossing_times.tolist(),variables=plan.x.tolist())
        rows.append(row);(a.outdir/'results.json').write_text(json.dumps(rows,indent=2)+'\n')
        print(json.dumps(row),flush=True)


if __name__=='__main__':main()
