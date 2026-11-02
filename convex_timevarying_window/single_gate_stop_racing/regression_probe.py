"""Replay failed PVAJ/time inputs without running or changing Gazebo."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from .planner import Config,Single,parking_points,route_signs


def main():
    p=argparse.ArgumentParser();p.add_argument('--previous',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True);p.add_argument('--budget-ms',type=float,default=90.)
    p.add_argument('--attempts',type=int,default=1)
    a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False);rows=[]
    for f,amp,gate in ((3.5,4.,1),(3.,4.,6)):
        name=f'seven_ours_f{f:g}_a{amp:g}_p000'
        logs=[json.loads(s) for s in (a.previous/name/'solves_live.jsonl').read_text().splitlines()]
        bad=[r for r in logs if r['gate']==gate and not r.get('accepted')]
        track,_=build_case(Case(f,amp,0.));cfg=Config(max_solve_ms=a.budget_ms)
        local=SCWindowTrack('probe',track.start,parking_points(track,cfg)[gate],(track.windows[gate],),(0,))
        began=time.perf_counter();model=Single(local,cfg,route_signs(track)[gate])
        print('BUILT',name,'seconds',time.perf_counter()-began,flush=True)
        for label,entry in (('first_failure',bad[0]),('last_failure',bad[-1])):
            head=entry['request_head'];model.failed_seed=None
            for attempt in range(a.attempts):
                row=dict(case=name,gate=gate+1,input=label,request_head=head,budget_ms=a.budget_ms,attempt=attempt)
                try:
                    plan=model.solve(np.asarray(head['pvaj']),head['made_at'],0)
                    row.update(accepted=True,solve_ms=plan.solve_ms,durations=plan.durations.tolist(),
                               point=plan.coefficients[0].sum(axis=0).tolist())
                except Exception as exc:row.update(accepted=False,error=repr(exc))
                row['stats']=model.solvers[0][0].value;rows.append(row)
                (a.outdir/'probe.json').write_text(json.dumps(rows,indent=2)+'\n')
                print(name,label,attempt,row['accepted'],row['stats']['optimization_ms'],
                      'check',row['stats']['local_check_ms'],flush=True)
                if row['accepted']:break


if __name__=='__main__':main()
