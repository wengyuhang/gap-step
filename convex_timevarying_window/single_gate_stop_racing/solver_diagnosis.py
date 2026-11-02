"""Controlled fixed-input gauge/budget diagnosis; no Gazebo flights."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from .planner import Config,Single,parking_points,route_signs
from .point_mapping import numeric


def main():
    p=argparse.ArgumentParser();p.add_argument('--previous',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    a.outdir.mkdir(parents=True,exist_ok=False);rows=[]
    for f,amp,gate in ((3.5,4.,1),(3.,4.,6)):
        name=f'seven_ours_f{f:g}_a{amp:g}_p000'
        logs=[json.loads(s) for s in (a.previous/name/'solves_live.jsonl').read_text().splitlines()]
        bad=[r for r in logs if r['gate']==gate and not r.get('accepted')]
        track,_=build_case(Case(f,amp,0.));cfg=Config()
        local=SCWindowTrack('probe',track.start,parking_points(track,cfg)[gate],(track.windows[gate],),(0,))
        began=time.perf_counter();model=Single(local,cfg,route_signs(track)[gate])
        print('BUILT',name,'seconds',time.perf_counter()-began,flush=True)
        for label,entry in (('first_failure',bad[0]),('last_failure',bad[-1])):
            head=entry['request_head']
            for gauge in (False,True):
                model.lower[2:-3]=-1. if gauge else -np.inf
                model.upper[2:-3]=1. if gauge else np.inf
                for budget in (90.,1000.):
                    model.cfg=replace(cfg,max_solve_ms=budget);model.failed_seed=None
                    row=dict(case=name,gate=gate+1,input=label,gauge=gauge,budget_ms=budget)
                    try:
                        plan=model.solve(np.asarray(head['pvaj']),head['made_at'],0)
                        row.update(accepted=True,solve_ms=plan.solve_ms,durations=plan.durations.tolist())
                    except Exception as exc:row.update(accepted=False,error=repr(exc))
                    row['stats']=model.solvers[0][0].value
                    for attempt in row['stats']['attempts']:
                        for r in attempt['rounds']:
                            x=np.asarray(r['variables']);r['mapped_point']=numeric(track.windows[gate].aperture,x[2:-3]).tolist()
                    rows.append(row);(a.outdir/'diagnosis.json').write_text(json.dumps(rows,indent=2)+'\n')
                    print(name,label,'gauge',gauge,'budget',budget,row['accepted'],row['stats']['optimization_ms'],
                          'check',row['stats']['local_check_ms'],flush=True)


if __name__=='__main__':main()
