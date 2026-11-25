"""Parameter-only diagnostics; frozen symbolic constraints and solver algorithm."""
import argparse
import json
from itertools import product
from dataclasses import replace
from pathlib import Path
import numpy as np
from .scene import scaled_track
from .planner import Planner
from .model import GenericQuad,planning_seed_parameters
from ..planner import Config
from ..scenario import workspace_bounds
from convex_timevarying_window.togt_point_only_mpc.trajectory import Plan


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',required=True)
    args=parser.parse_args();out=Path(args.outdir);out.mkdir(parents=True,exist_ok=False)
    previous=Path(__file__).parent/'results/native_sphere_scaled1886_f4_a4_v1_20261010'
    d=json.loads((previous/'plans.json').read_text())[0]
    old=Plan(d['made_at'],0,np.array(d['durations']),np.array(d['coefficients']),
        np.zeros(9),d['solve_ms'],0,'recorded',1)
    track=scaled_track(1.886280447);lo,hi=workspace_bounds()
    model=GenericQuad(air_density=1.210699200630188)
    cfg=Config(radius=.728,workspace_lower=lo,workspace_upper=hi,passage_margin=.04,
        compiler_optimization='-O1',**planning_seed_parameters(model))
    planner=Planner(track,cfg,model)
    locals_=[planner.prepare(i) for i in (0,1)]
    cases=[('initial',0,0.,np.r_[track.start,np.zeros(9)]),
        ('gate1_exit',1,5.493885952,np.concatenate([old.sample(5.493885952,k) for k in range(4)])),
        ('parked',1,14.137807872,np.concatenate([old.sample(14.137807872,k) for k in range(4)]))]
    results=[]
    for arrival,speed,tail in product((6.5,9.,12.,15.),(6.,9.,12.,16.),(1.,1.5)):
            row=dict(arrival_seed_speed=arrival,speed_seed=speed,initial_tail_scale=tail,cases=[])
            for name,index,now,state in cases:
                local=locals_[index]
                # These parameters affect numerical seeds/scaling only, not
                # symbolic geometry, native PWM limits, or the objective.
                local.cfg=replace(cfg,speed=speed,initial_tail_scale=tail,arrival_seed_speed=arrival)
                local.failed_seed=None;local.admitted=None;attempts=[]
                for request in range(3):
                    plan=None;error=None
                    try:plan=local.solve(state,now,0)
                    except RuntimeError as e:error=str(e)
                    stats=local.solvers[0][0].value
                    attempts.append(dict(accepted=plan is not None,error=error,stats=stats))
                    if plan is not None:break
                row['cases'].append(dict(name=name,attempts=attempts))
            results.append(row)
            (out/'probe.json').write_text(json.dumps(results,indent=2))
            print(arrival,speed,tail,[(c['name'],any(a['accepted'] for a in c['attempts'])) for c in row['cases']],flush=True)


if __name__=='__main__':main()
