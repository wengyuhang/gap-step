"""Small nominal-model passage regression; no PX4 or real-time claim."""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from .controller import make_controller
from .config import Config
from .geometry import GateProgress
from .rollout import hover_state, MASS


def scenario(moving):
    track,_=build_track(); w=track.windows[0]
    w.center0=np.array([0.,0.,4.]); w.angles0_rpy=np.array([0.,np.pi/2,0.])
    w.motion=replace(w.motion,translation_enabled=moving,rotation_enabled=moving,
        translation_amplitude=np.array([.1,.15,.1]),rotation_amplitude=np.deg2rad([3.,3.,3.]),
        translation_period=3.,rotation_period=3.,phase=.4)
    return replace(track,start=np.array([-3.,0.,4.]),goal=np.array([3.,0.,4.]),windows=(w,),order=(0,))


def run_case(moving,phase,seed,backend='cuda',samples=16384):
    track=scenario(moving);c=make_controller(track,replace(Config(),samples=samples),seed,backend);p=GateProgress(track,crossing=phase)
    w=track.windows[0];center,R,dc,_=w.state_at(0.);normal=R[:,2]*p.signs[0]
    offset={0:-2.,1:-.2,2:.2}[phase];speed={0:4.,1:3.,2:-3.}[phase]
    x=hover_state(center+normal*offset);x[3:6]=normal*speed+dc
    last=np.full(4,MASS*9.8066/4);rows=[];failure=None;done=False
    for step in range(150):
        t=step*.02
        chosen=c.select(x,t,p.first,p.crossing,True,None,last)
        if chosen is not None:c.publish(chosen,x,t)
        u=c.cached_action(x,t)
        if u is None:failure='cache_exhausted';break
        middle=c.rollout.integrate(x,u,.01);end=c.rollout.integrate(middle,u,.01)
        clear=float(c.rollout.geometry.interval_clearances(
            np.array([x[:3],middle[:3],end[:3]]),np.array([x[3:6],middle[3:6],end[3:6]]),
            np.array([t,t+.01,t+.02])).min())
        p.update(x[:3],t,middle[:3],t+.01,clear>0)
        p.update(middle[:3],t+.01,end[:3],t+.02,clear>0)
        rows.append(dict(t=t,position=x[:3].tolist(),speed=float(np.linalg.norm(x[3:6])),
            clearance=clear,phase=p.crossing,gate=p.first,ms=c.last_diagnostics['cycle_ms'],
            safe_samples=c.last_diagnostics['safe_samples']))
        x=end;last=u
        if clear<=0 or p.violation:failure=p.violation or 'clearance_failed';break
        if p.first==1 or (phase>0 and p.crossing==0):done=True;break
    if not done and failure is None:failure='timeout'
    return dict(moving=moving,initial_phase=phase,seed=seed,backend=backend,samples=samples,completed_exit=done,
        exit_side='forward' if p.first==1 else ('retreat' if done else None),failure=failure,
        time_s=(len(rows)*.02),max_ms=max((r['ms'] for r in rows),default=None),trace=rows)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--backend',choices=('cuda','cpu'),default='cuda')
    parser.add_argument('--samples',type=int,default=Config().samples);args=parser.parse_args()
    args.outdir.mkdir(parents=True,exist_ok=False);hashes={}
    for source in Path(__file__).parent.rglob('*'):
        rel=source.relative_to(Path(__file__).parent)
        if source.suffix not in ('.py','.cpp') or any(x in rel.parts for x in ('results','build','install','log')):continue
        hashes[str(rel)]=hashlib.sha256(source.read_bytes()).hexdigest()
        dest=args.outdir/'source_snapshot'/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
    results=[]
    for moving in (False,True):
        for phase in (0,1,2):
            for seed in (0,1,2):
                r=run_case(moving,phase,seed,args.backend,args.samples);results.append(r)
                print(json.dumps({k:v for k,v in r.items() if k!='trace'}),flush=True)
    report=dict(evidence='Nominal same-model, no TOGT, avoidance only; NOT Gazebo or real-time',
        cases=len(results),completed_exits=sum(r['completed_exit'] for r in results),
        source_sha256=hashes,results=results)
    (args.outdir/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
