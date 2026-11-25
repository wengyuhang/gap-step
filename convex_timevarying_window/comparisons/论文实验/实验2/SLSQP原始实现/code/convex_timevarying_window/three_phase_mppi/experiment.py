"""Independent same-model experiment. Does not emulate solver latency/PX4."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from .reference import Planner
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.diagnostic import pvaj_from_state
from .controller import Controller,Progress,config
from .events import reason,computation_safe

def snapshot(out):
    roots=[Path(__file__).parent,Path('convex_timevarying_window/dynamic_gate_mppi_v3'),Path('convex_timevarying_window/togt')]
    hashes={}
    for root in roots:
        for p in root.rglob('*'):
            if p.suffix not in ('.py','.cpp','.md') or any(x in p.relative_to(root).parts for x in ('results','build','__pycache__','install','log')):continue
            relative=p.resolve().relative_to(Path.cwd());dest=out/'source_snapshot'/relative
            dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
            hashes[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    (out/'source_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
    return hashes

def run(args):
    out=args.outdir;out.mkdir(parents=True,exist_ok=False);hashes=snapshot(out)
    track,opt=build_track();vary_track(track,args.phase,args.layout,args.motion_rate,args.translation_scale,args.rotation_scale)
    cfg=config(args.samples);tick=time.perf_counter();ctrl=Controller(track,cfg,args.seed);planner=Planner(track,opt)
    state=hover_state(track.start);previous=np.array([MASS*9.8066,0,0,0.]);progress=Progress(track,cfg,ctrl.engine.signs)
    # Compile and exercise both code paths before measurement.
    ctrl.select(state,0.,0,0,None,previous);ctrl.prepare_gate=0
    for j in range(3):
        warm=ctrl.select(state,0.,0,0,None,previous)
        if warm is not None:ctrl.active=warm
    ctrl.prepare_gate=None;ctrl.active=None
    warmup=time.perf_counter()-tick;ref=None;pending='initial';updates=[];logs=[];rows=[];failure=None;now=0.;next_control=0.
    started=time.perf_counter();live=(out/'cycles.jsonl').open('w',buffering=1)
    try:
        while now<args.duration:
            if pending=='gate_exit' and ctrl.active is not None:
                prefix,_,_=ctrl.prefix(ctrl.active,state,now,progress.first,progress.phase,previous)
                if computation_safe(prefix):
                    candidate=planner.solve(pvaj_from_state(ctrl.engine,state,previous),now,progress.first)
                    updates.append(dict(time=now,reason=pending,accepted=candidate is not None,solve_ms=planner.last_stats.get('elapsed_ms') if candidate is None else candidate.solve_ms))
                    if candidate is not None:ref=candidate;pending=None
            if now>=next_control-1e-8:
                sel=ctrl.select(state,now,progress.first,progress.phase,ref,previous)
                record=dict(ctrl.last);logs.append(record);live.write(json.dumps(record)+'\n')
                pending=pending or reason(ctrl.stage,avoiding=record['transit_avoidance'])
                if sel is not None:ctrl.active=sel
                elif ctrl.active is None:failure='no_admissible_policy';break
                else:
                    m,_,_=ctrl.prefix(ctrl.active,state,now,progress.first,progress.phase,previous)
                    if m[1]!=0 or ctrl.active.valid_until-now<.1:failure='no_admissible_successor';break
                prefix,_,_=ctrl.prefix(ctrl.active,state,now,progress.first,progress.phase,previous)
                if ref is not None and ctrl.stage=='TRANSIT':
                    r=ctrl.reference_array(ref,now,0,state)[0]
                    pending=pending or reason(ctrl.stage,position_error=np.linalg.norm(state[:3]-r[:3]),velocity_error=np.linalg.norm(state[3:6]-r[3:6]))
                if pending and computation_safe(prefix):
                    request_reason=pending;head=pvaj_from_state(ctrl.engine,state,previous)
                    candidate=planner.solve(head,now,progress.first)
                    updates.append(dict(time=now,reason=request_reason,accepted=candidate is not None,solve_ms=planner.last_stats.get('elapsed_ms') if candidate is None else candidate.solve_ms))
                    if candidate is not None:ref=candidate;pending=None
                next_control=now+.05
            action=ctrl.action(state,now,previous);new=ctrl.engine.integrate(state,action,.01)
            clear=ctrl.engine.geometry.interval_clearances(np.array([state[:3],new[:3]]),np.array([state[3:6],new[3:6]]),np.array([now,now+.01])).min()
            if progress.update(state[:3],now,new[:3],now+.01):pending='gate_exit'
            rows.append([now,*state,progress.first,progress.phase,clear]);previous=action;state=new;now+=.01
            if clear<=0:failure='executed_clearance';break
            if progress.violation:failure=progress.violation;break
            if progress.first==len(track.order) and np.linalg.norm(state[:3]-track.goal)<.55 and np.linalg.norm(state[3:6])<1:break
            if len(rows)%500==0:print(json.dumps(dict(t=now,gates=progress.first,stage=ctrl.stage,speed=float(np.linalg.norm(state[3:6])))),flush=True)
    except Exception as error:failure=repr(error)
    finally:live.close()
    completed=failure is None and now<=args.duration+1e-8 and progress.first==len(track.order) and np.linalg.norm(state[:3]-track.goal)<.55 and np.linalg.norm(state[3:6])<1
    if not completed and failure is None:failure='timeout'
    ms=np.array([x['cycle_ms'] for x in logs]);report=dict(method='three_phase_mppi',evidence='nominal same-model; no solver latency or PX4',completed=bool(completed),failure=failure,gates=progress.first,time=now,crossings=progress.crossings,updates=updates,
        cycles=dict(count=len(ms),median=float(np.median(ms)),p99=float(np.quantile(ms,.99)),max=float(ms.max()),over100=int(sum(ms>=100))),
        minimum_clearance=None if not rows else min(r[-1] for r in rows),warmup_s=warmup,wall_s=time.perf_counter()-started,config=asdict(cfg),condition={k:v for k,v in vars(args).items() if k!='outdir'},source_sha256=hashes)
    np.savez_compressed(out/'trace.npz',rows=np.asarray(rows),final_state=state)
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256','config','updates')},indent=2),flush=True)
    return report

def parser():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    for name,default in [('duration',60.),('phase',.4),('layout',0.),('motion-rate',1.),('translation-scale',1.),('rotation-scale',1.)]:p.add_argument('--'+name,type=float,default=default)
    p.add_argument('--samples',type=int,default=16384);p.add_argument('--seed',type=int,default=0);return p

if __name__=='__main__':run(parser().parse_args())
