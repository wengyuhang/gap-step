"""Seventeen-state prediction/execution consistency and racing experiment."""
import argparse
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import shutil
import time
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.diagnostic import pvaj_from_state
from .reference import Planner
from convex_timevarying_window.gs_mppi.progress import Progress
from .controller import Controller

def snapshot(out):
    roots=[Path(__file__).parent,Path('convex_timevarying_window/dynamic_gate_mppi_v3'),Path('convex_timevarying_window/three_phase_mppi'),
           Path('convex_timevarying_window/togt'),Path('nonconvex_timevarying_window/sc_dynatogt'),Path('convex_dynamic_seven_window_gazebo/x500_togt')]
    files=[Path('convex_timevarying_window/geometry.py'),Path('convex_timevarying_window/receding_gate_racing/scenario.py'),Path('convex_timevarying_window/gs_mppi/progress.py')]
    for root in roots:files+=list(root.glob('*.py'))+list(root.glob('*.cpp'))+list(root.glob('*.md'))+list((root/'native').glob('*.cpp'))
    hashes={}
    for f in files:
        relative=f.resolve().relative_to(Path.cwd());dest=out/'source_snapshot'/relative;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
        hashes[str(relative)]=hashlib.sha256(f.read_bytes()).hexdigest()
    return hashes

def run(args):
    args.outdir.mkdir(parents=True,exist_ok=False);hashes=snapshot(args.outdir)
    track,opt=build_track();vary_track(track,args.phase,0.,args.motion_rate,args.translation_scale,args.rotation_scale)
    velocity=np.zeros(3)
    if args.single:
        w=track.windows[0];c,r,*_=w.state_at(0);sign=-np.sign(r[:,2]@(track.start-c))
        track=replace(track,start=c-8*sign*r[:,2]+args.offset*r[:,0],goal=c+8*sign*r[:,2],windows=(w,),order=(0,));velocity=args.speed*sign*r[:,2]
    ctrl=Controller(track,args.samples,args.steps,args.seed,racing_guidance=args.experimental_racing_guidance,coupled_racing=args.coupled_racing)
    if args.coupled_racing:
        from .racing_reference import Planner as RacingPlanner
        planner=RacingPlanner(track,opt)
    else:planner=Planner(track,opt,racing_guidance=args.experimental_racing_guidance)
    x=hover_state(track.start);x[3:6]=velocity
    previous=np.array([MASS*9.8066,0,0,0.]);progress=Progress(track,ctrl.engine.config,ctrl.engine.signs,finite_entry_only=True)
    warm=time.perf_counter();ctrl.select(x,0.,0,0,None);ctrl.mean=None;ctrl.prepare_gate=None;ctrl.prior_time=None;warm=time.perf_counter()-warm
    now=0.;ref=None;pending='initial';failure=None;complete=False;was_avoiding=False;rows=[];cycles=[];updates=[];prediction_errors=[]
    with (args.outdir/'cycles.jsonl').open('w',buffering=1) as stream:
        try:
            while now<args.duration:
                if pending and not args.single:
                    tick=time.perf_counter();candidate=planner.solve(pvaj_from_state(ctrl.engine,x,previous),now,progress.first,budget=.085)
                    elapsed=1000*(time.perf_counter()-tick);accepted=candidate is not None and elapsed<100
                    updates.append(dict(time=now,reason=pending,accepted=accepted,wall_ms=elapsed))
                    if accepted:ref=candidate;pending=None
                action=ctrl.select(x,now,progress.first,progress.phase,ref,previous);cycles.append(ctrl.last);stream.write(json.dumps(ctrl.last)+'\n')
                if action is None:
                    failure='deadline' if ctrl.last.get('deadline_exceeded') else 'no_admissible_CTBR'
                    if hasattr(ctrl,'failure_data'):np.savez_compressed(args.outdir/'failed_selection.npz',**ctrl.failure_data)
                    break
                for _ in range(4):
                    # Direct selected CTBR, without any downstream PVA controller.
                    new=ctrl.engine.integrate(x,action,.005)
                    clear=float(ctrl.engine.geometry.interval_clearances(np.array([x[:3],new[:3]]),np.array([x[3:6],new[3:6]]),np.array([now,now+.005])).min())
                    if progress.update(x[:3],now,new[:3],now+.005):pending='gate_exit'
                    rows.append([now,*x,*action,progress.first,progress.phase,clear]);x=new;now+=.005;previous=action
                    if clear<=0:failure='executed_geometry';break
                    if progress.violation:failure=progress.violation;break
                prediction_errors.append(float(np.max(np.abs(x-ctrl.next_state))))
                if failure:break
                if args.single and progress.first==1:complete=True;break
                if progress.first==len(track.order) and np.linalg.norm(x[:3]-track.goal)<.55 and np.linalg.norm(x[3:6])<1:complete=True;break
                if ref is not None and ctrl.stage=='TRANSIT':
                    r=ctrl.reference(ref,now,x)[0]
                    if np.linalg.norm(x[:3]-r[:3])>.5 or np.linalg.norm(x[3:6]-r[3:6])>2:pending=pending or 'tracking_error'
                if ctrl.last['avoiding'] and not was_avoiding:pending=pending or 'obstacle_recovery'
                was_avoiding=ctrl.last['avoiding']
                if len(cycles)%250==0:print(json.dumps(dict(time=now,gates=progress.first,stage=ctrl.stage,speed=float(np.linalg.norm(x[3:6])))),flush=True)
        except Exception as error:failure=repr(error)
    if not complete and failure is None:failure='timeout'
    ms=np.array([r['cycle_ms'] for r in cycles]);planning=[u['wall_ms'] for u in updates]
    result=dict(method='direct_CTBR_MPPI_17state',completed=complete,failure=failure,gates=progress.first,time=now,
        crossings=progress.crossings,center_crossings=progress.centers,updates=updates,warmup_s=warm,source_sha256=hashes,
        evidence='same empirical 17-state CTBR plant in prediction and execution; paused solver clock; NOT PX4 or certification',
        condition={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        runtime_threads={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
        phase_units='radians',
        minimum_clearance=None if not rows else min(r[-1] for r in rows),prediction_execution_error_max=max(prediction_errors,default=0.),
        cycles=dict(count=len(ms),median=float(np.median(ms)),p99=float(np.quantile(ms,.99)),max=float(ms.max()),over100=int(sum(ms>=100))),
        planner=dict(count=len(planning),max=max(planning,default=0.),over100=sum(v>=100 for v in planning)))
    np.savez_compressed(args.outdir/'trace.npz',rows=np.asarray(rows),final_state=x,prediction_errors=prediction_errors)
    (args.outdir/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','updates','crossings','center_crossings')},indent=2),flush=True)
    return result

def parser():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--single',action='store_true')
    for name,default in [('duration',60.),('phase',0.),('motion-rate',1.),('translation-scale',1.),('rotation-scale',1.),('offset',0.),('speed',3.)]:p.add_argument('--'+name,type=float,default=default)
    p.add_argument('--matched-px4-parameters',action='store_true',help='Use the PX4 parameter profile of the 2026-09-23 center-motion comparison')
    variants=p.add_mutually_exclusive_group()
    variants.add_argument('--experimental-racing-guidance',action='store_true',help='Unvalidated development variant: inset TOGT points, reference handoff, corner-aware seed; default remains v7')
    variants.add_argument('--coupled-racing',action='store_true',help='Private dynamics-aware TOGT and optional TOGT control proposal during autonomous passage; experimental, latest PX4 trial failed tilt limit')
    for name,default in [('samples',16384),('steps',100),('seed',0)]:p.add_argument('--'+name,type=int,default=default)
    return p

if __name__=='__main__':run(parser().parse_args())
