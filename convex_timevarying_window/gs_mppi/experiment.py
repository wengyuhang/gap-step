"""Nominal direct-control validation; no claim of PX4 or continuous certification."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time
from dataclasses import replace
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from .progress import Progress
from convex_timevarying_window.three_phase_mppi.reference import Planner
from convex_timevarying_window.three_phase_mppi.events import reason
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from .controller import Controller
from .engine import advance

def run(args):
    args.outdir.mkdir(parents=True,exist_ok=False)
    track,opt=build_track();vary_track(track,args.phase,0.,args.motion_rate,1.,1.)
    if args.single:
        w=track.windows[0];center,r,*_=w.state_at(0.);sign=-np.sign(r[:,2]@(track.start-center))
        start=center-8*sign*r[:,2]+args.offset*r[:,0]
        track=replace(track,start=start,goal=center+8*sign*r[:,2],windows=(w,),order=(0,))
        velocity=args.speed*sign*r[:,2]
    else:velocity=np.zeros(3)
    ctrl=Controller(track,args.samples,args.seed,args.steps);state=np.r_[track.start,velocity]
    progress=Progress(track,ctrl.engine.config,ctrl.engine.signs);planner=Planner(track,opt)
    hashes={}
    for root in (Path(__file__).parent,Path('convex_timevarying_window/three_phase_mppi')):
        for f in root.glob('*'):
            if f.suffix not in ('.py','.cpp','.md'):continue
            relative=f.resolve().relative_to(Path.cwd());dest=args.outdir/'source_snapshot'/relative;dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(f,dest);hashes[str(relative)]=hashlib.sha256(f.read_bytes()).hexdigest()
    # Record local transitive dependencies, including the native TOGT source,
    # not just this method's entry points. Test/result trees are excluded.
    roots=('convex_timevarying_window/dynamic_gate_mppi_v3','convex_timevarying_window/togt',
           'nonconvex_timevarying_window/sc_dynatogt','convex_dynamic_seven_window_gazebo/x500_togt')
    files=[Path('convex_timevarying_window/geometry.py'),Path('convex_timevarying_window/receding_gate_racing/scenario.py')]
    for root in roots:
        files+=list(Path(root).glob('*.py'))+list((Path(root)/'native').glob('*.cpp'))
    for f in files:
        dest=args.outdir/'source_snapshot'/f;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
        hashes[str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
    started=time.perf_counter()
    ctrl.select(state,0.,0,0,None);ctrl.mean=None;ctrl.prior_time=None
    warmup=time.perf_counter()-started;now=0.;previous=np.zeros(3);jerk=np.zeros(3);ref=None;pending='initial';failure=None
    logs=[];updates=[];rows=[];trace=[];finished=False;was_avoiding=False
    with (args.outdir/'cycles.jsonl').open('w',buffering=1) as stream:
        try:
            while now<args.duration:
                if pending and not args.single:
                    head=BoundaryState(state[:3],state[3:],previous,jerk)
                    candidate=planner.solve(head,now,progress.first)
                    updates.append(dict(time=now,reason=pending,accepted=candidate is not None,solve_ms=None if candidate is None else candidate.solve_ms))
                    if candidate is not None:ref=candidate;pending=None
                action=ctrl.select(state,now,progress.first,progress.phase,ref)
                logs.append(ctrl.last);stream.write(json.dumps(ctrl.last)+'\n')
                if action is None:failure='no_current_filtered_control';break
                # Execution re-filters current command every integration step.
                interval=ctrl.dt
                for j in range(4):
                    a,info=ctrl.engine.filter(state,now,action)
                    if info['status'] or info['B']<0 or info['h_min']<0:failure='execution_filter_domain';break
                    dt=interval/4;new=advance(state,a,dt)
                    clear=float(ctrl.engine.geometry.interval_clearances(np.array([state[:3],new[:3]]),np.array([state[3:],new[3:]]),np.array([now,now+dt])).min())
                    if progress.update(state[:3],now,new[:3],now+dt):pending='gate_exit'
                    rows.append([now,*state,progress.first,progress.phase,clear,info['B'],info['residual']]);trace.append([now,*state,*a])
                    jerk=(a-previous)/dt;previous=a;state=new;now+=dt
                    if clear<=0:failure='executed_geometry';break
                    if progress.violation:failure=progress.violation;break
                if failure:break
                if args.single and progress.first==1:finished=True;break
                if progress.first==len(track.order) and np.linalg.norm(state[:3]-track.goal)<.55 and np.linalg.norm(state[3:])<1:finished=True;break
                if ref is not None and ctrl.stage=='TRANSIT':
                    r=ctrl.reference(ref,now,state)[0]
                    pending=pending or reason(ctrl.stage,position_error=np.linalg.norm(state[:3]-r[:3]),velocity_error=np.linalg.norm(state[3:]-r[3:6]))
                if ctrl.last['avoiding'] and not was_avoiding:pending=pending or 'obstacle_recovery'
                was_avoiding=ctrl.last['avoiding']
                if len(logs)%250==0:print(json.dumps(dict(time=now,gates=progress.first,stage=ctrl.stage)),flush=True)
        except Exception as error:failure=repr(error)
    if not finished and failure is None:failure='timeout'
    ms=np.array([r['cycle_ms'] for r in logs])
    result=dict(method='gs_inspired_direct_acceleration',completed=finished,failure=failure,gates=progress.first,time=now,
        crossings=progress.crossings,center_crossings=progress.centers,updates=updates,warmup_s=warmup,source_sha256=hashes,
        evidence='six-state nominal model, paused computation clock; not PX4 or a safety certificate',
        condition={k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        minimum_clearance=None if not rows else min(r[-3] for r in rows),
        cycles=dict(count=len(ms),median=float(np.median(ms)),p99=float(np.quantile(ms,.99)),max=float(ms.max()),over100=int(sum(ms>=100))))
    np.savez_compressed(args.outdir/'trace.npz',rows=np.asarray(rows),commands=np.asarray(trace),final_state=state)
    (args.outdir/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('source_sha256','updates')},indent=2),flush=True)
    return result

def parser():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);p.add_argument('--single',action='store_true')
    for name,default in [('duration',60.),('phase',0.),('motion-rate',1.),('offset',0.),('speed',3.)]:p.add_argument('--'+name,type=float,default=default)
    p.add_argument('--samples',type=int,default=16384);p.add_argument('--steps',type=int,default=100);p.add_argument('--seed',type=int,default=0);return p

if __name__=='__main__':run(parser().parse_args())
