"""Actuator-gap diagnostic, NOT Gazebo/PX4 and NOT a certified controller.

Keep the frozen six-state MPPI unchanged; apply its acceleration through the
existing empirical x500 CTBR attitude/motor plant. Stop at the first violation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr import CTBRRollout
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state,MASS
from convex_timevarying_window.dynamic_gate_mppi_v3.diagnostic import pvaj_from_state
from convex_timevarying_window.three_phase_mppi.reference import Planner
from .controller import Controller
from .progress import Progress

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--motion-rate',type=float,default=1.);args=parser.parse_args()
    args.outdir.mkdir(parents=True,exist_ok=False)
    track,opt=build_track();vary_track(track,0.,0.,args.motion_rate,1.,1.)
    ctrl=Controller(track);plant=CTBRRollout(track,ctrl.engine.config);planner=Planner(track,opt)
    x=hover_state(track.start);u=np.array([MASS*9.8066,0.,0.,0.]);progress=Progress(track,ctrl.engine.config,ctrl.engine.signs)
    sources=[Path(__file__),Path(__file__).with_name('controller.py'),Path(__file__).with_name('engine.py'),
             Path(__file__).with_name('kernel.cpp'),Path(__file__).with_name('progress.py'),
             Path('convex_timevarying_window/dynamic_gate_mppi_v3/ctbr.py'),
             Path('convex_timevarying_window/dynamic_gate_mppi_v3/native/rollout.cpp')]
    hashes={}
    for f in sources:
        f=f.resolve();relative=f.relative_to(Path.cwd());out=args.outdir/'source_snapshot'/relative;out.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(f,out);hashes[str(relative)]=hashlib.sha256(f.read_bytes()).hexdigest()
    ctrl.select(x[:6],0.,0,0,None);ctrl.mean=None;ctrl.prior_time=None
    t=0.;ref=None;pending='initial';rows=[];cycles=[];updates=[];failure=None;done=False
    while t<60.:
        if pending:
            tick=time.perf_counter();candidate=planner.solve(pvaj_from_state(plant,x,u),t,progress.first)
            updates.append(dict(time=t,reason=pending,accepted=candidate is not None,wall_ms=1000*(time.perf_counter()-tick)))
            if candidate is not None:ref=candidate;pending=None
        action=ctrl.select(x[:6],t,progress.first,progress.phase,ref);cycles.append(ctrl.last)
        if action is None:failure='no_current_filtered_control';break
        for _ in range(4):
            a,info=ctrl.engine.filter(x[:6],t,action)
            if info['status'] or info['B']<0 or info['h_min']<0:failure='filter_domain_in_actual_plant';break
            # Reused attitude/control-encoding primitive only. Neither the
            # old MPPI policy nor its recovery/planning logic is called.
            u=plant.feedback(x,x[:3],x[3:6],a);new=plant.integrate(x,u,.005)
            achieved=(new[3:6]-x[3:6])/.005
            clear=float(plant.geometry.interval_clearances(np.array([x[:3],new[:3]]),np.array([x[3:6],new[3:6]]),np.array([t,t+.005])).min())
            if progress.update(x[:3],t,new[:3],t+.005):pending='gate_exit'
            tilt=np.rad2deg(np.arccos(np.clip(1-2*(new[7]**2+new[8]**2),-1,1)))
            rows.append([t,*x,*a,*achieved,clear,info['B'],tilt]);x=new;t+=.005
            if clear<=0:failure='executed_geometry'
            elif progress.violation:failure=progress.violation
            elif np.linalg.norm(achieved)>8.+1e-6:failure='acceleration_limit'
            elif np.linalg.norm(x[3:6])>12.+1e-6:failure='speed_limit'
            elif tilt>42.+1e-6:failure='tilt_limit'
            elif np.linalg.norm(x[10:12])>3.839724354 or abs(x[12])>3.490658504:failure='bodyrate_limit'
            if failure:break
        if failure:break
        if progress.first==7 and np.linalg.norm(x[:3]-track.goal)<.55 and np.linalg.norm(x[3:6])<1:done=True;break
        if ref is not None and ctrl.stage=='TRANSIT':
            r=ctrl.reference(ref,t,x[:6])[0]
            if np.linalg.norm(x[:3]-r[:3])>.5 or np.linalg.norm(x[3:6]-r[3:6])>2:pending=pending or 'tracking_error'
        if ctrl.last['avoiding']:pending=pending or 'obstacle_recovery'
    rows=np.asarray(rows)
    result=dict(completed=done,failure=failure or (None if done else 'timeout'),time=t,gates=progress.first,
        minimum_clearance=None if len(rows)==0 else float(rows[:,-3].min()),
        acceleration_error_max=None if len(rows)==0 else float(np.linalg.norm(rows[:,18:21]-rows[:,21:24],axis=1).max()),
        cycles=cycles,updates=updates,source_sha256=hashes,motion_rate=args.motion_rate,
        evidence='17-state empirical CTBR/motor nominal plant; paused solver clock; NOT PX4/Gazebo; controller still predicts six states')
    np.savez_compressed(args.outdir/'trace.npz',rows=rows,final_state=x)
    (args.outdir/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('cycles','updates','source_sha256')},indent=2))

if __name__=='__main__':main()
