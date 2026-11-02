"""Controller validation against the existing independent PX4 execution model.

Not Gazebo, not a collision test. Always preserve failure and timing evidence.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import numpy as np
from .controller import MPC
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state
from convex_timevarying_window.single_gate_stop_racing.px4_model import native_dynamics, initial_state, DT


def reference(times):
    t=np.asarray(times); w=.7; s=np.sin(w*t); c=np.cos(w*t)
    p=np.c_[2*(1-c),s**3,3+.2*(1-c)]
    v=np.c_[2*w*s,3*w*s*s*c,.2*w*s]
    a=np.c_[2*w*w*c,3*w*w*(2*s*c*c-s**3),.2*w*w*c]
    return np.c_[p,v,a]


def run_case(mpc,tick,name,seconds,ref=None):
    if ref is None:
        ref=reference if name=='smooth_turn' else lambda ts: np.tile([0,0,3,0,0,0,0,0,0],(len(ts),1))
    x=initial_state(hover_state(ref([0.])[0,:3])); rows=[]; failure=None
    if name=='hover_recovery': x[:3]+=[.15,-.1,.1]
    mpc.controls=None; mpc.last_time=None; mpc.logs.clear()
    try:
        next_control=0.; command=mpc.previous.copy()
        for k in range(round(seconds/DT)):
            t=k*DT
            if t>=next_control-1e-9:
                command=mpc.command(x[:17],t,ref); next_control+=mpc.cfg.update_period
                rows.append([t,*x[:13],*command,float(np.linalg.norm(x[:3]-ref([t])[0,:3]))])
            # The digital PX4 filters are defined at this exact 4ms clock.
            x=np.asarray(tick(x,command,DT)).ravel()
            if not np.isfinite(x).all() or np.linalg.norm(x[:3]-ref([t+DT])[0,:3])>3:
                raise RuntimeError('execution_model_tracking_diverged')
    except Exception as exc: failure=repr(exc)
    q=np.asarray(rows); cycles=np.array([r['cycle_ms'] for r in mpc.logs])
    result=dict(case=name,failure=failure,completed=failure is None,duration_s=float(t+DT),
        plant_step_s=DT,control_period_target_s=mpc.cfg.update_period,
        position_error_max_m=float(q[:,-1].max()) if len(q) else None,
        position_error_final_m=float(q[-1,-1]) if len(q) else None,
        position_error_rms_m=float(np.sqrt(np.mean(q[:,-1]**2))) if len(q) else None,
        cycle_median_ms=float(np.median(cycles)),cycle_p95_ms=float(np.percentile(cycles,95)),
        cycle_max_ms=float(cycles.max()),cycles_over_10ms=int(np.sum(cycles>10)),
        cycles_over_100ms=int(np.sum(cycles>100)),
        evidence='closed-loop numerical PX4 execution model; NOT Gazebo or safety evidence')
    return result,q,list(mpc.logs)


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--plan-dir',type=Path)
    a=p.parse_args();a.outdir.mkdir(parents=True,exist_ok=False)
    began=time.perf_counter();mpc=MPC();build_s=time.perf_counter()-began
    tick,*_=native_dynamics();results=[]
    cases=[('hover_recovery',4.,None),('smooth_turn',12.,None)]
    if a.plan_dir:
        from convex_timevarying_window.global_gate_minco_probe.phase_descent import load_plan
        from convex_timevarying_window.global_gate_minco_probe.flight import Reference
        plan=load_plan(json.loads((a.plan_dir/'result.json').read_text()))
        ref=Reference(plan.sample(0),plan);ref.advance(0.)
        cases=[('frozen_global_reference',plan.end_time+1.,ref.sample)]
    for name,seconds,ref in cases:
        result,rows,logs=run_case(mpc,tick,name,seconds,ref);results.append(result)
        np.savez_compressed(a.outdir/(name+'.npz'),rows=rows)
        (a.outdir/(name+'_mpc.json')).write_text(json.dumps(logs,indent=2)+'\n')
        print(json.dumps(result),flush=True)
    (a.outdir/'result.json').write_text(json.dumps(dict(build_s=build_s,controller=mpc.metadata,
        config=asdict(mpc.cfg),results=results),indent=2)+'\n')
    if not all(r['completed'] for r in results): raise SystemExit(1)


if __name__=='__main__': main()
