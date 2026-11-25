"""Frozen-reference numerical validation of CTBR execution-time alignment."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from .aligned_controller import MPC
from convex_timevarying_window.single_gate_stop_racing.px4_model import native_dynamics, initial_state, DT
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state
from convex_timevarying_window.global_gate_minco_probe.phase_descent import load_plan
from convex_timevarying_window.global_gate_minco_probe.flight import Reference


def main():
    p=argparse.ArgumentParser(); p.add_argument('--plan-dir',type=Path,required=True)
    p.add_argument('--outdir',type=Path,required=True); p.add_argument('--delay',type=float,default=.024)
    p.add_argument('--control-period',type=float,default=.02)
    a=p.parse_args(); a.outdir.mkdir(parents=True,exist_ok=False)
    plan=load_plan(json.loads((a.plan_dir/'result.json').read_text())); ref=Reference(plan.sample(0),plan); ref.advance(0.)
    mpc=MPC(); tick=native_dynamics()[0]; x=initial_state(hover_state(plan.sample(0)))
    rows=[]; pending=[]; applied=mpc.previous.copy(); next_control=0.; failure=None; t=0.
    try:
        for k in range(round((plan.end_time+1)/DT)):
            t=k*DT
            if t>=next_control-1e-9:
                cmd=mpc.command(x[:17],t,ref.sample,execution_time=t+a.delay)
                pending.append((t+a.delay,cmd.copy()))
                mpc.command_sent(cmd,t+a.delay-DT)
                next_control+=a.control_period
            while pending and pending[0][0]<=t+1e-9: _,applied=pending.pop(0)
            error=np.linalg.norm(x[:3]-ref.sample([t])[0,:3])
            rows.append([t,*x,*applied,error])
            if error>3: raise RuntimeError('diagnostic_stop_error_over_3m')
            x=np.asarray(tick(x,applied,DT)).ravel()
    except Exception as exc: failure=repr(exc)
    rows=np.asarray(rows)
    result=dict(completed=failure is None,failure=failure,stop_s=t,position_error_max_m=float(rows[:,-1].max()),
        position_error_rms_m=float(np.sqrt(np.mean(rows[:,-1]**2))),delay_s=a.delay,control_period_s=a.control_period,
        cycle_ms_p50_p95_max=np.percentile([r['cycle_ms'] for r in mpc.logs],[50,95,100]).tolist(),
        reference_sha256=hashlib.sha256((a.plan_dir/'trajectory.npz').read_bytes()).hexdigest(),
        controller=mpc.metadata,evidence='Numerical model only, not Gazebo or collision/safety evidence')
    np.savez_compressed(a.outdir/'execution.npz',rows=rows)
    (a.outdir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    (a.outdir/'mpc.json').write_text(json.dumps(mpc.logs))
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__': main()
