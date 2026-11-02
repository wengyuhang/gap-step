"""PX4/Gazebo hover and smooth local motion; NOT a gate-racing experiment."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from .controller import MPC
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case, launch
from convex_timevarying_window.single_gate_stop_racing.metric_session import Session
from convex_timevarying_window.dynamic_gate_mppi_v3 import ctbr_probe
from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import set_parameter
from convex_timevarying_window.simple_two_gate_soft_only_mpc.px4 import PARAMETERS


def local_reference(times,start):
    t=np.asarray(times); z=np.clip((t-2.)/8.,0.,1.)
    # Smooth out-and-back excursion, zero V/A at both ends; no gate decisions.
    b=64*z**3*(1-z)**3
    d=192*z**2*(1-z)**2*(1-2*z)/8
    dd=(384*z*(1-z)*(1-5*z+5*z*z))/64
    active=(t>2)&(t<10);d*=active;dd*=active
    direction=np.array([-2.,1.,.2])
    return np.c_[start+b[:,None]*direction,d[:,None]*direction,dd[:,None]*direction]


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    a=p.parse_args();out=a.outdir.resolve()
    if out.exists():raise FileExistsError(out)
    track,_=build_case(Case(4.,4.,0.)); m=MPC()
    ref=lambda ts:local_reference(ts,track.start)
    m.command(hover_state(track.start),-1.,ref);m.controls=None;m.last_time=None;m.logs.clear()
    ctbr_probe.launch_variant=lambda track,assets,out:launch(track,assets,out,camera_helper=False)
    s=Session(out,track,consistent_velocity=False)
    for key,value in PARAMETERS.items():set_parameter(key,value)
    failure=None;rows=[]
    try:
        s.prepare()
        s.feedback=lambda x,t:m.command(x,t,ref)
        while s.read()[0]<12.:
            t,x,age=s.read()
            if s.failure:raise RuntimeError(s.failure)
            err=float(np.linalg.norm(x[:3]-ref([t])[0,:3]))
            rows.append([t,*x,err])
            if err>2. or age>.1:raise RuntimeError('smoke_tracking_or_telemetry_envelope')
            time.sleep(.01)
    except Exception as exc:failure=repr(exc)
    finally:
        s.close()
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000',
            '--req','pause: true'],capture_output=True,timeout=5)
    np.savez_compressed(out/'trace.npz',rows=np.array(rows))
    ms=np.array([l['cycle_ms'] for l in m.logs]);err=np.array(rows)[:,-1]
    result=dict(failure=failure,completed=failure is None,controller=m.metadata,
        max_tracking_error_m=float(err.max()),rms_tracking_error_m=float(np.sqrt(np.mean(err**2))),
        cycle_median_ms=float(np.median(ms)),cycle_p95_ms=float(np.percentile(ms,95)),
        cycle_max_ms=float(ms.max()),mpc_calls=len(ms),
        ground_truth_feedback_enabled=False,evidence='PX4/Gazebo local tracking only; not racing/collision validation')
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'mpc.json').write_text(json.dumps(m.logs,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    if failure:raise SystemExit(1)


if __name__=='__main__':main()
