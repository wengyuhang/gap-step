"""Run the unchanged MPC tracker on the complete existing 38-case union."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

import numpy as np

from convex_timevarying_window.comparisons.checked_soft_racing.run import (
    ROOT, analyze_run, conditions, save, source_hashes as planning_hashes)
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from convex_timevarying_window.gs_mppi.direct_ctbr.compare_targets import body_dynamics
from convex_timevarying_window.checked_soft_mpc_racing.mpc import Config


def source_hashes():
    result = planning_hashes()
    paths = list((ROOT/'convex_timevarying_window/checked_soft_mpc_racing').glob('*.py'))
    paths += [Path(__file__), Path(__file__).with_name('PROTOCOL.md')]
    paths += [ROOT/'convex_timevarying_window/dynamic_gate_mppi_v3'/name for name in
              ('ctbr_probe.py', 'ctbr.py', 'rollout.py', 'state_consistency.py')]
    paths += [ROOT/'复现/waypoint_mppi/gp_px4.py',
              ROOT/'convex_timevarying_window/comparisons/checked_soft_racing/adjudicate.py',
              ROOT/'convex_timevarying_window/receding_gate_racing/localization_audit.py',
              ROOT/'convex_dynamic_seven_window_gazebo/px4_togt_nmpc_experiment.py',
              ROOT/'convex_dynamic_seven_window_gazebo/online_safe_mppi_px4_experiment.py',
              ROOT/'convex_dynamic_seven_window_gazebo/convex_seven_dynamic_px4_togt.sdf',
              ROOT/'convex_dynamic_seven_window_gazebo/x500_togt/experiment.py']
    for path in paths: result[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def timing(values):
    a = np.asarray(values, dtype=float)
    return dict(count=len(a), mean=float(a.mean()), p99=float(np.percentile(a,99)), max=float(a.max()),
                over_20ms=int(np.sum(a>=20)), over_100ms=int(np.sum(a>=100))) if len(a) else None


def control_audit(result, execution):
    """Include failed calls and preparation calls; report racing subset too."""
    calls = result.get('mpc_cycles', [])
    times = timing([r['cycle_ms'] for r in calls])
    race = [r for r in calls if r['time']>=0]
    race_times = timing([r['cycle_ms'] for r in race])
    send = timing(execution[:, -1]) if len(execution) else None
    okay = bool(times and send and times['max']<100 and send['max']<100)
    return dict(mpc_cycle_ms=times, racing_mpc_cycle_ms=race_times, sender_cycle_ms=send,
        controller_deadline_passed=okay, mpc_failed_calls=sum('failure' in r for r in calls),
        inexact_qp_steps=sum(r.get('inexact_qp_steps',0) for r in calls),
        actual_racing_mpc_hz=float((len(race)-1)/(race[-1]['time']-race[0]['time'])) if len(race)>1 else None)


def inspect(path):
    r = json.loads((path/'result.json').read_text())
    out = dict(infrastructure_failure=r.get('failure')=="RuntimeError('arm failed')",
               physical_success=False, realtime_success=False, flight_time_s=r.get('flight_time_s'),
               last_time_s=r.get('last_time_s'), failure=r.get('failure'), collision=r.get('collision_detected',False))
    if out['infrastructure_failure']: return out
    try:
        out.update(analyze_run(path))
        out.update(control_audit(r, np.load(path/'execution.npz')['rows']))
        out['realtime_success'] &= out['controller_deadline_passed']
        data = np.genfromtxt(path/'telemetry.csv',delimiter=',',names=True)
        contacts = [c['first_contact_sim_time_s']-30 for c in r['contacts'] if c['first_contact_sim_time_s'] is not None]
        cutoff = min(contacts) if contacts else r['last_time_s']
        errors = data['error'][data['t'] <= cutoff]
        out.update(tracking_error_before_contact_max_m=float(np.max(errors)),
                   tracking_error_before_contact_rms_m=float(np.sqrt(np.mean(errors**2))),
                   tracking_statistics_end_s=cutoff)
        # Entire flight dynamics for the formal diagnostic; before-contact
        # dynamics are an additional diagnostic, never a substitute for it.
        dynamics = body_dynamics(path,r['last_time_s'])
        out.update(dynamics=dynamics, strict_dynamics_success=bool(out['realtime_success'] and dynamics['satisfies_direct_dynamics_limits']))
        if contacts: out['dynamics_before_contact'] = body_dynamics(path,cutoff)
    except Exception as exc:
        out.update(physical_success=False,realtime_success=False,analysis_error=repr(exc))
    try:
        out['preparation_assessment'] = assess(path)
        if out['preparation_assessment']['setup_failure']: out['infrastructure_failure'] = True
    except Exception as exc:
        out['preparation_assessment'] = dict(setup_failure=False,error=repr(exc))
    return out


def condition_rows(rows):
    """Select the permitted retry only after an infrastructure first attempt."""
    grouped = {}
    for r in rows: grouped.setdefault(tuple(r['condition_id']),[]).append(r)
    selected = []
    for attempts in grouped.values():
        attempts = sorted(attempts,key=lambda r:r['attempt'])
        if len(attempts)>2 or (len(attempts)>1 and not attempts[0]['infrastructure_failure']):
            raise ValueError('Invalid algorithm retry; do not select by outcome')
        selected.append(attempts[-1])
    return selected


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--resume',action='store_true'); a=parser.parse_args()
    out=a.outdir.resolve(); out.mkdir(parents=True,exist_ok=a.resume)
    hashes=source_hashes()
    if a.resume:
        protocol=json.loads((out/'protocol.json').read_text())
        if protocol['source_sha256']!=hashes: raise RuntimeError('Frozen sources changed')
        rows=json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else []
    else:
        development=ROOT/'convex_timevarying_window/checked_soft_mpc_racing/results/hard_mpc_v6_retry_20261006/result.json'
        dev=json.loads(development.read_text())
        for name in ('mpc.py','px4.py'):
            key='convex_timevarying_window/checked_soft_mpc_racing/'+name
            if hashes[key]!=dev['source_sha256'][key]: raise RuntimeError('Controller differs from declared development version')
        protocol=dict(method='checked_soft_mpc_racing',source_sha256=hashes,conditions=conditions(),
            mpc_config=asdict(Config()),max_flight_time_s=60.,development_reference=str(development),
            python=platform.python_version(),numpy=np.__version__,platform=platform.platform(),
            environment={k:os.environ.get(k) for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','GZ_SHOW_GUI')},
            created=time.time(),comparison='Frozen version, historical baselines, no per-case tuning')
        save(out/'protocol.json',protocol); rows=[]
        for relative in hashes:
            dst=out/'source_snapshot'/relative; dst.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(ROOT/relative,dst)
    for case,key in enumerate(conditions()):
        prior=sorted([r for r in rows if tuple(r['condition_id'])==key],key=lambda r:r['attempt'])
        if prior and (not prior[-1]['infrastructure_failure'] or prior[-1]['attempt']==2): continue
        for attempt in range(max((r['attempt'] for r in prior),default=0)+1,3):
            if source_hashes()!=hashes: raise RuntimeError('Frozen sources changed during matrix')
            rate,tr,rot,pid=key; phase=pid*2*math.pi/3
            path=out/f'r{rate}_t{tr}_a{rot}_p{pid}_attempt{attempt}'
            status=dict(state='running',case=case+1,total=38,condition_id=key,attempt=attempt,
                        pid=os.getpid(),run=str(path),started=time.time())
            save(out/'status.json',status);print(f'START {case+1}/38 {path.name}',flush=True)
            begin=time.monotonic();code=None;timed_out=False
            if not path.exists():
                command=[sys.executable,'-u','-m','convex_timevarying_window.checked_soft_mpc_racing.px4',
                    '--outdir',str(path),'--motion-rate',str(rate),'--translation-scale',str(tr),
                    '--rotation-scale',str(rot),'--phase',str(phase),'--max-flight-time','60']
                try:
                    process=subprocess.run(command,cwd=ROOT,capture_output=True,text=True,timeout=240)
                    output=process.stdout+'\n'+process.stderr;code=process.returncode
                except subprocess.TimeoutExpired as exc: output=repr(exc);timed_out=True
                path.mkdir(exist_ok=True);(path/'console.log').write_text(output)
            else:
                output=(path/'console.log').read_text() if (path/'console.log').exists() else ''
            row=dict(method='checked_soft_mpc_racing',condition_id=key,rate=rate,translation_scale=tr,
                rotation_scale=rot,phase=phase,phase_id=pid,layout=0.,attempt=attempt,run=str(path),
                wall_s=time.monotonic()-begin,returncode=code)
            if (path/'result.json').exists(): row.update(inspect(path))
            else:
                # Do not label arbitrary coding errors as infrastructure.
                started=(path/'solves_live.jsonl').exists()
                infra=not started and any(s in output for s in ('PX4 startup failed','no heartbeat','missing initial telemetry'))
                row.update(physical_success=False,realtime_success=False,infrastructure_failure=infra,
                           flight_time_s=None,failure='process_timeout' if timed_out else 'missing_result')
            rows.append(row);save(out/'summary.json',rows)
            save(out/'first_attempt_summary.json',[r for r in rows if r['attempt']==1])
            chosen=condition_rows(rows);save(out/'condition_summary.json',chosen)
            print('RESULT '+json.dumps(row,ensure_ascii=False),flush=True)
            if not row['infrastructure_failure']: break
    save(out/'status.json',dict(state='complete',conditions=len(condition_rows(rows)),attempts=len(rows),finished=time.time()))
    print('DONE '+str(out),flush=True)


if __name__=='__main__':main()
