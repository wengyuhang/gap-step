"""Evidence-based setup-failure classification; never overwrite raw trials.

A planner is first invoked at sim t=30. A vehicle that remains physically
immobile on the ground throughout t=10..29, despite an armed offboard climb
command, has not reached the frozen experiment's initial state. This is a
pre-planner simulator/actuation setup failure, not a failed racing trajectory.
Its raw attempt is retained; the protocol permits at most one setup retry.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from .run import ROOT, analyze_run, save, source_hashes


def assess(run):
    run = Path(run)
    result = json.loads((run/'result.json').read_text())
    ulog = ULog(str(run/'flight.ulg'))
    d = ulog.get_dataset('vehicle_local_position_groundtruth').data
    t = d['timestamp']*1e-6
    mask = (t>=10.) & (t<=29.)
    if mask.sum()<100 or t[mask][-1]<28.:
        return dict(setup_failure=False, reason='insufficient preparation evidence')
    positions = np.column_stack([d[k][mask] for k in ('x','y','z')])
    speeds = np.linalg.norm(np.column_stack([d[k][mask] for k in ('vx','vy','vz')]), axis=1)
    attitude = ulog.get_dataset('vehicle_attitude_groundtruth').data
    at = attitude['timestamp']*1e-6
    q = np.array([np.interp(t[mask][-1], at, attitude[k]) for k in ('q[1]','q[2]','q[3]','q[0]')])
    body_ned = positions[-1] + Rotation.from_quat(q).apply([0.,0.,-.24])
    body_enu = body_ned[[1,0,2]]*np.array([1.,1.,-1.])
    track, _ = build_track()
    error = float(np.linalg.norm(body_enu-track.start))
    displacement = float(np.linalg.norm(positions-positions[0], axis=1).max())
    status = ulog.get_dataset('vehicle_status').data
    status_mask = (status['timestamp']*1e-6>=20.) & (status['timestamp']*1e-6<=29.)
    ready = bool(status_mask.any() and np.all(status['arming_state'][status_mask]==2)
                 and np.all(status['nav_state'][status_mask]==14))
    sp = ulog.get_dataset('trajectory_setpoint').data
    sp_mask = (sp['timestamp']*1e-6>=20.) & (sp['timestamp']*1e-6<=29.)
    climb = bool(sp_mask.any() and np.all(sp['position[2]'][sp_mask]<-2.))
    before_planner = bool(result.get('solves') and min(s['time'] for s in result['solves'])>=0.)
    failure = bool(error>2. and displacement<.01 and speeds.max()<.01 and ready and climb and before_planner)
    return dict(setup_failure=failure, preparation_window_s=[10.,29.],
                preflight_position_error_m=error, preflight_truth_displacement_m=displacement,
                preflight_truth_max_speed_mps=float(speeds.max()), armed_offboard=ready,
                climb_setpoint_present=climb, planning_starts_after_preparation=before_planner,
                actual_initial_body_enu=body_enu.tolist(), required_initial_enu=track.start.tolist(),
                reason='immobile physical vehicle before first planning call' if failure else 'not this setup failure')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('batch', type=Path)
    args = p.parse_args()
    batch = args.batch.resolve()
    if json.loads((batch/'status.json').read_text())['state']!='complete':
        raise RuntimeError('Wait for the primary timed batch')
    if json.loads((batch/'protocol.json').read_text())['source_sha256'] != source_hashes():
        raise RuntimeError('Frozen sources changed')
    rows = json.loads((batch/'summary.json').read_text())
    save(batch/'adjudication_manifest.json', dict(
        adjudicator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        original_summary_sha256=hashlib.sha256((batch/'summary.json').read_bytes()).hexdigest(),
        note='Post-flight diagnosis of a failure before planning; original batch is never overwritten'))
    augmented, assessments = [], []
    for row in rows:
        if row.get('realtime_success') or row.get('infrastructure_failure'):
            augmented.append(row)
            continue
        try:
            verdict = assess(row['run'])
        except Exception as exc:
            verdict = dict(setup_failure=False, error=repr(exc))
        assessments.append(dict(run=row['run'], condition_id=row['condition_id'], **verdict))
        if not verdict['setup_failure']:
            augmented.append(row)
            continue
        augmented.append(dict(row, infrastructure_failure=True, infrastructure_adjudication=verdict))
        path = batch/'infrastructure_retry'/f'r{row["rate"]}_t{row["translation_scale"]}_a{row["rotation_scale"]}_p{row["phase_id"]}_attempt2'
        if path.exists():
            raise RuntimeError('Do not rerun an existing setup retry')
        path.parent.mkdir(exist_ok=True)
        cmd = [sys.executable, '-u', '-m', 'convex_timevarying_window.receding_gate_racing.px4_experiment',
               '--launch', '--checked-soft', '--tracking-preview', '.05', '--motion-rate', str(row['rate']),
               '--translation-scale', str(row['translation_scale']), '--rotation-scale', str(row['rotation_scale']),
               '--phase', str(row['phase']), '--outdir', str(path)]
        print('SETUP RETRY '+str(path), flush=True)
        tick = time.monotonic()
        try:
            proc = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=240)
            output = proc.stdout+'\n'+proc.stderr
        except subprocess.TimeoutExpired as exc:
            output = repr(exc)
        path.mkdir(exist_ok=True)
        (path/'console.log').write_text(output)
        fresh = {k: row[k] for k in ('method','condition_id','rate','translation_scale','rotation_scale','phase','phase_id','layout')}
        fresh.update(run=str(path), attempt=2, wall_s=time.monotonic()-tick, infrastructure_failure=False,
                     retry_reason='pre-planner immobility proven by saved ULog; raw attempt retained')
        if (path/'result.json').exists():
            r = json.loads((path/'result.json').read_text())
            fresh['infrastructure_failure'] = r.get('failure') == "RuntimeError('arm failed')"
            try:
                fresh.update(analyze_run(path))
            except Exception as exc:
                fresh.update(realtime_success=False, physical_success=False, analysis_error=repr(exc))
            if not fresh.get('realtime_success') and not fresh['infrastructure_failure']:
                try:
                    fresh['infrastructure_failure'] = assess(path)['setup_failure']
                except Exception:
                    pass
        else:
            fresh.update(realtime_success=False, physical_success=False, failure='missing_retry_result')
        augmented.append(fresh)
        save(batch/'adjudication.json', assessments)
        save(batch/'adjudicated_summary.json', augmented)
        print('RETRY RESULT '+json.dumps(fresh), flush=True)
    save(batch/'adjudication.json', assessments)
    save(batch/'adjudicated_summary.json', augmented)
    print('DONE: raw summary.json unchanged; adjudicated_summary.json retains both attempts', flush=True)


if __name__ == '__main__':
    main()
