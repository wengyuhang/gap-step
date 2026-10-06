"""Frozen amplitude sweep. No tuning or algorithm mutation between conditions."""
import contextlib
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import argparse
import shutil
import numpy as np
import scipy
from convex_timevarying_window.receding_gate_racing.analyze import analyze
from convex_timevarying_window.receding_gate_racing.localization_audit import audit


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True)
    p.add_argument('--continuation',action='store_true');p.add_argument('--tracking-preview',type=float,default=0.)
    p.add_argument('--checked',action='store_true');p.add_argument('--anticipatory',action='store_true')
    p.add_argument('--center-motion-grid',action='store_true',
                   help='rates 1/3/6/10 x translation amplitudes 1/2/4; fixed rotation amplitude')
    a=p.parse_args()
    if a.checked:a.continuation=True
    if a.anticipatory and not a.checked:p.error('anticipatory requires checked')
    a.outdir.mkdir(parents=True,exist_ok=False)
    source=Path('convex_timevarying_window/receding_gate_racing')
    hashes={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in source.glob('*.py')}
    extra=Path('convex_timevarying_window/continuation_racing/planner.py')
    extra_hash=hashlib.sha256(extra.read_bytes()).hexdigest() if a.continuation else None
    guard_source=Path('convex_timevarying_window/continuation_racing/guard.py')
    guard_hash=hashlib.sha256(guard_source.read_bytes()).hexdigest() if a.checked else None
    snapshot=a.outdir/'source_snapshot';snapshot.mkdir()
    for f in source.glob('*.py'):shutil.copyfile(f,snapshot/f.name)
    if a.continuation:shutil.copyfile(extra,snapshot/'continuation_planner.py')
    if a.checked:shutil.copyfile(guard_source,snapshot/'continuation_guard.py')
    if a.center_motion_grid:
        conditions=[(r,t,1,'translation') for r in (1,3,6,10) for t in (1,2,4)]
    else:
        conditions=[(r,t,t,'joint') for r in (1,10) for t in (1,2,4)]
        conditions += [(10,4,1,'translation'),(10,1,4,'rotation')]
    phases=[0,2*math.pi/3]
    protocol=dict(conditions=conditions,phases=phases,
                  design='center_motion_grid' if a.center_motion_grid else 'original_amplitude_matrix',
                  runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  source_sha256=hashes,
                  python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,max_flight_time_s=60,
                  continuation=a.continuation,continuation_source_sha256=extra_hash,tracking_preview_s=a.tracking_preview,
                  checked=a.checked,anticipatory=a.anticipatory,guard_source_sha256=guard_hash)
    (a.outdir/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[]
    for rate,translation,rotation,group in conditions:
        for phase_id,phase in enumerate(phases):
            for attempt in (1,2):
                assert hashes=={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in source.glob('*.py')}
                if a.continuation:assert extra_hash==hashlib.sha256(extra.read_bytes()).hexdigest()
                if a.checked:assert guard_hash==hashlib.sha256(guard_source.read_bytes()).hexdigest()
                path=a.outdir/f'r{rate}_t{translation}_a{rotation}_p{phase_id}_attempt{attempt}'
                cmd=[sys.executable,'-u','-m','convex_timevarying_window.receding_gate_racing.px4_experiment',
                     '--launch','--local-sweep','--motion-rate',str(rate),'--translation-scale',str(translation),
                     '--rotation-scale',str(rotation),'--phase',str(phase),'--outdir',str(path)]
                if a.continuation:cmd.extend(['--continuation','--tracking-preview',str(a.tracking_preview)])
                if a.checked:cmd.append('--checked')
                if a.anticipatory:cmd.append('--anticipatory')
                print('START',path.name,flush=True);tick=time.monotonic()
                timeout=False
                try:
                    proc=subprocess.run(cmd,text=True,capture_output=True,timeout=240)
                    output=proc.stdout+'\n'+proc.stderr
                except subprocess.TimeoutExpired as exc:
                    timeout=True;output=repr(exc)
                path.mkdir(exist_ok=True);(path/'console.log').write_text(output)
                row=dict(rate=rate,translation_scale=translation,rotation_scale=rotation,group=group,
                         phase=phase,phase_id=phase_id,attempt=attempt,run=str(path.resolve()),wall_s=time.monotonic()-tick)
                if not (path/'result.json').exists():
                    from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import parse_contacts
                    contacts=[parse_contacts(f) for f in path.glob('gate_*_contacts.pbtxt')]
                    collided=any(c['contact_pair_count']>0 for c in contacts)
                    # A timeout after contact is an algorithm failure, not a
                    # fresh infrastructure retry. Keep incomplete timing unknown.
                    row.update(infrastructure_failure=not collided,failure='collision_then_process_timeout' if collided else ('process_timeout' if timeout else 'missing_result'))
                    if collided:row.update(physical_success=False,realtime_success=False,collision=True,
                                            flight_time_s=None,cycle_ms=None,analysis_error='Incomplete flight log after collision')
                else:
                    r=json.loads((path/'result.json').read_text())
                    infra=r.get('failure')=="RuntimeError('arm failed')"
                    row.update(infrastructure_failure=infra)
                    if not infra:
                        try:
                            with contextlib.redirect_stdout(io.StringIO()) as captured:
                                checked=analyze(path);truth=audit(path)
                            (path/'analysis.log').write_text(captured.getvalue())
                            low=np.array(truth['world_body_position_min']);high=np.array(truth['world_body_position_max'])
                            room=bool(np.all(low>=np.array(r['config']['workspace_lower'])-.01) and np.all(high<=np.array(r['config']['workspace_upper'])+.01))
                            clearance=min(g['minimum_sampled_sphere_sleeve_clearance_m'] for g in truth['groundtruth_sampled_sphere_sleeve_clearance'])
                            physical=bool(checked['completed'] and checked['contact_free'] and checked['ordered_passages_passed'] and room and clearance>=0)
                            data=np.genfromtxt(path/'telemetry.csv',delimiter=',',names=True)
                            speed=np.sqrt(data['vx']**2+data['vy']**2+data['vz']**2)
                            row.update(physical_success=physical,realtime_success=physical and checked['deadline_passed'],
                                       flight_time_s=r['flight_time_s'],last_time_s=r['last_time_s'],
                                       gates=sum(g is not None for g in checked['ordered']),collision=r['collision_detected'],
                                       failure=r['failure'],room_passed=room,min_clearance_m=clearance,cycle_ms=checked['cycle_ms'],
                                       accepted_replans=checked['accepted_replans'],total_replans=checked['total_replans'],
                                       mean_speed_mps=float(np.trapz(speed,data['t'])/(data['t'][-1]-data['t'][0])),max_speed_mps=float(speed.max()))
                        except Exception as exc:
                            row.update(physical_success=False,realtime_success=False,failure=r['failure'],analysis_error=repr(exc))
                rows.append(row);(a.outdir/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
                print('RESULT',json.dumps(row),flush=True)
                if not row['infrastructure_failure']:break
    print('DONE',a.outdir,flush=True)


if __name__=='__main__':main()
