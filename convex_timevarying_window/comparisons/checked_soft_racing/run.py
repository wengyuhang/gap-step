"""Run the union of existing dynamic PX4 matrices, preserving every attempt."""
import argparse
import contextlib
import hashlib
import io
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

from convex_timevarying_window.receding_gate_racing.analyze import analyze
from convex_timevarying_window.receding_gate_racing.localization_audit import audit

ROOT = Path(__file__).resolve().parents[3]


def conditions():
    items = []
    for r in (1, 3, 6, 10):
        for t in (1, 2, 4):
            for p in (0, 1):
                items.append((r, t, 1, p))
    for r, t, a in [(r, a, a) for r in (1, 10) for a in (1, 2, 4)] + [(10, 4, 1), (10, 1, 4)]:
        for p in (0, 1):
            if (r, t, a, p) not in items:
                items.append((r, t, a, p))
    items.extend((r, 1, 1, 2) for r in (1, 3, 6, 10))
    assert len(items) == len(set(items)) == 38
    return items


def source_hashes():
    paths = []
    for directory in ('receding_gate_racing', 'continuation_racing', 'checked_soft_racing'):
        paths.extend((ROOT/'convex_timevarying_window'/directory).glob('*.py'))
    paths += [Path(__file__), Path(__file__).with_name('PROTOCOL.md')]
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def analyze_run(path):
    path = Path(path)
    r = json.loads((path/'result.json').read_text())
    with contextlib.redirect_stdout(io.StringIO()) as captured:
        check = analyze(path)
        truth = audit(path)
    (path/'analysis.log').write_text(captured.getvalue())
    low, high = np.array(truth['world_body_position_min']), np.array(truth['world_body_position_max'])
    room = bool(np.all(low >= np.array(r['config']['workspace_lower'])-.01)
                and np.all(high <= np.array(r['config']['workspace_upper'])+.01))
    clearance = min(g['minimum_sampled_sphere_sleeve_clearance_m'] for g in truth['groundtruth_sampled_sphere_sleeve_clearance'])
    physical = bool(check['completed'] and check['contact_free'] and check['ordered_passages_passed'] and room and clearance >= 0.)
    data = np.genfromtxt(path/'telemetry.csv', delimiter=',', names=True)
    speed = np.sqrt(data['vx']**2+data['vy']**2+data['vz']**2)
    return dict(physical_success=physical, realtime_success=physical and check['deadline_passed'],
                flight_time_s=r['flight_time_s'], last_time_s=r['last_time_s'],
                gates=sum(g is not None for g in check['ordered']), collision=r['collision_detected'],
                failure=r['failure'], room_passed=room, min_clearance_m=clearance, cycle_ms=check['cycle_ms'],
                accepted_replans=check['accepted_replans'], total_replans=check['total_replans'],
                mean_speed_mps=float(np.trapz(speed, data['t'])/(data['t'][-1]-data['t'][0])),
                max_speed_mps=float(speed.max()))


def save(path, value):
    pending = path.with_suffix('.pending')
    pending.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')
    pending.replace(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=args.resume)
    current_hashes = source_hashes()
    if args.resume:
        protocol = json.loads((out/'protocol.json').read_text())
        if protocol['source_sha256'] != current_hashes:
            raise RuntimeError('Frozen sources changed; do not mix versions in this batch')
        rows = json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else []
    else:
        rows = []
        protocol = dict(method='checked_soft_racing', source_sha256=current_hashes,
                        conditions=conditions(), phases_rad=[0., 2*math.pi/3, 4*math.pi/3],
                        tracking_preview_s=.05, max_flight_time_s=60., soft_weight=1., soft_buffer_m=.15,
                        python=platform.python_version(), numpy=np.__version__,
                        platform=platform.platform(), environment={k: os.environ.get(k) for k in ('OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','GZ_SHOW_GUI')},
                        comparison='historical frozen baselines; not simultaneous randomized repetitions')
        save(out/'protocol.json', protocol)
        for relative in current_hashes:
            destination = out/'source_snapshot'/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/relative, destination)
    completed = {tuple(r['condition_id']) for r in rows if not r['infrastructure_failure'] or r['attempt'] == 2}
    for case, (rate, translation, rotation, phase_id) in enumerate(conditions()):
        key = (rate, translation, rotation, phase_id)
        if key in completed:
            continue
        prior = [r['attempt'] for r in rows if tuple(r['condition_id']) == key]
        for attempt in range(max(prior, default=0)+1, 3):
            if current_hashes != source_hashes():
                raise RuntimeError('Frozen sources changed during experiment')
            path = out/f'r{rate}_t{translation}_a{rotation}_p{phase_id}_attempt{attempt}'
            phase = phase_id*2*math.pi/3
            command = [sys.executable, '-u', '-m', 'convex_timevarying_window.receding_gate_racing.px4_experiment',
                       '--launch', '--checked-soft', '--tracking-preview', '.05', '--motion-rate', str(rate),
                       '--translation-scale', str(translation), '--rotation-scale', str(rotation), '--phase', str(phase),
                       '--max-flight-time', '60', '--outdir', str(path)]
            save(out/'status.json', dict(state='running', case=case, total=38, condition_id=key,
                                       attempt=attempt, run=str(path), pid=os.getpid(), started=time.time()))
            print(f'START {case+1}/38 {path.name}', flush=True)
            began = time.monotonic()
            timed_out = False
            try:
                proc = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, timeout=240)
                output, returncode = proc.stdout+'\n'+proc.stderr, proc.returncode
            except subprocess.TimeoutExpired as exc:
                timed_out = True
                output, returncode = repr(exc), None
            path.mkdir(exist_ok=True)
            (path/'console.log').write_text(output)
            row = dict(method='checked_soft_racing', condition_id=key, rate=rate,
                       translation_scale=translation, rotation_scale=rotation, phase=phase,
                       phase_id=phase_id, layout=0., attempt=attempt, run=str(path),
                       wall_s=time.monotonic()-began, returncode=returncode)
            if not (path/'result.json').exists():
                from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import parse_contacts
                collided = any(parse_contacts(f)['contact_pair_count'] > 0 for f in path.glob('gate_*_contacts.pbtxt'))
                # Missing flight output after flight begins is not automatically
                # infrastructure. Never retry an observed contact or timeout flight.
                started_flight = (path/'solves_live.jsonl').exists()
                infra = not collided and not started_flight
                row.update(infrastructure_failure=infra, physical_success=False, realtime_success=False,
                           collision=collided, failure='process_timeout' if timed_out else 'missing_result',
                           flight_time_s=None, cycle_ms=None)
            else:
                result = json.loads((path/'result.json').read_text())
                infra = result.get('failure') == "RuntimeError('arm failed')"
                row.update(infrastructure_failure=infra)
                if not infra:
                    try:
                        row.update(analyze_run(path))
                    except Exception as exc:
                        row.update(physical_success=False, realtime_success=False,
                                   failure=result['failure'], analysis_error=repr(exc),
                                   flight_time_s=result.get('flight_time_s'), last_time_s=result.get('last_time_s'))
            rows.append(row)
            save(out/'summary.json', rows)
            print('RESULT '+json.dumps(row, ensure_ascii=False), flush=True)
            if not row['infrastructure_failure']:
                break
    save(out/'status.json', dict(state='complete', conditions=38, attempts=len(rows), finished=time.time()))
    print('DONE '+str(out), flush=True)


if __name__ == '__main__':
    main()
