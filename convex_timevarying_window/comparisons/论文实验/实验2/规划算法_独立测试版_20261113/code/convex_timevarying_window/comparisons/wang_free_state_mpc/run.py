"""Frozen 38-condition rerun; flight code remains exactly event-v4."""
import argparse
from dataclasses import asdict, replace
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

from convex_timevarying_window.comparisons.checked_soft_racing.run import ROOT, conditions, save
from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import inspect, condition_rows
from convex_timevarying_window.wang_free_state_mpc.experiment import source_hashes as flight_hashes
from convex_timevarying_window.wang_free_state_mpc.passage_audit import audit_run
from convex_timevarying_window.wang_free_state_mpc.planner import Config as PlanningConfig
from convex_timevarying_window.checked_soft_mpc_racing.mpc import Config as MPCConfig

DEVELOPMENT = ROOT/'convex_timevarying_window/wang_free_state_mpc/results/event_v4_20261007/protocol.json'


def source_hashes():
    result = flight_hashes()
    expected = json.loads(DEVELOPMENT.read_text())['source_sha256']
    if result != expected:
        raise RuntimeError('Flight/dependency sources differ from event-v4 development snapshot')
    for path in (Path(__file__), Path(__file__).with_name('PROTOCOL.md')):
        result[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    return result


def apply_passage(row, passage):
    row = dict(row)
    row.update(legacy_physical_success=row['physical_success'],
               legacy_realtime_success=row['realtime_success'],
               single_passage_success=bool(passage['passed']),
               passage_violations=passage['violations'])
    for key in ('physical_success', 'realtime_success', 'strict_dynamics_success'):
        if key in row:
            row[key] = bool(row[key] and passage['passed'])
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    out = args.outdir.resolve()
    hashes = source_hashes()
    matrix = conditions()
    assert len(matrix) == len(set(matrix)) == 38
    out.mkdir(parents=True, exist_ok=args.resume)
    if args.resume:
        protocol = json.loads((out/'protocol.json').read_text())
        if protocol['source_sha256'] != hashes:
            raise RuntimeError('Frozen matrix sources changed')
        rows = json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else []
        condition_rows(rows)  # Reject an invalid retry history before continuing.
    else:
        protocol = dict(method='wang_free_state_mpc_event_v4', source_sha256=hashes,
            development_reference=str(DEVELOPMENT), conditions=matrix, condensed=True,
            planning_config=asdict(replace(PlanningConfig(), free_points=True,
                workspace_lower=(-28., -30., .65), workspace_upper=(28., 22., 12.))),
            mpc_config=asdict(MPCConfig()), max_flight_time_s=60., process_timeout_s=240,
            created=time.time(), python=platform.python_version(), numpy=np.__version__,
            platform=platform.platform(),
            environment={k: os.environ.get(k) for k in
                         ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'GZ_SHOW_GUI')},
            acceptance='Legacy physical+realtime AND independent exactly-once/body-exit truth audit',
            nominal_dynamics='Diagnostic only', retry='Once, independently confirmed infrastructure only')
        save(out/'protocol.json', protocol)
        rows = []
        for relative in hashes:
            dest = out/'source_snapshot'/relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(ROOT/relative, dest)
    for case, key in enumerate(matrix):
        prior = sorted((r for r in rows if tuple(r['condition_id']) == key), key=lambda r: r['attempt'])
        if prior and (not prior[-1]['infrastructure_failure'] or prior[-1]['attempt'] == 2):
            continue
        for attempt in range(max((r['attempt'] for r in prior), default=0)+1, 3):
            if source_hashes() != hashes:
                raise RuntimeError('Frozen sources changed during matrix')
            rate, tr, rot, phase_id = key
            phase = phase_id*2*math.pi/3
            path = out/f'r{rate}_t{tr}_a{rot}_p{phase_id}_attempt{attempt}'
            save(out/'status.json', dict(state='running', case=case+1, total=len(matrix),
                condition_id=key, attempt=attempt, pid=os.getpid(), run=str(path), started=time.time()))
            print(f'START {case+1}/{len(matrix)} {path.name}', flush=True)
            began = time.monotonic()
            code, timed_out = None, False
            recovered = path.exists()
            if not recovered:
                command = [sys.executable, '-u', '-m', 'convex_timevarying_window.wang_free_state_mpc.px4',
                    '--condensed', '--outdir', str(path), '--motion-rate', str(rate),
                    '--translation-scale', str(tr), '--rotation-scale', str(rot),
                    '--phase', str(phase), '--max-flight-time', '60']
                try:
                    process = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=240)
                    output, code = process.stdout+'\n'+process.stderr, process.returncode
                except subprocess.TimeoutExpired as exc:
                    output, timed_out = repr(exc), True
                path.mkdir(exist_ok=True)
                (path/'console.log').write_text(output)
            else:
                output = (path/'console.log').read_text() if (path/'console.log').exists() else ''
            row = dict(method='wang_free_state_mpc_event_v4', condition_id=key,
                rate=rate, translation_scale=tr, rotation_scale=rot, phase=phase,
                phase_id=phase_id, layout=0., attempt=attempt, run=str(path),
                wall_s=time.monotonic()-began, returncode=code, recovered_existing_run=recovered)
            if (path/'result.json').exists():
                row.update(inspect(path))
                if not row['infrastructure_failure']:
                    try:
                        passage = audit_run(path)
                    except Exception as exc:
                        passage = dict(passed=False, violations=['audit_error'], error=repr(exc))
                    save(path/'passage_audit.json', passage)
                    row = apply_passage(row, passage)
            else:
                started = (path/'solves_live.jsonl').exists()
                infra = not started and any(s in output for s in
                    ('PX4 startup failed', 'no heartbeat', 'missing initial telemetry'))
                row.update(physical_success=False, realtime_success=False, infrastructure_failure=infra,
                    flight_time_s=None, failure='process_timeout' if timed_out else 'missing_result')
            rows.append(row)
            save(out/'summary.json', rows)
            save(out/'first_attempt_summary.json', [r for r in rows if r['attempt'] == 1])
            save(out/'condition_summary.json', condition_rows(rows))
            print('RESULT '+json.dumps(row, ensure_ascii=False), flush=True)
            if not row['infrastructure_failure']:
                break
    if source_hashes() != hashes:
        raise RuntimeError('Source mismatch after matrix')
    chosen = condition_rows(rows)
    save(out/'status.json', dict(state='complete', conditions=len(chosen), attempts=len(rows),
        passed=sum(bool(r.get('realtime_success')) for r in chosen), finished=time.time()))
    print('DONE '+str(out), flush=True)


if __name__ == '__main__':
    main()
