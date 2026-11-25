"""Run direct CTBR on the 2026-09-23 24-condition PX4/Gazebo matrix.

The old checked/Wang results are read-only; this script only creates new runs.
"""
import argparse
import contextlib
import hashlib
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from convex_timevarying_window.three_phase_mppi.report import audit_px4

ROOT = Path('convex_timevarying_window/comparisons/direct_ctbr_center_motion/results')
SOURCE_ROOTS = (
    Path('convex_timevarying_window/gs_mppi/direct_ctbr'),
    Path('convex_timevarying_window/dynamic_gate_mppi_v3'),
    Path('convex_timevarying_window/three_phase_mppi'),
    Path('convex_timevarying_window/togt'),
    Path('nonconvex_timevarying_window/sc_dynatogt'),
    Path('convex_dynamic_seven_window_gazebo/x500_togt'),
)
SOURCE_FILES = (
    Path('convex_timevarying_window/geometry.py'),
    Path('convex_timevarying_window/receding_gate_racing/scenario.py'),
    Path('convex_timevarying_window/gs_mppi/progress.py'),
    Path(__file__),
)


def conditions():
    return [(rate, translation, phase_id, phase)
            for rate in (1, 3, 6, 10)
            for translation in (1, 2, 4)
            for phase_id, phase in enumerate((0., 2 * math.pi / 3))]


def source_hashes():
    files = set(SOURCE_FILES)
    for root in SOURCE_ROOTS:
        files.update(root.glob('*.py'))
        files.update(root.glob('*.cpp'))
        files.update((root / 'native').glob('*.cpp'))
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}


def summarize(path, rate, translation, phase_id, phase, attempt, wall_s, timeout):
    row = dict(rate=rate, translation_scale=translation, rotation_scale=1,
               phase_id=phase_id, phase=phase, attempt=attempt,
               run=str(path.resolve()), wall_s=wall_s)
    result_path = path / 'result.json'
    if not result_path.exists():
        row.update(infrastructure_failure=True, physical_success=False,
                   realtime_success=False, flight_time_s=None,
                   failure='process_timeout' if timeout else 'missing_result')
        return row
    result = json.loads(result_path.read_text())
    row.update(infrastructure_failure=result.get('failure') == "RuntimeError('arm failed')",
               failure=result.get('failure'), flight_time_s=result.get('flight_time_s'),
               last_time_s=result.get('last_time_s'), gates=len(result.get('crossings', [])),
               collision=result.get('collision_detected'))
    try:
        with contextlib.redirect_stdout(io.StringIO()) as output:
            audit_px4(path, result)
        (path / 'audit.log').write_text(output.getvalue())
        audit = json.loads((path / 'audit.json').read_text())
        ordered = [x['gate'] for x in result['crossings']] == list(range(7))
        mppi_ms = [x['cycle_ms'] for x in result['solves']]
        togt_ms = [x['solve_ms'] for x in result['reference_updates'] if x.get('solve_ms') is not None]
        deadlines = audit['togt']['deadline_rejections']
        physical = bool(result['completed'] and ordered and not result['collision_detected']
                        and audit['min_sampled_body_clearance_m'] >= 0
                        and audit['all_six_workspace_faces_passed'])
        realtime = bool(physical and mppi_ms and max(mppi_ms) < 100
                        and all(x < 100 for x in togt_ms) and deadlines == 0)
        row.update(physical_success=physical, realtime_success=realtime,
                   ordered_passages_passed=ordered,
                   min_clearance_m=audit['min_sampled_body_clearance_m'],
                   room_passed=audit['all_six_workspace_faces_passed'],
                   max_speed_mps=audit['max_sampled_body_speed_mps'],
                   max_acceleration_mps2=audit['max_sampled_body_acceleration_mps2'],
                   max_tilt_deg=audit['max_truth_tilt_deg'],
                   mppi_max_ms=max(mppi_ms, default=None),
                   togt_max_ms=max(togt_ms, default=None),
                   togt_deadline_rejections=deadlines,
                   strict_dynamics_success=audit['strict_sampled_success'])
    except Exception as exc:
        row.update(physical_success=False, realtime_success=False,
                   analysis_error=repr(exc))
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir', type=Path, default=ROOT / 'direct_ctbr_20260924')
    p.add_argument('--resume', action='store_true')
    p.add_argument('--limit', type=int, default=24, help='Total matrix entries to attempt')
    a = p.parse_args()
    hashes = source_hashes()
    if a.resume:
        protocol = json.loads((a.outdir / 'protocol.json').read_text())
        if protocol['source_sha256'] != hashes:
            raise RuntimeError('Source changed since the frozen first run')
        rows = json.loads((a.outdir / 'summary.json').read_text())
    else:
        a.outdir.mkdir(parents=True, exist_ok=False)
        protocol = dict(matrix=conditions(), source_sha256=hashes,
                        max_flight_time_s=60, body_radius_m=.38594472,
                        rotation_scale=1, px4_profile='2026-09-23 checked/Wang',
                        direct_ctbr_samples=16384, direct_ctbr_horizon_s=2,
                        per_cycle_deadline_ms=100,
                        old_pva_tracking_preview_s=.05,
                        direct_ctbr_tracking_preview_s=None,
                        evidence='One deterministic trial per condition; 10 ms sampled truth, not a continuous safety certificate')
        (a.outdir / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
        rows = []
        (a.outdir / 'summary.json').write_text('[]\n')
    finished = {(x['rate'], x['translation_scale'], x['phase_id'])
                for x in rows if not x['infrastructure_failure'] or x['attempt'] == 2}
    for rate, translation, phase_id, phase in conditions()[:a.limit]:
        key = (rate, translation, phase_id)
        if key in finished:
            continue
        previous = [x for x in rows if (x['rate'], x['translation_scale'], x['phase_id']) == key]
        for attempt in range(len(previous) + 1, 3):
            if source_hashes() != hashes:
                raise RuntimeError('Source changed during the frozen matrix')
            path = a.outdir / f'r{rate}_t{translation}_a1_p{phase_id}_attempt{attempt}'
            if path.exists():
                raise FileExistsError(path)
            cmd = [sys.executable, '-u', '-m', 'convex_timevarying_window.gs_mppi.direct_ctbr.px4',
                   '--outdir', str(path), '--duration', '60',
                   '--motion-rate', str(rate), '--translation-scale', str(translation),
                   '--rotation-scale', '1', '--phase', str(phase),
                   '--matched-px4-parameters']
            print('START', path.name, flush=True)
            tick = time.monotonic()
            timeout = False
            try:
                proc = subprocess.run(cmd, capture_output=True, text=True, timeout=240,
                                      env={**os.environ, 'OPENBLAS_NUM_THREADS': '1',
                                           'OMP_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1'})
                output = proc.stdout + '\n' + proc.stderr
            except subprocess.TimeoutExpired as exc:
                timeout = True
                output = repr(exc)
            path.mkdir(parents=True, exist_ok=True)
            (path / 'console.log').write_text(output)
            row = summarize(path, rate, translation, phase_id, phase, attempt,
                            time.monotonic() - tick, timeout)
            rows.append(row)
            (a.outdir / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
            print('RESULT', json.dumps(row, ensure_ascii=False), flush=True)
            if not row['infrastructure_failure']:
                break
    print('DONE', str(a.outdir.resolve()), flush=True)


if __name__ == '__main__':
    main()
