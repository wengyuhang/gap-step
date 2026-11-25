"""Predeclared sparse transfer matches and two contemporary checked controls.

Run after the 38-case frozen batch. These are separate diagnostics, not extra
repeats selected to repair a failed primary condition.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

from .run import ROOT, analyze_run, save, source_hashes


CASES = [
    ('checked_control_r1_t1_a1_p0', False, 1., 1., 1., 0.),
    ('checked_control_r10_t4_a1_p120', False, 10., 4., 1., 2.0943951023931953),
    ('soft_rl_match_static_p04', True, 1., 0., 0., .4),
    ('soft_rl_match_dynamic_hard_p04', True, 10., 4., 4., .4),
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir', type=Path, required=True)
    p.add_argument('--batch', type=Path, required=True)
    args = p.parse_args()
    if json.loads((args.batch/'status.json').read_text())['state'] != 'complete':
        raise RuntimeError('Do not overlap timed PX4 flights')
    hashes = source_hashes()
    if hashes != json.loads((args.batch/'protocol.json').read_text())['source_sha256']:
        raise RuntimeError('Frozen algorithm changed')
    args.outdir.mkdir(parents=True, exist_ok=False)
    save(args.outdir/'protocol.json', dict(cases=CASES, primary_batch=str(args.batch.resolve()),
                                         source_sha256=hashes, runner_sha256=__import__('hashlib').sha256(Path(__file__).read_bytes()).hexdigest(),
                                         note='Separate sparse diagnostics; do not pool with primary matrix'))
    rows = []
    for name, soft, rate, translation, rotation, phase in CASES:
        for attempt in (1, 2):
            path = args.outdir/f'{name}_attempt{attempt}'
            cmd = [sys.executable, '-u', '-m', 'convex_timevarying_window.receding_gate_racing.px4_experiment',
                   '--launch', '--checked-soft' if soft else '--checked', '--tracking-preview', '.05',
                   '--motion-rate', str(rate), '--translation-scale', str(translation),
                   '--rotation-scale', str(rotation), '--phase', str(phase), '--outdir', str(path.resolve())]
            print('START '+name, flush=True)
            start = time.monotonic()
            try:
                proc = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True, timeout=240)
                console = proc.stdout+'\n'+proc.stderr
            except subprocess.TimeoutExpired as exc:
                console = repr(exc)
            path.mkdir(exist_ok=True)
            (path/'console.log').write_text(console)
            row = dict(name=name, method='Checked-Soft' if soft else 'Checked contemporary control',
                       rate=rate, translation_scale=translation, rotation_scale=rotation, phase=phase,
                       run=str(path.resolve()), attempt=attempt, wall_s=time.monotonic()-start)
            if (path/'result.json').exists():
                r = json.loads((path/'result.json').read_text())
                row['infrastructure_failure'] = r.get('failure') == "RuntimeError('arm failed')"
                if not row['infrastructure_failure']:
                    try:
                        row.update(analyze_run(path))
                    except Exception as exc:
                        row.update(physical_success=False, realtime_success=False, analysis_error=repr(exc))
            else:
                row.update(infrastructure_failure=not (path/'solves_live.jsonl').exists(),
                           realtime_success=False, failure='missing_result')
            rows.append(row)
            save(args.outdir/'summary.json', rows)
            print('RESULT '+json.dumps(row), flush=True)
            if not row['infrastructure_failure']:
                break


if __name__ == '__main__':
    main()
