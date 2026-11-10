"""Experiment-local entry; delegate to the unchanged frozen paired runner."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MODULE = 'convex_timevarying_window.comparisons.single_gate_swing.run'


def planned_cases(protocol):
    block = protocol['translation']
    static = block['static_case']
    return [(static['frequency_hz'], static['amplitude_m'])] + [
        (f, a) for a in block['amplitudes_m'] for f in block['frequencies_hz']]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', default='translation_v1_20261105')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id):
        raise ValueError('run-id must be a single directory name')
    protocol = json.loads((HERE/'protocol.json').read_text())
    from convex_timevarying_window.comparisons.single_gate_swing.run import CASES
    from convex_timevarying_window.comparisons.single_gate_swing.planner import Config
    if planned_cases(protocol) != CASES:
        raise RuntimeError('experiment and original runner case lists differ')
    if Config().max_solve_ms != protocol['translation']['wall_clock_solve_limit_ms']:
        raise RuntimeError('planning budget differs from frozen experiment protocol')
    preflight = HERE/'results/preflight_translation_v1_20261105'
    status = json.loads((preflight/'status.json').read_text())
    rows = json.loads((preflight/'summary.json').read_text())
    if status['state'] != 'complete' or not status.get('sources_unchanged') or len(rows) != 4 or not all(r['passed'] for r in rows):
        raise RuntimeError('static/low-dynamic preflight was not successfully completed')
    output = HERE/'results'/args.run_id
    if output.exists():
        raise FileExistsError(output)
    print('EXPERIMENT1_FORMAL_TRANSLATION', output, flush=True)
    subprocess.run([sys.executable, '-u', '-m', MODULE, '--outdir', str(output)], cwd=ROOT, check=True)
    from convex_timevarying_window.comparisons.论文实验.实验1.analyze_translation import analyze
    analyze(output)


if __name__ == '__main__':
    main()
