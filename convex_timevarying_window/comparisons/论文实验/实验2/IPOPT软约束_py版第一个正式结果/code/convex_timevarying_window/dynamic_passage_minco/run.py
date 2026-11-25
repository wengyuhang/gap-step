"""Frozen minimal single-window flight validation; no automatic tuning/repeats."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
from convex_timevarying_window.simple_two_gate_soft_only_mpc import run as shared
from convex_timevarying_window.comparisons.single_gate_translation.audit import audit


CASES = ((3.5, 2.),)
METHODS = ('ours', 'center')
MODULE = 'convex_timevarying_window.dynamic_passage_minco.flight'


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def sources(root):
    directories = ('dynamic_passage_minco', 'togt_point_only_mpc', 'togt_minco_mpc',
                   'simple_two_gate_soft_only_mpc', 'simple_two_gate_mpc',
                   'wang_free_state_mpc', 'event_triggered_mpc',
                   'comparisons/single_gate_translation')
    files = [p for d in directories for p in (root/'convex_timevarying_window'/d).rglob('*.py')
             if 'results' not in p.parts and '__pycache__' not in p.parts]
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def report(out, rows):
    lines = ['# 单窗相对运动安全约束：真实闭环验证', '',
             '原 W1 开口、固定平面、已知正弦平移、初相0。x500/PX4与原MPC不变。',
             '时间从起点飞至窗后6m目标；本单窗实验不返航，不与七窗完整时间直接比较。',
             '每方法一次；只有真实接触记录和正确穿越/到达共同决定成功。', '',
             '|频率 Hz|振幅 m|方法|结果|完整单窗时间 s|规划端到端最大 ms|',
             '|---:|---:|---|---|---:|---:|']
    for r in rows:
        outcome = ('证据不足' if r.get('audit_error') or r.get('infrastructure_failure') else
                   '碰撞' if r.get('collision_detected') else
                   '无碰撞完成' if r.get('passed') else '未完成')
        duration = f"{r['flight_time_s']:.3f}" if r.get('passed') else '—'
        raw = json.loads((Path(r['directory'])/'result.json').read_text())
        maximum = max((s['cycle_ms'] for s in raw['solves']), default=0.)
        lines.append(f"|{r['trial']['frequency']:g}|{r['trial']['amplitude']:g}|{'自由点' if r['trial']['method']=='ours' else '强制中心'}|{outcome}|{duration}|{maximum:.3f}|")
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--frequency', type=float)
    parser.add_argument('--amplitude', type=float)
    parser.add_argument('--method', choices=METHODS)
    parser.add_argument('--nominal-probe', default='development_probe_v6_20261009')
    args = parser.parse_args()
    if (args.frequency is None) != (args.amplitude is None): parser.error('Set both frequency and amplitude')
    cases = CASES if args.frequency is None else ((args.frequency, args.amplitude),)
    methods = METHODS if args.method is None else (args.method,)
    root = Path.cwd(); shared.ROOT = root
    out = args.outdir.resolve(); out.mkdir(parents=True, exist_ok=False)
    hashes = sources(root)
    save(out/'protocol.json', dict(cases=cases, methods=methods, repetitions=1, phase=0.,
         source_sha256=hashes, description=Path(__file__).with_name('README.md').read_text(),
         nominal_probe=args.nominal_probe, allow_algorithm_tuning_during_run=False))
    for rel in hashes:
        target = out/'source_snapshot'/rel; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/rel, target)
    rows = []; report(out, rows)
    for frequency, amplitude in cases:
        pair = []
        for method in methods:
            assert hashes == sources(root), 'Frozen source changed'
            trial = shared.Trial('passage', method, frequency, amplitude, 0., 0)
            path = out/trial.name
            save(out/'status.json', dict(state='running', pid=os.getpid(), method=method,
                                       completed=len(rows), directory=str(path)))
            print('START', trial.name, flush=True)
            row = shared.execute(trial, path, out/(trial.name+'.log'), 100,
                                 module=MODULE, audit_fn=audit)
            row['passed'] = bool(row.get('passed') and not row.get('audit_error')
                                 and not row.get('infrastructure_failure'))
            rows.append(row); pair.append(row)
            save(out/'summary.json', rows); report(out, rows)
            print('RESULT', trial.name, row.get('passed'), row.get('collision_detected'),
                  row.get('flight_time_s'), row.get('failure'), flush=True)
            if row.get('audit_error') or row.get('infrastructure_failure'):
                save(out/'status.json', dict(state='setup_or_evidence_failure', completed=len(rows)))
                return
        if len(pair) == 1: continue
        assert pair[0]['world_sha256'] == pair[1]['world_sha256']
        assert pair[0]['mpc_config'] == pair[1]['mpc_config']
        assert pair[0]['px4_parameters'] == pair[1]['px4_parameters']
        raw = [json.loads((Path(r['directory'])/'result.json').read_text()) for r in pair]
        cfg = [dict(r['config']) for r in raw]
        assert cfg[0].pop('free_points') != cfg[1].pop('free_points')
        assert cfg[0] == cfg[1]
    save(out/'status.json', dict(state='complete', completed=len(rows)))


if __name__ == '__main__': main()
