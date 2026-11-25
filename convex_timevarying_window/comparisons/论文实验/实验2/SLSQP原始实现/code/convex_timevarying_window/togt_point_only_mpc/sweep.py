"""Frozen 13-condition paired replay of the original one-gate contact sweep."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

from convex_timevarying_window.comparisons.single_gate_translation import sweep as original
from convex_timevarying_window.comparisons.single_gate_translation.audit import audit
from convex_timevarying_window.simple_two_gate_soft_only_mpc import run as shared
from convex_timevarying_window.simple_two_gate_soft_only_mpc.run import Trial, execute


ROUTES = original.ROUTES
METHODS = ('center', 'ours')
FLIGHT = 'convex_timevarying_window.togt_point_only_mpc.flight'


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def source_hashes():
    root = Path(__file__).resolve().parents[2]
    paths = [root/rel for rel in original.source_hashes()]
    paths += list(Path(__file__).parent.glob('*.py'))
    paths += [Path(__file__).with_name('SWEEP_PROTOCOL.md'),
              root/'convex_timevarying_window/togt_minco_mpc/trajectory.py',
              root/'convex_timevarying_window/comparisons/original_fixed_plane_ours/adapter.py',
              root/'convex_timevarying_window/comparisons/original_fixed_plane_deadline/adapter.py']
    return {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


def cases():
    return [(route, ROUTES[route][i])
            for i in range(5) for route in ROUTES if i < len(ROUTES[route])]


def report(out, rows):
    pairs = {(r['trial']['frequency'], r['trial']['amplitude'], r['trial']['method']): r
             for r in rows}
    lines = ['# 点式 TOGT/MINCO-s4：原尺寸单窗 13 条件配对升档', '',
             '每条件初相0、自由/中心各一次；实体窗框接触为安全主判据。',
             '用时为起点到窗后目标，不含七窗返航；未完成或碰撞无有效时间。', '',
             '|振幅 m|频率 Hz|自由点结果|自由点时间 s|中心结果|中心时间 s|自由点优势 s|',
             '|---:|---:|---|---:|---|---:|---:|']
    def label(row):
        if row is None: return '待运行'
        if row.get('collision_detected'): return '碰撞'
        if row.get('audit_error'): return '证据不足'
        if row.get('passed'): return '无碰撞完赛'
        return '未完成'
    for amplitude, frequency in sorted({(a, f) for route in ROUTES.values()
                                         for f, a in route}):
        free = pairs.get((frequency, amplitude, 'ours'))
        center = pairs.get((frequency, amplitude, 'center'))
        ft = f"{free['flight_time_s']:.3f}" if free and free.get('passed') else '—'
        ct = f"{center['flight_time_s']:.3f}" if center and center.get('passed') else '—'
        advantage = (f"{center['flight_time_s']-free['flight_time_s']:.3f}"
                     if free and center and free.get('passed') and center.get('passed') else '—')
        lines.append(f'|{amplitude:g}|{frequency:g}|{label(free)}|{ft}|'
                     f'{label(center)}|{ct}|{advantage}|')
    lines += ['', '只有双方无碰撞完赛时才比较用时。原始逐趟记录见同目录。',
              '这是固定条件单次观察，不是成功率估计。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    shared.ROOT = Path.cwd()
    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=args.resume)
    root = Path(__file__).resolve().parents[2]
    if args.resume:
        protocol = json.loads((out/'protocol.json').read_text())
        frozen = protocol['source_sha256']
        rows = json.loads((out/'summary.json').read_text()) if (out/'summary.json').exists() else []
        attempts = json.loads((out/'attempts.json').read_text()) if (out/'attempts.json').exists() else []
    else:
        frozen = source_hashes()
        save(out/'protocol.json', dict(routes=ROUTES, methods=METHODS, phase=0.,
            repeat_count=1, prior_pair_not_reused=True, source_sha256=frozen,
            text=Path(__file__).with_name('SWEEP_PROTOCOL.md').read_text()))
        for rel in frozen:
            dst = out/'source_snapshot'/rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(root/rel, dst)
        rows, attempts = [], []
        report(out, rows)
    completed = {(r['trial']['stage'], r['trial']['frequency'],
                  r['trial']['amplitude'], r['trial']['method']) for r in rows}
    for route, (frequency, amplitude) in cases():
        pair = []
        for method in METHODS:
            trial = Trial('single_'+route, method, frequency, amplitude, 0., 0)
            key = (trial.stage, frequency, amplitude, method)
            if key in completed:
                pair.append(next(r for r in rows if
                    (r['trial']['stage'], r['trial']['frequency'],
                     r['trial']['amplitude'], r['trial']['method']) == key))
                continue
            if source_hashes() != frozen:
                raise RuntimeError('Experiment sources changed after freeze')
            for attempt in range(2):
                name = trial.name+(f'_setup_retry{attempt}' if attempt else '')
                path = out/name
                if path.exists():
                    raise FileExistsError(f'Unrecorded attempt already exists: {path}')
                save(out/'status.json', dict(state='running', pid=os.getpid(), route=route,
                    frequency=frequency, amplitude=amplitude, method=method,
                    attempt=attempt, completed=len(rows)))
                print('START', name, flush=True)
                row = execute(trial, path, out/(name+'.log'), 100,
                              module=FLIGHT, audit_fn=audit)
                row.update(route=route, attempt=attempt)
                attempts.append(row)
                save(out/'attempts.json', attempts)
                if not row.get('infrastructure_failure') or attempt == 1:
                    break
            row['passed'] = bool(row.get('passed') and not row.get('collision_detected')
                                 and not row.get('infrastructure_failure')
                                 and not row.get('audit_error'))
            rows.append(row)
            pair.append(row)
            completed.add(key)
            save(out/'summary.json', rows)
            report(out, rows)
            print('RESULT', name, 'collision', row.get('collision_detected'),
                  'passed', row['passed'], 'time', row.get('flight_time_s'), flush=True)
            if row.get('infrastructure_failure') or (row.get('audit_error')
                                                      and not row.get('collision_detected')):
                save(out/'status.json', dict(state='evidence_or_setup_failure',
                    directory=str(path), completed=len(rows)))
                return
        original.check_pair(pair)
        left, right = (json.loads((Path(row['directory'])/'point_only_sources.json').read_text())
                       for row in pair)
        if left != right:
            raise RuntimeError('New-method source mismatch within pair')
    save(out/'status.json', dict(state='complete', completed=len(rows)))


if __name__ == '__main__':
    main()
