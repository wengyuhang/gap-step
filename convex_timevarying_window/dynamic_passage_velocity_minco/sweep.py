"""Frozen free-point single-gate difficulty sweep; no solver tuning or retries."""
import argparse
import json
import os
from pathlib import Path
import shutil
from convex_timevarying_window.dynamic_passage_minco.run import sources as base_sources
from convex_timevarying_window.simple_two_gate_soft_only_mpc import run as shared
from convex_timevarying_window.comparisons.single_gate_translation.audit import audit


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def sources(root):
    import hashlib
    result = base_sources(root)
    for p in Path(__file__).parent.glob('*.py'):
        result[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return result


def report(out, rows):
    lines = ['# 当前速度自由 MINCO：单窗升档', '',
        '仅自由点；原小窗W1、x500/PX4、初相0、两段C5 minimum-snap、当前MPC。',
        '目标/安全约束/首个收敛安全解返回策略均冻结；每条件一次，无自动重试。',
        '时间为起点到窗后6m目标，不是七窗返航时间。接触为碰撞证据，采样净空不代替接触。',
        '三路线各最多四步：固定4m升频、固定3Hz升幅、二者同步升档，每步0.5。',
        '各路线首次碰撞或未完成停止；求解失败不代表物理不可行。证据/设施异常整批停止。', '',
        '|幅度 m|频率 Hz|结果|完整单窗时间 s|规划最大 ms|',
        '|---:|---:|---|---:|---:|']
    for r in sorted(rows, key=lambda r:(r['trial']['amplitude'],r['trial']['frequency'])):
        collision = r.get('collision_detected', False)
        outcome = ('碰撞' if collision else '证据不足' if r.get('audit_error') or r.get('infrastructure_failure')
                   else '无碰撞完成' if r.get('passed') else '未完成')
        duration = f"{r['flight_time_s']:.3f}" if r.get('passed') else '—'
        raw_path = Path(r['directory'])/'result.json'
        raw = json.loads(raw_path.read_text()) if raw_path.exists() else {}
        maximum = max((s['cycle_ms'] for s in raw.get('solves', [])), default=None)
        timing = '—' if maximum is None else f'{maximum:.3f}'
        lines.append(f"|{r['trial']['amplitude']:g}|{r['trial']['frequency']:g}|{outcome}|{duration}|{timing}|")
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    root = Path.cwd(); shared.ROOT = root
    out = args.outdir.resolve(); out.mkdir(parents=True, exist_ok=False)
    hashes = sources(root)
    routes = {'frequency': [(3+.5*k,4.) for k in range(1,5)],
              'amplitude': [(3.,4+.5*k) for k in range(1,5)],
              'joint': [(3+.5*k,4+.5*k) for k in range(1,5)]}
    save(out/'protocol.json', dict(routes=routes, source_sha256=hashes, method='ours',
        phase=0, repetitions=1, retries=0, baseline='flight_forward_v2_f3_a4_20261009_01',
        module='convex_timevarying_window.dynamic_passage_velocity_minco.flight',
        stop_route='first collision or noncompletion', algorithm_changes=False))
    for rel in hashes:
        target = out/'source_snapshot'/rel; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root/rel, target)
    rows = []; stopped = {}; report(out, rows)
    for step in range(4):
        for route, cases in routes.items():
            if route in stopped: continue
            if sources(root) != hashes:
                save(out/'status.json', dict(state='source_changed', completed=len(rows), pid=os.getpid()))
                return
            f,a = cases[step]
            trial = shared.Trial(route, 'ours', f, a, 0., 0)
            path = out/trial.name
            save(out/'status.json', dict(state='running', pid=os.getpid(), completed=len(rows),
                route=route, frequency=f, amplitude=a, directory=str(path)))
            print('START', route, f, a, flush=True)
            row = shared.execute(trial, path, out/(trial.name+'.log'), 100,
                module='convex_timevarying_window.dynamic_passage_velocity_minco.flight', audit_fn=audit)
            row['route'] = route
            row['passed'] = bool(row.get('passed') and not row.get('audit_error')
                                 and not row.get('infrastructure_failure'))
            rows.append(row); save(out/'summary.json', rows); report(out, rows)
            print('RESULT', route, f, a, row['passed'], row.get('collision_detected'),
                  row.get('flight_time_s'), row.get('failure'), flush=True)
            if row.get('infrastructure_failure') or (row.get('audit_error') and not row.get('collision_detected')):
                save(out/'status.json', dict(state='setup_or_evidence_failure', completed=len(rows), pid=os.getpid()))
                return
            if not row['passed']:
                stopped[route] = dict(frequency=f, amplitude=a, collision=row.get('collision_detected'),
                                      failure=row.get('failure'), audit_error=row.get('audit_error'))
            save(out/'boundaries.json', stopped)
    save(out/'status.json', dict(state='complete', completed=len(rows), pid=os.getpid(), stopped=stopped))


if __name__ == '__main__': main()
