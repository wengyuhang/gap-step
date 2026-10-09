"""Sequential frozen difficulty sweep, including joint frequency/amplitude growth."""
import argparse
from collections import Counter
import json
from pathlib import Path
import shutil
import time

from convex_timevarying_window.simple_two_gate_mpc.run import execute, sources
from convex_timevarying_window.comparisons.comprehensive_wang.registry import Trial
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import ROOT, save, digest

BASELINE = ROOT/'convex_timevarying_window/simple_two_gate_mpc/results/matched_full_loop_v1_20261008'
METHODS = ('ours', 'center')
ROUTES = {
    'joint': [(1.25, 2.5), (1.5, 3.), (2., 4.), (3., 6.)],
    'frequency': [(1.25, 2.), (1.5, 2.), (2., 2.), (3., 2.)],
    'amplitude': [(1., 2.5), (1., 3.), (1., 4.), (1., 6.)],
}


def key(f, a, method):
    return f'{f:g}/{a:g}/{method}'


def midpoint(left, right):
    return tuple((x+y)/2 for x, y in zip(left, right))


def enrich(row):
    row = dict(row)
    path = Path(row['directory'])/'result.json'
    if path.exists():
        raw = json.loads(path.read_text())
        solves, controls = raw.get('solves', []), raw.get('mpc_cycles', [])
        row.update(planning_calls=len(solves), accepted=sum(s['accepted'] for s in solves),
                   planner_max_ms=max((s['cycle_ms'] for s in solves), default=0.),
                   mpc_max_ms=max((s['cycle_ms'] for s in controls), default=0.),
                   deadline_misses=sum(s['cycle_ms'] >= 100 for s in solves),
                   rejections=dict(Counter(s['rejection_reason'] for s in solves if not s['accepted'])))
    row['full_race_time_s'] = row.get('flight_time_s') if row.get('main_passed') else None
    checks = ('physical_success', 'sampled_envelope_passed', 'exit_before_next_entry',
              'no_reverse_after_entry', 'truth_returned')
    row['non_realtime_full_passed'] = bool(all(row.get(k, False) for k in checks)
                                         and row.get('workspace_center_margin_m', -1) >= 0)
    reasons = []
    if row.get('infrastructure_failure') or row.get('audit_error'):
        reasons.append('infrastructure_or_missing_evidence')
    else:
        if row.get('collision_detected'): reasons.append('physical_contact')
        if not row.get('sampled_envelope_passed'): reasons.append('sampled_safety')
        if row.get('body_exit_count', 0) < 7: reasons.append('incomplete_gates')
        if not row.get('no_reverse_after_entry'): reasons.append('reverse_crossing')
        if not row.get('truth_returned'): reasons.append('terminal_state')
        if row.get('workspace_center_margin_m', -1) < 0: reasons.append('workspace')
        if not row.get('realtime_passed'): reasons.append('online_deadline_or_control')
        if not row.get('main_passed') and not reasons: reasons.append('other_acceptance')
    row['failure_categories'] = reasons
    return row


def report(out, rows, boundaries):
    lines = ['# 频率/幅度递增：冻结算法仿真实验', '',
             '幅度为单侧；每条件各一次、原始相位。完整时间包含返回终点。',
             '严格失败包括在线超时；物理/采样完成与在线能力分列，不把失败中止时刻当成绩。', '',
             '|Hz|幅度 m|方法|严格完整时间 s|不含实时要求完整通过|退出窗数|规划最大 ms|MPC最大 ms|失败分类/原始原因|',
             '|---:|---:|---|---:|---|---:|---:|---:|---|']
    for r in rows:
        t = r['trial']
        vals = [t['frequency'], t['amplitude'], t['method'], r.get('full_race_time_s'),
                r['non_realtime_full_passed'], r.get('body_exit_count'),
                round(r.get('planner_max_ms', 0), 2), round(r.get('mpc_max_ms', 0), 2),
                ', '.join(r['failure_categories'])+' / '+str(r.get('failure'))]
        lines.append('| '+' | '.join(map(str, vals))+' |')
    lines += ['', '## 已观察到的粗档边界', '', '```json',
              json.dumps(boundaries, ensure_ascii=False, indent=2), '```', '',
              '中间档另见表；只有单初相单次结果，不假定难度单调，不推断全局不可行或随机成功率。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir', type=Path, required=True)
    a = p.parse_args()
    out = a.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    hashes = sources()
    for path in Path(__file__).parent.glob('*'):
        if path.suffix in ('.py', '.md'): hashes[str(path.relative_to(ROOT))] = digest(path)
    baseline_hashes = json.loads((BASELINE/'protocol.json').read_text())['source_sha256']
    # Previous pair only changed its reporting wrapper after flying; no flight/planner change allowed.
    changes = [rel for rel, h in baseline_hashes.items() if digest(ROOT/rel) != h]
    assert set(changes) <= {'convex_timevarying_window/simple_two_gate_mpc/run.py'}, changes
    save(out/'protocol.json', dict(routes=ROUTES, baseline=str(BASELINE), baseline_source_changes=changes,
         baseline_summary_sha256=digest(BASELINE/'summary.json'), source_sha256=hashes,
         text=Path(__file__).with_name('PROTOCOL.md').read_text()))
    for rel in hashes:
        target = out/'source_snapshot'/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/rel, target)
    rows = [enrich(r) for r in json.loads((BASELINE/'summary.json').read_text())]
    for r in rows: r['reused_baseline'] = True
    cache = {key(r['trial']['frequency'], r['trial']['amplitude'], r['trial']['method']): r for r in rows}
    attempts, boundaries = [], {}
    baseline_raw = {r['trial']['method']: json.loads((Path(r['directory'])/'result.json').read_text()) for r in rows}

    def frozen():
        assert all(digest(ROOT/rel) == h for rel, h in hashes.items()), 'Frozen source changed'

    def pair(f, amplitude, stage):
        for method in METHODS:
            k = key(f, amplitude, method)
            if k in cache: continue
            trial = Trial(stage, method, f, amplitude, 0., 0)
            for attempt in (1, 2):
                frozen()
                path = out/(trial.name+f'_attempt{attempt}')
                save(out/'status.json', dict(state='running', stage=stage, frequency=f,
                     amplitude=amplitude, method=method, attempt=attempt, finished_new=len(rows)-2))
                print('START', trial.name, attempt, flush=True)
                row = enrich(execute(trial, path, out/(path.name+'.log'), 100))
                attempts.append(row)
                save(out/'attempts.json', attempts)
                if not row.get('infrastructure_failure'): break
            if row.get('infrastructure_failure') or row.get('audit_error'):
                raise RuntimeError(f'Unresolved infrastructure/evidence: {path}')
            raw = json.loads((path/'result.json').read_text())
            assert raw['config'] == baseline_raw[method]['config']
            for field in ('mpc_config', 'parameters'):
                assert raw[field] == baseline_raw[method][field], field
            assert all(e['reason'] in ('initial', 'body_exit') for e in raw['planning_events'] if e['action']=='trigger')
            rows.append(row)
            cache[k] = row
            save(out/'summary.json', rows)
            report(out, rows, boundaries)
            print('RESULT', json.dumps({name: row.get(name) for name in ('name', 'main_passed',
                  'flight_time_s', 'non_realtime_full_passed', 'body_exit_count', 'failure_categories',
                  'failure', 'planner_max_ms', 'mpc_max_ms')}, ensure_ascii=False), flush=True)
        left, right = (cache[key(f, amplitude, method)] for method in METHODS)
        assert left['world_sha256'] == right['world_sha256']
        assert left['scene_manifest'] == right['scene_manifest']

    for route, levels in ROUTES.items():
        last = {m: (1., 2.) for m in METHODS}
        first = {}
        for f, amplitude in levels:
            pair(f, amplitude, route)
            for method in METHODS:
                if method in first: continue
                if cache[key(f, amplitude, method)]['main_passed']:
                    last[method] = (f, amplitude)
                else:
                    first[method] = dict(last_pass=last[method], first_fail=(f, amplitude))
            if len(first) == len(METHODS): break
        boundaries[route] = {m: first.get(m, dict(last_pass=last[m], first_fail=None)) for m in METHODS}
        save(out/'boundaries.json', boundaries)
        mids = sorted({midpoint(b['last_pass'], b['first_fail']) for b in first.values()})
        for f, amplitude in mids: pair(f, amplitude, route+'_midpoint')
    frozen()
    save(out/'verification.json', dict(source_count=len(hashes), source_changes=[],
         snapshot_changes=[rel for rel, h in hashes.items() if digest(out/'source_snapshot'/rel) != h],
         matched_configurations=True, matched_worlds=True, event_only=True))
    report(out, rows, boundaries)
    save(out/'status.json', dict(state='complete', new_trials=len(rows)-2, attempts=len(attempts), finished=time.time()))


if __name__ == '__main__': main()
