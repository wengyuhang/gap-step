"""User-amended sweep: flight completion and physical contact, not nominal limits."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import time

from .run import METHODS, ROUTES, BASELINE, key, midpoint, enrich
from convex_timevarying_window.simple_two_gate_mpc.run import execute, sources
from convex_timevarying_window.comparisons.comprehensive_wang.registry import Trial
from convex_timevarying_window.comparisons.comprehensive_wang.audit import audit
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from 复现.polynomial_dynamic_racing_2023.px4_original_track_legacy.run import ROOT, save, digest


def evaluate(row):
    row = enrich(row)
    row['simulation_passed'] = bool(row.get('completed') and row.get('body_exit_count') == 7
                                    and not row.get('collision_detected'))
    row['simulation_race_time_s'] = row.get('flight_time_s') if row['simulation_passed'] else None
    row['simulation_outcome'] = ('collision' if row.get('collision_detected') else
                                 'complete' if row['simulation_passed'] else 'incomplete')
    return row


def on_route(row, route):
    f, a = row['trial']['frequency'], row['trial']['amplitude']
    return {'joint': abs(a-2*f)<1e-10, 'frequency': a == 2., 'amplitude': f == 1.}[route]


def first_bracket(rows, route, method):
    values = sorted((r for r in rows if r['trial']['method'] == method and on_route(r, route)),
                    key=lambda r: (r['trial']['frequency'], r['trial']['amplitude']))
    last = None
    for row in values:
        point = (row['trial']['frequency'], row['trial']['amplitude'])
        if not row['simulation_passed']: return dict(last_pass=last, first_fail=point, outcome=row['simulation_outcome'])
        last = point
    return dict(last_pass=last, first_fail=None)


def report(out, rows, boundaries):
    lines = ['# 按真实仿真完成与碰撞判定的递增实验', '',
             '用户更新口径：不以100ms、名义动力学/包络超限否决。算法未改；时间包含七窗后返回终点。',
             '各条件各一次，不能称随机成功率或全局临界值。', '',
             '|频率 Hz|单侧幅度 m|方法|完整无碰撞时间 s|碰撞|结果|已退出窗数|',
             '|---:|---:|---|---:|---|---|---:|']
    for r in rows:
        vals = [r['trial']['frequency'], r['trial']['amplitude'], r['trial']['method'],
                r['simulation_race_time_s'], r.get('collision_detected'), r['simulation_outcome'], r.get('body_exit_count')]
        lines.append('| '+' | '.join(map(str, vals))+' |')
    lines += ['', '## 当前经验边界（含已测中间档）', '', '```json', json.dumps(boundaries, ensure_ascii=False, indent=2),
              '```', '', '更高档可能重新成功，不能外推为“超过此档必失败”。超时等原审计仍保留在summary.json。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir', type=Path, required=True)
    p.add_argument('--previous', type=Path, required=True)
    args = p.parse_args()
    out, previous = args.outdir.resolve(), args.previous.resolve()
    out.mkdir(parents=True, exist_ok=False)
    rows = [evaluate(r) for r in json.loads((previous/'summary.json').read_text())]
    status = json.loads((previous/'status.json').read_text())
    # Previous coordinator stopped, not its in-flight child. Finish and retain that one trial.
    if status['state'] == 'running':
        trial = Trial(status['stage'], status['method'], status['frequency'], status['amplitude'], 0., 0)
        path = previous/(trial.name+f"_attempt{status['attempt']}")
        if not any(r['directory'] == str(path) for r in rows):
            print('WAIT_EXISTING', path.name, flush=True)
            limit = time.monotonic()+900
            while not (path/'result.json').exists():
                if time.monotonic() > limit: raise RuntimeError('Existing trial result not available')
                time.sleep(2)
            # Result serialization and file writes may still be finishing.
            time.sleep(3)
            raw = json.loads((path/'result.json').read_text())
            preparation = assess(path)
            assert not preparation['setup_failure'], preparation
            row = dict(trial=asdict(trial), name=trial.name, pair=trial.pair, directory=str(path),
                       infrastructure_failure=False, preparation_assessment=preparation,
                       world_sha256=digest(path/'world.sdf'), mpc_config=raw['mpc_config'], px4_parameters=raw['parameters'],
                       scene_manifest=json.loads((path/'scene.json').read_text()), recovered_existing_trial=True)
            row.update(audit(path))
            rows.append(evaluate(row))
    save(previous/'amendment.json', dict(reason='User changed primary criteria to full flight time and physical collision',
         continued_in=str(out), original_coordinator_stopped=True, in_flight_trial_preserved=True))
    hashes = sources()
    for path in Path(__file__).parent.glob('*'):
        if path.suffix in ('.py', '.md'): hashes[str(path.relative_to(ROOT))] = digest(path)
    old_hashes = json.loads((previous/'protocol.json').read_text())['source_sha256']
    assert all(digest(ROOT/rel) == h for rel, h in old_hashes.items()), 'Original algorithm changed'
    save(out/'protocol.json', dict(source_sha256=hashes, previous=str(previous), routes=ROUTES,
         text=Path(__file__).with_name('SIMULATION_PROTOCOL.md').read_text()))
    for rel in hashes:
        dest = out/'source_snapshot'/rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/rel, dest)
    for r in rows: r['reused_previous'] = True
    cache = {key(r['trial']['frequency'], r['trial']['amplitude'], r['trial']['method']): r for r in rows}
    baseline_raw = {r['trial']['method']: json.loads((Path(r['directory'])/'result.json').read_text())
                    for r in rows if r['trial']['frequency']==1. and r['trial']['amplitude']==2.}
    attempts, boundaries = [], {}
    save(out/'summary.json', rows)

    def frozen():
        assert all(digest(ROOT/rel) == h for rel, h in hashes.items()), 'Frozen sources changed'

    def pair(f, amplitude, stage):
        for method in METHODS:
            k = key(f, amplitude, method)
            if k in cache: continue
            trial = Trial(stage, method, f, amplitude, 0., 0)
            for attempt in (1, 2):
                frozen()
                path = out/(trial.name+f'_attempt{attempt}')
                save(out/'status.json', dict(state='running', stage=stage, frequency=f, amplitude=amplitude,
                     method=method, attempt=attempt, completed_conditions=len(rows)))
                print('START', trial.name, attempt, flush=True)
                row = evaluate(execute(trial, path, out/(path.name+'.log'), 100))
                attempts.append(row)
                save(out/'attempts.json', attempts)
                if not row.get('infrastructure_failure'): break
            if row.get('infrastructure_failure') or row.get('audit_error'): raise RuntimeError(f'Missing valid trial evidence: {path}')
            raw = json.loads((path/'result.json').read_text())
            assert raw['config'] == baseline_raw[method]['config']
            for field in ('mpc_config', 'parameters'): assert raw[field] == baseline_raw[method][field]
            assert all(e['reason'] in ('initial', 'body_exit') for e in raw['planning_events'] if e['action']=='trigger')
            rows.append(row)
            cache[k] = row
            save(out/'summary.json', rows)
            report(out, rows, boundaries)
            print('RESULT', json.dumps({name: row.get(name) for name in ('name','simulation_race_time_s',
                  'simulation_outcome','body_exit_count','failure','planner_max_ms')}, ensure_ascii=False), flush=True)
        left, right = (cache[key(f, amplitude, method)] for method in METHODS)
        assert left['world_sha256'] == right['world_sha256'] and left['scene_manifest'] == right['scene_manifest']

    for route, levels in ROUTES.items():
        for f, amplitude in levels:
            pair(f, amplitude, route)
            if all(first_bracket(rows, route, m)['first_fail'] for m in METHODS): break
        brackets = {m: first_bracket(rows, route, m) for m in METHODS}
        mids = sorted({midpoint(b['last_pass'], b['first_fail']) for b in brackets.values() if b['first_fail'] and b['last_pass']})
        for f, amplitude in mids: pair(f, amplitude, route+'_midpoint')
        boundaries[route] = {m: first_bracket(rows, route, m) for m in METHODS}
        save(out/'boundaries.json', boundaries)
        report(out, rows, boundaries)
    frozen()
    save(out/'verification.json', dict(source_changes=[], source_count=len(hashes),
         snapshot_changes=[rel for rel, h in hashes.items() if digest(out/'source_snapshot'/rel) != h],
         matched_configs=True, matched_worlds=True))
    save(out/'status.json', dict(state='complete', conditions=len(rows), new_attempts=len(attempts)))


if __name__ == '__main__': main()
