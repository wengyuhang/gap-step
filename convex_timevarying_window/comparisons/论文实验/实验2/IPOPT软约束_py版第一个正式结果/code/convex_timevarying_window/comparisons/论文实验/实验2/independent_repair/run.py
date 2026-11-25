"""Frozen four-start regression on each complete target course independently."""
import argparse
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
from .smooth_augmented import Planner
from ..iteration_limit_only import run as previous
from ..basic_group.whole_course_difficulty import scenario, run as baseline
from ..attitude_repair.run import attitude_audit
from convex_timevarying_window.global_gate_minco_probe.planner import Plan

HERE = Path(__file__).resolve().parent
ROOT = scenario.ROOT
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.independent_repair.run'
read, save, digest = previous.read, previous.save, previous.digest


def sources():
    result = previous.sources()
    for folder in (HERE, ROOT/'convex_timevarying_window/tilt_heading_minco', HERE.parent/'attitude_repair'):
        for path in folder.rglob('*'):
            if path.is_file() and path.suffix in ('.py', '.md'):
                result[str(path.relative_to(ROOT))] = digest(path)
    return result


def frozen_plan(raw):
    return Plan(0., 0, np.array(raw['durations']), np.array(raw['coefficients']),
                np.array(raw['variables']), 0., 0., 'independent re-evaluation', 7)


def plan(level, method, out):
    from .. import plan as entry
    entry.Case = scenario.Case
    entry.build_case = scenario.build_case
    entry.manifest = scenario.manifest
    entry.Planner = lambda track, free: Planner(track, free, pose_function=scenario.pose)
    sys.argv = [entry.__name__, '--layout', 'togt_whole', '--level', level, '--method', method,
                '--outdir', str(out), '--seconds-per-seed', '300']
    entry.main()
    raw = read(out/'result.json')
    if raw['found']:
        save(out/'attitude_audit.json', attitude_audit(frozen_plan(raw)))


def assess(out, level, method):
    path = out/'plans'/f'{level}_{method}'
    raw = read(path/'result.json')
    assert raw['independent_cold_start'] and not raw['cross_condition_warm_start']
    assert not raw['amplitude_continuation']
    assert raw['attitude_model'] == 'north_tilt_heading_pi_over_2'
    assert read(path/'protocol.json')['scene'] == previous.canonical(scenario.manifest(scenario.Case(level=level)))
    row = baseline.assess(path, level, method)
    row.update(termination_counts=previous.termination_counts(raw), seconds_per_seed=300.,
               attitude_domain_passed=None, original_verdict='planning_not_found')
    if raw['found']:
        audit = read(path/'attitude_audit.json')
        row['attitude_domain_passed'] = audit['passed']
        if not audit['passed']:
            row.update(passed=False, verdict='nominal_unresolved')
    return row


def report(out, rows, state):
    lines = ['# 独立赛道求解回归', '', f'状态 {state}，已结束 {len(rows)}/9 条。', '',
             '每个完整目标赛道独立原四初值，每初值300秒。采用同一平滑姿态与数值恢复版本。',
             '时间含七窗后返航至零VAJ，只有完整名义通过的轨迹列出有效时间。', '',
             '|幅度/窗宽|频率倍数|实际频率 Hz|方法|结果|完整轨迹 s|四初值求解 s|',
             '|---|---:|---|---|---|---:|---:|']
    for row in previous.ordered(rows):
        lo, hi = scenario.manifest(scenario.Case(level=row['level']))['frequency_range_hz']
        duration = f"{row['full_trajectory_time_s']:.6f}" if row['passed'] else '—'
        lines.append(f"|{row['amplitude_ratio']:g}|{row['frequency_factor']:g}|{lo:g}–{hi:g}|{previous.NAMES[row['variant']]}|"
                     f"{baseline.driver.LABELS[row['verdict']]}|{duration}|{row['solve_s']:.3f}|")
    passed = sum(r['passed'] for r in rows)
    lines += ['', f'已结束记录中{passed}条名义通过，{len(rows)-passed}条未通过。',
              '姿态采用north tilt/heading pi/2，其航向定义与旧固定世界Y模型不同。',
              '原四候选及全部失败见plans，STL/地面独立复算见assessments，原节点和几何重算见validation.json。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def validate(out):
    assert sources() == read(out/'protocol.json')['source_sha256']
    assert previous.check_prior(sources()) == 2386
    rows = read(out/'summary.json')
    assert len(rows) == 9 and len({(r['level'], r['variant']) for r in rows}) == 9
    checks = []
    for row in previous.ordered(rows):
        path = Path(row['plan_directory'])
        assert digest(path/'result.json') == row['plan_result_sha256']
        again = assess(out, row['level'], row['variant'])
        assert again['verdict'] == row['verdict'] and again['full_trajectory_time_s'] == row['full_trajectory_time_s']
        raw = read(path/'result.json')
        track, _ = scenario.build_case(scenario.Case(level=row['level']))
        p = Planner(track, row['variant'] == 'ours', pose_function=scenario.pose)
        for seed, candidate in zip(p.seeds(), raw['candidates']):
            np.testing.assert_allclose(seed, candidate['initial_variables'], rtol=0, atol=2e-10)
        check = dict(level=row['level'], method=row['variant'], verdict=row['verdict'], independent_initialization=True)
        if raw['found']:
            plan = frozen_plan(raw)
            g, _, coefficients = p.evaluate(plan.x, p.witnesses())
            np.testing.assert_allclose(coefficients, plan.coefficients, rtol=1e-10, atol=1e-7)
            assert g.min() >= -1e-5
            geometric, passed = p.verify(plan)
            assert passed and len(geometric) == 56
            audit = attitude_audit(plan)
            assert audit['passed'] == read(path/'attitude_audit.json')['passed']
            check.update(original_node_minimum=float(g.min()), all_56_geometry_passed=True,
                         attitude_passed=audit['passed'])
        checks.append(check)
    save(out/'validation.json', dict(passed=True, records=9, original_frozen_sources_unchanged=2386,
        all_assessments_recomputed=True, initializations_recomputed=True, checks=checks))


def run(out):
    with open('/tmp/convex_exp2_independent_repair.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        (out/'assessments').mkdir()
        frozen = sources()
        previous.check_prior(frozen)
        queue = [(r['level'], r['variant']) for r in previous.prior_rows()]
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
            targets=queue, seconds_per_seed=300., original_frozen_sources_unchanged=2386,
            independent_cold_starts=True, gazebo_run=False))
        for rel, h in frozen.items():
            target = out/'source_snapshot'/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            old = previous.PRIOR/'source_snapshot'/rel
            if old.is_file() and digest(old) == h:
                os.link(old, target)
            else:
                shutil.copyfile(ROOT/rel, target)
        active, rows = [], []
        save(out/'summary.json', rows)

        def status(state):
            save(out/'status.json', dict(state=state, pid=os.getpid(), completed=len(rows), total=9,
                active=[dict(level=l, method=m, pid=p.pid) for p, log, l, m in active], pending=queue))

        try:
            while queue or active:
                assert sources() == frozen
                while queue and len(active) < 3:
                    level, method = queue.pop(0)
                    log = (out/f'{level}_{method}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE, 'plan', '--level', level,
                        '--method', method, '--outdir', str(out/'plans'/f'{level}_{method}')],
                        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                    active.append((process, log, level, method))
                    print('START', level, method, process.pid, flush=True)
                status('running')
                for item in list(active):
                    process, log, level, method = item
                    code = process.poll()
                    if code is None:
                        continue
                    log.close()
                    active.remove(item)
                    if code:
                        raise RuntimeError(f'child exited {level} {method} {code}')
                    row = assess(out, level, method)
                    save(out/'assessments'/f'{level}_{method}.json', row)
                    rows.append(row)
                    save(out/'summary.json', previous.ordered(rows))
                    report(out, rows, 'running')
                    print('RESULT', level, method, row['verdict'], row['full_trajectory_time_s'], flush=True)
                if queue or active:
                    time.sleep(3)
            status('validating')
            validate(out)
            report(out, rows, 'complete')
            status('complete')
            print('COMPLETE', len(rows), flush=True)
        except BaseException as exc:
            for process, log, level, method in active:
                if process.poll() is None:
                    process.terminate()
                process.wait()
                log.close()
            save(out/'error.json', dict(type=type(exc).__name__, message=str(exc)))
            status('stopped')
            report(out, rows, 'stopped')
            raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('run', 'plan', 'validate'))
    parser.add_argument('--outdir', required=True, type=Path)
    parser.add_argument('--level')
    parser.add_argument('--method', choices=('ours', 'center'))
    args = parser.parse_args()
    out = args.outdir.resolve()
    if args.action == 'run':
        run(out)
    elif args.action == 'validate':
        validate(out)
    else:
        plan(args.level, args.method, out)


if __name__ == '__main__':
    main()
