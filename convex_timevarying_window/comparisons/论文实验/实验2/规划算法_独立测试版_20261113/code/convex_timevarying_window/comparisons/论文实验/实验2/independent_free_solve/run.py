"""Independent four-cold-start free-point regression, without center solves."""
import argparse
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
from .planner import Planner
from ..independent_repair import run as base

HERE = Path(__file__).resolve().parent
ROOT = base.ROOT
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.independent_free_solve.run'
PRIOR = HERE.parent/'results/independent_cold_regression_v2_20261107'
LEVELS = [r['level'] for r in base.previous.prior_rows() if r['variant'] == 'ours']
read, save, digest = base.read, base.save, base.digest


def sources():
    result = base.sources()
    for path in HERE.rglob('*'):
        if path.is_file() and path.suffix in ('.py', '.md'):
            result[str(path.relative_to(ROOT))] = digest(path)
    return result


def solve(level, out):
    base.Planner = Planner
    base.plan(level, 'ours', out)


def assess(out, level):
    row = base.assess(out, level, 'ours')
    raw = read(Path(row['plan_directory'])/'result.json')
    assert not raw['center_solution_used'] and not raw['center_subproblem_solved']
    assert raw['crossing_points_free_from_first_iteration']
    row['planning_total_s'] = raw['build_s']+raw['solve_s']
    return row


def report(out, rows, state):
    lines = ['# 独立自由选点回归', '', f'状态 {state}，已结束 {len(rows)}/5 条。', '',
        '每条赛道独立四个解析初值，时间、选点和PVAJ从第一次优化起全部开放。',
        '七窗后返航总时长为时间指标，全部初值和失败成本计入规划耗时。', '',
        '|幅度/窗宽|频率倍数|结果|完整轨迹 s|全部规划成本 s|',
        '|---|---:|---|---:|---:|']
    for row in base.previous.ordered(rows):
        duration = f"{row['full_trajectory_time_s']:.6f}" if row['passed'] else '—'
        lines.append(f"|{row['amplitude_ratio']:g}|{row['frequency_factor']:g}|"
            f"{base.baseline.driver.LABELS[row['verdict']]}|{duration}|{row['planning_total_s']:.3f}|")
    lines += ['', '所有候选与终止原因见plans。名义轨迹评估，动力学为节点硬约束。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def validate(out):
    assert sources() == read(out/'protocol.json')['source_sha256']
    rows = read(out/'summary.json')
    assert len(rows) == 5 and {r['level'] for r in rows} == set(LEVELS)
    checks = []
    for row in rows:
        path = Path(row['plan_directory'])
        raw = read(path/'result.json')
        assert digest(path/'result.json') == row['plan_result_sha256']
        again = assess(out, row['level'])
        assert again['verdict'] == row['verdict']
        assert again['full_trajectory_time_s'] == row['full_trajectory_time_s']
        track, _ = base.scenario.build_case(base.scenario.Case(level=row['level']))
        p = Planner(track, True, pose_function=base.scenario.pose)
        assert len(raw['candidates']) == 4
        for seed, candidate in zip(p.seeds(), raw['candidates']):
            np.testing.assert_array_equal(seed, candidate['initial_variables'])
        check = dict(level=row['level'], verdict=row['verdict'], independent_free_initialization=True)
        if raw['found']:
            plan = base.frozen_plan(raw)
            g, _, c = p.evaluate(plan.x, p.witnesses())
            np.testing.assert_allclose(c, plan.coefficients, rtol=1e-10, atol=1e-7)
            assert g.min() >= -1e-5
            geometry, passed = p.verify(plan)
            assert passed and len(geometry) == 56
            assert base.attitude_audit(plan)['passed']
            final = raw['candidates'][raw['selected_seed']]['statistics']['records'][-1]
            assert final['optimizer_converged'] and final['global_passed']
            check.update(original_node_minimum=float(g.min()), all_56_geometry_passed=True)
        checks.append(check)
    save(out/'validation.json', dict(passed=True, records=len(rows), original_frozen_sources_unchanged=base.previous.check_prior(sources()),
        all_assessments_recomputed=True, all_twenty_initializations_recomputed=True,
        no_center_subproblem_or_solution=True, source_hashes_unchanged=True, checks=checks))


def run(out):
    with open('/tmp/convex_exp2_independent_free.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        (out/'assessments').mkdir()
        frozen = sources()
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
            levels=LEVELS, center_solution_used=False, independent_free_cold_starts=True, gazebo_run=False))
        for rel, h in frozen.items():
            target = out/'source_snapshot'/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            old = PRIOR/'source_snapshot'/rel
            if old.is_file() and digest(old) == h:
                os.link(old, target)
            else:
                shutil.copyfile(ROOT/rel, target)
        queue, active, rows = LEVELS.copy(), [], []
        save(out/'summary.json', rows)
        report(out, rows, 'running')

        def status(state):
            save(out/'status.json', dict(state=state, pid=os.getpid(), completed=len(rows), total=5,
                active=[dict(level=l, pid=p.pid) for p, log, l in active], pending=queue))

        try:
            while queue or active:
                assert sources() == frozen
                while queue and len(active) < 2:
                    level = queue.pop(0)
                    log = (out/f'{level}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE, 'solve',
                        '--outdir', str(out/'plans'/f'{level}_ours'), '--level', level],
                        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                    active.append((process, log, level))
                    print('START', level, process.pid, flush=True)
                status('running')
                for item in list(active):
                    process, log, level = item
                    code = process.poll()
                    if code is None:
                        continue
                    log.close()
                    active.remove(item)
                    if code:
                        raise RuntimeError(f'worker exited {level} {code}')
                    row = assess(out, level)
                    rows.append(row)
                    save(out/'summary.json', base.previous.ordered(rows))
                    report(out, rows, 'running')
                    print('RESULT', level, row['verdict'], row['full_trajectory_time_s'], flush=True)
                if queue or active:
                    time.sleep(3)
            status('validating')
            validate(out)
            report(out, rows, 'complete')
            status('complete')
        except BaseException as exc:
            for process, log, level in active:
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
    parser.add_argument('action', choices=('run', 'solve', 'validate'))
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--level', choices=base.scenario.LEVELS)
    args = parser.parse_args()
    if args.action == 'run':
        run(args.outdir.resolve())
    elif args.action == 'validate':
        validate(args.outdir.resolve())
    else:
        solve(args.level, args.outdir.resolve())


if __name__ == '__main__':
    main()
