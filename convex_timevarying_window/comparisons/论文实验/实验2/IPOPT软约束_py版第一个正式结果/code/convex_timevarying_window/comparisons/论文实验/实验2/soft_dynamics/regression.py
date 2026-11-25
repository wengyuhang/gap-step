"""Frozen independent numerical-repair regression with unchanged geometry."""
import argparse
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

import numpy as np

from . import run as common
from .planner import Planner
from ..no_tilt_comparison.run import sources, preserve_old_records
from ..no_tilt_comparison.worker import METHODS

MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.no_tilt_comparison.worker'
LEVELS = ('static', 'mixed_base', 'a25_f6', 'a75_f6')


def report(out, rows):
    lines = ['# 独立求解修复回归', '',
        f'已完成{len(rows)}/{len(LEVELS)*len(METHODS)}条规划。全部使用固定32等分积分、零倾角罚项和原四独立时间初值。', '',
        '单位秒，包含七窗后返航。碰撞格直接显示碰撞，名义球/STL检查与Gazebo实飞分开。', '',
        '|条件|本文自由点·软动力学|本文中心·软动力学|动态TOGT自由点|动态TOGT中心|', '|---|---:|---:|---:|---:|']
    for level in LEVELS:
        by = {r['variant']: r for r in rows if r['level'] == level}
        good = [r['full_trajectory_time_s'] for r in by.values() if r['passed']]
        cells = []
        for method in METHODS:
            row = by.get(method)
            value = common.label(row, 'planning')
            if row and row['passed'] and good and abs(row['full_trajectory_time_s']-min(good)) < 1e-9:
                value = '**'+value+'**'
            cells.append(value)
        lines.append('|'+level+'|'+'|'.join(cells)+'|')
    lines += ['', '每个条件各方法从当前几何和解析时长独立求解。原始四初值、全部候选、目标和约束证据保留。',
        '本文初始PVAJ由MINCO线性系统计算，求解时完整PVAJ保持自由。软超限量用等价辅助变量表示，原C++目标另行重算并核对。',
        '当前结果为规划回归，真实PX4/Gazebo执行需另列。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def validate(out):
    checks = []
    for row in common.read(out/'planning_summary.json'):
        folder = Path(row['directory'])
        raw = common.read(folder/'result.json')
        assert len(raw['candidates']) == 4 and raw['tilt_penalty_weight'] == 0.
        assert not raw.get('external_solution_used', False)
        if row['variant'] in ('ours', 'center'):
            track, _ = common.scenario.build_case(common.scenario.Case(level=row['level']))
            planner = Planner(track, row['variant'] == 'ours', pose_function=common.scenario.pose)
            for seed, candidate in zip(planner.seeds(), raw['candidates']):
                np.testing.assert_array_equal(seed, candidate['initial_variables'])
            good = [c for c in raw['candidates'] if c['found']]
            assert bool(good) == raw['found']
            if raw['found']:
                plan = common.frozen_plan(raw)
                value, _, penalty = planner.objective(plan.x)
                np.testing.assert_allclose(value, raw['objective'], rtol=1e-10, atol=1e-7)
                assert abs(value-min(c['objective'] for c in good)) < 1e-7
                margins, _, coefficients = planner.evaluate(plan.x, planner.witnesses())
                np.testing.assert_allclose(coefficients, plan.coefficients, atol=1e-7, rtol=1e-10)
                assert margins.min() >= -1e-5 and planner.verify(plan)[1]
                check_calls = [call for call in raw['optimizer_calls']
                    if call['stage'] in ('equivalent_soft_epigraph', 'direct_soft_slsqp') and call['success']]
                expected = 'Optimization terminated successfully' if raw.get('numerical_solver') == 'slsqp' else 'Solve_Succeeded'
                assert check_calls and all(call['status'] == expected for call in check_calls)
        again = common.assess(out, row['level'], row['variant'])
        assert again['outcome'] == row['outcome']
        checks.append(dict(level=row['level'], variant=row['variant'], outcome=row['outcome'],
            original_four_analytic_initializers_verified=True, geometry_recomputed=True))
    assert len(checks) == len(LEVELS)*len(METHODS)
    assert sources() == common.read(out/'protocol.json')['source_sha256']
    prior = common.read(out/'prior_results_sha256.json')
    assert all(common.previous.digest(Path(p)) == h for p, h in prior.items())
    common.save(out/'validation.json', dict(passed=True, checks=checks,
        frozen_sources_unchanged=True, prior_raw_records_unchanged=len(prior), external_solution_inputs=[]))


def run(out):
    with open('/tmp/convex_soft_solver_regression.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        frozen = sources()
        common.snapshot(out, frozen)
        common.save(out/'prior_results_sha256.json', preserve_old_records())
        common.save(out/'protocol.json', dict(levels=LEVELS, methods=METHODS, source_sha256=frozen,
            protocol=(Path(__file__).parent/'PROTOCOL.md').read_text(), tests_passed=18,
            fixed_quadrature_intervals=32, tilt_penalty_weight=0., external_solution_inputs=[],
            independent_cold_starts=4, gazebo_run=False))
        queue = [(level, method) for level in LEVELS for method in METHODS]
        active, rows = [], []
        common.save(out/'planning_summary.json', rows)
        try:
            while queue or active:
                assert sources() == frozen
                while queue and len(active) < 4:
                    level, method = queue.pop(0)
                    log = (out/f'{level}_{method}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE,
                        '--level', level, '--method', method, '--outdir', str(out/'plans'/f'{level}_{method}')],
                        cwd=common.ROOT, stdout=log, stderr=subprocess.STDOUT)
                    active.append((process, log, level, method))
                    print('START', level, method, process.pid, flush=True)
                for item in list(active):
                    process, log, level, method = item
                    if process.poll() is None:
                        continue
                    active.remove(item)
                    log.close()
                    if process.returncode:
                        raise RuntimeError(f'{level} {method} worker exit {process.returncode}')
                    row = common.assess(out, level, method)
                    rows.append(row)
                    common.save(out/'planning_summary.json', rows)
                    print('RESULT', level, method, row['outcome'], row['full_trajectory_time_s'], flush=True)
                report(out, rows)
                common.save(out/'status.json', dict(state='running', pid=os.getpid(), completed=len(rows),
                    total=len(LEVELS)*len(METHODS), active=[dict(pid=p.pid, level=l, method=m) for p, log, l, m in active]))
                if queue or active:
                    time.sleep(10)
            validate(out)
            common.save(out/'status.json', dict(state='complete', pid=os.getpid(), completed=len(rows), validation_passed=True))
        except BaseException as exc:
            for process, log, level, method in active:
                if process.poll() is None:
                    process.terminate()
                process.wait()
                log.close()
            common.save(out/'error.json', dict(error=repr(exc), traceback=traceback.format_exc()))
            common.save(out/'status.json', dict(state='stopped', pid=os.getpid(), completed=len(rows), error=repr(exc)))
            raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    run(parser.parse_args().outdir.resolve())
