"""Freeze, run, and independently assess the two-method ten-condition matrix."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

from .planner import Planner
from ..basic_group.whole_course_difficulty import scenario
from ..basic_group.trajectory_only.collision import evaluate
from ..independent_repair.run import frozen_plan, attitude_audit
from ..gazebo_matrix import run as previous, archive
from ..radius_ablation.execution import audit as flight_audit

HERE = Path(__file__).resolve().parent
ROOT = scenario.ROOT
RESULTS = HERE.parent/'results'
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics'
METHODS = ('ours', 'center')
NAMES = dict(ours='本文自由点·软动力学', center='本文中心·软动力学')


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    temporary = path.with_name(path.name+'.new')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def sources():
    result = previous.sources()
    for path in HERE.rglob('*'):
        if path.is_file() and not {'build', '__pycache__'}.intersection(path.relative_to(HERE).parts):
            result[str(path.relative_to(ROOT))] = previous.digest(path)
    for relative in ('convex_dynamic_seven_window_gazebo/x500_togt/native/togt_x500.cpp',
        'convex_dynamic_seven_window_gazebo/x500_togt/native/build/libtogt_x500.so',
        str((HERE/'native/build/libpolynomial_penalty.so').relative_to(ROOT)),
        'convex_timevarying_window/comparisons/论文实验/实验2/radius_ablation/execution.py'):
        result[relative] = previous.digest(ROOT/relative)
    return result


def old_records():
    records = {}
    base = RESULTS/'gazebo_all_selected_v1_20261107'
    for relative, entries in read(base/'record_sha256.json').items():
        for name, expected in entries.items():
            path = base/relative/name
            assert previous.digest(path) == expected, path
            records[str(path)] = expected
    for dirname in ('independent_full_matrix_v2_20261107', 'wang2023_full_v1_20261107'):
        for path in (RESULTS/dirname/'plans').glob('*/result.json'):
            records[str(path)] = previous.digest(path)
    return records


def snapshot(out, frozen):
    prior = RESULTS/'gazebo_a75_f6_ours_view_v1_20261107/source_snapshot'
    for relative, expected in frozen.items():
        target = out/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        source = prior/relative
        if source.is_file() and previous.digest(source) == expected:
            os.link(source, target)
        else:
            shutil.copyfile(ROOT/relative, target)


def assess(out, level, method):
    folder = out/'plans'/f'{level}_{method}'
    raw = read(folder/'result.json')
    row = dict(level=level, variant=method, directory=str(folder), result_sha256=previous.digest(folder/'result.json'),
        found=raw['found'], build_s=raw['build_s'], solve_s=raw['solve_s'],
        planning_total_s=raw['build_s']+raw['solve_s'], full_trajectory_time_s=raw.get('nominal_time_s'),
        objective=raw.get('objective'), dynamic_soft_integral=raw.get('dynamic_soft_integral'),
        outcome='planning_not_found', passed=False, frame_collision=False)
    if not raw['found']:
        return row
    track, _ = scenario.build_case(scenario.Case(level=level))
    geometry = evaluate(raw['coefficients'], raw['durations'], track, scenario.ASSETS/'meshes')
    save(folder/'assessment.json', geometry)
    collision = any(c['outcome'] == 'collision' for c in geometry['checks'])
    clear = all(c['outcome'] == 'clear' for c in geometry['checks'])
    task = geometry['ordered_exactly_once'] and geometry['endpoints_passed']
    outcome = 'collision' if collision else 'clear' if clear and task else 'task_invalid' if not task else 'unresolved'
    diagnostic = read(folder/'attitude_audit.json')
    row.update(outcome=outcome, passed=outcome == 'clear', frame_collision=collision,
        trajectory_distance_m=geometry['trajectory_distance_m'], ground_outcome_excluded=geometry['ground'],
        ordered_exactly_once=geometry['ordered_exactly_once'], endpoints_passed=geometry['endpoints_passed'],
        diagnostic_attitude_domain_passed=diagnostic['passed'],
        diagnostic_maximum_body_rates_radps=diagnostic['maximum_body_rates_radps'],
        diagnostic_minimum_motor_n=diagnostic['minimum_motor_n'],
        diagnostic_maximum_motor_n=diagnostic['maximum_motor_n'])
    return row


def label(row, stage):
    if row is None:
        return '待运行'
    outcome = row['outcome'] if stage == 'planning' else row['verdict']
    if outcome == 'collision' or row.get('frame_collision'):
        return '碰撞'
    if outcome == 'planning_not_found':
        return '未找到规划'
    if outcome in ('unresolved', 'evidence_failure'):
        return '证据不足'
    if not row['passed']:
        return '未完成'
    key = 'full_trajectory_time_s' if stage == 'planning' else 'full_reference_execution_time_s'
    return f"{row[key]:.3f}"


def report(out, rows, stage, state):
    filename = 'PLANNING.md' if stage == 'planning' else 'FLIGHT.md'
    lines = [f"# {'规划' if stage == 'planning' else '实飞'}结果", '',
        f'状态为{state}，已登记{len(rows)}/20项。', '',
        '单位秒，地面项单列排除，碰撞格只显示碰撞。各行双方有效时间最短项加粗。', '',
        '|幅度/窗宽|频率倍数|本文自由点·软动力学|本文中心·软动力学|', '|---|---:|---:|---:|']
    for level in scenario.LEVELS:
        by = {r['variant']: r for r in rows if r['level'] == level}
        cells = [label(by.get(m), stage) for m in METHODS]
        key = 'full_trajectory_time_s' if stage == 'planning' else 'full_reference_execution_time_s'
        good = [r[key] for r in by.values() if r.get('passed')]
        for i, method in enumerate(METHODS):
            if good and by.get(method, {}).get('passed') and abs(by[method][key]-min(good)) < 1e-9:
                cells[i] = '**'+cells[i]+'**'
        ratio, factor = (0., 1.) if level == 'static' else scenario.CONDITIONS[level]
        lines.append('|'+('静态' if level == 'static' else f'{ratio:g}')+f'|{factor:g}|'+'|'.join(cells)+'|')
    if stage == 'planning':
        lines += ['', '完整八段参考时间含七窗后返航。目标为时间加原生TOGT动力学软积分，按总目标选择四初值最优候选。',
            '动力学超限诊断单列，几何与任务验收决定上表分类。此表为名义规划证据。', '',
            '|条件|方法|参考路程 m|动力学软积分|完整目标|全部规划成本 s|', '|---|---|---:|---:|---:|---:|']
        for row in ordered(rows):
            number = lambda key: '—' if row.get(key) is None else f'{row[key]:.6f}'
            lines.append(f"|{row['level']}|{NAMES[row['variant']]}|{number('trajectory_distance_m')}|{number('dynamic_soft_integral')}|{number('objective')}|{row['planning_total_s']:.3f}|")
    else:
        lines += ['', '新实飞执行原参考全部时长，竞赛时钟和状态时间达到T后结束，取消旧0.55m/1m/s提前结束规则。',
            '本表指标为完整参考执行用时，真实末端误差和速度单列。精确零误差停稳的到达时刻没有用参考T替代。', '',
            '|条件|方法|真实七窗计数|末端误差 m|末端速度 m/s|', '|---|---|---:|---:|---:|']
        for row in ordered(rows):
            position = '—' if row.get('truth_goal_error_m') is None else f"{row['truth_goal_error_m']:.6f}"
            speed = '—' if row.get('truth_goal_speed_mps') is None else f"{row['truth_goal_speed_mps']:.6f}"
            lines.append(f"|{row['level']}|{NAMES[row['variant']]}|{row.get('actual_gate_count', '—')}|{position}|{speed}|")
    (out/filename).write_text('\n'.join(lines)+'\n')


def ordered(rows):
    return sorted(rows, key=lambda r: (scenario.LEVELS.index(r['level']), METHODS.index(r['variant'])))


def validate_planning(out):
    assert sources() == read(out/'protocol.json')['source_sha256']
    checks = []
    for row in ordered(read(out/'planning_summary.json')):
        folder = Path(row['directory'])
        raw = read(folder/'result.json')
        assert previous.digest(folder/'result.json') == row['result_sha256']
        track, _ = scenario.build_case(scenario.Case(level=row['level']))
        planner = Planner(track, row['variant'] == 'ours', pose_function=scenario.pose,
            dynamics_model=raw.get('dynamics_model', 'togt'),
            dynamic_limit_fraction=raw.get('dynamic_limit_fraction', 1.))
        assert len(raw['candidates']) == 4 and raw['all_four_attempted']
        for seed, candidate in zip(planner.seeds(), raw['candidates']):
            np.testing.assert_array_equal(seed, candidate['initial_variables'])
            assert not any(r.get('status') == 'wall_budget' for r in candidate['statistics']['records'])
        good = [c for c in raw['candidates'] if c['found']]
        assert bool(good) == raw['found']
        check = dict(level=row['level'], variant=row['variant'], outcome=row['outcome'], all_initializers_equal=True)
        if raw['found']:
            plan = frozen_plan(raw)
            cost, _, penalty = planner.objective(plan.x)
            np.testing.assert_allclose(cost, raw['objective'], rtol=1e-10, atol=1e-7)
            np.testing.assert_allclose(penalty, raw['dynamic_soft_integral'], rtol=1e-10, atol=1e-7)
            assert abs(cost-min(c['objective'] for c in good)) < 1e-7
            g, _, curve = planner.evaluate(plan.x, planner.witnesses())
            np.testing.assert_allclose(curve, plan.coefficients, rtol=1e-10, atol=1e-7)
            assert g.min() >= -1e-5
            geometry, passed = planner.verify(plan)
            assert len(geometry) == 56 and passed
            again = assess(out, row['level'], row['variant'])
            assert again['outcome'] == row['outcome']
            diagnostic = attitude_audit(plan)
            np.testing.assert_allclose(diagnostic['maximum_body_rates_radps'], row['diagnostic_maximum_body_rates_radps'], atol=1e-8)
            check.update(objective_recomputed=True, minimum_hard_constraint=float(g.min()), geometry_passed=True)
        checks.append(check)
    assert len(checks) == 20
    assert sources() == read(out/'protocol.json')['source_sha256']
    assert all(previous.digest(Path(path)) == expected for path, expected in read(out/'prior_results_sha256.json').items())
    save(out/'planning_validation.json', dict(passed=True, records=20, checks=checks,
        prior_results_unchanged=True, frozen_sources_unchanged=True, four_initializers_per_condition=True))


def plan_matrix(out):
    with open('/tmp/convex_exp2_soft_dynamics_plan.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        frozen = sources()
        snapshot(out, frozen)
        save(out/'prior_results_sha256.json', old_records())
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(),
            source_sha256=frozen, cases=[scenario.manifest(scenario.Case(level=l)) for l in scenario.LEVELS],
            methods=list(METHODS), total=20, wall_time_limit_s_per_seed=None,
            dynamic_objective='unchanged native x500 TOGT penalty with arbitrary polynomial derivatives'))
        queue = [(level, method) for level in scenario.LEVELS for method in METHODS]
        active, rows = [], []
        save(out/'planning_summary.json', rows)
        try:
            while queue or active:
                assert sources() == frozen
                while queue and len(active) < 4:
                    level, method = queue.pop(0)
                    log = (out/f'{level}_{method}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE+'.worker',
                        '--level', level, '--method', method, '--outdir', str(out/'plans'/f'{level}_{method}')],
                        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                    active.append((process, log, level, method))
                    print('PLAN_START', level, method, process.pid, flush=True)
                for item in list(active):
                    process, log, level, method = item
                    code = process.poll()
                    if code is None:
                        continue
                    log.close()
                    active.remove(item)
                    if code:
                        raise RuntimeError(f'worker exited {level} {method} {code}')
                    row = assess(out, level, method)
                    rows.append(row)
                    save(out/'planning_summary.json', ordered(rows))
                    print('PLAN_RESULT', level, method, row['outcome'], row['full_trajectory_time_s'], flush=True)
                report(out, rows, 'planning', 'running')
                save(out/'planning_status.json', dict(state='running', pid=os.getpid(), completed=len(rows), total=20,
                    active=[dict(pid=p.pid, level=l, method=m) for p, log, l, m in active], pending=queue))
                if queue or active:
                    time.sleep(5)
            save(out/'planning_status.json', dict(state='validating', pid=os.getpid(), completed=20, total=20))
            validate_planning(out)
            report(out, rows, 'planning', 'complete')
            save(out/'planning_status.json', dict(state='complete', pid=os.getpid(), completed=20, total=20, validation_passed=True))
        except BaseException as exc:
            for process, log, level, method in active:
                if process.poll() is None:
                    process.terminate()
                process.wait()
                log.close()
            save(out/'planning_error.json', dict(type=type(exc).__name__, message=str(exc)))
            save(out/'planning_status.json', dict(state='stopped', pid=os.getpid(), completed=len(rows), failure=repr(exc)))
            raise


def fly_matrix(out):
    assert read(out/'planning_validation.json')['passed']
    assert sources() == read(out/'protocol.json')['source_sha256']
    with open('/tmp/convex_exp2_gazebo_matrix.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        (out/'flights').mkdir(exist_ok=False)
        rows = []
        plans = read(out/'planning_summary.json')
        save(out/'flight_summary.json', rows)
        for index, level in enumerate(scenario.LEVELS):
            for method in METHODS if index%2 == 0 else tuple(reversed(METHODS)):
                assert sources() == read(out/'protocol.json')['source_sha256']
                plan = next(r for r in plans if (r['level'], r['variant']) == (level, method))
                if not plan['found']:
                    rows.append(dict(level=level, variant=method, passed=False, verdict='planning_not_found',
                        frame_collision=False, full_reference_execution_time_s=None, actual_gate_count=None))
                else:
                    if shutil.disk_usage(ROOT).free < 3*2**30:
                        raise RuntimeError('insufficient_disk_for_native_flight')
                    source = Path(plan['directory'])/'result.json'
                    folder = out/'flights'/f'{level}_{method}'
                    log = (out/f'{level}_{method}_flight.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE+'.flight', '--source', str(source),
                        '--method', method, '--outdir', str(folder)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                    save(out/'flight_status.json', dict(state='flying', pid=os.getpid(), worker_pid=process.pid,
                        level=level, method=method, completed=len(rows), total=20))
                    print('FLY', level, method, process.pid, flush=True)
                    code = process.wait(timeout=900)
                    log.close()
                    if code or not (folder/'comparison_audit.json').exists():
                        save(out/'flight_status.json', dict(state='stopped', level=level, method=method,
                            completed=len(rows), worker_exit_code=code))
                        raise RuntimeError(f'flight worker or evidence failure {level} {method} {code}')
                    row = dict(read(folder/'comparison_audit.json'), level=level, variant=method, directory=str(folder),
                        reference_source_sha256=previous.digest(source), attempt=1)
                    archive.pack(folder)
                    with archive.restored(folder):
                        again = flight_audit(folder)
                    for key in ('passed', 'verdict', 'frame_collision', 'actual_gate_count', 'full_reference_execution_time_s'):
                        assert again[key] == row[key]
                    rows.append(row)
                    print('FLIGHT_RESULT', level, method, row['verdict'], row['full_reference_execution_time_s'], flush=True)
                save(out/'flight_summary.json', ordered(rows))
                report(out, rows, 'flight', 'running')
                save(out/'flight_status.json', dict(state='running', pid=os.getpid(), completed=len(rows), total=20))
        assert len(rows) == 20
        for level in scenario.LEVELS:
            folders = [Path(r['directory']) for r in rows if r['level'] == level and r.get('directory')]
            if len(folders) == 2:
                a, b = [read(f/'result.json') for f in folders]
                assert a['parameters'] == b['parameters'] and a['mpc_config'] == b['mpc_config']
                assert previous.digest(folders[0]/'world.sdf') == previous.digest(folders[1]/'world.sdf')
        assert sources() == read(out/'protocol.json')['source_sha256']
        assert all(previous.digest(Path(path)) == expected for path, expected in read(out/'prior_results_sha256.json').items())
        save(out/'flight_validation.json', dict(passed=True, records=20,
            native_flights_reaudited=sum(bool(r.get('directory')) for r in rows),
            same_world_and_mpc_and_parameters=True, prior_results_unchanged=True, frozen_sources_unchanged=True,
            complete_reference_clock_used=True, old_early_threshold_removed=True))
        report(out, rows, 'flight', 'complete')
        save(out/'flight_status.json', dict(state='complete', pid=os.getpid(), completed=20, total=20, validation_passed=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('plan', 'validate-plan', 'flight'))
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    actions = {'plan': plan_matrix, 'validate-plan': validate_planning, 'flight': fly_matrix}
    actions[args.action](args.outdir.resolve())


if __name__ == '__main__':
    main()
