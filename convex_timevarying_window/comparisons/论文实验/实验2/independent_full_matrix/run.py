"""Complete the frozen factorial with the current independent solver."""
import argparse
from dataclasses import asdict
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
from ..independent_free_solve import run as current
from ..basic_group.whole_course_difficulty import scenario, layout
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import KF

HERE = Path(__file__).resolve().parent
ROOT = current.ROOT
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.independent_full_matrix.run'
ORIGINAL = HERE.parent/'results/togt_whole_course_factorial_v1_20261106'
CURRENT = HERE.parent/'results/independent_free_regression_v1_20261107'
METHODS = current.base.baseline.driver.METHODS
NAMES = current.base.baseline.driver.NAMES
LABELS = current.base.baseline.driver.LABELS
Planner = current.Planner
read, save, digest = current.read, current.save, current.digest


def sources():
    result = current.sources()
    for path in HERE.rglob('*'):
        if path.is_file() and path.suffix in ('.py', '.md'):
            result[str(path.relative_to(ROOT))] = digest(path)
    return result


def ordered(rows):
    return sorted(rows, key=lambda row: (scenario.LEVELS.index(row['level']), METHODS.index(row['variant'])))


def diagnostic(audit):
    limits = np.array([3.8, 3.8, 3.49])
    high, low = float(KF*1000**2), float(KF*150**2)
    rates = np.asarray(audit['maximum_body_rates_radps'])
    return dict(attitude_domain_passed=audit['passed'], maximum_body_rates_radps=rates.tolist(),
        planning_body_rate_limits_radps=limits.tolist(),
        dense_rate_within_planning_limits=bool(np.isfinite(rates).all() and np.all(rates <= limits)),
        maximum_motor_n=audit['maximum_motor_n'], minimum_motor_n=audit['minimum_motor_n'],
        dense_motor_within_planning_reserve=bool(audit['maximum_motor_n'] <= .9*high),
        dense_motor_within_physical_limits=bool(audit['minimum_motor_n'] >= low and audit['maximum_motor_n'] <= high),
        planning_motor_upper_n=.9*high, physical_motor_limits_n=[low, high],
        dynamics_evidence='5ms numerical diagnostic; fixed north tilt/heading pi/2 nominal model')


def time_comparable(row):
    return bool(row['passed'] and row.get('attitude_domain_passed')
        and row.get('dense_rate_within_planning_limits') and row.get('dense_motor_within_physical_limits'))


def retained(frozen):
    result = {}
    for prior in (ORIGINAL, CURRENT):
        protocol = read(prior/'protocol.json')
        assert all(frozen.get(k) == value for k, value in protocol['source_sha256'].items())
        assert read(prior/'validation.json')['passed'] if prior == CURRENT else read(prior/'status.json')['validation_passed']
        for row in read(prior/'summary.json'):
            if prior == ORIGINAL and row['variant'] not in ('dynamic_togt', 'dynamic_togt_center'):
                continue
            key = row['level'], row['variant']
            assert key not in result
            path = Path(row['plan_directory'])
            assert digest(path/'result.json') == row['plan_result_sha256']
            expected = current.base.previous.canonical(scenario.manifest(scenario.Case(level=row['level'])))
            if prior == ORIGINAL:
                case = next(c for c in protocol['cases'] if c['case']['level'] == row['level'])
                assert case == expected
                if row['level'] == 'static':
                    current.base.baseline.static_reuse_evidence(row['variant'])
                else:
                    assert read(path/'protocol.json') == expected
            else:
                assert read(path/'protocol.json')['scene'] == expected
            result[key] = dict(source_directory=str(path), prior=str(prior),
                source_sha256_count=len(protocol['source_sha256']),
                all_source_contents_unchanged=True, target_scene_equal=True,
                files_sha256={str(p.relative_to(path)): digest(p) for p in path.rglob('*')
                    if p.is_file() and '__pycache__' not in p.parts})
    assert len(result) == 25
    return result


def solve(level, method, out):
    assert method in ('ours', 'center')
    current.base.Planner = Planner
    current.base.plan(level, method, out)


def assess(out, level, method):
    path = out/'plans'/f'{level}_{method}'
    raw = read(path/'result.json')
    assert raw['method'] == method and len(raw['candidates']) == 4
    good = [c for c in raw['candidates'] if c['found' if method in ('ours', 'center') else 'success']]
    assert bool(good) == raw['found']
    rho, factor = (0., 1.) if level == 'static' else scenario.CONDITIONS[level]
    reuse = read(path/'matrix_reuse.json') if (path/'matrix_reuse.json').exists() else None
    if reuse:
        for rel, expected in reuse['files_sha256'].items():
            assert digest(Path(reuse['source_directory'])/rel) == expected
            assert digest(path/rel) == expected
    row = dict(level=level, variant=method, amplitude_ratio=rho, frequency_factor=factor,
        plan_directory=str(path), plan_result_sha256=digest(path/'result.json'),
        build_s=raw['build_s'], solve_s=raw['solve_s'],
        planning_total_s=raw['build_s']+raw['solve_s'], gazebo_run=False,
        reused_plan=bool(reuse), cost_origin='retained original measurement' if reuse else 'new measurement',
        reuse_source=reuse['source_directory'] if reuse else None)
    if method in ('ours', 'center'):
        assert raw['independent_cold_start'] and not raw['center_solution_used']
        assert not raw['amplitude_continuation'] and not raw['cross_condition_warm_start']
        assert raw['crossing_points_free_from_first_iteration'] == (method == 'ours')
        assert read(path/'protocol.json')['scene'] == current.base.previous.canonical(scenario.manifest(scenario.Case(level=level)))
    if not raw['found']:
        return dict(row, passed=False, verdict='planning_not_found', full_trajectory_time_s=None,
            collision_detected=None, check_s=0., scene_assumption_passed=None, time_comparison_eligible=False)
    metric = 'nominal_time_s' if method in ('ours', 'center') else 'objective'
    assert abs(raw['candidates'][raw['selected_seed']][metric]-min(c[metric] for c in good)) < 1e-8
    track, _ = scenario.build_case(scenario.Case(level=level))
    assessment = current.base.baseline.evaluate(raw['coefficients'], raw['durations'], track, scenario.ASSETS/'meshes')
    assert abs(assessment['full_trajectory_time_s']-raw['nominal_time_s']) < 1e-8
    plan = current.base.frozen_plan(raw)
    audit = current.base.attitude_audit(plan)
    dynamics = diagnostic(audit)
    save(out/'assessments'/f'{level}_{method}_geometry.json', assessment)
    save(out/'assessments'/f'{level}_{method}_attitude.json', audit)
    save(out/'assessments'/f'{level}_{method}_dynamics.json', dynamics)
    row.update(**assessment, **layout.foreign_diagnostics(assessment), **dynamics)
    row['time_comparison_eligible'] = time_comparable(row)
    return row


def report(out, rows, state):
    lines = ['# 实验二独立求解完整对比', '', f'状态 {state}，已结束 {len(rows)}/40 项。', '',
        '固定原七窗与运动，静态加3×3幅频矩阵，初相0。本文两方使用同一当前独立求解器，各自四初值。',
        '时间包含七窗后返航至零VAJ，单位秒。表中时间属于名义几何无碰撞完成。†表示额外5ms诊断存在姿态/角速/物理电机问题。', '',
        '|幅度/窗宽|频率倍数|本文自由点|本文强制中心|动态TOGT自由点|动态TOGT中心|',
        '|---|---:|---:|---:|---:|---:|']
    comparisons = []
    for level in scenario.LEVELS:
        by = {r['variant']: r for r in rows if r['level'] == level}
        rho, factor = (0., 1.) if level == 'static' else scenario.CONDITIONS[level]
        cells = []
        for method in METHODS:
            row = by.get(method)
            cell = LABELS['pending'] if row is None else LABELS[row['verdict']]
            if row is not None and row['passed']:
                cell = f"{row['full_trajectory_time_s']:.3f}"+('†' if not row['time_comparison_eligible'] else '')
            cells.append(cell)
        lines.append('|'+('静态' if level == 'static' else f'{rho:g}')+f'|{factor:g}|'+'|'.join(cells)+'|')
        item = dict(level=level)
        for a, b, key in [('ours', 'center', 'ours_vs_center_saving_percent'),
                          ('ours', 'dynamic_togt', 'ours_vs_togt_saving_percent'),
                          ('dynamic_togt', 'dynamic_togt_center', 'togt_free_vs_center_saving_percent')]:
            x, y = by.get(a), by.get(b)
            item[key] = 100*(y['full_trajectory_time_s']-x['full_trajectory_time_s'])/y['full_trajectory_time_s'] if x and y and time_comparable(x) and time_comparable(y) else None
        comparisons.append(item)
    lines += ['', '各窗基准频率0.30/0.40/0.50/0.60/0.55/0.45/0.35Hz。幅度为各窗实际水平开口宽度的比例。',
        '', '## 节点间动力学与全部成本', '',
        '|条件|方法|几何结果|5ms角速|5ms电机物理界|90%推力预留界|完整规划 s|来源|',
        '|---|---|---|---|---|---|---:|---|']
    for row in ordered(rows):
        word = lambda key: '—' if key not in row else '在界内' if row[key] else '超限/待定'
        lines.append(f"|{row['level']}|{NAMES[row['variant']]}|{LABELS[row['verdict']]}|"
            f"{word('dense_rate_within_planning_limits')}|{word('dense_motor_within_physical_limits')}|"
            f"{word('dense_motor_within_planning_reserve')}|{row['planning_total_s']:.3f}|{row['cost_origin']}|")
    lines += ['', '时间优势仅在双方名义几何完成、姿态诊断有效、5ms角速度在原界内且电机在实际模型界内时计算，见analysis.json。',
        '动力学约束仍为固定节点硬约束，5ms结果属于额外数值诊断。名义几何与动力学诊断分开，完整连续动力学与实飞分别评估。',
        '25条冻结结果保留原始文件哈希及原测量成本，15条新算。全部40条重新几何/姿态/动力学评估。',
        '本文每初值300秒，动态TOGT沿用原180秒预算及原目标择解；所有内部失败与全部成本保留。',
        '', '![赛道布局](course_layout.png)']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(out/'analysis.json', dict(state=state, completed=len(rows), total=40, comparisons=comparisons,
        by_method={m: dict(geometric_no_collision=sum(r['variant'] == m and r['passed'] for r in rows),
            collision=sum(r['variant'] == m and r['verdict'] == 'nominal_collision' for r in rows),
            planning_not_found=sum(r['variant'] == m and r['verdict'] == 'planning_not_found' for r in rows),
            time_comparison_eligible=sum(r['variant'] == m and r['time_comparison_eligible'] for r in rows)) for m in METHODS}))


def validate(out):
    frozen = read(out/'protocol.json')['source_sha256']
    assert sources() == frozen
    rows = read(out/'summary.json')
    assert len(rows) == 40 and {(r['level'], r['variant']) for r in rows} == {(l, m) for l in scenario.LEVELS for m in METHODS}
    checks = []
    for row in ordered(rows):
        path = Path(row['plan_directory'])
        raw = read(path/'result.json')
        assert digest(path/'result.json') == row['plan_result_sha256']
        again = assess(out, row['level'], row['variant'])
        for key in ('verdict', 'passed', 'full_trajectory_time_s', 'time_comparison_eligible',
                    'dense_rate_within_planning_limits', 'dense_motor_within_physical_limits'):
            assert again.get(key) == row.get(key)
        check = dict(level=row['level'], method=row['variant'], verdict=row['verdict'],
            time_comparison_eligible=row['time_comparison_eligible'])
        if row['variant'] in ('ours', 'center'):
            track, _ = scenario.build_case(scenario.Case(level=row['level']))
            p = Planner(track, row['variant'] == 'ours', pose_function=scenario.pose)
            assert current.base.previous.canonical(asdict(p.cfg)) == raw['config']
            for seed, candidate in zip(p.seeds(), raw['candidates']):
                np.testing.assert_array_equal(seed, candidate['initial_variables'])
            check['four_analytic_initializations_recomputed'] = True
            if raw['found']:
                plan = current.base.frozen_plan(raw)
                g, _, c = p.evaluate(plan.x, p.witnesses())
                np.testing.assert_allclose(c, plan.coefficients, rtol=1e-10, atol=1e-7)
                geometry, passed = p.verify(plan)
                assert g.min() >= -1e-5 and passed and len(geometry) == 56
                final = raw['candidates'][raw['selected_seed']]['statistics']['records'][-1]
                assert final['optimizer_converged'] and final['global_passed']
                check.update(original_node_minimum=float(g.min()), all_56_geometry_passed=True)
        checks.append(check)
    retained(frozen)
    save(out/'validation.json', dict(passed=True, records=40, current_source_entries=len(frozen),
        all_assessments_recomputed=True, all_80_ours_initializations_recomputed=True,
        inherited_files_and_sources_rechecked=True, source_hashes_unchanged=True, checks=checks))


def run(out):
    with open('/tmp/convex_exp2_independent_full.lock', 'a') as lock, open('/tmp/convex_exp2_independent_free.lock', 'a') as old_lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(old_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        frozen = sources()
        reuse = retained(frozen)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        (out/'assessments').mkdir()
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
            cases=[scenario.manifest(scenario.Case(level=l)) for l in scenario.LEVELS],
            methods=METHODS, total=40, retained=25, new_plans=15, workers=4,
            ours_seconds_per_seed=300., togt_original_seconds_per_seed=180., gazebo_run=False,
            no_cross_condition_or_method_warm_start=True))
        prior_sources = read(CURRENT/'protocol.json')['source_sha256']
        for rel, h in frozen.items():
            target = out/'source_snapshot'/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            old = CURRENT/'source_snapshot'/rel
            if prior_sources.get(rel) == h and old.is_file():
                os.link(old, target)
            else:
                shutil.copyfile(ROOT/rel, target)
        for name in ('course_layout.png', 'course_layout.pdf'):
            shutil.copyfile(ORIGINAL/name, out/name)
        save(out/'geometry_audit.json', layout.audit())
        save(out/'motion_audit.json', layout.motion_audit())
        assert read(out/'geometry_audit.json')['passed'] and read(out/'motion_audit.json')['passed']
        rows, active = [], []
        queue = [(l, m) for l in scenario.LEVELS for m in ('ours', 'center') if (l, m) not in reuse]

        def status(state):
            save(out/'status.json', dict(state=state, pid=os.getpid(), completed=len(rows), total=40,
                active=[dict(level=l, method=m, pid=p.pid) for p, log, l, m in active], pending=queue))

        def record(level, method):
            row = assess(out, level, method)
            rows.append(row)
            save(out/'assessments'/f'{level}_{method}.json', row)
            save(out/'summary.json', ordered(rows))
            report(out, rows, 'running')
            print('RESULT', level, method, row['verdict'], row['full_trajectory_time_s'], flush=True)
            if row['scene_assumption_passed'] is False:
                raise RuntimeError('nonincident frame risk requires diagnosis '+level+' '+method)

        save(out/'summary.json', rows)
        status('retaining_and_reassessing')
        try:
            for (level, method), evidence in reuse.items():
                path = out/'plans'/f'{level}_{method}'
                shutil.copytree(evidence['source_directory'], path, ignore=shutil.ignore_patterns('__pycache__'))
                save(path/'matrix_reuse.json', evidence)
                record(level, method)
                status('retaining_and_reassessing')
            while queue or active:
                assert sources() == frozen
                while queue and len(active) < 4:
                    level, method = queue.pop(0)
                    log = (out/f'{level}_{method}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE, 'solve', '--level', level,
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
                        raise RuntimeError(f'worker exited {level} {method} {code}')
                    record(level, method)
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
    parser.add_argument('action', choices=('run', 'solve', 'validate'))
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--level', choices=scenario.LEVELS)
    parser.add_argument('--method', choices=('ours', 'center'))
    args = parser.parse_args()
    if args.action == 'run':
        run(args.outdir.resolve())
    elif args.action == 'validate':
        validate(args.outdir.resolve())
    else:
        solve(args.level, args.method, args.outdir.resolve())


if __name__ == '__main__':
    main()
