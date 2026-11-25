"""Independent free/center starts followed by one release of the best center result."""
import argparse
from dataclasses import asdict
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import casadi as ca
import numpy as np

from convex_timevarying_window.single_gate_stop_racing import point_mapping
from . import run
from .phase_initialization import Planner
from ..basic_group.whole_course_difficulty import scenario
from ..basic_group.trajectory_only.collision import evaluate
from ..independent_repair.run import frozen_plan, attitude_audit
from ..safety import frame_aperture

LABELS = dict(ours='独立自由点', center='强制中心', release='最佳中心开放选点')


def make_planner(track, free_points):
    return Planner(track, free_points, solver='ipopt', dynamics_model='togt',
        dynamic_limit_fraction=.95, extra_clearance_m=.05, initial_count=8,
        sampling_seed=0, pose_function=scenario.pose)


def release_initial(planner, center):
    """Insert the center point parameters while preserving the optimized curve."""
    original = np.asarray(center['variables'])
    parameters = np.concatenate([point_mapping.inverse(aperture, np.zeros(2))
        for aperture in planner.point_apertures])
    initial = np.r_[original[:planner.n], parameters, original[planner.n:]]
    if initial.shape != (planner.nvar,):
        raise ValueError('Expected the strict-center trajectory variables')
    coefficients = np.asarray(planner.curve_values(initial)).ravel().reshape(
        planner.n, 3, 8).transpose(0, 2, 1)
    np.testing.assert_allclose(coefficients, center['coefficients'], rtol=0., atol=1e-8)
    objective = planner.objective(initial)[0]
    np.testing.assert_allclose(objective, center['objective'], rtol=1e-10, atol=1e-7)
    return initial, dict(passed=True, all_point_variables_open=True,
        free_point_variables=len(parameters), center_initial_time_s=center['nominal_time_s'],
        coefficient_maximum_error=float(np.max(np.abs(coefficients-center['coefficients']))),
        center_objective=center['objective'], free_initial_objective=objective)


def worker(out, job_id):
    job = run.read(out/'jobs'/f'{job_id}.json')
    folder = out/'plans'/job_id
    folder.mkdir(parents=True, exist_ok=False)
    began = time.perf_counter()
    track, _ = scenario.build_case(scenario.Case(level=job['level']))
    planner = make_planner(track, job['method'] != 'center')
    releasing = job['method'] == 'release'
    if releasing:
        center = run.read(out/'best_center_result.json')
        initial, equivalence = release_initial(planner, center)
        run.save(folder/'initial_equivalence.json', equivalence)
    else:
        initial = planner.seeds()[job['seed']]
    planner.seeds = lambda: [initial.copy()]
    description = ('best verified strict-center result with all crossing points opened' if releasing
        else 'independent sampled gate times, queried centers and analytic MINCO PVAJ')
    run.save(folder/'initial.json', dict(job=job, variables=initial.tolist(),
        durations_s=np.exp(initial[:planner.n]).tolist(),
        points=np.asarray(planner.point_function(initial)).T.tolist(),
        coefficients=np.asarray(planner.curve_values(initial)).ravel().tolist(),
        nvar=planner.nvar, point_variables=planner.velocity_start-planner.n,
        optimized_initial_solution_used=releasing, source=description))
    factory = ca.nlpsol

    def normal_initialization(name, plugin, nlp, options):
        options = dict(options)
        for key in ('ipopt.warm_start_bound_push', 'ipopt.warm_start_mult_bound_push',
                'ipopt.warm_start_slack_bound_push'):
            options.pop(key, None)
        return factory(name, plugin, nlp, options)

    ca.nlpsol = normal_initialization
    build_s = time.perf_counter()-began
    try:
        with (folder/'progress.jsonl').open('w') as log:
            def progress(row):
                log.write(json.dumps(row, ensure_ascii=False)+'\n')
                log.flush()
            plan, result = planner.solve_all(np.inf, progress)
    finally:
        ca.nlpsol = factory
    result.update(job=job, method=job['method'], initial_sample_index=job['seed'],
        case=dict(layout='togt_whole', level=job['level']), build_s=build_s,
        config=asdict(planner.cfg), all_four_attempted=False, all_starts_attempted=True,
        number_initial_starts=1, initialization=description,
        optimized_initial_solution_used=releasing, other_method_solution_used=releasing,
        center_solution_used=releasing, external_solution_used=releasing,
        center_result_initialization=releasing, point_variables=planner.velocity_start-planner.n,
        additional_clearance_m=.05, crossing_points_mapped_into_inset=planner.free_points,
        center_point_initialization=True, global_initial_time_scaling=False, sampling_seed=0,
        finite_safety_radius_m=.62594472, verification_safety_radius_m=.62094472,
        numerical_initialization='preceding normal IPOPT settings; three tiny push overrides removed')
    if plan is not None:
        cost, _, penalty = planner.objective(plan.x)
        result.update(variables=plan.x.tolist(), durations=plan.durations.tolist(),
            coefficients=plan.coefficients.tolist(), nominal_time_s=float(plan.durations.sum()),
            objective=cost, dynamic_soft_integral=penalty, made_at=0.,
            crossing_times_s=plan.crossing_times.tolist(),
            crossing_points=np.asarray(planner.point_function(plan.x)).T.tolist())
        run.save(folder/'attitude_audit.json', attitude_audit(plan))
    run.save(folder/'result.json', result)
    print('RESULT', job_id, result['found'], result.get('nominal_time_s'), flush=True)


def assess(out, job, code, process_wall, planner, track):
    row = dict(job=job['id'], method=job['method'], seed=job['seed'], passed=False,
        exit_code=code, observed_process_wall_s=process_wall,
        outcome='运行异常' if code else '未找到合格规划')
    if code:
        return row
    source = out/'plans'/job['id']/'result.json'
    raw = run.read(source)
    row.update(found=raw['found'], nominal_time_s=raw.get('nominal_time_s'),
        objective=raw.get('objective'), solve_s=raw['solve_s'], build_s=raw['build_s'],
        source=str(source), finite_calls=len(raw['optimizer_calls']))
    if not raw['found']:
        return row
    plan = frozen_plan(raw)
    _, full_passed = planner.verify(plan)
    value, _, penalty = planner.objective(plan.x)
    np.testing.assert_allclose(value, raw['objective'], rtol=1e-10, atol=1e-7)
    geometry = evaluate(raw['coefficients'], raw['durations'], track, scenario.ASSETS/'meshes')
    clear = all(item['outcome'] == 'clear' for item in geometry['checks'])
    task = geometry['ordered_exactly_once'] and geometry['endpoints_passed']
    margins = [float(frame_aperture(window.aperture).boundary_distance(
        window.world_to_local(point, at)[0])) for window, point, at in zip(track.windows,
        np.asarray(planner.point_function(plan.x)).T, plan.crossing_times)]
    inset = min(margins) >= .62594472-1e-7
    row.update(passed=bool(full_passed and clear and task and inset),
        trajectory_distance_m=geometry['trajectory_distance_m'], dynamic_soft_integral=penalty)
    row['outcome'] = '通过' if row['passed'] else '复查未通过'
    run.save(source.parent/'assessment.json', geometry)
    run.save(source.parent/'validation.json', dict(passed=row['passed'], all_56_passed=full_passed,
        mesh_clear=clear, task_passed=task, inset_membership_passed=inset,
        knot_frame_distances_m=margins, objective_recomputed=value))
    return row


def report(out, rows, best, state, elapsed, level):
    lines = ['# 独立自由点与最佳中心释放', '',
        f'条件{level}，状态{state}，完成{len(rows)}次求解，墙钟{elapsed:.3f}秒。', '',
        '自由点与强制中心各8组独立时间初值，点初值均在到达时刻的中心，PVAJ解析MINCO生成。',
        '16次独立求解结束后，仅取完整复查合格的最佳中心结果，开放全部25选点参数再求解一次。',
        '最终自由点从8个独立结果和1个中心初始化结果中按原完整目标择优。',
        '动力学95%软界、附加安全距离0.15米、原内缩映射/目标/32积分/安全复查保持。', '',
        '|分支|样本|结果|完整七窗及返航参考/s|完整目标|求解/s|', '|---|---:|---|---:|---:|---:|']
    for row in sorted(rows, key=lambda r:(r['method'], r['seed'])):
        def number(key):
            return '—' if row.get(key) is None else f"{row[key]:.6f}"
        lines.append(f"|{LABELS[row['method']]}|{row['seed']}|{row['outcome']}|"
            f"{number('nominal_time_s')}|{number('objective')}|{number('solve_s')}|")
    lines += ['', '## 分支择优', '', '|分支|合格数|最佳参考/s|', '|---|---:|---:|']
    for method in LABELS:
        selected = best[method]
        count = sum(r['passed'] for r in rows if r['method']==method)
        attempted = sum(r['method']==method for r in rows)
        value = '—' if selected is None else f"{selected['nominal_time_s']:.6f}"
        lines.append(f'|{LABELS[method]}|{count}/{attempted}|{value}|')
    lines += ['', '本批为规划验证，计时包含七窗后返航。所有失败和求解成本保留。',
        '八时间使用原Sobol种子0与逐段旅行/运动相位规则，所有内部时间/PVAJ正式优化继续开放。',
        '每次有限IPOPT最多5000步、最多24轮安全补点，无墙钟截断，其他数值配置沿用上一轮。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def coordinator(out, level):
    with open('/tmp/convex_exp2_best_center_release.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'jobs').mkdir()
        frozen = {path:value for path,value in run.sources().items() if not path.endswith('.md')}
        run.snapshot(out, frozen)
        previous = out.parent/'spatial_phase_minco_eightstarts_v1_20261109'
        protected = run.read(previous/'protocol.json')['prior_sha256']
        for folder in ('spatial_phase_minco_eightstarts_v1_20261109',
                'center_first_phase_eightstarts_v1_20261109', 'center_phase_minco_eightstarts_v1_20261109'):
            protected.update({str(path):run.previous.digest(path) for path in (out.parent/folder).rglob('*.json')
                if 'source_snapshot' not in path.parts})
        run.save(out/'protocol.json', dict(level=level, total_independent_solves=16, maximum_solves=17,
            processes=16, starts_per_method=8, center_point_initialization=True,
            initialization='independent phase-sampled times and analytic MINCO PVAJ', sampling_seed=0,
            global_initial_time_scaling=False, source_sha256=frozen, prior_sha256=protected,
            independent_stage_uses_optimized_results=False, release_uses_best_verified_center=True,
            release_solves=1, selection='lowest original objective among verified free-point results',
            dynamic_limit_fraction=.95, extra_clearance_m=.05, passage_margin_m=.15,
            finite_safety_radius_m=.62594472, verification_safety_radius_m=.62094472,
            finite_max_iterations=5000, safety_exchange_rounds=24, wall_budget_per_seed=None,
            quadrature_intervals=32, point_variables_free=25, point_variables_center=0,
            physical_course_unchanged=True, blas_threads=1, solver='preceding normal IPOPT configuration'))
        track, _ = scenario.build_case(scenario.Case(level=level))
        validators = {method:make_planner(track, method!='center') for method in ('ours', 'center')}
        validators['release'] = validators['ours']
        rows, best, active = [], dict.fromkeys(LABELS), {}
        began = time.perf_counter()
        env = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')
        module = __spec__.name

        def stage(jobs, name):
            starts = []
            for job in jobs:
                run.save(out/'jobs'/f"{job['id']}.json", job)
                log = (out/f"{job['id']}.log").open('w')
                process = subprocess.Popen([sys.executable, '-u', '-m', module, 'worker',
                    '--outdir', str(out), '--job', job['id']], cwd=run.ROOT,
                    stdout=log, stderr=subprocess.STDOUT, env=env)
                started = time.perf_counter()
                starts.append(started)
                active[job['id']] = (process, log, job, started)
            print('STAGE_STARTED', name, {key:value[0].pid for key,value in active.items()}, flush=True)
            while active:
                for key, (process, log, job, started) in list(active.items()):
                    code = process.poll()
                    if code is None:
                        continue
                    log.close()
                    del active[key]
                    row = assess(out, job, code, time.perf_counter()-started,
                        validators[job['method']], track)
                    rows.append(row)
                    selected = best[job['method']]
                    if row['passed'] and (selected is None or row['objective']<selected['objective']):
                        best[job['method']] = row.copy()
                        run.save(out/f"best_{job['method']}_result.json", run.read(Path(row['source'])))
                    print('FINISHED', key, row['outcome'], row.get('nominal_time_s'), flush=True)
                elapsed = time.perf_counter()-began
                run.save(out/'summary.json', rows)
                run.save(out/'best.json', best)
                report(out, rows, best, name, elapsed, level)
                run.save(out/'status.json', dict(state=name, pid=os.getpid(), finished=len(rows),
                    maximum_total=17, active={key:item[0].pid for key,item in active.items()},elapsed_s=elapsed))
                if active:
                    time.sleep(15.)
            return dict(wall_s=time.perf_counter()-min(starts), startup_spread_s=max(starts)-min(starts))

        try:
            jobs = [dict(id=f'{method}_{seed}', method=method, seed=seed, level=level)
                for seed in range(8) for method in ('ours','center')]
            first_stage = stage(jobs, 'independent_planning')
            initial_checks = []
            for seed in range(8):
                pair = {}
                for method in ('ours','center'):
                    recorded = run.read(out/'plans'/f'{method}_{seed}'/'initial.json')
                    np.testing.assert_allclose(recorded['variables'], validators[method].seeds()[seed], rtol=0., atol=1e-10)
                    assert recorded['point_variables']==(25 if method=='ours' else 0)
                    assert not recorded['optimized_initial_solution_used']
                    pair[method] = recorded
                np.testing.assert_array_equal(pair['ours']['durations_s'], pair['center']['durations_s'])
                np.testing.assert_allclose(pair['ours']['points'], pair['center']['points'], rtol=0., atol=1e-12)
                np.testing.assert_allclose(pair['ours']['coefficients'], pair['center']['coefficients'], rtol=0., atol=1e-8)
                initial_checks.append(dict(seed=seed, reproduced=True, paired_curves_identical=True))
            release_stage = None
            if best['center'] is not None:
                center = best['center']
                release_stage = stage([dict(id='release_best', method='release', seed=center['seed'],
                    level=level, center_source=center['source'], center_objective=center['objective'])], 'best_center_release')
            qualified = [row for row in rows if row['method']!='center' and row['passed']]
            selected = min(qualified, key=lambda row:row['objective'], default=None)
            run.save(out/'selected.json', selected)
            if selected is not None:
                run.save(out/'best_result.json', run.read(Path(selected['source'])))
            assert {path:value for path,value in run.sources().items() if not path.endswith('.md')}==frozen
            assert all(run.previous.digest(run.ROOT/path)==value for path,value in frozen.items())
            assert all(run.previous.digest(Path(path))==value for path,value in protected.items())
            elapsed = time.perf_counter()-began
            run.save(out/'validation.json', dict(passed=True, all_independent_collected=True,
                initial_checks=initial_checks, source_and_prior_sha256_passed=True,
                release_attempted=release_stage is not None, best_center_job=None if best['center'] is None else best['center']['job'],
                independent_stage=first_stage, release_stage=release_stage, total_wall_s=elapsed,
                sum_solve_s=sum(row.get('solve_s',0.) for row in rows), gazebo_runs=0))
            report(out, rows, best, 'complete', elapsed, level)
            run.save(out/'status.json', dict(state='complete', finished=len(rows), total=len(rows),elapsed_s=elapsed))
            print('COMPLETE', selected, flush=True)
        except BaseException as error:
            for process, log, *_ in active.values():
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=30)
                log.close()
            run.save(out/'status.json', dict(state='stopped',failure=repr(error),finished=len(rows)))
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('run','worker'))
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--level', choices=scenario.LEVELS, default='a25_f6')
    parser.add_argument('--job')
    args = parser.parse_args()
    out = args.outdir.resolve()
    if args.action=='worker':
        if args.job is None:
            parser.error('worker requires --job')
        worker(out, args.job)
    else:
        coordinator(out, args.level)


if __name__=='__main__':
    main()
