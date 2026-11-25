"""Generate eight C++ CMA-ES initial curves and run the frozen SLSQP solver."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent
CPUS = [0, 2, 4, 6, 8, 10, 12, 14]


def save(path, value):
    temporary = path.with_suffix(path.suffix+'.writing')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def quality_key(candidate):
    q = candidate['quality']
    if q['sampled_feasible']:
        return 0, q['trajectory_time_s'], 0, 0
    return 1, q['maximum_violation'], q['mean_square_violation'], q['trajectory_time_s']


def parallel(commands, log_directory, status_path, phase, level):
    start = time.perf_counter()
    logs = []
    active = {}
    finished = []
    for seed, command in enumerate(commands):
        log = (log_directory/f'worker_{seed}.log').open('w')
        logs.append(log)
        cpu = CPUS[seed]
        process = subprocess.Popen(command, cwd=PACKAGE, stdout=log, stderr=subprocess.STDOUT,
                                   preexec_fn=lambda cpu=cpu: os.sched_setaffinity(0, {cpu}))
        active[seed] = process
    while active:
        for seed, process in list(active.items()):
            code = process.poll()
            if code is None:
                continue
            finished.append(dict(seed=seed, exit_code=code))
            del active[seed]
            print('WORKER_COMPLETE', level, phase, seed, code, flush=True)
        save(status_path, dict(state='running', level=level, phase=phase, completed=len(finished),
                               active=[dict(seed=s, pid=p.pid) for s, p in active.items()],
                               phase_wall_s=time.perf_counter()-start, coordinator_pid=os.getpid()))
        time.sleep(.2)
    for log in logs:
        log.close()
    save(log_directory/'PROCESS_RESULTS.json', finished)
    if any(row['exit_code'] != 0 for row in finished):
        raise RuntimeError(f'Worker failed in {level} {phase}, inspect {log_directory}')
    return time.perf_counter()-start


def select_eight(generation_directory):
    selected = []
    for seed in range(8):
        result = json.loads((generation_directory/f'seed_{seed}.json').read_text())
        candidate = result['best']
        if not candidate['quality']['finite']:
            raise RuntimeError(f'CMA restart {seed} has no finite candidate')
        selected.append(dict(candidate, source_seed=seed))
    for seed, candidate in enumerate(selected):
        for previous in selected[:seed]:
            difference = np.max(abs(np.asarray(candidate['arrivals'])-np.asarray(previous['arrivals'])))
            if difference < .05:
                raise RuntimeError('Independent CMA winners duplicate an arrival schedule')
    return selected


def materialize_inputs(cases, destination, level, selected):
    for method in ('ours', 'center'):
        metadata = json.loads((cases/'inputs'/f'{level}_{method}'/'manifest.json').read_text())
        n, vs = metadata['n'], metadata['velocity_start']
        for seed, candidate in enumerate(selected):
            original = metadata['initials'][seed]['original']
            original[:n] = candidate['log_times']
            original[vs:] = candidate['VAJ']
            metadata['initials'][seed]['original'] = original
        metadata['initialization'] = 'eight quality-ranked distinct schedules from C++ CMA-ES and position-only MINCO'
        target = destination/'inputs'/f'{level}_{method}'
        target.mkdir(parents=True)
        save(target/'manifest.json', metadata)


def validate_initials(inputs, level, selected):
    sys.path.insert(0, str(PACKAGE))
    import worker
    track, _ = worker.scenario.build_case(worker.scenario.Case(level=level))
    validation = []
    for method in ('ours', 'center'):
        planner = worker.Planner(track, method == 'ours', pose_function=worker.scenario.pose)
        metadata = json.loads((inputs/'inputs'/f'{level}_{method}'/'manifest.json').read_text())
        points = np.concatenate([worker.point_mapping.initial(w.aperture) for w in track.windows]) if method == 'ours' else np.empty(0)
        for seed, candidate in enumerate(selected):
            initial = np.r_[candidate['log_times'], points, candidate['VAJ']]
            residuals, _, coefficients = planner.evaluate(initial, planner.witnesses())
            native = np.asarray(candidate['coefficients_flat']).reshape(planner.n, 3, 8).transpose(0, 2, 1)
            coefficient_error = float(np.max(abs(coefficients-native)))
            np.testing.assert_allclose(coefficients, native, rtol=1e-8, atol=1e-7)
            q = candidate['quality']
            maximum = max(0., -float(residuals.min()))
            if abs(maximum-q['maximum_violation']) > 1e-7:
                raise RuntimeError('C++ and frozen solver constraint quality differ')
            assert np.asarray(metadata['initials'][seed]['original'][:planner.n]).tolist() == candidate['log_times']
            validation.append(dict(method=method, seed=seed, coefficient_error=coefficient_error,
                                   residual_maximum_error=abs(maximum-q['maximum_violation']), passed=True))
    return validation


def write_report(out, rows):
    lines = ['# CMA-ES 八初值验证', '', '每档共用八条中心点 MINCO 初值，段时间由 C++ CMA-ES 搜索，实际 VAJ 从 MINCO 曲线提取。后端采用冻结原始硬约束 SLSQP，每方法八路并行。轨迹时间含七窗后返航。', '',
             '| 条件 | 方法 | 结果 | 完整轨迹／s | 初值生成／s | 后端八路／s | 总计算／s | 合格初值 |',
             '|---|---|---|---:|---:|---:|---:|---:|---:|']
    for row in rows:
        duration = '未找到' if row['time_s'] is None else f"{row['time_s']:.3f}"
        method = '自由点' if row['method'] == 'ours' else '中心'
        lines.append(f"| {row['level']} | {method} | {row['outcome']} | {duration} | {row['initial_generation_wall_s']:.3f} | {row['backend_wall_s']:.3f} | {row['total_wall_s']:.3f} | {row['qualified']}/8 |")
    lines += ['', '总计算为本档一次导出与八路初值搜索、选择/验证、该方法八路后端求解和复查的墙钟之和。源码编译成本单列。两方法共用初值生成成本，各方法独立计入完整该项成本。', '',
              '初值质量评价为原有限节点约束。最终结果沿原版有限求解、浮点 Bernstein、实体网格、顺序和起终点检查判定，地面字段单列。本批进行规划及名义几何评估。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cases', type=Path, required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--generations', type=int, default=200)
    parser.add_argument('--population', type=int, default=32)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    start = time.perf_counter()
    protocol = dict(levels=['a50_f6', 'a75_f6'], methods=['ours', 'center'], starts_per_method=8, workers=8,
                    search_dimensions=8, generations=args.generations, population=args.population, sigma_log_time=.12,
                    random_seeds=[20261113+i for i in range(8)], archive_arrival_separation_s=.05, selection='one best curve from each of eight independent CMA restarts',
                    candidate_ranking='feasibility; feasible duration; infeasible maximum violation then mean-square violation',
                    point_initialization='window center at actual arrival', VAJ_initialization='position-only MINCO actual derivatives',
                    backend='frozen independent_free_solve.planner.Planner', seconds_per_backend_start=300,
                    center_release=False, search_implementation='C++ libcmaes and native TOGT MincoSnap',
                    python_role='frozen constraint export, orchestration, physical equivalence audit and original SLSQP backend',
                    original_baseline=str(PACKAGE/'outputs/original_eight_ten_v1_20261112/run/SUMMARY.json'))
    save(args.out/'PROTOCOL.json', protocol)
    rows = []
    for level in protocol['levels']:
        generation_begin = time.perf_counter()
        model = args.models/level
        generation = args.out/'generation'/level
        generation.mkdir(parents=True)
        commands = [[str(args.binary), str(model), str(seed), str(args.generations), str(args.population), str(generation/f'seed_{seed}.json')] for seed in range(8)]
        cma_wall = parallel(commands, generation, args.out/'status.json', 'initial_generation', level)
        selected = select_eight(generation)
        inputs = args.out/'cases'
        materialize_inputs(args.cases, inputs, level, selected)
        validation = validate_initials(inputs, level, selected)
        export = json.loads((model/'model.json').read_text())
        generation_wall = time.perf_counter()-generation_begin+export['export_wall_s']
        save(generation/'INITIALS.json', dict(level=level, count=8, candidates=selected, cma_wall_s=cma_wall,
                                            initial_generation_wall_s=generation_wall, equivalence_validation=validation,
                                            sampled_feasible_count=sum(c['quality']['sampled_feasible'] for c in selected)))
        print('INITIALS_COMPLETE', level, json.dumps(dict(count=8, generation_wall_s=generation_wall,
              feasible=sum(c['quality']['sampled_feasible'] for c in selected), maxima=[c['quality']['maximum_violation'] for c in selected])), flush=True)
        for method in protocol['methods']:
            group = args.out/'groups'/level/method
            group.mkdir(parents=True)
            commands = [[sys.executable, '-u', str(PACKAGE/'worker.py'), '--cases', str(inputs), '--level', level,
                         '--method', method, '--seed', str(seed), '--out', str(group/f'seed_{seed}')] for seed in range(8)]
            backend_wall = parallel(commands, group, args.out/'status.json', method, level)
            candidates = [json.loads((group/f'seed_{seed}'/'result.json').read_text()) for seed in range(8)]
            qualified = [r for r in candidates if r['passed']]
            best = min(qualified, key=lambda r: r['nominal_time_s']) if qualified else None
            if best:
                save(group/'BEST_RESULT.json', best)
            row = dict(level=level, method=method, outcome='窗口复查通过' if best else '未找到',
                       time_s=best['nominal_time_s'] if best else None, qualified=len(qualified),
                       best_seed=best['seed'] if best else None, initial_generation_wall_s=generation_wall,
                       backend_wall_s=backend_wall, total_wall_s=generation_wall+backend_wall,
                       source=str(group))
            rows.append(row)
            save(args.out/'SUMMARY.json', rows)
            write_report(args.out, rows)
            print('METHOD_COMPLETE', json.dumps(row, ensure_ascii=False), flush=True)
    save(args.out/'status.json', dict(state='complete', methods=len(rows), elapsed_s=time.perf_counter()-start))


if __name__ == '__main__':
    main()
