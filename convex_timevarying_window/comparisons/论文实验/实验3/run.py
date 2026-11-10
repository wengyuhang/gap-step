"""Four-method independent single-gate stress test, stopped by center non-find."""
import argparse
import fcntl
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from . import planning, scenario

HERE = Path(__file__).resolve().parent
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验3.run'
METHODS = tuple(planning.LABELS)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def source_hashes():
    roots = [HERE, HERE.parent/'实验2/soft_dynamics', HERE.parent/'实验2/independent_repair',
        HERE.parent/'实验2/independent_free_solve']
    files = {path for root in roots for path in root.rglob('*.py') if 'results' not in path.parts}
    files.update([scenario.ROOT/'convex_timevarying_window/simple_two_gate_soft_only_mpc/px4.py',
        scenario.ROOT/'convex_timevarying_window/global_gate_minco_probe/planner.py',
        scenario.ROOT/'convex_timevarying_window/comparisons/论文实验/实验2/planner.py',
        scenario.ASSETS/'meshes/gate_01_rectangle_sleeve.stl',
        scenario.ASSETS/'build/libMixedGateMotion.so'])
    return {str(path):digest(path) for path in sorted(files)}


def row_for(out, job, code, wall):
    path = out/'plans'/job['id']/'result.json'
    row = dict(job=job['id'], level=job['level'], method=job['method'], seed=job['seed'],
        source=str(path), process_wall_s=wall, exit_code=code, found=False)
    if code or not path.exists():
        row['outcome'] = '运行异常'
        return row
    raw = planning.read(path)
    row.update({key:raw.get(key) for key in ('found', 'nominal_time_s', 'objective', 'solve_s',
        'build_s', 'geometry_verdict', 'trajectory_distance_m', 'postcheck_rejected')})
    row['outcome'] = raw.get('geometry_verdict', '未找到合格规划') if raw['found'] else '未找到合格规划'
    row['finite_calls'] = len(raw['optimizer_calls'])
    row['iterations'] = sum(item.get('iterations', 0) for item in raw['optimizer_calls'])
    return row


def selected_rows(rows, level):
    return {method:min((row for row in rows if row['level'] == level and
        row['method'] == method and row['found']), key=lambda row:row['objective'], default=None)
        for method in METHODS}


def report(out, rows, selections, state, elapsed):
    planning.save(out/'summary.json', rows)
    planning.save(out/'selected.json', selections)
    lines = ['# 实验3 单窗高速平移安全对比', '',
        f'状态 {state}。已收集 {len(rows)} 条候选，墙钟 {elapsed:.1f} s。', '',
        '任务包含起点到窗口及窗口到终点两段，起终点各距平面8米。',
        '规划按原目标择优，瞬时消融和动态TOGT的后置碰撞检查不参与择解。', '',
        '|幅度/m|频率/Hz|'+'|'.join(planning.LABELS.values())+'|',
        '|---:|---:|'+'---|'*len(METHODS)]
    for level, best in selections.items():
        values = []
        for method in METHODS:
            row = best[method]
            if row is None:
                values.append('未找到')
            else:
                classification = {'nominal_safe_complete':'无名义碰撞',
                    'nominal_collision':'名义碰撞', 'nominal_task_invalid':'任务不合格',
                    'nominal_unresolved':'检查未决'}.get(row['geometry_verdict'], '证据不足')
                values.append(f"{row['nominal_time_s']:.3f} s，{classification}")
        lines.append(f"|{level}|{level}|"+'|'.join(values)+'|')
    lines += ['', '本表为规划结果。Gazebo接触与实际完整任务用时单列。',
        '未找到表示固定八初值、每次有限优化5000步及最多24轮交换下未获合格结果。']
    (out/'PLANNING.md').write_text('\n'.join(lines)+'\n')


def coordinator(out, processes):
    with open('/tmp/convex_exp3_single_gate.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'jobs').mkdir()
        frozen = source_hashes()
        planning.save(out/'protocol.json', dict(created_date='2026-11-10', methods=planning.LABELS,
            scene=scenario.manifest(scenario.Case()), escalation='k=0,1,2,...; A=k m, f=k Hz for k>0',
            stopping='first level with zero qualified strict-center candidates from all eight starts',
            initial_starts=8, center_release_starts_per_free_method=1,
            center_release_requires_qualified_center=True, processes=processes,
            finite_max_iterations=5000, maximum_exchange_rounds=24, wall_budget_per_seed=None,
            dynamic_limit_fraction=.95, passage_margin_m=.15, witness_slack_m=.005,
            verification_radius_m=.6209447197013662, mapping_radius_m=.6259447197013662,
            physical_body_radius_m=.3859447197013662, quadrature_intervals=32,
            tilt_penalty_weight=0., cold_initial_points='center', sampling_seed=0,
            selection='lowest original time plus dynamic soft-integral objective',
            posthoc_collision_not_selection_for=['instant', 'togt_center'], source_sha256=frozen))
        rows, selections, active = [], {}, {}
        began = time.perf_counter()
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
            OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', PYTHONPATH=str(scenario.ROOT))

        def stage(jobs, level, name):
            queue = list(jobs)
            while queue or active:
                while queue and len(active) < processes:
                    job = queue.pop(0)
                    planning.save(out/'jobs'/f"{job['id']}.json", job)
                    log = (out/f"{job['id']}.log").open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE, 'worker',
                        '--outdir', str(out), '--job', job['id']], cwd=scenario.ROOT,
                        stdout=log, stderr=subprocess.STDOUT, env=env)
                    active[job['id']] = (process, log, job, time.perf_counter())
                for key, (process, log, job, started) in list(active.items()):
                    code = process.poll()
                    if code is None:
                        continue
                    log.close()
                    del active[key]
                    row = row_for(out, job, code, time.perf_counter()-started)
                    rows.append(row)
                    print('FINISHED', key, row['outcome'], row.get('nominal_time_s'), flush=True)
                    if code or row['outcome'] == '运行异常':
                        raise RuntimeError(f'Worker failure {key}; inspect its unchanged raw log')
                selections[str(level)] = selected_rows(rows, level)
                report(out, rows, selections, name, time.perf_counter()-began)
                planning.save(out/'status.json', dict(state=name, pid=os.getpid(), level=level,
                    finished=len(rows), queued=len(queue), active={key:item[0].pid for key,item in active.items()},
                    elapsed_s=time.perf_counter()-began))
                if queue or active:
                    time.sleep(5.)

        try:
            level = 0
            while True:
                planning.save(out/f'scene_{level:02d}.json', scenario.manifest(scenario.Case(level)))
                jobs = [dict(id=f'l{level:02d}_{method}_{seed}', level=level, method=method, seed=seed)
                    for method in METHODS for seed in range(8)]
                stage(jobs, level, 'independent_planning')
                best = selected_rows(rows, level)
                if best['center'] is not None:
                    stage([dict(id=f'l{level:02d}_{method}_release', level=level, method=method,
                        seed=best['center']['seed'], center_source=best['center']['source'])
                        for method in ('ours', 'instant')], level, 'best_center_release')
                selections[str(level)] = selected_rows(rows, level)
                report(out, rows, selections, 'level_complete', time.perf_counter()-began)
                print('LEVEL_COMPLETE', level, selections[str(level)], flush=True)
                if best['center'] is None:
                    break
                level += 1
            assert all(digest(path) == value for path,value in frozen.items())
            planning.save(out/'validation.json', dict(passed=True, sources_unchanged=True,
                first_center_nonfind_level=level, all_eight_center_starts_attempted_each_level=True,
                total_candidates=len(rows), formal_gazebo_runs=0))
            report(out, rows, selections, 'planning_complete', time.perf_counter()-began)
            planning.save(out/'status.json', dict(state='planning_complete', pid=os.getpid(),
                last_level=level, finished=len(rows), active={}, elapsed_s=time.perf_counter()-began))
            print('PLANNING_COMPLETE', level, flush=True)
        except BaseException as error:
            for process, log, *_ in active.values():
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=30)
                log.close()
            planning.save(out/'status.json', dict(state='stopped', failure=repr(error),
                finished=len(rows), elapsed_s=time.perf_counter()-began))
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('run', 'worker'))
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--job')
    parser.add_argument('--processes', type=int, default=16)
    args = parser.parse_args()
    out = args.outdir.resolve()
    if args.action == 'worker':
        planning.worker(out, planning.read(out/'jobs'/f'{args.job}.json'))
    else:
        coordinator(out, args.processes)


if __name__ == '__main__':
    main()
