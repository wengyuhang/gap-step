"""Three sequential levels, four native planners, unit TOGT dynamics weight."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from .export_exchange_dynamics import export as export_dynamics
from .export_plan_initial import export as export_initial
from .run_hard_from_togt import read, save


ROOT = Path(__file__).resolve().parents[6]
EXPERIMENT = ROOT/'convex_timevarying_window/comparisons/论文实验/实验2'
OUT = EXPERIMENT/'results/dynamics_exchange_weight1_three_v1_20261112'
BIN = Path('/tmp/window_cpp_runtime_build')
ASSETS = EXPERIMENT/'cpp_runtime/results/assets_v1_20261110'
GLOBAL_GRAPH = EXPERIMENT/'results/global_uniform_hard_v1_20261111/graphs/static_ours'
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.cpp_runtime.tools.'
LEVELS = ('a25_f6', 'a50_f6', 'a75_f6')
LABELS = dict(dynamic_togt_center='动态 TOGT 中心', dynamic_togt='动态 TOGT 自由点',
              center='本文中心', ours='本文自由点')
ENV = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1')


def execute(command, log):
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('w') as stream:
        completed = subprocess.run([str(x) for x in command], cwd=ROOT, env=ENV,
                                   stdout=stream, stderr=subprocess.STDOUT)
    if completed.returncode:
        raise RuntimeError(f'Process exited {completed.returncode}; see {log}')


def export_graph(level, method, folder):
    graph = folder/'graphs_base'/method
    if graph.exists():
        return graph
    tool = 'export_togt' if method.startswith('dynamic_togt') else 'export_runtime'
    command = [sys.executable, '-m', MODULE+tool, '--level', level,
               '--method', method, '--outdir', graph]
    if tool == 'export_togt':
        command.extend(['--dynamics-weight', '1'])
    began = time.perf_counter()
    execute(command, folder/'logs'/f'export_{method}.log')
    save(folder/'logs'/f'export_{method}_timing.json', dict(wall_s=time.perf_counter()-began))
    if tool == 'export_togt':
        execute([BIN/'togt_planner', graph, folder/'validation'/method, 'verify', 0],
                folder/'logs'/f'verify_{method}.log')
    else:
        # The source-plan importer evaluates this legacy diagnostic function.
        shutil.copy2(GLOBAL_GRAPH/'global_uniform_dynamics.casadi', graph/'global_uniform_dynamics.casadi')
        export_dynamics(graph)
    return graph


def run_togt(method, graph, folder):
    selected = folder/method/'result.json'
    timing_path = folder/method/'timing.json'
    if selected.exists() and timing_path.exists():
        return read(selected), read(timing_path)
    began = time.perf_counter()
    with ThreadPoolExecutor(max_workers=8) as pool:
        tasks = []
        for seed in range(8):
            candidate = folder/method/'candidates'/f'seed_{seed}'
            tasks.append(pool.submit(execute, [BIN/'togt_planner', graph, candidate, 'solve', seed],
                                     candidate/'console.log'))
        for task in tasks:
            task.result()
    candidates = [read(folder/method/'candidates'/f'seed_{seed}'/'result.json') for seed in range(8)]
    converged = [candidate for candidate in candidates if candidate['found']]
    chosen = min(converged, key=lambda row: row['objective']) if converged else dict(found=False)
    chosen = dict(chosen, method=method, scenario_variant='higher_swing_length_2p20')
    timing = dict(candidate_count=8, converged=len(converged),
                  wall_s=time.perf_counter()-began,
                  selected_seed=chosen.get('initial_sample_index'),
                  candidate_solver_s=[row['solve_s'] for row in candidates])
    save(selected, chosen)
    save(timing_path, timing)
    return chosen, timing


def run_ours(method, graph, folder):
    selected = folder/method/'result.json'
    if not selected.exists():
        execute([BIN/'sip_planner', graph, folder/method, 0, '--exchange-hard-reset-togt'],
                folder/method/'console.log')
    return read(selected)


def assess(level, method, folder, result):
    if not result.get('found'):
        return None
    path = folder/method/'assessment.json'
    execute([BIN/'mesh_check', folder/method/'result.json', ASSETS/level/'scene.json', path],
            folder/method/'mesh.log')
    if method in ('ours', 'center'):
        execute([BIN/'dynamics_check', folder/method/'result.json',
                 folder/'graphs'/method/'exchange_dynamics.json',
                 folder/method/'dynamics_assessment.json'], folder/method/'dynamics.log')
        if not read(folder/method/'dynamics_assessment.json')['passed']:
            raise RuntimeError(f'Independent dynamics verification failed for {level}/{method}')
    return read(path)


def record(level, method, folder, result, timing, initial_s=0.):
    assessment = assess(level, method, folder, result)
    if not result.get('found'):
        outcome = '未找到合格规划'
    elif assessment['collision_detected']:
        outcome = '碰撞'
    elif assessment['passed']:
        outcome = '无碰撞完成'
    elif assessment['ordered_exactly_once'] and assessment['endpoints_passed']:
        outcome = '几何检查未决'
    else:
        outcome = '未完成'
    calls = result.get('optimizer_calls', [])
    return dict(level=level, method=method, outcome=outcome,
        full_trajectory_time_s=result.get('nominal_time_s'),
        initial_generation_s=initial_s, optimization_wall_s=timing,
        total_compute_s=initial_s+timing,
        finite_nlp_count=len(calls), total_iterations=sum(row['iterations'] for row in calls),
        dynamic_rows=calls[-1].get('dynamic_constraint_rows') if calls else None,
        geometry_collision=None if assessment is None else assessment['collision_detected'],
        source=str((folder/method/'result.json').resolve()))


def publish(rows):
    save(OUT/'SUMMARY.json', rows)
    status = read(OUT/'status.json')
    status['completed']=len(rows)
    save(OUT/'status.json', status)
    lines = ['# 动力学补点与单位 TOGT 罚项的三组对比', '',
        '2026-11-12。相同高摆幅七窗赛道、原初相、完整起点到终点含返航。', '',
        '两个动态 TOGT 使用时间权重 1、动力学积分罚项权重 1，各自 8 候选并行后按目标择优。'
        '本文两版各使用本组动态 TOGT 中心最优候选作为唯一初值，采用逐段正时间映射和动力学半无限补点，'
        '每次几何或动力学补点后重新从该初值开始。本文动力学为 95% 限值，归一化复查容差 1e-6。', '',
        '本文总计算时间包含动态 TOGT 中心 8 候选生成墙钟、轨迹转换墙钟与本方法 C++ 全部求解/复查墙钟。'
        '候选并行时间按墙钟统计。图导出准备单独记录。', '',
        '| 难度 | 方法 | 完整参考时间 / s | 初值生成 / s | 本方法求解 / s | 总计算 / s | 有限问题数 | 最终动力学约束 |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for level in LEVELS:
        for method in LABELS:
            matches = [r for r in rows if r['level']==level and r['method']==method]
            if not matches:
                continue
            row=matches[0]
            flight=f"{row['full_trajectory_time_s']:.3f}" if row['outcome']=='无碰撞完成' else row['outcome']
            count=str(row['dynamic_rows']) if row['dynamic_rows'] is not None else '—'
            lines.append(f"| {level} | {LABELS[method]} | {flight} | {row['initial_generation_s']:.3f} | "
                f"{row['optimization_wall_s']:.3f} | {row['total_compute_s']:.3f} | {row['finite_nlp_count']} | {count} |")
    (OUT/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT/'coordinator.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rows = read(OUT/'SUMMARY.json') if (OUT/'SUMMARY.json').exists() else []
        completed = {(row['level'], row['method']) for row in rows}
        for level in LEVELS:
            folder=OUT/'groups'/level
            save(OUT/'status.json', dict(state='running', level=level, completed=len(rows), pid=os.getpid()))
            print('LEVEL', level, flush=True)
            graphs={method:export_graph(level,method,folder) for method in LABELS}
            center_result=None
            center_timing=None
            for method in ('dynamic_togt_center','dynamic_togt'):
                result,timing=run_togt(method,graphs[method],folder)
                if method=='dynamic_togt_center':
                    center_result,center_timing=result,timing
                if (level,method) not in completed:
                    rows.append(record(level,method,folder,result,timing['wall_s']))
                    completed.add((level,method));publish(rows)
                    print('DONE', level,method,rows[-1]['outcome'],result.get('nominal_time_s'),flush=True)
            if not center_result['found']:
                raise RuntimeError(f'No dynamic TOGT center initial trajectory for {level}')
            conversions={}
            for method in ('center','ours'):
                graph=folder/'graphs'/method
                if not graph.exists():
                    began=time.perf_counter()
                    export_initial(graphs[method],folder/'dynamic_togt_center/result.json',graph)
                    conversions[method]=time.perf_counter()-began
                    save(graph/'conversion_timing.json',dict(wall_s=conversions[method]))
                else:
                    conversions[method]=read(graph/'conversion_timing.json')['wall_s']
            with ThreadPoolExecutor(max_workers=2) as pool:
                tasks={pool.submit(run_ours,method,folder/'graphs'/method,folder):method
                       for method in ('center','ours') if (level,method) not in completed}
                for task in as_completed(tasks):
                    method=tasks[task];result=task.result()
                    rows.append(record(level,method,folder,result,result['solve_s'],
                        center_timing['wall_s']+conversions[method]))
                    completed.add((level,method));publish(rows)
                    print('DONE',level,method,rows[-1]['outcome'],result.get('nominal_time_s'),flush=True)
        save(OUT/'status.json',dict(state='complete',completed=len(rows),pid=os.getpid()))
        print('COMPLETE',len(rows),flush=True)


if __name__=='__main__':
    main()
