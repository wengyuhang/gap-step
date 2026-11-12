"""Run ten levels sequentially: dynamic TOGT center, then two SIP variants.

All optimizers are C++ binaries. This Python entry only exports mathematical
graphs, schedules processes, and records wall-clock provenance.
"""
from concurrent.futures import ThreadPoolExecutor
import fcntl
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

from .export_plan_initial import export as export_initial


LEVELS = ('static', 'mixed_base', 'a25_f3', 'a25_f6', 'a50_f1',
          'a50_f3', 'a50_f6', 'a75_f1', 'a75_f3', 'a75_f6')
METHODS = ('center', 'ours')
ROOT = Path(__file__).resolve().parents[6]
EXP = ROOT / 'convex_timevarying_window/comparisons/论文实验/实验2'
CPP = EXP / 'cpp_runtime'
OUT = EXP / 'results/global_hard_togt_init_full_v1_20261112'
BIN = Path('/tmp/window_cpp_runtime_build')
PYTHON = Path(sys.executable)
GLOBAL_GRAPH = EXP / 'results/global_uniform_hard_v1_20261111/graphs/static_ours'
TOGT_GRAPHS = CPP / 'results/graphs_v1_20261110'
ASSETS = CPP / 'results/assets_v1_20261110'
STATIC_PROBE = EXP / 'results/global_uniform_hard_dyn_togt_center_seed_v1_20261112'
STATIC_SOURCE = EXP / 'results/native_cpp_10x5_v1_20261110/plans/static_dynamic_togt_center/result.json'


def read(path):
    return json.loads(path.read_text())


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.writing')
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n')
    temporary.replace(path)


def execute(command, log):
    log.parent.mkdir(parents=True, exist_ok=True)
    with log.open('w') as stream:
        return subprocess.run([str(x) for x in command], stdout=stream,
                              stderr=subprocess.STDOUT, cwd=ROOT).returncode


def baseline(level, group):
    selected = group / 'dynamic_togt_center/result.json'
    timing = group / 'dynamic_togt_center/timing.json'
    if selected.exists() and timing.exists():
        return read(selected), read(timing)
    graph = TOGT_GRAPHS / f'{level}_dynamic_togt_center'
    candidates = group / 'dynamic_togt_center/candidates'
    jobs = []
    for seed in range(8):
        folder = candidates / f'seed_{seed}'
        result = folder / 'result.json'
        if not result.exists():
            jobs.append((seed, folder, result))
    began = time.perf_counter()
    with ThreadPoolExecutor(max_workers=8) as pool:
        tasks = [pool.submit(execute,
            [BIN/'togt_planner', graph, folder, 'solve', seed],
            folder/'run.log') for seed, folder, _ in jobs]
        for (seed, _, result), task in zip(jobs, tasks):
            if task.result() != 0 or not result.exists():
                raise RuntimeError(f'{level} dynamic TOGT center seed {seed} failed; logs preserved')
    wall = time.perf_counter() - began
    results = [read(candidates/f'seed_{seed}'/'result.json') for seed in range(8)]
    feasible = [r for r in results if r['found']]
    metadata = dict(level=level, candidate_count=8, successful_candidates=len(feasible),
                    parallel_candidate_wall_s=wall,
                    candidate_solve_s=[r['solve_s'] for r in results])
    if not feasible:
        chosen = dict(found=False, method='dynamic_togt_center',
                      case=dict(layout='togt_whole', level=level))
    else:
        chosen = min(feasible, key=lambda r: (r['objective'], r['initial_sample_index']))
        chosen = dict(chosen, scenario_variant='higher_swing_length_2p20')
        metadata['selected_seed'] = chosen['initial_sample_index']
        metadata['selected_objective'] = chosen['objective']
    save(selected, chosen)
    save(timing, metadata)
    return chosen, metadata


def prepare_graph(level, method, baseline_result, group):
    initialized = group / 'graphs' / method
    if initialized.exists():
        return read(initialized/'initial_diagnostic.json').get('conversion_wall_s', 0.)
    base = group / 'graphs_base' / method
    if not base.exists():
        command = [PYTHON, '-m',
            'convex_timevarying_window.comparisons.论文实验.实验2.cpp_runtime.tools.export_runtime',
            '--level', level, '--method', method, '--outdir', base]
        if execute(command, group/'logs'/f'export_{method}.log') != 0:
            raise RuntimeError(f'{level} {method} graph export failed')
    shutil.copy2(GLOBAL_GRAPH/'global_uniform_dynamics.casadi', base/'global_uniform_dynamics.casadi')
    shutil.copy2(GLOBAL_GRAPH/'global_uniform_dynamics.json', base/'global_uniform_dynamics.json')
    began = time.perf_counter()
    export_initial(base, baseline_result, initialized)
    wall = time.perf_counter() - began
    diagnostic = read(initialized/'initial_diagnostic.json')
    diagnostic['conversion_wall_s'] = wall
    save(initialized/'initial_diagnostic.json', diagnostic)
    return wall


def same_trajectory(left, right):
    import numpy as np
    return (left.get('found') and right.get('found') and
        np.max(np.abs(np.asarray(left['durations'])-np.asarray(right['durations']))) < 1e-6 and
        np.max(np.abs(np.asarray(left['coefficients'])-np.asarray(right['coefficients']))) < 1e-6)


def solve_sip(level, method, group, baseline_result):
    folder = group / method
    target = folder / 'result.json'
    if target.exists():
        return read(target)
    if folder.exists() and any(folder.iterdir()):
        # The previous coordinator may have been interrupted between finite NLP
        # rounds. Preserve its raw log instead of silently truncating it.
        archive = OUT / 'interrupted_attempts' / level / method
        archive.parent.mkdir(parents=True, exist_ok=True)
        if archive.exists():
            raise RuntimeError(f'Interrupted-attempt archive already exists: {archive}')
        folder.rename(archive)
    if level == 'static' and same_trajectory(baseline_result, read(STATIC_SOURCE)):
        source = STATIC_PROBE / ('center' if method == 'center' else 'free') / 'result.json'
        if source.exists():
            folder.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            save(folder/'reuse.json', dict(source=str(source.resolve()),
                reason='Same exact dynamic TOGT center trajectory; one-seed hard solve already completed'))
            return read(target)
    graph = group / 'graphs' / method
    if execute([BIN/'sip_planner', graph, folder, 0, '--sampled-hard'],
               folder/'run.log') != 0 or not target.exists():
        raise RuntimeError(f'{level} {method} C++ SIP solve failed; logs preserved')
    return read(target)


def assess(level, name, group, raw):
    if not raw.get('found'):
        return None
    target = group / name / 'assessment.json'
    if not target.exists():
        scene = ASSETS / level / 'scene.json'
        if execute([BIN/'mesh_check', group/name/'result.json', scene, target],
                   group/name/'mesh_check.log') != 0:
            raise RuntimeError(f'{level} {name} mesh check failed')
    return read(target)


def publish(rows):
    save(OUT/'SUMMARY.json', rows)
    existing = read(OUT/'status.json') if (OUT/'status.json').exists() else {}
    if existing.get('state') == 'running':
        existing['completed_rows'] = len(rows)
        save(OUT/'status.json', existing)
    lines = ['# 全程均匀采样动力学硬约束：动态 TOGT 中心初值', '',
        '2026-11-12。10 组按 LEVELS 顺序逐组运行；每组先并行求动态 TOGT 中心的 8 个原候选，取目标最小的收敛解；'
        '随后本文中心与自由点各从该同一轨迹出发，仅一个初值。两者仍采用 257 个全程等时刻、95% 动力学硬约束与原几何补点。'
        '补点后的有限 NLP 传递上一轮变量及约束乘子。C++ 执行优化，Python 只导出图与调度。', '',
        '计算时间为实际墙钟；本文总时间 = 动态 TOGT 中心候选墙钟 + 轨迹转换墙钟 + 本文 C++ 求解墙钟。'
        '图导出是离线准备，单独记录而不计入该在线式计算链。', '',
        '| 难度 | 方法 | 结果 | 全程规划时间 / s | 初值生成 / s | 本文求解 / s | 合计计算 / s | 几何碰撞 | 补点轮数 |',
        '| --- | --- | --- | ---: | ---: | ---: | ---: | --- | ---: |']
    for row in rows:
        value = lambda key: '—' if row.get(key) is None else f'{row[key]:.3f}'
        lines.append(f"| {row['level']} | {row['method']} | {row['outcome']} | {value('flight_time_s')} | "
                     f"{value('initial_generation_s')} | {value('method_solve_s')} | {value('total_compute_s')} | "
                     f"{row.get('collision', '—')} | {row.get('exchange_rounds', '—')} |")
    (OUT/'REPORT.md').write_text('\n'.join(lines) + '\n')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    with (OUT/'coordinator.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rows = read(OUT/'SUMMARY.json') if (OUT/'SUMMARY.json').exists() else []
        done = {(r['level'], r['method']) for r in rows}
        for level in LEVELS:
            group = OUT/'groups'/level
            save(OUT/'status.json', dict(state='running', level=level, completed_rows=len(rows)))
            print('LEVEL', level, flush=True)
            baseline_result, timing = baseline(level, group)
            baseline_assessment = assess(level, 'dynamic_togt_center', group, baseline_result)
            if (level, 'dynamic_togt_center') not in done:
                rows.append(dict(level=level, method='dynamic_togt_center',
                    outcome='收敛' if baseline_result['found'] else '未找到',
                    flight_time_s=baseline_result.get('nominal_time_s'),
                    initial_generation_s=0.,
                    method_solve_s=timing['parallel_candidate_wall_s'],
                    total_compute_s=timing['parallel_candidate_wall_s'],
                    collision=None if baseline_assessment is None else baseline_assessment.get('collision_detected'),
                    exchange_rounds=0))
                done.add((level, 'dynamic_togt_center'));publish(rows)
            if not baseline_result['found']:
                for method in METHODS:
                    if (level, method) not in done:
                        rows.append(dict(level=level, method=method, outcome='无 TOGT 初值',
                            flight_time_s=None, initial_generation_s=timing['parallel_candidate_wall_s'],
                            method_solve_s=None, total_compute_s=None, collision=None, exchange_rounds=None))
                        done.add((level, method));publish(rows)
                continue
            conversions = {method: prepare_graph(level, method,
                group/'dynamic_togt_center/result.json', group) for method in METHODS}
            # Two methods may solve concurrently, but the next level waits for both.
            with ThreadPoolExecutor(max_workers=2) as pool:
                tasks = {method: pool.submit(solve_sip, level, method, group, baseline_result)
                         for method in METHODS if (level, method) not in done}
                for method, task in tasks.items():
                    raw = task.result()
                    assessment = assess(level, method, group, raw)
                    init_time = timing['parallel_candidate_wall_s'] + conversions[method]
                    rows.append(dict(level=level, method=method,
                        outcome='收敛且几何复查通过' if raw.get('found') else '未找到',
                        flight_time_s=raw.get('nominal_time_s'),
                        initial_generation_s=init_time,
                        method_solve_s=raw['solve_s'],
                        total_compute_s=init_time+raw['solve_s'],
                        collision=None if assessment is None else assessment.get('collision_detected'),
                        exchange_rounds=len(raw.get('optimizer_calls', []))-1))
                    done.add((level, method));publish(rows)
                    print('DONE', level, method, raw.get('found'), raw.get('nominal_time_s'), flush=True)
        save(OUT/'status.json', dict(state='complete', completed_rows=len(rows)))
        print('COMPLETE', len(rows), flush=True)


if __name__ == '__main__':
    main()
