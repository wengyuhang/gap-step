"""Frozen ten-course assessment of the independent Wang mathematical core."""
import argparse
from dataclasses import asdict, replace
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
from . import rollout
from ..independent_full_matrix import run as previous
from 复现.wang2023_verified import Planner
from 复现.wang2023_verified import x500

HERE = Path(__file__).resolve().parent
ROOT = previous.ROOT
CORE = ROOT/'复现/wang2023_verified'
PRIOR = HERE.parent/'results/independent_full_matrix_v2_20261107'
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.wang2023.run'
read, save, digest = previous.read, previous.save, previous.digest
LABELS = dict(previous.LABELS, planning_not_found='未找到收敛候选')


def sources():
    result = previous.sources()
    for directory in (HERE, CORE):
        for path in directory.rglob('*'):
            if path.is_file() and not {'.git', '.build', '__pycache__', 'results'}.intersection(path.relative_to(directory).parts):
                if path.suffix in ('.py', '.cpp', '.hpp', '.md', '.json') or path.name == 'LICENSE':
                    result[str(path.relative_to(ROOT))] = digest(path)
    return result


def assessment(raw):
    track, _ = rollout.scenario.build_case(rollout.scenario.Case(level=raw['case']['level']))
    if not raw['durations']:
        return dict(passed=False, verdict='planning_not_found', collision_detected=None,
            full_trajectory_time_s=None, failure=raw['failure'], time_comparison_eligible=False,
            reference_task_completed=False)
    geometry = previous.current.base.baseline.evaluate(raw['coefficients'], raw['durations'], track, rollout.scenario.ASSETS/'meshes')
    plan = previous.current.base.frozen_plan(dict(raw, variables=[]))
    audit = previous.current.base.attitude_audit(plan)
    row = dict(**geometry, **previous.layout.foreign_diagnostics(geometry), **previous.diagnostic(audit))
    row['time_comparison_eligible'] = previous.time_comparable(row) and raw['found']
    row['reference_task_completed'] = raw['found']
    row['attitude_audit'] = audit
    row['executed_prefix_time_s'] = float(sum(raw['durations']))
    if not raw['found']:
        row['full_trajectory_time_s'] = None
    return row


def solve(level, out):
    out.mkdir(parents=True, exist_ok=False)
    frozen = sources()
    save(out/'protocol.json', dict(scene=rollout.scenario.manifest(rollout.scenario.Case(level=level)),
        source_sha256=frozen, settings=asdict(rollout.Settings()), ideal_tracking=True,
        baseline_reference=str(PRIOR), dynamic_motion_clock='absolute race clock; wall solve time separate'))
    with (out/'progress.jsonl').open('w') as log:
        def progress(row):
            line = json.dumps(row)
            log.write(line+'\n')
            log.flush()
            print(line, flush=True)
        raw = rollout.solve(level, progress=progress)
    save(out/'result.json', raw)
    if raw['durations']:
        np.savez_compressed(out/'trajectory.npz', coefficients=raw['coefficients'], durations=raw['durations'])
    began = time.perf_counter()
    evaluated = assessment(raw)
    save(out/'assessment.json', evaluated)
    save(out/'status.json', dict(state='complete', found=raw['found'], verdict=evaluated['verdict'],
        nominal_time_s=raw['nominal_time_s'] if raw['found'] else None,
        planning_s=raw['build_s']+raw['solve_s'], assessment_s=time.perf_counter()-began,
        time_comparison_eligible=evaluated['time_comparison_eligible']))
    assert sources() == frozen
    print('COMPLETE', level, raw['found'], raw['nominal_time_s'], evaluated['verdict'],
        evaluated['time_comparison_eligible'], flush=True)


def prior_rows(frozen):
    protocol = read(PRIOR/'protocol.json')
    assert read(PRIOR/'validation.json')['passed']
    assert all(frozen.get(k) == value for k, value in protocol['source_sha256'].items())
    rows = read(PRIOR/'summary.json')
    assert len(rows) == 40
    expected = [rollout.scenario.manifest(rollout.scenario.Case(level=l)) for l in rollout.scenario.LEVELS]
    assert protocol['cases'] == previous.current.base.previous.canonical(expected)
    for row in rows:
        assert digest(Path(row['plan_directory'])/'result.json') == row['plan_result_sha256']
    return rows


def record(out, level):
    path = out/'plans'/level
    raw, assessed = read(path/'result.json'), read(path/'assessment.json')
    rho, factor = (0., 1.) if level == 'static' else rollout.scenario.CONDITIONS[level]
    cycles = [r['cycle_ms'] for r in raw['requests']]
    return dict(assessed, level=level, variant='wang2023', amplitude_ratio=rho, frequency_factor=factor,
        plan_directory=str(path), plan_result_sha256=digest(path/'result.json'),
        build_s=raw['build_s'], solve_s=raw['solve_s'], planning_total_s=raw['build_s']+raw['solve_s'],
        request_count=len(cycles), local_cycle_ms=dict(minimum=min(cycles), median=float(np.median(cycles)),
            maximum=max(cycles), below_100ms=sum(c <= 100. for c in cycles)),
        gate_count=raw['gate_count'], failure=raw['failure'], gazebo_run=False)


def report(out, rows, state):
    old = read(out/'prior_summary.json')
    methods = (*previous.METHODS, 'wang2023')
    all_rows = old+rows
    lines = ['# Wang2023同十赛道比较', '', f'状态 {state}，Wang已结束 {len(rows)}/10 条。', '',
        '固定原小七窗、四平移三摆动、初相0。参考时间包含七窗后返回原终点零VAJ，单位秒。',
        '数值为名义几何无碰撞完成。†表示额外5ms姿态、角速度或单电机诊断未全部满足。', '',
        '|幅度/窗宽|频率倍数|本文自由点|本文中心|动态TOGT自由点|动态TOGT中心|Wang2023独立复现|',
        '|---|---:|---:|---:|---:|---:|---:|']
    for level in rollout.scenario.LEVELS:
        by = {r['variant']: r for r in all_rows if r['level'] == level}
        rho, factor = (0., 1.) if level == 'static' else rollout.scenario.CONDITIONS[level]
        cells = []
        for method in methods:
            row = by.get(method)
            cell = '待算' if row is None else LABELS[row['verdict']]
            if row and row['passed']:
                cell = f"{row['full_trajectory_time_s']:.3f}"+('†' if not row['time_comparison_eligible'] else '')
            cells.append(cell)
        lines.append('|'+('静态' if level == 'static' else f'{rho:g}')+f'|{factor:g}|'+'|'.join(cells)+'|')
    lines += ['', '基准频率0.30/0.40/0.50/0.60/0.55/0.45/0.35Hz，乘表内倍数。幅度为各窗实际水平开口宽度的比例。', '',
        '## Wang原始时长与成本', '',
        '|条件|几何结果|完整参考 s|无碰撞完赛|角速在界|电机在物理界|建图/编译 s|求解合计 s|局部请求中位/最大 ms|',
        '|---|---|---:|---|---|---|---:|---:|---:|']
    for row in rows:
        duration = '—' if row['full_trajectory_time_s'] is None else f"{row['full_trajectory_time_s']:.3f}"
        word = lambda key: '—' if key not in row else '是' if row[key] else '否'
        cycle = row['local_cycle_ms']
        lines.append(f"|{row['level']}|{LABELS[row['verdict']]}|{duration}|{word('passed')}|"
            f"{word('dense_rate_within_planning_limits')}|{word('dense_motor_within_physical_limits')}|"
            f"{row['build_s']:.3f}|{row['solve_s']:.3f}|{cycle['median']:.1f}/{cycle['maximum']:.1f}|")
    counts = {method: dict(geometric_no_collision=sum(r['passed'] for r in all_rows if r['variant'] == method),
        collision=sum(r['collision_detected'] is True for r in all_rows if r['variant'] == method),
        eligible=sum(r['time_comparison_eligible'] for r in all_rows if r['variant'] == method)) for method in methods}
    comparisons = []
    for row in rows:
        ours = next(r for r in old if r['level'] == row['level'] and r['variant'] == 'ours')
        eligible = row['time_comparison_eligible'] and ours['time_comparison_eligible']
        comparisons.append(dict(level=row['level'], both_eligible=eligible,
            ours_time_s=ours['full_trajectory_time_s'], wang_time_s=row['full_trajectory_time_s'],
            ours_saving_percent=100*(row['full_trajectory_time_s']-ours['full_trajectory_time_s'])/row['full_trajectory_time_s'] if eligible else None))
    lines += ['', '## 复现与证据范围', '',
        'Wang数学核对应IROS2023原文Eq18–32及TableIII N2/M4，采用论文链接的LBFGS-Lite。原桥的收敛返回点一致性问题已修复。',
        's4、rho1e4、每段32积分区间、x500模型、前方巡航固定末端、整机出窗触发和四时间初值属于本任务实例化，详见PROTOCOL.md与复现README。',
        '原文总推力与角速度范数是积分软罚。单电机物理界为公共额外诊断，几何无碰撞与动力学可行分别列示。',
        '当前评估采用理想PVAJ跟踪，构图/编译与求解另计。每局部四初值各2000ms/5000迭代，未把计算延迟加入赛道运动时钟。',
        '所有正式赛道仅运行本冻结版本一次，所有初值、失败及原始碰撞见证保留。旧40条通过原场景与文件哈希核对后引用，没有重算或改写。',
        '有效时间优势仅在双方几何完成且公共额外诊断全部满足时计算，analysis.json保存逐组条件。', '',
        '[Wang原论文](https://arxiv.org/abs/2306.14461)', '', '![共同赛道](course_layout.png)']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(out/'analysis.json', dict(state=state, completed=len(rows), total=10, counts=counts, comparisons=comparisons,
        all_local_requests=sum(r['request_count'] for r in rows),
        requests_below_100ms=sum(r['local_cycle_ms']['below_100ms'] for r in rows)))
    save(out/'comparison_50.json', all_rows)


def validate(out):
    frozen = read(out/'protocol.json')['source_sha256']
    current = sources()
    if current != frozen:
        repair = read(out/'HARNESS_REPAIR.json')
        runner = str((HERE/'run.py').relative_to(ROOT))
        assert repair['file'] == runner
        assert repair['original_sha256'] == frozen[runner]
        assert repair['repaired_sha256'] == current[runner]
        assert current == dict(frozen, **{runner: repair['repaired_sha256']})
    (out/'validation_assessments').mkdir(exist_ok=True)
    assert all(digest(out/'source_snapshot'/rel) == value for rel, value in frozen.items())
    assert prior_rows(frozen) == read(out/'prior_summary.json')
    rows = read(out/'summary.json')
    assert [r['level'] for r in rows] == list(rollout.scenario.LEVELS)
    checks = []
    settings = rollout.Settings()
    cfg = replace(rollout.model_config(settings), compile_functions=False)
    for row in rows:
        path = Path(row['plan_directory'])
        raw = read(path/'result.json')
        assert digest(path/'result.json') == row['plan_result_sha256']
        assert read(path/'protocol.json')['source_sha256'] == frozen
        assert raw['config'] == previous.current.base.previous.canonical(asdict(rollout.model_config(settings)))
        again = assessment(raw)
        for key in ('verdict', 'passed', 'collision_detected', 'full_trajectory_time_s', 'time_comparison_eligible',
                    'dense_rate_within_planning_limits', 'dense_motor_within_physical_limits'):
            assert again.get(key) == row.get(key)
        save(out/'validation_assessments'/f"{row['level']}.json", again)
        track, _ = rollout.scenario.build_case(rollout.scenario.Case(level=row['level']))
        reference = rollout.Reference(track, settings, rollout.scenario.ASSETS/'meshes')
        head = np.zeros((3, settings.order))
        head[:, 0] = track.start
        now, blocks, durations = 0., [], []
        for index, request in enumerate(raw['requests']):
            np.testing.assert_allclose(request['head_state'], head, atol=1e-9)
            assert request['race_time_s'] == now
            np.testing.assert_array_equal(request['tail_state'], reference.tails[index])
            window = track.windows[index]
            planner = Planner([lambda at, w=window: rollout.scenario.pose(w, at)[0]], cfg,
                flatness_function=x500.flatness, yaw=lambda at: 0*at+np.pi/2)
            parameters = planner.parameters(head, reference.tails[index], now)
            trajectories = []
            for scale, candidate in zip(settings.temporal_scales, request['candidates']):
                stats = candidate['stats']
                seed = reference.seed(planner, head, now, index, scale)
                np.testing.assert_array_equal(seed, stats['initial_variables'])
                trajectory, terms, _ = planner.unpack(stats['variables'], parameters)
                np.testing.assert_allclose(trajectory.coefficients, candidate['coefficients'], rtol=1e-10, atol=1e-7)
                np.testing.assert_allclose(trajectory.segment_durations, candidate['durations'], rtol=1e-12)
                np.testing.assert_allclose(terms.sum()/cfg.rho, stats['objective_scaled'], rtol=1e-8, atol=1e-8)
                at = now+trajectory.segment_durations[0]
                np.testing.assert_allclose(trajectory.sample(at), window.state_at(at)[0], atol=1e-7)
                if stats['converged']:
                    assert stats['returned_point_source'] == 'accepted_lbfgs_iterate'
                trajectories.append(trajectory)
            good = [i for i, c in enumerate(request['candidates']) if c['stats']['eligible']]
            selected = min(good, key=lambda i: request['candidates'][i]['stats']['objective_scaled']) if good else None
            assert selected == request['selected_seed']
            if selected is None:
                break
            trajectory = trajectories[selected]
            event = rollout.passage(trajectory, window, reference.normals[index], reference.body_depths[index])
            assert event == request['event']
            if not event['passed']:
                break
            end = trajectory.end if index == 6 else event['body_exit_s']
            c, t = rollout.prefix(trajectory, end)
            blocks.extend(c)
            durations.extend(t)
            now = float(end)
            head = np.column_stack([trajectory.sample(now, d) for d in range(settings.order)])
        np.testing.assert_allclose(blocks, raw['coefficients'], rtol=1e-10, atol=1e-7)
        np.testing.assert_allclose(durations, raw['durations'], rtol=1e-12)
        assert raw['found'] == (len(raw['requests']) == 7 and all(r.get('event', {}).get('passed') for r in raw['requests']))
        checks.append(dict(level=row['level'], local_requests=len(raw['requests']),
            all_seeds_objectives_coefficients_events_and_prefixes_recomputed=True, common_assessment_recomputed=True))
    assert sources() == current
    save(out/'validation.json', dict(passed=True, records=10, previous_records=40,
        source_entries=len(frozen), old_sources_and_results_unchanged=True, checks=checks))


def run(out):
    with open('/tmp/convex_exp2_wang2023.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        frozen = sources()
        old = prior_rows(frozen)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
            cases=[rollout.scenario.manifest(rollout.scenario.Case(level=l)) for l in rollout.scenario.LEVELS],
            settings=asdict(rollout.Settings()), total=10, workers=3, prior_result_directory=str(PRIOR),
            prior_summary_sha256=digest(PRIOR/'summary.json'), ideal_tracking=True, gazebo_run=False))
        for rel, value in frozen.items():
            target = out/'source_snapshot'/rel
            target.parent.mkdir(parents=True, exist_ok=True)
            old_path = PRIOR/'source_snapshot'/rel
            if old_path.is_file() and digest(old_path) == value:
                os.link(old_path, target)
            else:
                shutil.copyfile(ROOT/rel, target)
        shutil.copyfile(PRIOR/'course_layout.png', out/'course_layout.png')
        save(out/'prior_summary.json', old)
        rows, active, queue = [], [], list(rollout.scenario.LEVELS)
        save(out/'summary.json', rows)

        def status(state):
            save(out/'status.json', dict(state=state, pid=os.getpid(), completed=len(rows), total=10,
                active=[dict(level=l, pid=p.pid) for p, log, l in active], pending=queue))

        try:
            while queue or active:
                assert sources() == frozen
                while queue and len(active) < 3:
                    level = queue.pop(0)
                    log = (out/f'{level}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE, 'solve', '--level', level,
                        '--outdir', str(out/'plans'/level)], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
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
                    row = record(out, level)
                    rows.append(row)
                    rows.sort(key=lambda r: rollout.scenario.LEVELS.index(r['level']))
                    save(out/'summary.json', rows)
                    report(out, rows, 'running')
                    print('RESULT', level, row['verdict'], row['full_trajectory_time_s'], flush=True)
                if queue or active:
                    time.sleep(3)
            status('validating')
            validate(out)
            report(out, rows, 'complete')
            status('complete')
            print('COMPLETE', len(rows), flush=True)
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
    parser.add_argument('action', choices=('solve', 'run', 'validate'))
    parser.add_argument('--outdir', required=True, type=Path)
    parser.add_argument('--level', choices=rollout.scenario.LEVELS)
    args = parser.parse_args()
    if args.action == 'solve':
        solve(args.level, args.outdir.resolve())
    elif args.action == 'run':
        run(args.outdir.resolve())
    else:
        validate(args.outdir.resolve())


if __name__ == '__main__':
    main()
