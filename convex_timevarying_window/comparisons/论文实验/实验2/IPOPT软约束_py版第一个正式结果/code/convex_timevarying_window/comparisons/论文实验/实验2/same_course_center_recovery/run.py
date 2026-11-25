"""Five-course regression of the same-course centre-first recovery policy."""
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
from .planner import Planner, embed_center
from ..independent_repair import run as base

HERE = Path(__file__).resolve().parent
ROOT = base.ROOT
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.same_course_center_recovery.run'
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
    out.mkdir(parents=True, exist_ok=False)
    frozen = sources()
    case = base.scenario.Case(level=level)
    track, _ = base.scenario.build_case(case)
    save(out/'protocol.json', dict(scene=base.scenario.manifest(case), source_sha256=frozen,
        algorithm='same_course_center_first', seconds_per_seed=300., cross_course_warm_start=False))
    start = time.perf_counter()
    planner = Planner(track, True, pose_function=base.scenario.pose)
    free_build_s = time.perf_counter()-start
    save(out/'status.json', dict(state='solving', pid=os.getpid(), level=level))
    with (out/'progress.jsonl').open('w') as log:
        def progress(row):
            import json
            line = json.dumps(row)
            log.write(line+'\n')
            log.flush()
            print(line, flush=True)
        plan, result = planner.solve_all(300., progress)
    result.update(case=dict(layout='togt_whole', level=level), config=asdict(planner.cfg),
        free_graph_build_s=free_build_s, build_s=free_build_s+result.get('center_build_s', 0.),
        planning_total_s=free_build_s+result.get('center_build_s', 0.)+result['solve_s'],
        independent_cold_center=True)
    if plan is not None:
        result.update(durations=plan.durations.tolist(), coefficients=plan.coefficients.tolist(),
            variables=plan.x.tolist(), nominal_time_s=float(plan.durations.sum()), made_at=0.)
        np.savez_compressed(out/'trajectory.npz', durations=plan.durations, coefficients=plan.coefficients, variables=plan.x)
    save(out/'result.json', result)
    if plan is not None:
        assessment = base.baseline.evaluate(plan.coefficients, plan.durations, track, base.scenario.ASSETS/'meshes')
        save(out/'assessment.json', assessment)
        save(out/'attitude_audit.json', base.attitude_audit(plan))
    assert sources() == frozen
    save(out/'status.json', dict(state='complete', pid=os.getpid(), found=result['found']))
    print('COMPLETE', level, result['found'], result.get('nominal_time_s'), result['solve_s'], flush=True)


def assess(out, level):
    path = out/'plans'/level
    raw = read(path/'result.json')
    row = dict(level=level, plan_directory=str(path), result_sha256=digest(path/'result.json'),
        solve_s=raw['solve_s'], build_s=raw['build_s'], planning_total_s=raw['planning_total_s'],
        selected_stage=raw['selected_stage'], free_refinement_converged=raw['free_refinement_converged'],
        center_time_s=raw.get('center_time_s'), free_candidate_time_s=raw.get('free_candidate_time_s'))
    if not raw['found']:
        return dict(row, passed=False, verdict='planning_not_found', full_trajectory_time_s=None)
    check = read(path/'assessment.json')
    healthy = read(path/'attitude_audit.json')['passed']
    row.update(**check, attitude_domain_passed=healthy)
    if not healthy:
        row.update(passed=False, verdict='nominal_unresolved')
    return row


def report(out, rows, state):
    lines = ['# 同赛道中心初始化回归', '', f'状态 {state}，已结束 {len(rows)}/5 条。', '',
        '全部运动固定为目标赛道；每条赛道自己求中心四初值，再开放选点精修。中心未找到则自由四初值。',
        '每条赛道的中心阶段均重新计算，中心与自由阶段成本全计入。七窗返航总时长为时间指标。', '',
        '|幅度/窗宽|频率倍数|结果|中心 s|自由精修 s|最终完整轨迹 s|返回阶段|全部规划成本 s|',
        '|---|---:|---|---:|---:|---:|---|---:|']
    for row in sorted(rows, key=lambda r: LEVELS.index(r['level'])):
        rho, factor = base.scenario.CONDITIONS[row['level']]
        number = lambda value: '—' if value is None else f'{value:.6f}'
        stage = dict(free_refinement='自由精修', center_incumbent='沿用中心基准', free_cold_fallback='自由冷启动').get(row['selected_stage'], '未找到')
        lines.append(f"|{rho:g}|{factor:g}|{base.baseline.driver.LABELS[row['verdict']]}|{number(row['center_time_s'])}|"
            f"{number(row['free_candidate_time_s'])}|{number(row['full_trajectory_time_s'] if row['passed'] else None)}|{stage}|{row['planning_total_s']:.3f}|")
    lines += ['', '每个阶段分别保存优化器收敛与安全证据。沿用中心基准时，自由阶段是否收敛单独标记。',
        '模型采用north tilt/heading pi/2；归一化SQP终止1e-5、原节点验收1e-5，完整浮点几何/STL/地面检查。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def validate(out):
    assert sources() == read(out/'protocol.json')['source_sha256']
    rows = read(out/'summary.json')
    assert len(rows) == 5 and {r['level'] for r in rows} == set(LEVELS)
    checks = []
    for row in rows:
        path = Path(row['plan_directory'])
        raw = read(path/'result.json')
        assert digest(path/'result.json') == row['result_sha256']
        assert raw['independent_cold_center']
        track, _ = base.scenario.build_case(base.scenario.Case(level=row['level']))
        p = Planner(track, True, pose_function=base.scenario.pose)
        if raw['found']:
            plan = base.frozen_plan(raw)
            g, _, c = p.evaluate(plan.x, p.witnesses())
            assert g.min() >= -1e-5
            np.testing.assert_allclose(c, plan.coefficients, rtol=1e-10, atol=1e-7)
            geometric, passed = p.verify(plan)
            assert passed and len(geometric) == 56
            again = base.baseline.evaluate(c, plan.durations, track, base.scenario.ASSETS/'meshes')
            attitude = base.attitude_audit(plan)
            if not attitude['passed']:
                again.update(passed=False, verdict='nominal_unresolved')
            assert again['verdict'] == row['verdict']
            assert attitude['passed'] == row['attitude_domain_passed']
            if raw.get('center_reference'):
                embedded = embed_center(p, raw['center_reference']['variables'])
                np.testing.assert_array_equal(embedded, raw['free_initial_variables'])
                assert plan.durations.sum() <= raw['center_time_s']+1e-8
            if raw['selected_stage'] == 'free_refinement':
                assert raw['free_refinement_converged']
                final = raw['free_stage']['candidates'][0]['statistics']['records'][-1]
                assert final['optimizer_converged'] and final['global_passed']
        checks.append(dict(level=row['level'], verdict=row['verdict'], selected_stage=row['selected_stage']))
    save(out/'validation.json', dict(passed=True, records=len(rows), all_assessments_recomputed=True,
        independent_course_inputs=True, source_hashes_unchanged=True, checks=checks))


def run(out):
    with open('/tmp/convex_exp2_same_course_center.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        frozen = sources()
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
            levels=LEVELS, all_center_stages_fresh=True, gazebo_run=False))
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
                while queue and len(active) < 3:
                    level = queue.pop(0)
                    log = (out/f'{level}.log').open('w')
                    args = [sys.executable, '-u', '-m', MODULE, 'solve', '--outdir', str(out/'plans'/level), '--level', level]
                    process = subprocess.Popen(args, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
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
                    save(out/'summary.json', rows)
                    report(out, rows, 'running')
                    print('RESULT', level, row['verdict'], row['full_trajectory_time_s'], row['selected_stage'], flush=True)
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
    out = args.outdir.resolve()
    if args.action == 'run':
        run(out)
    elif args.action == 'validate':
        validate(out)
    else:
        solve(args.level, out)


if __name__ == '__main__':
    main()
