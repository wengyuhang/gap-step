"""Run the unchanged four-start planner with an infinite wall-clock budget."""
import argparse
from dataclasses import asdict
import fcntl
import json
import os
from pathlib import Path
import shutil
import time

import numpy as np

from ..independent_free_solve.planner import Planner
from ..independent_repair.run import attitude_audit
from ..basic_group.whole_course_difficulty import scenario
from ..basic_group.trajectory_only.collision import evaluate
from ..gazebo_matrix import run as original

HERE = Path(__file__).resolve().parent
ROOT = scenario.ROOT
PRIOR = HERE.parent/'results/independent_full_matrix_v2_20261107/plans/a25_f6_ours/result.json'


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    temporary = path.with_name(path.name+'.new')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def report(out, rows, state, comparison=None):
    lines = ['# 0.25／6原自由点方法取消墙钟求解上限', '', f'状态为{state}，已结束{len(rows)}/4个初值。', '',
        '每初值墙钟预算无上限，算法自身的收敛和迭代终止规则见protocol.json。',
        '时间为七窗后返航完整参考时长，求解耗时单列。', '',
        '|初值|求解结果|完整参考 s|该初值求解耗时 s|终止状态|', '|---:|---|---:|---:|---|']
    for row in rows:
        duration = '—' if row['nominal_time_s'] is None else f"{row['nominal_time_s']:.6f}"
        lines.append(f"|{row['seed']}|{'获得合格候选' if row['found'] else '未找到合格候选'}|{duration}|{row['solve_s']:.3f}|{row['termination']}|")
    if comparison:
        lines += ['', '## 完整规划时间比较', '', '|方法|完整参考 s|', '|---|---:|',
            f"|原300秒/初值自由点|{comparison['previous_time_s']:.6f}|",
            f"|本轮无墙钟上限自由点|{comparison['new_time_s']:.6f}|",
            '|原动态TOGT自由点|21.676888|', '',
            f"本轮建图{comparison['build_s']:.3f}秒，四初值求解合计{comparison['solve_s']:.3f}秒。",
            '名义几何结果与节点间动力学诊断分别保存在assessment.json及attitude_audit.json。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    with open('/tmp/convex_exp2_unlimited_a25_f6.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        frozen = original.sources()
        for path in HERE.glob('*'):
            if path.suffix in ('.py', '.md'):
                frozen[str(path.relative_to(ROOT))] = original.digest(path)
        snapshot = HERE.parent/'results/radius090_a25_f6_v1_20261107/execution_source_snapshot'
        for relative, expected in frozen.items():
            target = out/'source_snapshot'/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            prior = snapshot/relative
            if prior.is_file() and original.digest(prior) == expected:
                os.link(prior, target)
            else:
                shutil.copyfile(ROOT/relative, target)
        save(out/'protocol.json', dict(case=dict(layout='togt_whole', level='a25_f6'),
            scene=scenario.manifest(scenario.Case(level='a25_f6')),
            wall_time_limit_s_per_seed=None, wall_time_limit_disabled=True,
            effective_sqp_max_iterations=2000, augmented_restoration_rounds=16,
            inner_lbfgs_max_iterations=120, geometry_exchange_max_rounds=24,
            radius_m=.38594472, independent_four_analytic_initializations=True,
            source_sha256=frozen, protocol=(HERE/'PROTOCOL.md').read_text()))
        save(out/'status.json', dict(state='building', pid=os.getpid(), completed=0, total=4))
        track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
        began = time.perf_counter()
        planner = Planner(track, pose_function=scenario.pose)
        build_s = time.perf_counter()-began
        assert planner.cfg.radius == .38594472
        previous = read(PRIOR)
        assert previous['config'] == json.loads(json.dumps(asdict(planner.cfg)))
        for seed, prior in zip(planner.seeds(), previous['candidates']):
            np.testing.assert_array_equal(seed, prior['initial_variables'])
        save(out/'build.json', dict(build_s=build_s, variables=planner.nvar, config=asdict(planner.cfg)))
        rows = []
        report(out, rows, 'running')
        try:
            with (out/'progress.jsonl').open('w') as log:
                def progress(row):
                    log.write(json.dumps(row, ensure_ascii=False)+'\n')
                    log.flush()
                    if row.get('stage') == 'seed_finished':
                        rows.append(dict(seed=row['seed'], found=row['found'], nominal_time_s=row['nominal_time_s'],
                            solve_s=row['elapsed_s']-(rows[-1]['elapsed_s'] if rows else 0.),
                            elapsed_s=row['elapsed_s'], termination='见result.json的原始求解状态'))
                        save(out/'seed_progress.json', rows)
                        report(out, rows, 'running')
                        print(json.dumps(row, ensure_ascii=False), flush=True)
                    save(out/'status.json', dict(state='solving', pid=os.getpid(), completed=len(rows), total=4,
                        current_seed=row.get('temporal_seed', row.get('seed')),
                        last_progress=row, wall_time_limit_disabled=True))
                plan, result = planner.solve_all(seconds_per_seed=np.inf, progress=progress)
            result.update(build_s=build_s, case=dict(layout='togt_whole', level='a25_f6'),
                method='ours', config=asdict(planner.cfg), wall_time_limit_s_per_seed=None,
                wall_time_limit_disabled=True)
            if plan is not None:
                result.update(durations=plan.durations.tolist(), coefficients=plan.coefficients.tolist(),
                    variables=plan.x.tolist(), made_at=0., nominal_time_s=float(plan.durations.sum()),
                    crossing_times_s=plan.crossing_times.tolist(),
                    crossing_points=np.array(planner.point_function(plan.x)).T.tolist())
                np.savez_compressed(out/'trajectory.npz', durations=plan.durations,
                    coefficients=plan.coefficients, variables=plan.x)
            save(out/'result.json', result)
            assert len(result['candidates']) == 4 and result['all_four_attempted']
            assert previous['config'] == json.loads(json.dumps(asdict(planner.cfg)))
            for seed, candidate, prior in zip(planner.seeds(), result['candidates'], previous['candidates']):
                np.testing.assert_array_equal(seed, candidate['initial_variables'])
                np.testing.assert_array_equal(seed, prior['initial_variables'])
                assert not any(r.get('status') == 'wall_budget' for r in candidate['statistics']['records'])
            rows = []
            for candidate in result['candidates']:
                stats = candidate['statistics']
                rows.append(dict(seed=candidate['seed'], found=candidate['found'],
                    nominal_time_s=candidate['nominal_time_s'], solve_s=stats['solve_s'],
                    termination=stats['records'][-1]['status']))
            save(out/'seed_summary.json', rows)
            validation = dict(passed=True, independent_four_initializations_equal_to_original=True,
                wall_clock_timeouts=0, all_original_source_contents_unchanged=True, selected_plan_found=plan is not None)
            comparison = None
            if plan is not None:
                g, _, curve = planner.evaluate(plan.x, planner.witnesses())
                np.testing.assert_allclose(curve, plan.coefficients, rtol=1e-10, atol=1e-7)
                assert g.min() >= -1e-5
                geometry, passed = planner.verify(plan)
                assert passed and len(geometry) == 56
                assessment = evaluate(plan.coefficients, plan.durations, track, scenario.ASSETS/'meshes')
                save(out/'assessment.json', assessment)
                save(out/'attitude_audit.json', attitude_audit(plan))
                validation.update(original_node_minimum=float(g.min()), all_56_geometry_passed=True,
                    ordered_exactly_once=assessment['ordered_exactly_once'], endpoints_passed=assessment['endpoints_passed'])
                comparison = dict(previous_time_s=previous['nominal_time_s'], new_time_s=float(plan.durations.sum()),
                    build_s=build_s, solve_s=result['solve_s'])
                save(out/'comparison.json', comparison)
            assert all(original.digest(ROOT/relative) == expected for relative, expected in frozen.items())
            save(out/'validation.json', validation)
            report(out, rows, 'complete', comparison)
            save(out/'status.json', dict(state='complete', pid=os.getpid(), completed=4, total=4,
                found=plan is not None, nominal_time_s=result.get('nominal_time_s'), validation_passed=True))
            print('COMPLETE', json.dumps(read(out/'status.json')), flush=True)
        except BaseException as exc:
            save(out/'error.json', dict(type=type(exc).__name__, message=str(exc)))
            save(out/'status.json', dict(state='stopped', pid=os.getpid(), completed=len(rows), failure=repr(exc)))
            raise


if __name__ == '__main__':
    main()
