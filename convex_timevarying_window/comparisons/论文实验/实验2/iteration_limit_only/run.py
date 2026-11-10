"""Independent cold starts with one changed SLSQP option."""
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
import trimesh
from .planner import Planner
from ..basic_group.whole_course_difficulty import run as baseline, scenario
from ..basic_group.trajectory_only.collision import CheckConfig

HERE = Path(__file__).resolve().parent
ROOT = scenario.ROOT
PRIOR = HERE.parent/'results/togt_whole_course_factorial_v1_20261106'
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.iteration_limit_only.run'
read, save, digest = baseline.driver.read, baseline.driver.save, baseline.driver.digest
METHODS = ('ours', 'center')
NAMES = dict(ours='自由点', center='强制中心')


def sources():
    result = baseline.sources()
    for path in HERE.rglob('*'):
        if path.is_file() and path.suffix in ('.py', '.md'):
            result[str(path.relative_to(ROOT))] = digest(path)
    return result


def prior_rows():
    return [r for r in read(PRIOR/'summary.json')
            if r['variant'] in METHODS and r['verdict'] == 'planning_not_found']


def check_prior(frozen):
    old = read(PRIOR/'protocol.json')['source_sha256']
    changed = [p for p, h in old.items() if frozen.get(p) != h]
    if changed:
        raise RuntimeError('original frozen source changed '+repr(changed))
    return len(old)


def canonical(value):
    import json
    return json.loads(json.dumps(value))


def plan(level, method, out):
    from .. import plan as entry
    old_row = next(r for r in prior_rows() if (r['level'], r['variant']) == (level, method))
    old_path = Path(old_row['plan_directory'])
    old = read(old_path/'result.json')
    assert digest(old_path/'result.json') == old_row['plan_result_sha256']
    scene = canonical(scenario.manifest(scenario.Case(level=level)))
    assert scene == read(old_path/'protocol.json')['scene']

    def independent_planner(track, free_points):
        p = Planner(track, free_points, pose_function=scenario.pose, max_iterations=2000)
        seeds = p.seeds()
        assert canonical(asdict(p.cfg)) == old['config']
        errors = [float(np.max(np.abs(x-np.array(c['initial_variables']))))
                  for x, c in zip(seeds, old['candidates'])]
        assert len(seeds) == len(old['candidates']) == 4 and max(errors) < 2e-10
        save(out/'input_equivalence.json', dict(passed=True, original_result_sha256=digest(old_path/'result.json'),
            original_directory=str(old_path), scene_equal=True, config_equal=True,
            initial_variables_max_abs_difference=errors, independently_generated_four_seeds=True,
            inherited_trajectory=False, amplitude_continuation=False, max_iterations=2000,
            unchanged_seconds_per_seed=180., unchanged_objective='log1p(sum(T))',
            unchanged_attitude='original fixed world Y', original_frozen_sources=check_prior(sources())))
        return p

    entry.Case = scenario.Case
    entry.build_case = scenario.build_case
    entry.manifest = scenario.manifest
    entry.Planner = independent_planner
    sys.argv = [entry.__name__, '--layout', 'togt_whole', '--level', level,
                '--method', method, '--outdir', str(out), '--seconds-per-seed', '180']
    entry.main()


def termination_counts(raw):
    counts = {}
    for candidate in raw['candidates']:
        for record in candidate['statistics']['records']:
            status = record.get('status', record.get('stage', 'unknown'))
            counts[status] = counts.get(status, 0)+1
    return counts


def assess(out, level, method):
    path = out/'plans'/f'{level}_{method}'
    raw = read(path/'result.json')
    equivalence = read(path/'input_equivalence.json')
    assert equivalence['passed'] and not equivalence['inherited_trajectory']
    assert raw['max_iterations'] == 2000 and raw['independent_cold_start']
    assert not raw['amplitude_continuation'] and not raw['cross_condition_warm_start']
    for call in raw['backend_calls']:
        assert call['options_before'] == dict(maxiter=400, ftol=1e-7, disp=False)
        assert call['options_after'] == dict(maxiter=2000, ftol=1e-7, disp=False)
    row = baseline.assess(path, level, method)
    old_row = next(r for r in prior_rows() if (r['level'], r['variant']) == (level, method))
    return dict(row, original_verdict=old_row['verdict'], original_solve_s=old_row['solve_s'],
                termination_counts=termination_counts(raw), independent_input_audit=equivalence,
                max_iterations=2000, seconds_per_seed=180.)


def ordered(rows):
    return sorted(rows, key=lambda r: (scenario.LEVELS.index(r['level']), METHODS.index(r['variant'])))


def report(out, rows, state):
    lines = ['# 仅放宽迭代上限的独立求解', '', f'状态 {state}，已完成 {len(rows)}/9 条。', '',
             '每个条件直接使用自身完整运动和原四初值，只将 SLSQP 上限 400 改为 2000。每初值仍为180秒。',
             '原固定Y姿态、log1p总时间、模型、约束、验收和择解保持一致。原9条均未找到合格规划。',
             '时间含七窗后返航至零VAJ，为规划轨迹结果。', '',
             '|幅度/窗宽|频率倍数（Hz范围）|方法|新结果|合格轨迹时间 s|四初值求解 s|终止记录|',
             '|---|---|---|---|---:|---:|---|']
    for row in ordered(rows):
        lo, hi = scenario.manifest(scenario.Case(level=row['level']))['frequency_range_hz']
        t = row['full_trajectory_time_s']
        cell = f'{t:.6f}' if row['passed'] else '—'
        terms = ' / '.join(f'{k} ×{v}' for k, v in row['termination_counts'].items())
        lines.append(f"|{row['amplitude_ratio']:g}|{row['frequency_factor']:g}（{lo:g}–{hi:g}）|{NAMES[row['variant']]}|"
                     f"{baseline.driver.LABELS[row['verdict']]}|{cell}|{row['solve_s']:.3f}|{terms}|")
    lines += ['', '全部原始四候选见 plans，逐条件复算见 assessments。未找到记录不代表物理无解。',
              '合格要求优化器收敛、原节点硬约束及全部几何复查通过，并独立复算整条轨迹与所有实体窗框和地面。',
              '这是名义球/STL浮点数值评估。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')


def validate(out):
    protocol = read(out/'protocol.json')
    assert sources() == protocol['source_sha256']
    assert check_prior(sources()) == 2386
    rows = read(out/'summary.json')
    assert {(r['level'], r['variant']) for r in rows} == {(r['level'], r['variant']) for r in prior_rows()}
    checks = []
    for row in ordered(rows):
        path = Path(row['plan_directory'])
        assert digest(path/'result.json') == row['plan_result_sha256']
        again = assess(out, row['level'], row['variant'])
        assert again['verdict'] == row['verdict']
        assert again['full_trajectory_time_s'] == row['full_trajectory_time_s']
        assert again['termination_counts'] == row['termination_counts']
        for check in again.get('checks', []):
            if check['outcome'] == 'collision':
                mesh = trimesh.load_mesh(scenario.ASSETS/'meshes'/f"gate_{check['gate']}_sleeve.stl")
                distance = trimesh.proximity.closest_point_naive(mesh, np.array(check['witness']['local_position'])[None])[1][0]
                assert distance-CheckConfig().radius < -CheckConfig().numerical_guard_m
        checks.append(dict(level=row['level'], method=row['variant'], verdict=row['verdict'],
                           plan_result_sha256=row['plan_result_sha256']))
    save(out/'validation.json', dict(passed=True, records=len(rows), original_sources_unchanged=2386,
        all_assessments_recomputed=True, source_sha256_unchanged=True, independent_cold_starts=True, checks=checks))


def run(out):
    with open('/tmp/convex_exp2_iteration_limit_only.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out.mkdir(parents=True, exist_ok=False)
        (out/'plans').mkdir()
        (out/'assessments').mkdir()
        frozen = sources()
        count = check_prior(frozen)
        targets = prior_rows()
        assert len(targets) == 9
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
            prior=str(PRIOR), prior_protocol_sha256=digest(PRIOR/'protocol.json'),
            prior_summary_sha256=digest(PRIOR/'summary.json'), original_sources_unchanged=count,
            targets=[dict(level=r['level'], method=r['variant']) for r in targets],
            max_iterations=2000, seconds_per_seed=180., parallel_workers=3, independent_cold_starts=True,
            checker=asdict(CheckConfig()), gazebo_run=False))
        for rel, h in frozen.items():
            dest = out/'source_snapshot'/rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            old = PRIOR/'source_snapshot'/rel
            if old.is_file() and digest(old) == h:
                os.link(old, dest)
            else:
                shutil.copyfile(ROOT/rel, dest)
        priority = [('a25_f3', 'ours'), ('a75_f3', 'ours'), ('a75_f3', 'center'),
                    ('a75_f1', 'ours'), ('a50_f6', 'center')]
        pairs = [(r['level'], r['variant']) for r in targets]
        queue = priority+[p for p in pairs if p not in priority]
        active, rows = [], []
        save(out/'summary.json', rows)
        report(out, rows, 'running')

        def status(state):
            save(out/'status.json', dict(state=state, pid=os.getpid(), completed=len(rows), total=9,
                active=[dict(level=l, method=m, pid=p.pid) for p, log, l, m in active],
                pending=[dict(level=l, method=m) for l, m in queue]))

        try:
            while queue or active:
                assert sources() == frozen
                while queue and len(active) < 3:
                    level, method = queue.pop(0)
                    log = (out/f'{level}_{method}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE, 'plan', '--level', level,
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
                        raise RuntimeError(f'child exited {level} {method} {code}')
                    row = assess(out, level, method)
                    save(out/'assessments'/f'{level}_{method}.json', row)
                    rows.append(row)
                    save(out/'summary.json', ordered(rows))
                    report(out, rows, 'running')
                    print('RESULT', level, method, row['verdict'], row['full_trajectory_time_s'], flush=True)
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
    parser.add_argument('action', choices=('run', 'plan', 'validate'))
    parser.add_argument('--outdir', required=True, type=Path)
    parser.add_argument('--level')
    parser.add_argument('--method', choices=METHODS)
    args = parser.parse_args()
    out = args.outdir.resolve()
    if args.action == 'run':
        run(out)
    elif args.action == 'validate':
        validate(out)
    else:
        plan(args.level, args.method, out)


if __name__ == '__main__':
    main()
