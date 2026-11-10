"""One frozen batch of forty plans and serial native Gazebo executions."""
import argparse
import fcntl
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.time_mapping import durations_from_k, k_from_durations
from ..soft_dynamics import run as common
from ..soft_dynamics.planner import Planner
from ..static_calibration.planner import Objective
from ..radius_ablation.execution import audit
from ..independent_repair.run import frozen_plan
from .worker import METHODS

HERE = Path(__file__).resolve().parent
ROOT, RESULTS = common.ROOT, common.RESULTS
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.no_tilt_comparison'
TOTAL = len(common.scenario.LEVELS)*len(METHODS)
read, save, digest = common.read, common.save, common.previous.digest


def sources():
    result = common.sources()
    for path in HERE.rglob('*'):
        if path.is_file() and not {'__pycache__', 'build'}.intersection(path.relative_to(HERE).parts):
            result[str(path.relative_to(ROOT))] = digest(path)
    return result


def preserve_old_records():
    result = common.old_records()
    prior = RESULTS/'soft_dynamics_paired_v1_20261107'
    for path in prior.rglob('*'):
        if path.is_file() and 'source_snapshot' not in path.relative_to(prior).parts:
            result[str(path)] = digest(path)
    return result


def check_frozen(out):
    assert sources() == read(out/'protocol.json')['source_sha256']


def validate_planning(out):
    checks = []
    for row in read(out/'planning_summary.json'):
        check_frozen(out)
        folder = Path(row['directory'])
        raw = read(folder/'result.json')
        assert digest(folder/'result.json') == row['result_sha256']
        assert raw['tilt_penalty_weight'] == 0. and not raw['tilt_penalty_enabled']
        assert len(raw['candidates']) == 4
        track, _ = common.scenario.build_case(common.scenario.Case(level=row['level']))
        check = dict(level=row['level'], variant=row['variant'], found=raw['found'])
        if row['variant'] in ('ours', 'center'):
            planner = Planner(track, row['variant'] == 'ours', pose_function=common.scenario.pose)
            seeds = planner.seeds()
            good = [c for c in raw['candidates'] if c['found']]
            if raw['found']:
                plan = frozen_plan(raw)
                cost, _, penalty = planner.objective(plan.x)
                np.testing.assert_allclose([cost, penalty], [raw['objective'], raw['dynamic_soft_integral']], atol=1e-7, rtol=1e-10)
                g, _, coefficients = planner.evaluate(plan.x, planner.witnesses())
                np.testing.assert_allclose(coefficients, plan.coefficients, atol=1e-7, rtol=1e-10)
                assert g.min() >= -1e-5 and planner.verify(plan)[1]
                check['minimum_hard_constraint'] = float(g.min())
        else:
            planner = Objective(track, row['variant'].endswith('_center'))
            initial = planner.initial_guess()
            times = durations_from_k(initial[:8])
            seeds = []
            for scale in (1., .8, 1.2, 1.5):
                seed = initial.copy()
                seed[:8] = k_from_durations(times*scale)
                seeds.append(seed)
            good = [c for c in raw['candidates'] if c['success']]
            if raw['found']:
                cost, _ = planner.value_and_gradient(np.asarray(raw['variables']))
                np.testing.assert_allclose(cost, raw['objective'], atol=1e-7, rtol=1e-10)
                forward = planner.forward(raw['variables'])
                coefficients = forward.trajectory.coefficients*forward.durations[:, None, None]**np.arange(8)[None, :, None]
                np.testing.assert_allclose(coefficients, raw['coefficients'], atol=1e-7, rtol=1e-10)
                assert all(raw['safe_point_membership'])
        for initial, candidate in zip(seeds, raw['candidates']):
            np.testing.assert_array_equal(initial, candidate['initial_variables'])
        assert bool(good) == raw['found']
        if raw['found']:
            np.testing.assert_allclose(raw['objective'], min(c['objective'] for c in good), atol=1e-7, rtol=1e-10)
            again = common.assess(out, row['level'], row['variant'])
            assert again['outcome'] == row['outcome']
        check.update(four_independent_initializers_rebuilt=True, selected_objective_recomputed=True,
            frame_geometry_recomputed=True, tilt_weight_zero=True)
        checks.append(check)
    assert len(checks) == TOTAL
    save(out/'planning_validation.json', dict(passed=True, records=TOTAL, checks=checks))


def plan_matrix(out):
    active, rows = [], []
    queue = [(level, method) for level in common.scenario.LEVELS for method in METHODS]
    save(out/'planning_summary.json', rows)
    try:
        while queue or active:
            check_frozen(out)
            while queue and len(active) < 4:
                level, method = queue.pop(0)
                log = (out/f'{level}_{method}.log').open('w')
                process = subprocess.Popen([sys.executable, '-u', '-m', MODULE+'.worker',
                    '--level', level, '--method', method, '--outdir', str(out/'plans'/f'{level}_{method}')],
                    cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                active.append((process, log, level, method))
                print('PLAN_START', level, method, process.pid, flush=True)
            for item in list(active):
                process, log, level, method = item
                if process.poll() is None:
                    continue
                log.close()
                active.remove(item)
                if process.returncode:
                    raise RuntimeError(f'planning worker {level} {method} exit {process.returncode}')
                row = common.assess(out, level, method)
                rows.append(row)
                save(out/'planning_summary.json', rows)
                print('PLAN_RESULT', level, method, row['outcome'], row['full_trajectory_time_s'], flush=True)
            save(out/'status.json', dict(state='planning', pid=os.getpid(), completed=len(rows), total=TOTAL,
                active=[dict(pid=p.pid, level=l, method=m) for p, log, l, m in active], pending=queue))
            if queue or active:
                time.sleep(10)
        validate_planning(out)
        check_frozen(out)
    finally:
        for process, log, level, method in active:
            if process.poll() is None:
                process.terminate()
            process.wait()
            log.close()


def validate_flight(out, rows):
    for row in rows:
        if not row.get('directory'):
            continue
        with common.archive.restored(Path(row['directory'])):
            again = audit(Path(row['directory']))
        for key in ('passed', 'verdict', 'frame_collision', 'actual_gate_count', 'full_reference_execution_time_s'):
            assert again[key] == row[key], (row['level'], row['variant'], key)
    for level in common.scenario.LEVELS:
        records = [read(Path(r['directory'])/'result.json') for r in rows
            if r['level'] == level and r.get('directory')]
        for record in records[1:]:
            assert record['parameters'] == records[0]['parameters']
            assert record['mpc_config'] == records[0]['mpc_config']
        worlds = [digest(Path(r['directory'])/'world.sdf') for r in rows
            if r['level'] == level and r.get('directory')]
        assert len(set(worlds)) <= 1
    assert len(rows) == TOTAL
    save(out/'flight_validation.json', dict(passed=True, records=TOTAL,
        native_flights_reaudited=sum(bool(r.get('directory')) for r in rows),
        same_world_and_mpc_and_parameters=True, complete_reference_clock_used=True))


def fly_matrix(out):
    assert read(out/'planning_validation.json')['passed']
    plans, rows = read(out/'planning_summary.json'), []
    save(out/'flight_summary.json', rows)
    for index, level in enumerate(common.scenario.LEVELS):
        for method in METHODS if index%2 == 0 else tuple(reversed(METHODS)):
            check_frozen(out)
            plan = next(r for r in plans if (r['level'], r['variant']) == (level, method))
            if not plan['found']:
                row = dict(level=level, variant=method, passed=False, verdict='planning_not_found',
                    frame_collision=False, full_reference_execution_time_s=None, actual_gate_count=None)
            else:
                if shutil.disk_usage(ROOT).free < 3*2**30:
                    raise RuntimeError('insufficient_disk_for_native_flight')
                source = Path(plan['directory'])/'result.json'
                for attempt in (1, 2):
                    folder = out/'flights'/f'{level}_{method}'/f'attempt_{attempt}'
                    folder.parent.mkdir(parents=True, exist_ok=True)
                    log = (out/f'{level}_{method}_flight_attempt_{attempt}.log').open('w')
                    process = subprocess.Popen([sys.executable, '-u', '-m', MODULE+'.flight',
                        '--source', str(source), '--method', method, '--outdir', str(folder)],
                        cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
                    save(out/'status.json', dict(state='flying', pid=os.getpid(), worker_pid=process.pid,
                        level=level, method=method, attempt=attempt, completed=len(rows), total=TOTAL))
                    print('FLY', level, method, attempt, process.pid, flush=True)
                    try:
                        code = process.wait(timeout=900)
                    except subprocess.TimeoutExpired:
                        process.terminate()
                        process.wait(timeout=20)
                        raise RuntimeError('flight_worker_timeout')
                    finally:
                        log.close()
                    audit_file = folder/'comparison_audit.json'
                    activation = folder/'reference_activation.json'
                    if code or not audit_file.is_file():
                        inactive = activation.is_file() and not read(activation)['activated']
                        if inactive and attempt == 1:
                            save(folder/'startup_incident.json', dict(worker_exit_code=code,
                                reference_activated=False, action='same_configuration_reinitialization'))
                            common.archive.pack(folder)
                            continue
                        raise RuntimeError(f'flight worker or evidence failure {level} {method} exit {code}')
                    row = dict(read(audit_file), level=level, variant=method, directory=str(folder),
                        reference_source_sha256=digest(source), attempt=attempt)
                    common.archive.pack(folder)
                    with common.archive.restored(folder):
                        again = audit(folder)
                    for key in ('passed', 'verdict', 'frame_collision', 'actual_gate_count', 'full_reference_execution_time_s'):
                        assert again[key] == row[key]
                    print('FLIGHT_RESULT', level, method, row['verdict'], row['full_reference_execution_time_s'], flush=True)
                    break
            rows.append(row)
            save(out/'flight_summary.json', rows)
    validate_flight(out, rows)


def run(out):
    with open('/tmp/convex_exp2_no_tilt.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with open('/tmp/convex_exp2_gazebo_matrix.lock', 'a') as flight_lock:
            fcntl.flock(flight_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            out.mkdir(parents=True, exist_ok=False)
            (out/'plans').mkdir()
            (out/'flights').mkdir()
            save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(),
                source_sha256=sources(), methods=METHODS, total=TOTAL, tilt_penalty_weight=0.,
                cases=[common.scenario.manifest(common.scenario.Case(level=l)) for l in common.scenario.LEVELS]))
            common.snapshot(out, read(out/'protocol.json')['source_sha256'])
            save(out/'prior_results_sha256.json', preserve_old_records())
            try:
                plan_matrix(out)
                fly_matrix(out)
                check_frozen(out)
                prior = read(out/'prior_results_sha256.json')
                assert all(digest(Path(path)) == expected for path, expected in prior.items())
                from .report import export
                export(out)
                save(out/'final_checks.json', dict(passed=True, original_records_unchanged=len(prior),
                    source_snapshot_unchanged=True, planning_and_native_flight_validation_passed=True))
                save(out/'status.json', dict(state='complete', pid=os.getpid(), completed=TOTAL, total=TOTAL))
            except BaseException as exc:
                save(out/'error.json', dict(error=repr(exc), traceback=traceback.format_exc()))
                save(out/'status.json', dict(state='stopped', pid=os.getpid(), failure=repr(exc)))
                raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    run(args.outdir.resolve())


if __name__ == '__main__':
    main()
