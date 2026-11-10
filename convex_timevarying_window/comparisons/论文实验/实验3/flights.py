"""Serial execution of each selected single-gate reference, with native audits."""
import argparse
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from . import planning, run, scenario
from .audit import audit
from ..实验2.gazebo_matrix.archive import pack, restored

MODULE = 'convex_timevarying_window.comparisons.论文实验.实验3.execution'


def report(out, rows, selections, state):
    planning.save(out/'flight_summary.json', rows)
    by_key = {(str(row['level']), row['method']):row for row in rows}
    lines = ['# 实验3 Gazebo/PX4实飞', '', f'状态 {state}，已收集 {len(rows)} 趟实际执行。', '',
        '用时包含起点到窗后终点。碰撞以原生窗框接触判定，地面接触单列。', '',
        '|幅度/m|频率/Hz|'+'|'.join(planning.LABELS.values())+'|',
        '|---:|---:|'+'---|'*len(planning.LABELS)]
    for level, selected in selections.items():
        values = []
        for method in planning.LABELS:
            row = by_key.get((level, method))
            if selected[method] is None:
                values.append('未找到')
            elif row is None:
                values.append('待执行')
            elif row.get('frame_collision'):
                values.append('碰撞')
            elif row.get('passed'):
                values.append(f"{row['flight_time_s']:.3f} s")
            else:
                values.append('未完成' if row.get('audit_available') else '证据不足')
        lines.append(f'|{level}|{level}|'+ '|'.join(values)+'|')
    lines += ['', '原生ULog、窗位姿、两路完整接触流及其SHA保留在各flight子目录。',
        '原参考逐值保持，未设置终点误差或末速验收阈值，末端测量另列。']
    (out/'FLIGHT.md').write_text('\n'.join(lines)+'\n')


def coordinator(out):
    with open('/tmp/convex_exp2_gazebo_matrix.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if (out/'flight_status.json').exists():
            raise FileExistsError('This frozen flight batch already has a coordinator record')
        while True:
            status = planning.read(out/'status.json')
            if status['state'] == 'planning_complete':
                break
            if status['state'] == 'stopped':
                raise RuntimeError('Planning stopped; inspect original planning incident')
            os.kill(status['pid'], 0)
            time.sleep(15.)
        selections = planning.read(out/'selected.json')
        sources = run.source_hashes()
        for relative in ('convex_timevarying_window/togt_style_mpc/shooting_controller.py',
                'convex_timevarying_window/togt_style_mpc/aligned_session.py',
                'convex_timevarying_window/togt_style_mpc/flatness.py',
                'convex_timevarying_window/dynamic_gate_mppi_v3/ctbr_probe.py',
                'convex_timevarying_window/comparisons/论文实验/实验2/gazebo_matrix/reference.py'):
            file = scenario.ROOT/relative
            sources[str(file)] = run.digest(file)
        planning.save(out/'flight_protocol.json', dict(source_sha256=sources, initial_phase=0.,
            repetitions=1, cpu_policy='controller 0-3; plant 4-23', no_planning_during_flight=True,
            reference='unaltered selected polynomials', start_epoch_sim_s=30.,
            no_posthoc_safe_candidate_substitution=True, state_guard_s=.1,
            full_reference_clock_finish=True, endpoint_threshold=False,
            ground_contacts_reported_separately=True))
        rows, active = [], None
        began = time.perf_counter()
        jobs = [(int(level), method, row) for level,best in selections.items()
            for method,row in best.items() if row is not None]
        (out/'flights').mkdir(exist_ok=False)
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
            OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMBA_CACHE_DIR='/tmp/convex_trajectory_only_numba',
            ACADOS_SOURCE_DIR='/home/jack/wyh/dependencies/acados-togt-v0.5.4',
            LD_LIBRARY_PATH='/home/jack/wyh/dependencies/acados-togt-v0.5.4/lib',
            PYTHONPATH=str(scenario.ROOT), GZ_SHOW_GUI='0')
        try:
            for level, method, selected in jobs:
                key = f'l{level:02d}_{method}'
                folder = out/'flights'/key
                log = (out/f'{key}_flight.log').open('w')
                source = Path(selected['source'])
                process = subprocess.Popen(['taskset', '-c', '0-3', sys.executable, '-u', '-m', MODULE,
                    '--source', str(source), '--variant', method, '--outdir', str(folder)],
                    cwd=scenario.ROOT, stdout=log, stderr=subprocess.STDOUT, env=env)
                active = process
                print('FLIGHT_STARTED', key, process.pid, flush=True)
                while process.poll() is None:
                    planning.save(out/'flight_status.json', dict(state='flying', pid=os.getpid(),
                        active_pid=process.pid, active_job=key, completed=len(rows), total=len(jobs),
                        elapsed_s=time.perf_counter()-began))
                    time.sleep(5.)
                log.close()
                active = None
                raw_path = folder/'result.json'
                audit_path = folder/'comparison_audit.json'
                row = dict(level=level, method=method, folder=str(folder), exit_code=process.returncode,
                    audit_available=audit_path.exists(), activated=False)
                marker = folder/'reference_activation.json'
                if marker.exists():
                    row['activated'] = planning.read(marker)['activated']
                if not audit_path.exists():
                    row['failure'] = 'missing_independent_flight_evidence'
                    rows.append(row)
                    report(out, rows, selections, 'stopped')
                    raise RuntimeError(f'Flight infrastructure/evidence failure {key}; raw records retained')
                result = planning.read(audit_path)
                row.update(result)
                rows.append(row)
                reference = np.load(folder/'reference_trajectory.npz')
                original = planning.read(source)
                np.testing.assert_array_equal(reference['durations'], original['durations'])
                np.testing.assert_array_equal(reference['coefficients'], original['coefficients'])
                assert planning.read(folder/'contact_manifest.json')['sensors'] == 2
                assert all(run.digest(path) == value for path,value in sources.items())
                pack(folder)
                with restored(folder):
                    repeated = audit(folder)
                assert repeated == result
                planning.save(folder/'validation.json', dict(passed=True, reference_arrays_unchanged=True,
                    source_sha256_passed=True, native_archive_restore_audit_identical=True,
                    sensor_groups=2, task_activated=row['activated']))
                report(out, rows, selections, 'flying')
                print('FLIGHT_FINISHED', key, row['verdict'], row.get('flight_time_s'), flush=True)
            planning.save(out/'flight_validation.json', dict(passed=True, formal_runs=len(rows),
                total_selected=len(jobs), archive_restoration_reaudits_passed=True,
                reference_arrays_unchanged=True, sources_unchanged=True,
                endpoint_threshold_used=False, elapsed_s=time.perf_counter()-began))
            report(out, rows, selections, 'complete')
            planning.save(out/'flight_status.json', dict(state='complete', pid=os.getpid(),
                completed=len(rows), total=len(jobs), active_pid=None, elapsed_s=time.perf_counter()-began))
            print('FLIGHTS_COMPLETE', len(rows), flush=True)
        except BaseException as error:
            if active is not None and active.poll() is None:
                active.terminate()
                active.wait(timeout=30)
            planning.save(out/'flight_status.json', dict(state='stopped', pid=os.getpid(),
                failure=repr(error), completed=len(rows), elapsed_s=time.perf_counter()-began))
            raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    coordinator(args.outdir.resolve())
