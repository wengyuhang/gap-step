"""Catalogue all selected references and execute the paired Gazebo matrix."""
import argparse
from dataclasses import asdict
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback
import numpy as np
from . import archive
from .audit import audit
from ..wang2023 import run as nominal
from ..basic_group.whole_course_difficulty import scenario

HERE, ROOT = Path(__file__).resolve().parent, nominal.ROOT
NOMINAL = HERE.parent/'results/wang2023_full_v1_20261107'
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.gazebo_matrix'
METHODS = ('ours', 'center', 'dynamic_togt', 'dynamic_togt_center', 'wang2023')
NAMES = ('本文自由点', '本文中心', '动态TOGT自由点', '动态TOGT中心', 'Wang独立复现')
read, digest = nominal.read, nominal.digest


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.new')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temporary.replace(path)


def sources():
    result = nominal.sources()
    directories = (HERE, ROOT/'convex_timevarying_window/togt_style_mpc',
        ROOT/'convex_timevarying_window/single_gate_stop_racing',
        ROOT/'convex_timevarying_window/simple_two_gate_soft_only_mpc',
        ROOT/'convex_timevarying_window/global_gate_minco_probe')
    for directory in directories:
        for path in directory.rglob('*.py'):
            if not {'results', 'build', '.build', '__pycache__', 'native'}.intersection(path.relative_to(directory).parts):
                result[str(path.relative_to(ROOT))] = digest(path)
    for path in HERE.glob('*.md'):
        result[str(path.relative_to(ROOT))] = digest(path)
    for relative in ('convex_timevarying_window/comparisons/论文实验/实验2/static_calibration/assets/build/libMixedGateMotion.so',
                     'convex_timevarying_window/comparisons/论文实验/实验2/assets/MixedGateMotion.cc'):
        result[relative] = digest(ROOT/relative)
    return result


def catalogue(out):
    assert read(NOMINAL/'validation.json')['passed']
    rows = read(NOMINAL/'comparison_50.json')
    assert len(rows) == 50 and len({(r['level'], r['variant']) for r in rows}) == 50
    out.mkdir(parents=True, exist_ok=False)
    records = []
    for level in scenario.LEVELS:
        for method in METHODS:
            row = next(r for r in rows if r['level'] == level and r['variant'] == method)
            source = Path(row['plan_directory'])/'result.json'
            assert digest(source) == row['plan_result_sha256']
            raw = read(source)
            folder = out/'references'/f'{level}_{method}'
            folder.mkdir(parents=True)
            shutil.copyfile(source, folder/'source_result.json')
            record = dict(level=level, variant=method, reference_directory=str(folder),
                source_result=str(source), source_result_sha256=digest(source), found=raw['found'],
                nominal_verdict=row['verdict'], nominal_dynamics_eligible=row['time_comparison_eligible'],
                nominal_time_s=raw.get('nominal_time_s') if raw['found'] else None,
                planning_total_s=row.get('planning_total_s', raw['build_s']+raw['solve_s']))
            if raw['found']:
                coefficients, durations = np.asarray(raw['coefficients']), np.asarray(raw['durations'])
                assert coefficients.shape == (len(durations), 8, 3)
                assert np.isfinite(coefficients).all() and np.isfinite(durations).all() and np.all(durations > 0.)
                np.savez_compressed(folder/'reference.npz', coefficients=coefficients, durations=durations)
                record['reference_sha256'] = digest(folder/'reference.npz')
            save(folder/'manifest.json', record)
            records.append(record)
    assert sum(r['found'] for r in records) == 49
    save(out/'catalogue.json', records)
    save(out/'catalogue_origin.json', dict(directory=str(NOMINAL), comparison_sha256=digest(NOMINAL/'comparison_50.json'),
        total=50, available_trajectories=49, planning_not_found=1))
    return records


def report(out, rows, state):
    lines = ['# 五方法十赛道Gazebo/PX4执行', '', f'状态 {state}，已登记 {len(rows)}/50 项。', '',
        '执行现有选中轨迹，统一x500/PX4和北图航向解析参考MPC。完整时间从统一起跑时刻到七窗后返航停稳，单位秒。',
        '完成条件为七窗整机出面、每窗正向一次、返回距起点0.55m以内且真值速度低于1m/s，并且全部原生接触记录为零。', '',
        '|幅度/窗宽|频率倍数|'+'|'.join(NAMES)+'|', '|---|---:|---:|---:|---:|---:|---:|']
    labels = dict(collision='碰撞', not_completed='未完成', missed_gate='漏窗', startup_failure='启动失败',
        evidence_failure='证据不足', planning_not_found='未找到规划', terminal_speed_not_met='终速未达标')
    for level in scenario.LEVELS:
        by = {r['variant']: r for r in rows if r['level'] == level}
        rho, factor = (0., 1.) if level == 'static' else scenario.CONDITIONS[level]
        cells = []
        for method in METHODS:
            row = by.get(method)
            cells.append('待飞' if row is None else f"{row['flight_time_s']:.3f}" if row['passed'] else labels.get(row['verdict'], row['verdict']))
        lines.append('|'+('静态' if level == 'static' else f'{rho:g}')+f'|{factor:g}|'+'|'.join(cells)+'|')
    lines += ['', '七窗基准频率0.30/0.40/0.50/0.60/0.55/0.45/0.35Hz。运动与规划阶段保持相同初相及绝对时钟。', '',
        '## 各次执行记录', '', '|条件|方法|结果|七窗计数|参考时间 s|闭环时间 s|真值跟踪最大误差 m|原因|',
        '|---|---|---|---:|---:|---:|---:|---|']
    for row in sorted(rows, key=lambda r: (scenario.LEVELS.index(r['level']), METHODS.index(r['variant']))):
        number = lambda key: '—' if row.get(key) is None else f"{row[key]:.3f}"
        lines.append(f"|{row['level']}|{NAMES[METHODS.index(row['variant'])]}|{row['verdict']}|"
            f"{row.get('actual_gate_count', '—')}|{number('nominal_time_s')}|{number('flight_time_s')}|"
            f"{number('truth_tracking_max_m')}|{str(row.get('failure') or '').replace('|', '/')}|")
    lines += ['', '每条件原初相一次。名义碰撞/超限的49条现有参考均进入执行清单，原未找到的一条登记为未找到。',
        '记录ULog、原生窗框/地面接触、窗口真实位姿、MPC输入/估计状态与参考哈希，原始日志无损压缩并保存原文SHA。',
        '启动和起飞准备时间单独保留。原Wang已生成的局部执行前缀按原时钟拼接为冻结参考，本轮统一评估轨迹的飞控闭环执行。',
        '协议、源码和参数见PROTOCOL.md及protocol.json。']
    (out/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(out/'analysis.json', dict(state=state, completed=len(rows), total=50,
        by_method={m: dict(no_collision_complete=sum(r['variant'] == m and r['passed'] for r in rows),
            collision=sum(r['variant'] == m and r.get('collision_detected') is True for r in rows),
            unfinished=sum(r['variant'] == m and not r['passed'] and r.get('collision_detected') is not True and r['verdict'] != 'planning_not_found' for r in rows),
            planning_not_found=sum(r['variant'] == m and r['verdict'] == 'planning_not_found' for r in rows)) for m in METHODS}))


def execute(record, folder):
    log = folder.parent/(folder.name+'.log')
    with log.open('w') as stream:
        process = subprocess.Popen([sys.executable, '-u', '-m', MODULE+'.flight',
            '--reference-dir', record['reference_directory'], '--outdir', str(folder)],
            cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        try:
            return process.wait(timeout=900)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=10)
            raise RuntimeError('flight_worker_wall_timeout')


def row_for(record, folder, code):
    audited = read(folder/'comparison_audit.json') if (folder/'comparison_audit.json').is_file() else dict(
        passed=False, verdict='startup_failure' if not (folder/'result.json').is_file() else 'evidence_failure',
        flight_time_s=None, collision_detected=None, failure='worker_exit_'+str(code))
    return dict(audited, level=record['level'], variant=record['variant'], directory=str(folder),
        nominal_time_s=record['nominal_time_s'], source_result_sha256=record['source_result_sha256'],
        reference_sha256=record.get('reference_sha256'), worker_exit_code=code)


def validate(out):
    protocol = read(out/'protocol.json')
    assert sources() == protocol['source_sha256']
    catalogue_rows, rows = read(out/'catalogue.json'), read(out/'summary.json')
    assert len(rows) == 50
    checks = []
    for record in catalogue_rows:
        assert digest(Path(record['source_result'])) == record['source_result_sha256']
        row = next(r for r in rows if r['level'] == record['level'] and r['variant'] == record['variant'])
        if not record['found']:
            assert row['verdict'] == 'planning_not_found'
            continue
        assert digest(Path(record['reference_directory'])/'reference.npz') == record['reference_sha256']
        folder = Path(row['directory'])
        with archive.restored(folder):
            assert digest(folder/'reference_trajectory.npz') == record['reference_sha256']
            again = audit(folder)
        for key in ('passed', 'verdict', 'collision_detected', 'actual_gate_count', 'exactly_once', 'flight_time_s'):
            assert again[key] == row[key]
        checks.append(dict(level=row['level'], variant=row['variant'], verdict=row['verdict'],
            raw_logs_hashes_verified=True, actual_motion_and_contacts_recomputed=True))
    assert len(checks) == 49 and sources() == protocol['source_sha256']
    save(out/'validation.json', dict(passed=True, planned_records=50, flight_records=49, checks=checks,
        raw_references_and_scenes_unchanged=True, all_native_archives_decompressed_and_reverified=True))


def run(out):
    with open('/tmp/convex_exp2_gazebo_matrix.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        records = catalogue(out)
        (out/'flights').mkdir()
        frozen = sources()
        image = subprocess.check_output(['docker', 'image', 'inspect', 'px4io/px4-sitl-gazebo:latest', '--format', '{{.Id}}'], text=True).strip()
        save(out/'protocol.json', dict(protocol=(HERE/'PROTOCOL.md').read_text(), source_sha256=frozen,
            image_id=image, cases=[scenario.manifest(scenario.Case(level=l)) for l in scenario.LEVELS],
            methods=METHODS, total=50, executable=49, max_flight_s=180, gazebo_run=True,
            tracking_reference='unchanged polynomial coefficients and absolute times',
            finish_position_tolerance_m=.55, finish_truth_speed_tolerance_mps=1.))
        for relative, value in frozen.items():
            target = out/'source_snapshot'/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            original = NOMINAL/'source_snapshot'/relative
            if original.is_file() and digest(original) == value:
                os.link(original, target)
            else:
                shutil.copyfile(ROOT/relative, target)
        rows = []

        def status(state, record=None, directory=None):
            save(out/'status.json', dict(state=state, pid=os.getpid(), completed=len(rows), total=50,
                current=record, directory=None if directory is None else str(directory),
                disk_available_GiB=shutil.disk_usage(ROOT).free/2**30))

        save(out/'summary.json', rows)
        report(out, rows, 'running')
        try:
            for index, level in enumerate(scenario.LEVELS):
                for method in METHODS if index%2 == 0 else tuple(reversed(METHODS)):
                    assert sources() == frozen
                    record = next(r for r in records if r['level'] == level and r['variant'] == method)
                    if not record['found']:
                        row = dict(level=level, variant=method, passed=False, verdict='planning_not_found',
                            flight_time_s=None, collision_detected=None, nominal_time_s=None)
                    else:
                        if shutil.disk_usage(ROOT).free < 3*2**30:
                            raise RuntimeError('insufficient_disk_for_next_flight')
                        directory = out/'flights'/f'{level}_{method}'
                        for attempt in (1, 2):
                            folder = directory/f'attempt_{attempt}'
                            directory.mkdir(exist_ok=True)
                            status('flying', dict(level=level, variant=method, attempt=attempt), folder)
                            print('FLY', level, method, attempt, flush=True)
                            code = execute(record, folder)
                            row = row_for(record, folder, code)
                            folder.mkdir(parents=True, exist_ok=True)
                            activation = read(folder/'reference_activation.json') if (folder/'reference_activation.json').is_file() else dict(activated=False)
                            archive.pack(folder)
                            if row['verdict'] != 'startup_failure' or activation['activated'] or attempt == 2:
                                break
                        if code or row['verdict'] in ('evidence_failure', 'startup_failure'):
                            save(directory/'infrastructure_failure.json', row)
                            raise RuntimeError('execution_infrastructure_or_evidence_error '+level+' '+method)
                    rows.append(row)
                    save(out/'summary.json', rows)
                    report(out, rows, 'running')
                    status('running')
                    print('RESULT', level, method, row['verdict'], row['flight_time_s'], flush=True)
            status('validating')
            validate(out)
            report(out, rows, 'complete')
            status('complete')
            print('COMPLETE', len(rows), flush=True)
        except BaseException as exc:
            save(out/'error.json', dict(type=type(exc).__name__, message=str(exc), traceback=traceback.format_exc()))
            status('stopped')
            report(out, rows, 'stopped')
            raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('catalogue', 'run', 'validate'))
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    if args.action == 'catalogue':
        catalogue(args.outdir.resolve())
    elif args.action == 'validate':
        validate(args.outdir.resolve())
    else:
        run(args.outdir.resolve())


if __name__ == '__main__':
    main()
