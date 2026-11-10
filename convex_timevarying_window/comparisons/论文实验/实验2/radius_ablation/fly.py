"""Execute original and smaller-radius frozen references under one finish rule."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

from ..basic_group.whole_course_difficulty import scenario
from ..basic_group.trajectory_only.collision import evaluate, CheckConfig, BODY_RADIUS
from ..gazebo_matrix import archive, run as original
from .execution import audit

HERE = Path(__file__).resolve().parent
ROOT = scenario.ROOT
MODULE = 'convex_timevarying_window.comparisons.论文实验.实验2.radius_ablation.execution'
BASE = HERE.parent/'results/independent_full_matrix_v2_20261107/plans/a25_f6_ours/result.json'


def read(path):
    return json.loads(path.read_text())


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def frozen_sources():
    result = original.sources()
    for path in HERE.rglob('*'):
        if path.is_file() and path.suffix in ('.py', '.md') and '__pycache__' not in path.parts:
            result[str(path.relative_to(ROOT))] = original.digest(path)
    return result


def report(out, plans, flights):
    lines = ['# 0.25／6组规划半径对比', '',
        '时间为完整七窗后返航的多项式总时长。原球使用原冻结结果，小球从原四解析初值独立求解。', '',
        '|方法|规划球半径 m|规划时间 s|参考路程 m|原尺寸球窗框结果|', '|---|---:|---:|---:|---|']
    for row in plans:
        lines.append(f"|{row['variant']}|{row['radius_m']:.6f}|{row['duration_s']:.6f}|{row['path_length_m']:.6f}|{row['original_sphere_frame_outcome']}|")
    (out/'PLANNING.md').write_text('\n'.join(lines)+'\n')
    lines = ['# 0.25／6组Gazebo完整参考执行', '',
        '原球与小球使用相同x500/PX4、MPC和原生窗框接触判定。各执行一次，地面接触单列。',
        '用时从竞赛时钟0开始，到完整参考T已执行且状态时间达到T。取消旧0.55m／1m/s提前结束规则。',
        '该指标为完整参考执行用时，末端真实位置误差和速度单列。精确零误差停稳的物理到达时间未定义。', '',
        '|方法|窗框结果|真实七窗计数|完整参考执行用时 s|末端误差 m|末端速度 m/s|', '|---|---|---:|---:|---:|---:|']
    for row in flights:
        duration = '碰撞' if row['frame_collision'] else f"{row['full_reference_execution_time_s']:.3f}" if row['passed'] else '未完成'
        result = '碰撞' if row['frame_collision'] else '无窗框接触' if row['passed'] else '未完成'
        lines.append(f"|{row['variant']}|{result}|{row['actual_gate_count']}|{duration}|{row['truth_goal_error_m']:.6f}|{row['truth_goal_speed_mps']:.6f}|")
    (out/'FLIGHT.md').write_text('\n'.join(lines)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    assert read(out/'planning_validation.json')['passed']
    new = out/'planning/result.json'
    if not read(new)['found']:
        raise RuntimeError('New radius did not produce a selected trajectory')
    with open('/tmp/convex_exp2_gazebo_matrix.lock', 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        (out/'flights').mkdir(exist_ok=False)
        frozen = frozen_sources()
        for relative, expected in frozen.items():
            target = out/'execution_source_snapshot'/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            prior = out/'source_snapshot'/relative
            if prior.is_file() and original.digest(prior) == expected:
                os.link(prior, target)
            else:
                shutil.copyfile(ROOT/relative, target)
        sources = dict(original_radius=BASE, small_radius=new)
        source_hashes = {name: original.digest(path) for name, path in sources.items()}
        save(out/'execution_protocol.json', dict(source_sha256=frozen, source_results_sha256=source_hashes,
            finish_definition='execute_full_reference_T_and_report_terminal_truth', old_terminal_threshold_used=False,
            image_id=subprocess.check_output(['docker', 'image', 'inspect', 'px4io/px4-sitl-gazebo:latest', '--format', '{{.Id}}'], text=True).strip(),
            scene=scenario.manifest(scenario.Case(level='a25_f6')), reference_retiming=False, trials_per_variant=1))
        track, _ = scenario.build_case(scenario.Case(level='a25_f6'))
        plans, flights = [], []
        for variant, source in sources.items():
            raw = read(source)
            full = evaluate(raw['coefficients'], raw['durations'], track, scenario.ASSETS/'meshes')
            small = evaluate(raw['coefficients'], raw['durations'], track, scenario.ASSETS/'meshes',
                CheckConfig(radius=raw['config']['radius']))
            save(out/(variant+'_geometry_original_sphere.json'), full)
            save(out/(variant+'_geometry_planning_sphere.json'), small)
            collision = any(c['outcome'] == 'collision' for c in full['checks'])
            clear = all(c['outcome'] == 'clear' for c in full['checks'])
            plans.append(dict(variant=variant, radius_m=raw['config']['radius'],
                duration_s=raw['nominal_time_s'], path_length_m=full['trajectory_distance_m'],
                original_sphere_radius_m=BODY_RADIUS,
                original_sphere_frame_outcome='碰撞' if collision else '无碰撞' if clear else '证据不足'))
        save(out/'planning_comparison.json', plans)
        save(out/'flight_summary.json', flights)
        report(out, plans, flights)
        for variant, source in sources.items():
            assert frozen_sources() == frozen
            assert original.digest(source) == source_hashes[variant]
            if shutil.disk_usage(ROOT).free < 3*2**30:
                raise RuntimeError('insufficient_disk_for_native_recording')
            folder = out/'flights'/variant
            save(out/'flight_status.json', dict(state='flying', variant=variant, coordinator_pid=os.getpid()))
            with (out/(variant+'_flight.log')).open('w') as stream:
                process = subprocess.Popen([sys.executable, '-u', '-m', MODULE, '--source', str(source),
                    '--variant', variant, '--outdir', str(folder)], cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
                save(out/'flight_status.json', dict(state='flying', variant=variant,
                    coordinator_pid=os.getpid(), worker_pid=process.pid))
                code = process.wait(timeout=900)
            if code or not (folder/'comparison_audit.json').exists():
                save(out/'flight_status.json', dict(state='stopped', variant=variant, worker_exit_code=code))
                raise RuntimeError('physical_worker_or_evidence_failure '+variant)
            row = dict(read(folder/'comparison_audit.json'), variant=variant, directory=str(folder))
            archive.pack(folder)
            with archive.restored(folder):
                again = audit(folder)
            for key in ('passed', 'verdict', 'frame_collision', 'actual_gate_count', 'full_reference_execution_time_s'):
                assert again[key] == row[key]
            flights.append(row)
            save(out/'flight_summary.json', flights)
            report(out, plans, flights)
            print('FLIGHT_RESULT', variant, row['verdict'], row['full_reference_execution_time_s'], flush=True)
        assert frozen_sources() == frozen
        assert all(original.digest(path) == source_hashes[name] for name, path in sources.items())
        physical = [read(Path(r['directory'])/'result.json') for r in flights]
        assert physical[0]['parameters'] == physical[1]['parameters']
        assert physical[0]['mpc_config'] == physical[1]['mpc_config']
        assert original.digest(Path(flights[0]['directory'])/'world.sdf') == original.digest(Path(flights[1]['directory'])/'world.sdf')
        save(out/'execution_validation.json', dict(passed=True, independent_native_reaudits=2,
            source_hashes_unchanged=True, same_world_and_mpc_and_parameters=True,
            original_terminal_threshold_removed=True, complete_reference_execution_measured=True))
        save(out/'flight_status.json', dict(state='complete', coordinator_pid=os.getpid(), completed=2))


if __name__ == '__main__':
    main()
