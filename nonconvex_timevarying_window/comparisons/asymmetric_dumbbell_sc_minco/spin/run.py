"""Offline spin-rate sweep with unchanged component-initialized optimization."""
import argparse
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from ..scenario import VERTICES, room
from ..run import save_json
from .scenario import build_case, local_coordinates
from .planner import Planner


def audit(plan, window, cfg):
    total = float(plan.durations.sum())
    times = np.linspace(0., total, int(np.ceil(total/.001))+1)+plan.made_at
    states = [plan.sample_many(times, d) for d in range(5)]
    _, rates, motors = flat_actuators.map(len(times))(*[s.T for s in states[1:]])
    rates, motors = np.asarray(rates), np.asarray(motors)
    q = local_coordinates(window, states[0], times)
    rho = cfg.radius+cfg.frame_thickness+cfg.passage_margin
    clearance = np.hypot(window.aperture.boundary_distance(q[:, :2]), q[:, 2])-rho
    low, high = KF*150**2, KF*1000**2
    return dict(role='independent_1ms_sampling_not_continuous_dynamics_certificate',
                sample_count=len(times), dt_max_s=float(np.diff(times).max()),
                minimum_clearance_after_all_margins_m=float(clearance.min()),
                minimum_rotor_thrust_N=float(motors.min()), maximum_rotor_thrust_N=float(motors.max()),
                maximum_rotor_upper_violation_fraction=max(0., float(motors.max()/high-1)),
                maximum_rotor_lower_violation_fraction=max(0., float((low-motors.min())/high)),
                maximum_abs_body_rates_rad_s=np.max(abs(rates), axis=1).tolist(),
                maximum_body_rate_violation_fraction=max(0., float(np.max(
                    np.max(abs(rates), axis=1)/np.array([3.8, 3.8, 3.49])-1))),
                workspace_lower_violation_m=np.maximum(np.asarray(cfg.workspace_lower)-states[0].min(axis=0), 0.).tolist(),
                workspace_upper_violation_m=np.maximum(states[0].max(axis=0)-np.asarray(cfg.workspace_upper), 0.).tolist(),
                nominal_path_length_m=float(np.linalg.norm(np.diff(states[0], axis=0), axis=1).sum()))


def report(outdir, results):
    lines = ['# 平移叠加自旋：单非凸窗口', '',
             '平移固定3 Hz / 2 m，初相零；只改变绕原窗口参考原点的法向自旋频率。',
             '当前算法不变：机体内缩分区初值、原PV-MINCO、原约束与局部交换。',
             '**均为离线名义规划，未进行Gazebo飞行，不以本报告宣称实际零碰撞。**', '',
             '| 自旋/圈每秒 | 转速/rpm | 当前验收 | 名义完整起终点/s | 过窗时刻/s | 求解/ms | 选点区域 |',
             '|---:|---:|---|---:|---:|---:|---|']
    for r in results:
        f = r.get('final')
        lines.append(f"| {r['spin_hz']:g} | {60*r['spin_hz']:g} | {'通过' if r['accepted'] else '未接纳'} | "
                     f"{format(f['nominal_total_time_s'], '.6f') if f else '—'} | "
                     f"{format(f['crossing_time_s'], '.6f') if f else '—'} | "
                     f"{r['stats']['optimization_ms']:.3f} | {f['room'] if f else '—'} |")
    lines += ['', '## 独立1ms审计（不用于接纳）', '',
              '| 自旋/圈每秒 | 最小采样净空/m | 单桨上界超限/% | 角速度超限/% |',
              '|---:|---:|---:|---:|']
    for r in results:
        if r['accepted']:
            a = r['posthoc_audit']
            lines.append(f"| {r['spin_hz']:g} | {a['minimum_clearance_after_all_margins_m']:.6f} | "
                         f"{100*a['maximum_rotor_upper_violation_fraction']:.4f} | "
                         f"{100*a['maximum_body_rate_violation_fraction']:.4f} |")
    lines += ['', '## 每个初值的结果', '']
    for r in results:
        lines += [f"### {r['spin_hz']:g} 圈/秒", '',
                  f"离线构建/准备 {r['offline_build_ms']:.3f} ms；JIT={r['jit']}。", '']
        for b in r['stats']['branches']:
            statuses = [v['status'] for a in b['stats'].get('attempts', []) for v in a['rounds']]
            lines.append(f"- 区域{b['component']['index']}：{'通过' if b['accepted'] else '未接纳'}，"
                         f"{b['stats'].get('optimization_ms', 0.):.3f} ms；{statuses}。")
        lines.append('')
    lines += ['## 解释范围', '',
              '- 内缩域在窗口自身坐标系计算，自旋不改变其连通分区；SC始终映射原物理开口。',
              '- 符号姿态、有限边距离、局部复查和独立审计均考虑自旋，不能将姿态冻结。',
              '- 只在法向多项式确定的近窗区间做Bernstein/正弦/余弦乘积包络，仍非Arb证书。',
              '- 动力学只在原节点强制；采样超限如实列出，不称完整连续动力学可行。',
              '- 未接纳不等于碰撞或证明场景不可行；仍可能遗漏时间/速度初值分支。',
              '- 本轮每档一次，不估计成功率；名义时间不是闭环竞速时间。', '',
              '![初值与最终选点](spin_points.png)', '']
    (outdir/'REPORT.md').write_text('\n'.join(lines))


def draw(outdir, results, components):
    count = len(results)
    fig, axes = plt.subplots(1, count, figsize=(5*count, 4.5), squeeze=False)
    boundary = np.vstack([VERTICES, VERTICES[0]])
    for ax, r in zip(axes[0], results):
        ax.fill(*boundary.T, color='#eeeeee')
        ax.plot(*boundary.T, color='#303030', lw=1.4)
        for c in components:
            ax.fill(*np.asarray(c.region.exterior.coords).T, color='#b7d8bf')
            ax.scatter(*c.point, marker='*', color='#2563a6', s=90)
        if r['accepted']:
            ax.scatter(*r['final']['q'], s=65, color='#bf4b2e', zorder=5)
        ax.set(title=f"Spin {r['spin_hz']:g} rev/s\n"+
                     (f"Nominal {r['final']['nominal_total_time_s']:.3f} s" if r['accepted'] else 'No accepted plan'),
               xlabel='Gate-local horizontal (m)', ylabel='Gate-local vertical (m)')
        ax.set_aspect('equal')
        ax.set_xlim(-1.85, 3.45)
        ax.set_ylim(-1.85, 1.85)
        ax.grid(alpha=.2)
    fig.suptitle('3 Hz / 2 m translation + in-plane spin; blue = initial, orange = final; no flight')
    fig.tight_layout()
    fig.savefig(outdir/'spin_points.png', dpi=170)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--spin-hz', type=float, nargs='+', default=[0., .5, 1., 2.])
    parser.add_argument('--jit', action='store_true')
    args = parser.parse_args()
    if len(set(args.spin_hz)) != len(args.spin_hz):
        raise ValueError('Do not overwrite duplicate conditions')
    args.outdir.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[4]
    paths = list(Path(__file__).parent.glob('*.py'))+list(Path(__file__).parent.parent.glob('*.py'))
    paths += list((Path(__file__).parent.parent/'component_initialization').glob('*.py'))
    paths += [root/p for p in (
        'convex_timevarying_window/single_gate_stop_racing/planner.py',
        'convex_timevarying_window/single_gate_stop_racing/solver.py',
        'convex_timevarying_window/single_gate_stop_racing/geometry.py',
        'convex_timevarying_window/single_gate_stop_racing/local_bounds.py',
        'convex_timevarying_window/dynamic_passage_minco/safety.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/hard_planner.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/hard_dynamics.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/trajectory.py',
        'nonconvex_timevarying_window/sc_dynatogt/sc_mapping.py')]
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    cfg = Config(compile_objective=args.jit, max_solve_ms=5000.)
    began = time.perf_counter()
    mapping = SCDiskMap.fit(VERTICES)
    fit_ms = 1000*(time.perf_counter()-began)
    mapping.save(args.outdir/'sc_map.npz')
    save_json(args.outdir/'manifest.json', dict(
        created_at=datetime.now(ZoneInfo('America/Indiana/Winamac')).isoformat(),
        config=asdict(cfg), translation_frequency_hz=3., translation_amplitude_m=2.,
        spin_hz=args.spin_hz, spin_phase_rad=0., rotation_center='original gate reference origin',
        source_sha256=hashes, offline_sc_fit_ms=fit_ms, flight_executed=False,
        protocol='Same component initialization, objective, constraints and solver; motion adapter only.'))
    print(json.dumps(dict(stage='SC_fit', offline_ms=fit_ms)), flush=True)
    results = []
    for spin_hz in args.spin_hz:
        folder = args.outdir/f'spin_{spin_hz:g}'
        folder.mkdir()
        track = build_case(spin_hz)
        began = time.perf_counter()
        planner = Planner(track, cfg, mapping)
        build_ms = 1000*(time.perf_counter()-began)
        print(json.dumps(dict(stage='built', spin_hz=spin_hz, offline_build_ms=build_ms)), flush=True)
        result = dict(spin_hz=spin_hz, jit=args.jit, accepted=False,
                      offline_build_ms=build_ms, flight_executed=False)
        try:
            plan = planner.solve(np.r_[track.start, np.zeros(9)])
            at = float(plan.crossing_times[0])
            q = mapping.map_unconstrained(plan.x[2:4])
            c, R, dc, dR = track.windows[0].state_at(at)
            frame_velocity = dc+dR[:, :2]@q
            drone_velocity = plan.sample(at, 1)
            result.update(accepted=True, final=dict(q=q.tolist(), room=room(q),
                          nominal_total_time_s=float(plan.durations.sum()), crossing_time_s=at,
                          spin_angle_at_crossing_rad=float(np.remainder(2*np.pi*spin_hz*at, 2*np.pi)),
                          world_point=plan.sample(at).tolist(), velocity_mps=drone_velocity.tolist(),
                          local_frame_point_velocity_mps=frame_velocity.tolist(),
                          relative_velocity_gate_coordinates_mps=(R.T@(drone_velocity-frame_velocity)).tolist()))
            np.savez(folder/'plan.npz', coefficients=plan.coefficients, durations=plan.durations, x=plan.x)
            result['posthoc_audit'] = audit(plan, track.windows[0], cfg)
        except RuntimeError as exc:
            if str(exc) != 'no_accepted_component':
                raise
            result['failure'] = str(exc)
        result['stats'] = planner.solvers[0][0].stats()
        for row in result['stats']['branches']:
            subdir = folder/f"component_{row['component']['index']}"
            subdir.mkdir()
            if row['accepted']:
                data = {k: np.asarray(v) for k, v in row['plan'].items()}
                np.savez(subdir/'plan.npz', **data)
                candidate = Plan(0., 0, data['durations'], data['coefficients'], data['x'], 0., 0., 'audit', 1)
                row['posthoc_audit'] = audit(candidate, track.windows[0], cfg)
            row['points_outside_physical_polygon'] = sum(
                not track.windows[0].aperture.contains(v['q'], tolerance=1e-7) for v in row['evaluations'])
            save_json(subdir/'result.json', row)
        save_json(folder/'result.json', result)
        results.append(result)
        save_json(args.outdir/'summary.json', results)
        report(args.outdir, results)
        print(json.dumps(dict(spin_hz=spin_hz, accepted=result['accepted'], final=result.get('final'),
                              solve_ms=result['stats']['optimization_ms'])), flush=True)
    draw(args.outdir, results, planner.components)
    changed = [p for p, h in hashes.items() if hashlib.sha256((root/p).read_bytes()).hexdigest() != h]
    save_json(args.outdir/'source_verification.json', dict(unchanged=not changed, changed=changed))
    if changed:
        raise RuntimeError('Sources changed during experiment: '+str(changed))
    print('FINISHED '+str(args.outdir), flush=True)


if __name__ == '__main__':
    main()
