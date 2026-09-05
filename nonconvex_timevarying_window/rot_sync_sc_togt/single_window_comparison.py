"""Paired single-window experiment: fixed waypoint racing versus RotSync.

The baseline fixes a SAFE gate-local point before optimizing arrival times.
Its world position follows the known gate rotation. It has no Sync segment.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime
from pathlib import Path
import platform
import time
from types import SimpleNamespace

import numpy as np
from scipy.optimize import brentq
from shapely.geometry import Point, Polygon
from shapely.ops import polylabel

from nonconvex_timevarying_window.sc_dynatogt.dynamics import constraint_extrema
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import durations_from_k, k_from_durations
from .collision import body_rotations, cuboid_window_collision
from .experiments import _sampled_dynamic_validation, _write_json, _write_summary
from .optimizer import RotSyncObjective, RotSyncOptimizationConfig
from .scenarios import build_smoke_scenario, preprocess_shape_catalog


class FixedWaypointObjective(RotSyncObjective):
    """Two minimum-snap pieces through one fixed local waypoint; optimize times."""

    def __init__(self, scenario, config):
        if len(scenario.windows) != 1:
            raise ValueError("this experiment requires exactly one window")
        super().__init__(scenario, config)
        point = polylabel(Polygon(scenario.windows[0].safe_polygon), tolerance=1e-5)
        self.fixed_q = np.asarray(point.coords[0])
        self.dimension = 2

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (2,) or not np.all(np.isfinite(values)):
            raise ValueError("fixed waypoint variables must be two finite time parameters")
        return values

    def initial_guess(self):
        window = self.scenario.windows[0]
        anchors = np.asarray([self.scenario.start_state.position, window.center,
                              self.scenario.goal_state.position])
        durations = np.maximum(np.linalg.norm(np.diff(anchors, axis=0), axis=1)
                               / self.config.initial_speed,
                               self.config.minimum_initial_free_duration)
        return k_from_durations(durations)

    def forward(self, x):
        durations = durations_from_k(self.split(x))
        crossing = float(durations[0])
        point = self.scenario.windows[0].world_point(self.fixed_q, crossing)
        trajectory = MincoSnap(self.scenario.start_state, self.scenario.goal_state,
                               point[None, :], durations)
        return SimpleNamespace(trajectory=trajectory, crossing_times=np.array([crossing]),
                               local_points=self.fixed_q[None, :], durations=durations)


def solve_objective(objective):
    started = time.perf_counter()
    result = _minimize_togt_lbfgs(objective.value_and_gradient, objective.initial_guess(),
                                objective.config.lbfgs_config())
    elapsed = time.perf_counter() - started
    forward = objective.forward(result.x)
    return result, forward, elapsed


def audit(scenario, forward, config, *, dt=0.001):
    """Common independent grid, exact prescribed waypoint check, all-time body audit.

    This is numerical sampling, not a continuous-time certificate. Both sides of
    segment junctions are included because control-related derivatives can jump.
    """
    trajectory = forward.trajectory
    knots = np.cumsum(trajectory.durations)[:-1]
    grid = np.unique(np.r_[np.linspace(0, trajectory.total_time,
                                      int(np.ceil(trajectory.total_time / dt)) + 1),
                           knots, knots - 1e-8, knots + 1e-8, forward.crossing_times])
    grid = grid[(grid >= 0) & (grid <= trajectory.total_time)]
    positions = trajectory.evaluate(grid)
    rotations = body_rotations(trajectory, grid, parameters=config.quadrotor)
    window = scenario.windows[0]
    collisions, clearances = [], []
    for instant, position, rotation in zip(grid, positions, rotations):
        hit, clearance = cuboid_window_collision(window, instant, position, rotation, scenario.body)
        collisions.append(hit)
        clearances.append(clearance)
    collisions = np.asarray(collisions)
    clearances = np.asarray(clearances)
    # Reuse the exact same limit definitions while sampling at identical times.
    class AuditGrid:
        def sample(self, **kwargs):
            return trajectory.sample(times=grid)

        def evaluate(self, *args, **kwargs):
            return trajectory.evaluate(*args, **kwargs)

    extrema = constraint_extrema(AuditGrid(), parameters=config.quadrotor)
    dynamic = _sampled_dynamic_validation(SimpleNamespace(extrema=extrema), config)
    crossing = float(forward.crossing_times[0])
    expected = window.world_point(forward.local_points[0], crossing)
    crossing_error = float(np.linalg.norm(trajectory.evaluate(crossing) - expected))
    safe = Polygon(window.safe_polygon).covers(Point(forward.local_points[0]))
    endpoint_error = max(float(np.max(np.abs(np.stack([trajectory.evaluate(t, d)
                         for d in range(4)]) - state.matrix)))
                         for t, state in [(0, scenario.start_state),
                                          (trajectory.total_time, scenario.goal_state)])
    signed = (positions - window.center) @ window.normal
    # Detect additional plane crossings (including a sample exactly at zero).
    nonzero = np.flatnonzero(np.abs(signed) > 1e-9)
    brackets = [(grid[a], grid[b]) for a, b in zip(nonzero[:-1], nonzero[1:])
                if signed[a] * signed[b] < 0]
    crossings = [brentq(lambda t: float((trajectory.evaluate(t) - window.center) @ window.normal),
                        a, b) for a, b in brackets]
    sequence_ok = len(crossings) == 1 and signed[0] < 0 < signed[-1]
    finite = clearances[np.isfinite(clearances)]
    passed = bool(safe and crossing_error < 1e-8 and endpoint_error < 1e-8 and sequence_ok
                  and not collisions.any() and dynamic['sampled_dynamic_limits_satisfied'])
    values = dict(trajectory_validation_pass=passed, collision_free=not bool(collisions.any()),
                  colliding_samples=int(collisions.sum()), audit_samples=len(grid), audit_dt=dt,
                  minimum_frame_clearance=float(finite.min()) if finite.size else None,
                  first_collision_time=float(grid[collisions][0]) if collisions.any() else None,
                  endpoint_error=endpoint_error, crossing_error=crossing_error,
                  plane_crossings=crossings, ordered_once=sequence_ok, q_in_safe_region=bool(safe),
                  **dynamic, extrema=extrema)
    return values, dict(time=grid, position=positions, clearance=clearances,
                        speed=np.linalg.norm(trajectory.evaluate(grid, 1), axis=1),
                        collision=collisions)


def plot_pair(scenario, records, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    window = scenario.windows[0]
    boundary = np.vstack([window.physical_polygon, window.physical_polygon[0]])
    axes[0].plot(*boundary.T, color='black', lw=2, label='Physical aperture')
    for method, record in records.items():
        forward, data = record['forward'], record['data']
        local = np.stack([window.rotated_basis(t).T @ (p - window.center)
                          for t, p in zip(data['time'], data['position'])])
        near = np.abs((data['position'] - window.center) @ window.normal) <= window.clearance_distance
        axes[0].plot(*local[near].T, label=method, lw=2)
        axes[0].scatter(*forward.local_points[0], s=45)
        finite = np.isfinite(data['clearance'])
        axes[1].plot(data['time'][finite] - forward.crossing_times[0],
                     data['clearance'][finite] * 1000, label=method)
        axes[2].plot(data['time'], data['speed'], label=method)
    axes[0].set(xlabel='Gate local u (m)', ylabel='Gate local v (m)', aspect='equal',
                title='Centre projection during crossing')
    axes[1].set(xlabel='Time relative to centre crossing (s)', ylabel='Body-to-frame clearance (mm)',
                title='Whole cuboid to physical boundary')
    axes[2].axhline(7, color='red', ls='--', label='Speed limit')
    axes[2].set(xlabel='Flight time (s)', ylabel='Speed (m/s)', title='Speed profile')
    for ax in axes:
        ax.legend(fontsize=8)
        ax.grid(alpha=.2)
    fig.suptitle(f'{scenario.name}: fixed waypoint (no Sync) vs RotSync')
    fig.savefig(output, dpi=170)
    plt.close(fig)


def write_report(root, rows):
    lines = ['# 单窗口固定点竞速与同步穿越对比', '',
             '固定点基线在优化前选定安全区最大内接圆中心，仅优化两段 MINCO 时间；',
             '穿越时刻经过随窗口旋转的该点，不包含 Sync。完整方法联合优化点与自由段、同步段时间。',
             '这是一组两种完整方法的比较，点优化和同步结构同时不同，不能单独归因于 Sync。', '',
             '两组使用相同物理窗口、机体、初终状态、代价权重和 L-BFGS 设置；',
             '采用独立 1 ms 网格及分段接口双侧检查，结果属于名义模型的数值验证。',
             '每个确定性场景运行一次，汇总是这些测试实例的结果，不是随机总体成功率。', '',
             '|场景|方法|飞行/s|求解/s|最大速度/(m/s)|最小净距/mm|碰撞|动力学合格|轨迹合格|优化收敛|',
             '|---|---|---:|---:|---:|---:|---|---|---|---|']
    for row in rows:
        lines.append(f"|{row['scenario']}|{row['method']}|{row['total_time']:.4f}|"
                     f"{row['solve_time']:.2f}|{row['max_velocity']:.5f}|"
                     f"{row['minimum_frame_clearance'] * 1000:.2f}|"
                     f"{not row['collision_free']}|{row['dynamic_pass']}|"
                     f"{row['trajectory_pass']}|{row['optimizer_success']}|")
    lines += ['', '各场景 comparison.png 显示窗口坐标中的轨迹、穿越附近净距及速度曲线。',
              'config.json 保存完整配置；result.json、trajectory.csv 和 summary.csv 保存数值。']
    lines += ['', '## 当前测试实例汇总', '']
    for method in ['Fixed-WP', 'RotSync']:
        selected = [r for r in rows if r['method'] == method]
        valid = [r for r in selected if r['trajectory_pass']]
        if selected:
            duration = float(np.mean([r['total_time'] for r in valid])) if valid else float('nan')
            runtime = float(np.median([r['solve_time'] for r in selected]))
            lines.append(f'- {method}：{len(valid)}/{len(selected)} 轨迹合格；'
                         f'合格轨迹平均飞行时间 {duration:.4f} s，求解耗时中位数 {runtime:.3f} s。')
    pairs = []
    for case in dict.fromkeys(r['scenario'] for r in rows):
        by_method = {r['method']: r for r in rows if r['scenario'] == case and r['trajectory_pass']}
        if len(by_method) == 2:
            pairs.append(by_method['RotSync']['total_time'] / by_method['Fixed-WP']['total_time'] - 1)
    if pairs:
        lines.append(f'- {len(pairs)} 对共同合格场景中，RotSync 相对 Fixed-WP 的飞行时间变化'
                     f'均值为 {np.mean(pairs)*100:+.2f}%，范围 '
                     f'{min(pairs)*100:+.2f}% 至 {max(pairs)*100:+.2f}%。')
    lines += ['', '![对比汇总](comparison_summary.png)']
    (root / 'REPORT.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def plot_summary(root, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    cases = list(dict.fromkeys(row['scenario'] for row in rows))
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), constrained_layout=True, sharex=True)
    for j, method in enumerate(['Fixed-WP', 'RotSync']):
        selected = [next(r for r in rows if r['scenario'] == c and r['method'] == method)
                    for c in cases]
        x = np.arange(len(cases)) + (j - .5) * .36
        for ax, key in zip(axes, ['total_time', 'solve_time']):
            bars = ax.bar(x, [r[key] for r in selected], width=.36, label=method)
            for bar, row in zip(bars, selected):
                if not row['trajectory_pass']:
                    bar.set_hatch('///')
            ax.grid(axis='y', alpha=.2)
            ax.legend()
    axes[0].set(ylabel='Flight duration (s)', title='Single-window paired comparison (hatched = invalid trajectory)')
    axes[1].set(ylabel='Optimizer wall time (s)', yscale='log')
    axes[1].set_xticks(np.arange(len(cases)), cases, rotation=25, ha='right')
    fig.savefig(root / 'comparison_summary.png', dpi=170)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outdir', type=Path)
    parser.add_argument('--shapes', nargs='+', default=['L', 'U', 'star'], choices=['L', 'U', 'star'])
    parser.add_argument('--omegas', nargs='+', type=float, default=[0.0, 0.75, 1.5])
    parser.add_argument('--max-iterations', type=int, default=120)
    parser.add_argument('--vertex-count', type=int, default=256)
    parser.add_argument('--quadrature-order', type=int, default=64)
    args = parser.parse_args(argv)
    root = args.outdir or Path(__file__).parent / 'results' / ('single_window_' + datetime.now().strftime('%Y%m%d_%H%M%S'))
    root.mkdir(parents=True, exist_ok=False)
    config = RotSyncOptimizationConfig(max_iterations=args.max_iterations,
                                      samples_per_segment=11, dynamics_weight=0.1)
    started = time.perf_counter()
    catalog = preprocess_shape_catalog(shape_names=args.shapes, vertex_count=args.vertex_count,
                                      quadrature_order=args.quadrature_order)
    _write_json(root / 'protocol.json', dict(config=config, shapes=args.shapes, omegas=args.omegas,
                theta0=0.3, preprocessing_seconds=time.perf_counter()-started,
                vertex_count=args.vertex_count, quadrature_order=args.quadrature_order,
                python=platform.python_version(), platform=platform.platform(),
                fixed_point='safe polygon polylabel, tolerance 1e-5 m', audit_dt=0.001,
                baseline='fixed gate-local waypoint, moving world target at optimized arrival, no Sync'))
    rows = []
    for shape in args.shapes:
        base = build_smoke_scenario({'L': catalog[shape]})
        # Oblique approach exposes the route-choice benefit without changing it between methods.
        base = replace(base, start_state=BoundaryState(np.array([-4.5, -0.8, 1.8])),
                       goal_state=BoundaryState(np.array([4.5, 0.8, 1.8])))
        for omega in args.omegas:
            scenario = replace(base, name=f'{shape}_omega{omega:g}',
                               windows=(replace(base.windows[0], omega=omega),))
            directory = root / scenario.name
            directory.mkdir()
            window = scenario.windows[0]
            _write_json(directory / 'config.json', dict(start=scenario.start_state, goal=scenario.goal_state,
                        body=scenario.body, center=window.center, normal=window.normal,
                        basis=window.plane_basis, theta0=window.theta0, omega=omega,
                        thickness=window.thickness, rho=window.rho, config=config,
                        physical_polygon=window.physical_polygon, safe_polygon=window.safe_polygon))
            records = {}
            for method, cls in [('Fixed-WP', FixedWaypointObjective), ('RotSync', RotSyncObjective)]:
                print(f'START {scenario.name} {method}', flush=True)
                objective = cls(scenario, config)
                result, forward, solve_time = solve_objective(objective)
                validation, data = audit(scenario, forward, config)
                row = dict(scenario=scenario.name, shape=shape, omega=omega, method=method,
                           total_time=float(forward.trajectory.total_time), solve_time=solve_time,
                           max_velocity=validation['extrema']['max_velocity'],
                           minimum_frame_clearance=validation['minimum_frame_clearance'],
                           collision_free=validation['collision_free'],
                           dynamic_pass=validation['sampled_dynamic_limits_satisfied'],
                           trajectory_pass=validation['trajectory_validation_pass'],
                           optimizer_success=bool(result.success), iterations=int(result.nit))
                rows.append(row)
                target = directory / method
                target.mkdir()
                _write_json(target / 'result.json', dict(**row, x=result.x, selected_q=forward.local_points,
                            crossing_times=forward.crossing_times, durations=forward.trajectory.durations,
                            message=str(result.message), objective=float(result.fun), validation=validation))
                np.savetxt(target / 'trajectory.csv', np.column_stack([data['time'], data['position'],
                           data['speed'], data['clearance'], data['collision'].astype(int)]), delimiter=',',
                           header='time,x,y,z,speed,frame_clearance,collision', comments='')
                records[method] = dict(forward=forward, data=data)
                _write_summary(root / 'summary.csv', rows)
                write_report(root, rows)
                print(f"DONE {row}", flush=True)
            plot_pair(scenario, records, directory / 'comparison.png')
    plot_summary(root, rows)
    print(f'OUTPUT {root.resolve()}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
