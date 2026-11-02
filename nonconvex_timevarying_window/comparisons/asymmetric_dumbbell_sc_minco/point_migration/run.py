"""Isolate gate point selection. No MINCO, physical-body feasibility, or flight."""
import argparse
from datetime import datetime
import hashlib
from pathlib import Path
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from ..planner import inverse_along_corridor, recenter_at_left
from ..run import save_json
from ..scenario import VERTICES, LEFT_INITIAL_POINT, build_case, room


TIMES = (1.0, 1.02, 1.1, 1.2)
OPTIONS = dict(maxiter=400, maxls=80, ftol=1e-14, gtol=1e-10)


def point_cost(q, track, at):
    center, rotation = track.windows[0].state_at(at)[:2]
    world = center + rotation[:, :2] @ np.asarray(q)
    delta = np.stack([world-track.start, world-track.goal])
    lengths = np.linalg.norm(delta, axis=1)
    gradient = rotation[:, :2].T @ np.sum(delta/lengths[:, None], axis=0)
    return float(lengths.sum()), gradient


def solve_point(mapping, track, at):
    began = time.perf_counter()
    initial_d = inverse_along_corridor(mapping, LEFT_INITIAL_POINT)
    preparation_ms = 1000*(time.perf_counter()-began)
    np.testing.assert_allclose(mapping.map_unconstrained(initial_d), LEFT_INITIAL_POINT, atol=1e-7)
    evaluations, accepted = [], [initial_d.copy()]

    def fun(d):
        q, jac = mapping.map_unconstrained_with_jacobian(d)
        value, gradient = point_cost(q, track, at)
        evaluations.append(dict(d=d.copy(), q=q.copy(), cost_m=value))
        return value, jac.T @ gradient

    began = time.perf_counter()
    result = minimize(fun, initial_d, jac=True, method='L-BFGS-B',
                      callback=lambda d: accepted.append(d.copy()), options=OPTIONS)
    solve_ms = 1000*(time.perf_counter()-began)
    polygon = Polygon(VERTICES)
    body_radius = .38594472
    initial_q = mapping.map_unconstrained(initial_d)
    final_q = mapping.map_unconstrained(result.x)
    # For the fixed original track, p(q) has x=c_x+q_x and z=3.2+q_y.
    # If the straight crossing is inside, triangle inequality proves a 12 m optimum.
    center = track.windows[0].state_at(at)[0]
    straight_q = np.array([-center[0], 0.])
    straight_interior = bool(polygon.contains(Point(straight_q)))
    # All frozen control times have their unconstrained point inside or to the left.
    if not straight_interior and straight_q[0] >= -1.6:
        raise ValueError('Analytic boundary reference only implemented for the left exterior control')
    reference_q = straight_q if straight_interior else np.array([-1.6, 0.])
    reference_cost = point_cost(reference_q, track, at)[0]
    trace = [dict(iteration=i, d=d, q=mapping.map_unconstrained(d),
                  cost_m=point_cost(mapping.map_unconstrained(d), track, at)[0])
             for i, d in enumerate(accepted)]
    # This samples the mapping of parameter-space segments, NOT physical straight lines.
    # It is an audit/display, not a continuous interval certificate.
    curves = []
    for a, b in zip(accepted[:-1], accepted[1:]):
        curves.append(np.array([mapping.map_unconstrained((1-s)*a+s*b)
                                for s in np.linspace(0., 1., 129)]))
    path = np.concatenate(curves) if curves else initial_q[None, :]
    all_q = np.array([e['q'] for e in evaluations])
    outside_evals = sum(not polygon.buffer(1e-7).covers(Point(q)) for q in all_q)
    outside_path = sum(not polygon.buffer(1e-7).covers(Point(q)) for q in path)
    minimum_distance = min(polygon.boundary.distance(Point(q)) for q in path)
    value, grad_q = point_cost(final_q, track, at)
    return dict(query_time_s=at, optimizer_success=bool(result.success),
                optimizer_message=str(result.message), iterations=int(result.nit),
                evaluations_count=int(result.nfev), solve_ms=solve_ms,
                offline_initial_inverse_ms=preparation_ms,
                physical_initial_q=initial_q, final_q=final_q, final_room=room(final_q),
                initial_cost_m=point_cost(initial_q, track, at)[0], final_cost_m=value,
                analytic_reference_q=reference_q, analytic_reference_cost_m=reference_cost,
                reference_is_attained_interior=straight_interior,
                cost_gap_m=value-reference_cost, gradient_q=grad_q,
                gradient_d=result.jac, physical_evaluations_outside=outside_evals,
                sampled_parameter_path_outside=outside_path,
                sampled_parameter_path_min_boundary_distance_m=minimum_distance,
                sampled_parameter_path_has_body_infeasible_points=minimum_distance<body_radius,
                initial_jacobian_condition=float(np.linalg.cond(mapping.jacobian_unconstrained(initial_d))),
                final_body_boundary_margin_m=polygon.boundary.distance(Point(final_q))-body_radius,
                evaluations=evaluations, accepted_trace=trace, sampled_parameter_path=path,
                window_center=center, window_velocity=track.windows[0].state_at(at)[2],
                flight_executed=False, full_planner_safety_verified=False)


def draw(outdir, results):
    main = [r for r in results if r['query_time_s'] == 1.2]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    outline = np.vstack([VERTICES, VERTICES[0]])
    eroded = Polygon(VERTICES).buffer(-.38594472)
    for ax, row in zip(axes[:2], main):
        ax.fill(outline[:, 0], outline[:, 1], color='#eeeeee')
        ax.plot(outline[:, 0], outline[:, 1], color='black', lw=1.5)
        for part in eroded.geoms:
            p = np.asarray(part.exterior.coords)
            ax.fill(p[:, 0], p[:, 1], color='#a9d6ab', alpha=.65)
        path = np.array(row['sampled_parameter_path'])
        ax.plot(path[:, 0], path[:, 1], color='#326aa6', lw=1.4,
                label='Mapped optimizer steps (not flight)')
        points = np.array([a['q'] for a in row['accepted_trace']])
        ax.scatter(points[:, 0], points[:, 1], s=17, c='#326aa6')
        ax.scatter(*row['physical_initial_q'], s=130, marker='*', c='#e48922', zorder=5, label='Only initial point')
        ax.scatter(*row['final_q'], s=65, marker='o', c='#b62737', zorder=5, label='Final point')
        ax.set(title=f"{row['normalization']} | {row['iterations']} iterations",
               xlabel='Gate-local horizontal (m)', ylabel='Gate-local vertical (m)',
               xlim=(-1.85, 3.45), ylim=(-1.85, 1.85), aspect='equal')
        ax.legend(fontsize=7, loc='lower right')
        ax.grid(alpha=.2)
        axes[2].plot([a['iteration'] for a in row['accepted_trace']],
                     [a['cost_m'] for a in row['accepted_trace']], '.-', label=row['normalization'])
    axes[2].axhline(12., color='black', ls='--', label='Analytic optimum: 12 m')
    axes[2].set(xlabel='Accepted optimizer iteration', ylabel='Geometric broken-line length (m)',
                title='Point-selection objective, not flight time')
    axes[2].legend(fontsize=8)
    axes[2].grid(alpha=.2)
    fig.suptitle('Same physical polygon, same left seed, one L-BFGS solve; fixed query t=1.2 s\n'
                 'Green: body-radius erosion only. Blue optimizer path need NOT be body-safe.', fontsize=11)
    fig.tight_layout()
    fig.savefig(outdir/'migration.png', dpi=170)
    plt.close(fig)


def report(outdir, results):
    lines = ['# 同一窗口的单左初值纯选点验证', '',
             '本轮只做几何选点；不是 MINCO 完整规划，不是飞行或连续安全证明。', '',
             '原 3 Hz / 2 m 运动、12 边物理窗口与起终点不变。固定查询时刻，最小化起点—选点—终点的总路程。',
             '所有初值均为局部 (-1,0)；每行只运行一次普通 L-BFGS-B，无分区枚举、额外试探或右侧奖励。', '',
             '| 时刻 s | SC 坐标归一化 | 最终局部点 m | 区域 | 迭代/评估 | 求解 ms | 路程 m | 参考差 m |',
             '|---:|---|---|---|---:|---:|---:|---:|']
    for r in results:
        q = r['final_q']
        lines.append(f"| {r['query_time_s']:g} | {r['normalization']} | ({q[0]:.7f},{q[1]:.7f}) | "
                     f"{r['final_room']} | {r['iterations']}/{r['evaluations_count']} | {r['solve_ms']:.3f} | "
                     f"{r['final_cost_m']:.9f} | {r['cost_gap_m']:.3g} |")
    lines += ['', '## 怎样从左到右', '',
              '在 1.2 s，窗口参考中心 x=-1.1755705 m；原起终点直线穿过世界 x=0，',
              '对应窗口局部 q=(1.1755705,0)，自然在右室，并非人为指定右室目标。',
              'SC 使用原物理域；一次常规局部优化可改变两个坐标变量并进入右室。',
              '原归一化与左归一化均保留。左归一化仅是同一映射的等价坐标变换。', '',
              '## 不能混淆', '',
              '- 原窗口内缩机体半径后为两块；蓝线只是优化过程，中间点可以不满足机体安全。',
              '- 所有评估点及绘图采样的物理域合法性见 summary.json；采样不称区间证书。',
              '- 1.1 s 对照的最优点在开域边界，只能渐近逼近；不能把求解器成功标志当精确达到。',
              '- 这不是原完整动态硬约束求解的成功，不证明 SC 比其他自由选点表示更快。',
              '- 既有完整 MINCO 单左初值局部求解失败；single_seed_poll 的成功用了额外非局部试探。',
              '- 12 m 是几何目标，不是竞速时间。固定查询时刻没有做可达性检查。', '',
              '![优化选点过程，不是飞行轨迹](migration.png)', '']
    (outdir/'REPORT.md').write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[4]
    sources = [Path(__file__), Path(__file__).parents[1]/'scenario.py',
               Path(__file__).parents[1]/'planner.py',
               root/'nonconvex_timevarying_window/sc_dynatogt/sc_mapping.py']
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    began = time.perf_counter()
    original = SCDiskMap.fit(VERTICES)
    fit_ms = 1000*(time.perf_counter()-began)
    original.save(args.outdir/'sc_map.npz')
    began = time.perf_counter()
    left_centered = recenter_at_left(original)
    recenter_ms = 1000*(time.perf_counter()-began)
    track = build_case()
    manifest = dict(created_at=datetime.now().isoformat(), source_sha256=hashes,
                    experiment='point_selection_only_not_full_MINCO_or_flight',
                    initial_point=LEFT_INITIAL_POINT, vertices=VERTICES,
                    query_times_s=TIMES, optimizer='L-BFGS-B', options=OPTIONS,
                    offline_sc_fit_ms=fit_ms, offline_recenter_ms=recenter_ms,
                    motion='original 3 Hz / 2 m translation; no spin',
                    body_radius_for_diagnostic_only_m=.38594472,
                    flight_executed=False)
    save_json(args.outdir/'manifest.json', manifest)
    results = []
    for name, mapping in [('original', original), ('left_centered', left_centered)]:
        for at in TIMES:
            row = solve_point(mapping, track, at)
            row['normalization'] = name
            results.append(row)
            print({k: row[k] for k in ['normalization','query_time_s','final_q','iterations',
                                      'solve_ms','physical_evaluations_outside']}, flush=True)
            save_json(args.outdir/'summary.json', results)
    draw(args.outdir, results)
    report(args.outdir, results)
    assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    print('FINISHED '+str(args.outdir), flush=True)


if __name__ == '__main__':
    main()
