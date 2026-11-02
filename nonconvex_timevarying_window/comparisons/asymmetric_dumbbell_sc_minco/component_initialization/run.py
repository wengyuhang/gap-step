"""Frozen offline comparison: original left seed versus erosion components."""
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
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from ..analyze import audit_plan
from ..run import save_json
from ..scenario import Case, VERTICES, LEFT_INITIAL_POINT, build_case, room
from ..solver import solve as solve_local
from .planner import Planner, inverse_seed


def write_report(outdir, results):
    lines = ['# 连通区域初始化：单窗口验证', '',
             '按0.38594472 m机体半径内缩；原目标、约束、四时间种子及局部复查不变。',
             '没有Gazebo飞行；下表是通过当前有限约束与局部几何复查的名义轨迹，不是实飞成绩。', '',
             '| 场景 | 初值策略 | 接纳 | 最终区域 | 名义完整起终点时间/s | 求解/ms |',
             '|---|---|---|---|---:|---:|']
    for r in results:
        f = r.get('final', {})
        lines.append(f"| {r['case']['frequency']:g} Hz / {r['case']['amplitude']:g} m | "
                     f"{r['method']} | {'是' if r['accepted'] else '否'} | {f.get('room', '—')} | "
                     f"{format(f['nominal_total_time_s'], '.6f') if f else '—'} | "
                     f"{r['stats']['optimization_ms']:.3f} |")
    lines += ['', '## 自动分区与逐块结果', '']
    for r in results:
        if r['method'] != 'components':
            continue
        lines.append(f"### {r['case']['frequency']:g} Hz / {r['case']['amplitude']:g} m")
        lines.append('')
        for row in r['stats']['branches']:
            c = row['component']
            lines.append(f"- 连通块 {c['index']}：初值 {c['point']}；"
                         f"{'接纳' if row['accepted'] else '未接纳'}；"
                         f"求解 {row['stats'].get('optimization_ms', 0.):.3f} ms。")
        lines += ['', f"离线内缩/选点 {r['stats']['offline_erosion_ms']:.3f} ms；"
                      f"SC逆映射 {r['stats']['offline_inverse_ms']:.3f} ms。", '']
    lines += ['## 独立1 ms审计', '',
              '不参与优化或接纳；离散采样不是连续动力学证书。', '',
              '| 场景/策略 | 最小采样净空（含框/余量）/m | 单桨上界超限/% | 角速度最大相对超限/% |',
              '|---|---:|---:|---:|']
    for r in results:
        if not r['accepted']:
            continue
        a = r['posthoc_audit']
        rate_violation = max(0., float(np.max(np.asarray(a['maximum_abs_body_rates_rad_s']) /
                                              a['body_rate_limits_rad_s']-1)))
        lines.append(f"| {r['case']['amplitude']:g} m/{r['method']} | "
                     f"{a['minimum_frame_clearance_after_all_margins_m']:.6f} | "
                     f"{100*a['maximum_relative_rotor_upper_violation']:.4f} | {100*rate_violation:.4f} |")
    lines += ['', '## 证据边界', '',
              '- 每块初值继续使用原SC物理域；不是强制留在该内缩块，也不加入离散区域约束。',
              '- 本轮不使用非单调跳步，不读取旧右室解；不能称单左初值跨室成功。',
              '- 几何内缩是GEOS圆弧数值近似；局部复查是浮点包络，不是Arb证书。',
              '- 每块只用原局部流程首条接纳轨迹；最终最短仅相对于本轮已接纳候选，不是全局最优。',
              '- 5 s为双方各自总求解预算；未JIT/JIT版本分别保存，离线准备不计入求解。',
              '- 失败是未找到接纳解，不是证明该区域物理不可行。', '',
              '![自动初值与最终选点](component_points.png)', '']
    (outdir/'REPORT.md').write_text('\n'.join(lines))


def draw(outdir, results, components):
    proposed = [r for r in results if r['method'] == 'components']
    fig, axes = plt.subplots(1, len(proposed), figsize=(6*len(proposed), 5), squeeze=False)
    outline = np.vstack([VERTICES, VERTICES[0]])
    for ax, r in zip(axes[0], proposed):
        ax.fill(*outline.T, color='#eeeeee')
        ax.plot(*outline.T, color='#303030', lw=1.5, label='Original frame centreline')
        for c in components:
            ax.fill(*np.asarray(c.region.exterior.coords).T, color='#b7d8bf', alpha=.8,
                    label='Body-radius erosion' if c.index == 0 else None)
            ax.scatter(*c.point, marker='*', s=130, c='#2563a6', zorder=4,
                       label='Automatic initial points' if c.index == 0 else None)
            ax.annotate(f'C{c.index}', c.point, xytext=(5, 9), textcoords='offset points')
        for row in r['stats']['branches']:
            if row['accepted']:
                ax.scatter(*row['final_q'], marker='o', s=60, color='#bf4b2e', zorder=5)
        if r['accepted']:
            ax.scatter(*r['final']['q'], marker='o', s=130, facecolors='none',
                       edgecolors='#bf4b2e', lw=2, zorder=6, label='Selected final crossing point')
        c = r['case']
        ax.set(title=f"{c['frequency']:g} Hz / {c['amplitude']:g} m: "
                     f"{'accepted nominal plan' if r['accepted'] else 'no accepted plan'}",
               xlabel='Gate-local horizontal coordinate (m)', ylabel='Gate-local vertical coordinate (m)')
        ax.set_aspect('equal')
        ax.set_xlim(-1.8, 3.4)
        ax.set_ylim(-1.8, 1.8)
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc='lower right')
    fig.suptitle('One seed per eroded component; optimization of the original moving gate')
    fig.tight_layout()
    fig.savefig(outdir/'component_points.png', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--jit', action='store_true')
    parser.add_argument('--no-static', action='store_true')
    args = parser.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[4]
    paths = list(Path(__file__).parent.glob('*.py')) + list(Path(__file__).parent.parent.glob('*.py'))
    paths += [root/p for p in (
        'convex_timevarying_window/single_gate_stop_racing/planner.py',
        'convex_timevarying_window/single_gate_stop_racing/solver.py',
        'convex_timevarying_window/single_gate_stop_racing/geometry.py',
        'convex_timevarying_window/single_gate_stop_racing/local_bounds.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/hard_planner.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/hard_dynamics.py',
        'convex_timevarying_window/dynamic_passage_velocity_minco/trajectory.py',
        'convex_timevarying_window/comparisons/original_fixed_plane_ours/adapter.py',
        'nonconvex_timevarying_window/sc_dynatogt/sc_mapping.py')]
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    cfg = Config(compile_objective=args.jit, max_solve_ms=5000.)
    began = time.perf_counter()
    mapping = SCDiskMap.fit(VERTICES)
    fit_ms = 1000*(time.perf_counter()-began)
    mapping.save(args.outdir/'sc_map.npz')
    save_json(args.outdir/'manifest.json', dict(
        created_at=datetime.now(ZoneInfo('America/Indiana/Winamac')).isoformat(),
        config=asdict(cfg), source_sha256=hashes, offline_sc_fit_ms=fit_ms,
        erosion_radius_m=cfg.radius, check_radius_m=cfg.radius+cfg.frame_thickness+cfg.passage_margin,
        flight_executed=False, protocol='Only spatial initialization changes; original local solver reused.'))
    print(json.dumps(dict(stage='SC_fit', offline_ms=fit_ms)), flush=True)
    cases = ([] if args.no_static else [Case(3., 0.)]) + [Case(3., 2.)]
    results = []
    for case in cases:
        track = build_case(case)
        began = time.perf_counter()
        planner = Planner(track, cfg, mapping)
        build_ms = 1000*(time.perf_counter()-began)
        print(json.dumps(dict(stage='built', case=asdict(case), offline_build_ms=build_ms,
                              seeds=[c.point.tolist() for c in planner.components])), flush=True)
        for method in ('original_left', 'components'):
            folder = args.outdir/case.name/method
            folder.mkdir(parents=True)
            planner.evaluations, planner.map_ms = [], 0.
            planner._map_cached_d = planner._map_cached_value = None
            planner.solvers[0][0].value = {}
            planner.initial_d = inverse_seed(mapping, LEFT_INITIAL_POINT)
            result = dict(method=method, case=asdict(case), jit=args.jit, accepted=False,
                          shared_offline_build_including_component_preparation_ms=build_ms,
                          flight_executed=False)
            try:
                state = np.r_[track.start, np.zeros(9)]
                plan = (solve_local(planner, state, 0., 0) if method == 'original_left'
                        else planner.solve(state))
                q = mapping.map_unconstrained(plan.x[2:4])
                result.update(accepted=True, final=dict(q=q.tolist(), room=room(q),
                              nominal_total_time_s=float(plan.durations.sum()),
                              crossing_time_s=float(plan.crossing_times[0])))
                np.savez(folder/'plan.npz', coefficients=plan.coefficients, durations=plan.durations, x=plan.x)
                result['posthoc_audit'] = audit_plan(folder, case,
                    cfg.radius+cfg.frame_thickness+cfg.passage_margin)
            except RuntimeError as exc:
                if str(exc) not in ('no_safe_passage_under_budget', 'no_accepted_component'):
                    raise
                result['failure'] = str(exc)
            result['stats'] = planner.solvers[0][0].stats()
            if method == 'original_left':
                result['initial_q'] = LEFT_INITIAL_POINT.tolist()
                result['evaluations'] = planner.evaluations
            else:
                for row in result['stats']['branches']:
                    branch_dir = folder/f"component_{row['component']['index']}"
                    branch_dir.mkdir()
                    if row['accepted']:
                        np.savez(branch_dir/'plan.npz', **{k: np.asarray(v) for k, v in row['plan'].items()})
                        row['posthoc_audit'] = audit_plan(branch_dir, case,
                            cfg.radius+cfg.frame_thickness+cfg.passage_margin)
                    row['points_outside_physical_polygon'] = sum(
                        not track.windows[0].aperture.contains(v['q'], tolerance=1e-7)
                        for v in row['evaluations'])
                    save_json(branch_dir/'result.json', row)
            save_json(folder/'result.json', result)
            results.append(result)
            save_json(args.outdir/'summary.json', results)
            print(json.dumps(dict(case=asdict(case), method=method, accepted=result['accepted'],
                                  final=result.get('final'), solve_ms=result['stats']['optimization_ms'])), flush=True)
    draw(args.outdir, results, planner.components)
    write_report(args.outdir, results)
    changed = [p for p, h in hashes.items() if hashlib.sha256((root/p).read_bytes()).hexdigest() != h]
    save_json(args.outdir/'source_verification.json', dict(unchanged=not changed, changed=changed))
    if changed:
        raise RuntimeError('Sources changed during experiment: '+str(changed))
    print('FINISHED '+str(args.outdir), flush=True)


if __name__ == '__main__':
    main()
