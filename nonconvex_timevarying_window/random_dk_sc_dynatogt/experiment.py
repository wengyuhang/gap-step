"""Run the fixed-plane spinning-U local Random-DK experiment."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from shapely.geometry import Polygon

from nonconvex_timevarying_window.phase_governed_sc_tracking.experiment import _build_counterexample
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_sc_dynatogt_fixed_wp import FreeSCWaypointObjective, FixedWaypointObjective
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import compare_fixed_wp_counterexample as common
from nonconvex_timevarying_window.rot_sync_sc_togt.collision import _slab_cross_section
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs

from .safety import dynamics_check
from .search import SearchConfig, search


HERE = Path(__file__).resolve().parent


def jsonable(value):
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, np.generic):
        return jsonable(value.item())
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write_json(path, value):
    path.write_text(json.dumps(jsonable(value), ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def final_audit(scenario, forward, config):
    if len(scenario.windows) != 1:
        raise ValueError("this final audit adapter supports exactly one window")
    audit, data = common._EXPERIMENT.audit_solution(scenario, forward, config, dt=0.0002)
    # Extend the historical boundary-curtain audit to the declared solid exterior.
    polygon = Polygon(scenario.windows[0].physical_polygon)
    outside = 0
    for t, p, rotation in zip(data["time"], data["position"], data["body_rotation"]):
        section = _slab_cross_section(scenario.windows[0], t, p, rotation, scenario.body, tolerance=1e-9)
        if section is not None and not section.is_empty and not polygon.contains(section):
            outside += 1
    audit["solid_exterior_violating_samples"] = outside
    audit["trajectory_validation_pass"] = audit["trajectory_validation_pass"] and outside == 0
    return audit, data


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260909)
    parser.add_argument("--per-scale", type=int, default=100)
    parser.add_argument("--d-scales", type=float, nargs="+", default=(0.02, 0.05, 0.10))
    parser.add_argument("--k-scales", type=float, nargs="+", default=(0.01, 0.03, 0.05))
    parser.add_argument("--initialization", choices=("native", "fixed-wp"), default="native")
    parser.add_argument("--baseline-json", type=Path, help="Replay an existing baseline from this algorithm; never silently reuse a different scene")
    args = parser.parse_args(argv)
    protocol = SearchConfig(seed=args.seed, per_scale=args.per_scale,
                            d_scales=tuple(args.d_scales), k_scales=tuple(args.k_scales))
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "protocol.json", dict(search=asdict(protocol), obstacle="closed complement of physical aperture", sphere_dt=0.0002,
        sphere_refine_dt=0.00005, dynamics_dt=0.001, final_dt=0.0002,
        trigger="nominal screening failure", ranking="flight time among all-screen-pass only", k_perturbation="direct additive",
        scenario="balanced_U_zero_thickness_w18_p1p1", initialization=args.initialization, safety_radius="body circumscribed radius + 0.015 m"))
    begin = time.perf_counter()
    print("Preparing frozen U geometry", flush=True)
    _, config, _, scenario, track, _, fixed_d = _build_counterexample()
    objective = FreeSCWaypointObjective(track, config)
    scene = dict(name=scenario.name, omega=scenario.windows[0].omega, phase=scenario.windows[0].theta0,
                 thickness=scenario.windows[0].thickness, radius=scenario.windows[0].rho,
                 body_half_extents=scenario.body.half_extents, size_ratio=common.RATIO,
                 center=scenario.windows[0].center, normal=scenario.windows[0].normal,
                 start=scenario.start_state.matrix, goal=scenario.goal_state.matrix,
                 physical_polygon=scenario.windows[0].physical_polygon, safe_polygon=scenario.windows[0].safe_polygon)
    write_json(output / "scene.json", scene)
    scene_hash = hashlib.sha256(json.dumps(jsonable(scene), sort_keys=True).encode()).hexdigest()
    solve_begin = time.perf_counter()
    if args.baseline_json:
        baseline = json.loads(args.baseline_json.read_text(encoding="utf-8"))
        if baseline["scene_sha256"] != scene_hash or baseline["config"] != jsonable(asdict(config)):
            raise ValueError("baseline scene/config mismatch")
        center = np.asarray(baseline["x"])
        baseline["replayed_from"] = str(args.baseline_json.resolve())
        baseline["current_run_solve_seconds"] = 0.0
    else:
        print(f"Solving native SC-DynaTOGT; initialization={args.initialization}", flush=True)
        initial = objective.initial_guess()
        initialization_record = dict(mode=args.initialization)
        if args.initialization == "fixed-wp":
            fixed = FixedWaypointObjective(track, config, fixed_d)
            fixed_result = _minimize_togt_lbfgs(fixed.value_and_gradient, fixed.initial_guess(), config)
            initial = np.r_[fixed_result.x, fixed_d]
            initialization_record.update(fixed_x=fixed_result.x, fixed_objective=float(fixed_result.fun),
                                         fixed_optimizer_success=bool(fixed_result.success))
        result = _minimize_togt_lbfgs(objective.value_and_gradient, initial, config)
        center = np.asarray(result.x)
        baseline = dict(x=center, initial_x=initial, objective=float(result.fun), optimizer_success=bool(result.success),
                        optimizer_message=str(result.message), iterations=int(result.nit), scene_sha256=scene_hash,
                        config=asdict(config), initialization=initialization_record, solve_seconds=time.perf_counter() - solve_begin)
        baseline["current_run_solve_seconds"] = baseline["solve_seconds"]
    forward = objective.forward(center)
    baseline["flight_time"] = float(forward.trajectory.total_time)
    baseline["dynamics_diagnostic"] = dynamics_check(forward.trajectory, config)
    write_json(output / "baseline.json", baseline)
    np.savez_compressed(output / "baseline_trajectory.npz", x=center, coefficients=forward.trajectory.coefficients, durations=forward.durations)
    print(f"Baseline T={baseline['flight_time']:.9f}, x={center.tolist()}, dynamics={baseline['dynamics_diagnostic']}", flush=True)
    search_begin = time.perf_counter()
    log = (output / "candidates.jsonl").open("x", encoding="utf-8")

    def record(row):
        log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
        log.flush()
        if row["id"] % 25 == 0:
            print(f"candidate {row['id']}: {row['screen']['reason']} ({row['screen_seconds']:.3f}s)", flush=True)

    try:
        rows, ranked = search(objective, center, scenario, config, protocol, on_record=record)
    finally:
        log.close()
    search_seconds = time.perf_counter() - search_begin
    selected, final_records = None, []
    audit_begin = time.perf_counter()
    for row in ranked:
        candidate = objective.forward(row["x"])
        print(f"Final whole-body audit of candidate {row['id']}", flush=True)
        try:
            audit, data = final_audit(scenario, candidate, config)
        except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
            audit, data = dict(trajectory_validation_pass=False, failure_reasons=["numerical_failure"], error=str(exc)), None
        final_records.append(dict(id=row["id"], audit=audit))
        write_json(output / f"final_audit_{row['id']}.json", audit)
        if data is not None:
            common._EXPERIMENT.save_raw_trajectory(output / f"final_candidate_{row['id']}", data)
        if audit["trajectory_validation_pass"]:
            selected = row
            np.savez_compressed(output / "selected_trajectory.npz", x=row["x"], coefficients=candidate.trajectory.coefficients, durations=candidate.durations)
            break
    reasons = Counter(row["screen"]["reason"] for row in rows[1:])
    groups = {}
    for row in rows[1:]:
        key = f"scale_{row['level']}_{row['mode']}"
        group = groups.setdefault(key, dict(count=0, sphere_pass=0, all_pass=0))
        group["count"] += 1
        spheres = row["screen"].get("spheres", [])
        group["sphere_pass"] += int(len(spheres) == len(scenario.windows) and all(s["passed"] for s in spheres))
        group["all_pass"] += int(row["screen"]["passed"])
    summary = dict(status="SAMPLED_FEASIBLE" if selected else "NO_FEASIBLE_CANDIDATE_FOUND", search_config=asdict(protocol),
                   baseline=baseline, nominal_screen=rows[0]["screen"], random_candidates=len(rows) - 1,
                   screen_feasible=len(ranked), selected=selected, rejection_counts=reasons, groups=groups,
                   final_audits=final_records, search_seconds=search_seconds,
                   final_audit_seconds=time.perf_counter() - audit_begin, total_seconds=time.perf_counter() - begin,
                   evidence="sampled nominal-model validation; no continuous certificate or global infeasibility claim")
    write_json(output / "result.json", summary)
    report = ["# Random-DK SC-DynaTOGT：自旋 U 单窗实验", "",
              f"固定中心/平面，零厚度均衡 U，尺寸比 1.9，18 rad/s，初相位 1.1 rad。SC 初值方式：{baseline.get('initialization', {'mode': 'native'})['mode']}；直接扰动 K/D，不重新优化候选。",
              "球体筛选使用物理开口补集，r_s 为整机外接球半径加 15 mm。所有硬要求通过后，才按飞行时间排序；最终才调用整机检测。", "",
              f"- 状态：`{summary['status']}`", f"- 原始 SC：T={baseline['flight_time']:.9f} s；优化器停止状态={baseline['optimizer_success']}；候选筛选={rows[0]['screen']['reason']}。",
              f"- 随机候选：{len(rows)-1}；全部筛选通过：{len(ranked)}；最终整机检测次数：{len(final_records)}。",
              f"- D 扰动尺度：{list(protocol.d_scales)}；K 直接扰动尺度：{list(protocol.k_scales)}；每尺度 {protocol.per_scale} 个。",
              f"- 基线来源：{baseline.get('replayed_from', '本次重新求解')}；本次 SC 求解耗时 {baseline['current_run_solve_seconds']:.3f} s。",
              f"- 扰动搜索耗时：{search_seconds:.3f} s；本次总耗时：{summary['total_seconds']:.3f} s。",
              f"- 首个淘汰原因统计（短路检查，不是所有违规的统计）：{dict(reasons)}", "",
              "|尺度|类型|数量|球体筛选通过|全部筛选通过|", "|---|---|---:|---:|---:|"]
    for key, group in groups.items():
        _, level, mode = key.split("_")
        report.append(f"|{level}|{mode}|{group['count']}|{group['sphere_pass']}|{group['all_pass']}|")
    report += ["", f"最终选中：{('候选 ' + str(selected['id']) + '，T=' + str(selected['flight_time']) + ' s') if selected else '无；没有将违规最少者作为结果返回。'}",
               "", "所有安全与动力学结果是采样证据。未找到合格候选只说明当前固定邻域、随机种子和预算搜索失败；不是全局不可行结论。未对每条候选运行整机检测，未进行加速比对照。", ""]
    (output / "REPORT.md").write_text("\n".join(report), encoding="utf-8")
    print(json.dumps(jsonable({k: summary[k] for k in ("status", "random_candidates", "screen_feasible", "rejection_counts", "search_seconds")}), ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
