"""Three-window open track using the original joint SC/MINCO objective."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.optimizer import JointTOGTObjective, _minimize_togt_lbfgs
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_sc_dynatogt_fixed_wp import _RotatingSCWindowAdapter
from nonconvex_timevarying_window.phase_governed_sc_tracking.experiment import _build_counterexample

from .experiment import final_audit as single_final_audit, jsonable, write_json, common
from .safety import dynamics_check
from .search import SearchConfig, search


class MultiWindowObjective:
    """Adapt arbitrary ordered window counts without single-window vector slicing."""

    def __init__(self, scenario, config):
        track = SCWindowTrack(
            name=scenario.name, start=scenario.start_state.position, goal=scenario.goal_state.position,
            windows=tuple(_RotatingSCWindowAdapter(w) for w in scenario.windows),
            order=tuple(range(len(scenario.windows))),
        )
        self.joint = JointTOGTObjective(track, config)
        self.joint.start_state = scenario.start_state
        self.joint.end_state = scenario.goal_state
        self.config = config
        self.dimension = self.joint.dimension

    def initial_guess(self):
        return self.joint.initial_guess()

    def value_and_gradient(self, x):
        return self.joint.scipy_value_and_gradient(x)

    def forward(self, x):
        f = self.joint.forward(x)
        return SimpleNamespace(trajectory=f.trajectory, crossing_times=f.traversal_times,
                               local_points=f.local_points, durations=f.durations, method="Random-DK SC-DynaTOGT")


def build_three_u_track():
    _, config, _, single, _, _, fixed_d = _build_counterexample()
    centers = ((0., 0., 1.8), (9., 1., 2.3), (18., -0.8, 1.4))
    phases = (1.1, 0.3, 2.0)
    windows = tuple(replace(single.windows[0], name=f"U_{i+1}", center=np.asarray(c), theta0=phase)
                    for i, (c, phase) in enumerate(zip(centers, phases)))
    scenario = replace(single, name="three_U_open_w18", windows=windows,
                       start_state=BoundaryState(np.asarray((-4.5, -0.8, 1.8))),
                       goal_state=BoundaryState(np.asarray((22.5, 0.8, 1.8))),
                       description="Three parallel fixed planes, balanced U apertures spinning at 18 rad/s; ordered once, open 3D slalom.")
    return scenario, config, np.tile(fixed_d, len(windows))


def audit_multi(scenario, forward, config, *, audit_one=single_final_audit, on_window=None):
    """Audit each physical window over the entire trajectory, then AND all results.

    Reuses the independently refined single-window cuboid audit. Its repeated
    full-flight dynamics computation is an explicit final-stage cost, never a
    candidate screening shortcut or a per-window trajectory decomposition.
    """
    count = len(scenario.windows)
    times = np.asarray(forward.crossing_times)
    if times.shape != (count,) or np.shape(forward.local_points) != (count, 2):
        raise ValueError("one point and crossing time required per window")
    ordered = bool(np.all(np.isfinite(times)) and np.all(np.diff(times) > 0))
    per_window = []
    for i, window in enumerate(scenario.windows):
        sub_scenario = replace(scenario, windows=(window,))
        sub_forward = SimpleNamespace(trajectory=forward.trajectory, crossing_times=times[i:i+1],
                                      local_points=forward.local_points[i:i+1], crossing_local_index=0,
                                      durations=forward.durations)
        started = time.perf_counter()
        try:
            audit, data = audit_one(sub_scenario, sub_forward, config)
        except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
            audit, data = dict(trajectory_validation_pass=False, failure_reasons=["numerical_failure"], error=str(exc)), None
        record = dict(window_index=i, window_name=window.name, seconds=time.perf_counter()-started, audit=audit)
        per_window.append(record)
        if on_window:
            on_window(record, data)
    return dict(trajectory_validation_pass=ordered and all(r["audit"]["trajectory_validation_pass"] for r in per_window),
                prescribed_order_increasing=ordered, crossing_times=times, per_window=per_window,
                evidence="full-trajectory per-window <=0.2 ms refined nominal-model sampled audit; not continuous certification")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=20260909)
    parser.add_argument("--per-scale", type=int, default=1000)
    parser.add_argument("--d-scales", type=float, nargs="+", default=(.25, .5, 1., 2.))
    parser.add_argument("--k-scales", type=float, nargs="+", default=(.1, .25, .5, 1.))
    parser.add_argument("--initialization", choices=("native", "fixed-wp"), default="native")
    parser.add_argument("--baseline-json", type=Path)
    args = parser.parse_args(argv)
    sampling = SearchConfig(seed=args.seed, per_scale=args.per_scale,
                            d_scales=tuple(args.d_scales), k_scales=tuple(args.k_scales))
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    write_json(output / "protocol.json", dict(search=asdict(sampling), scenario="three_U_open_w18",
               initialization=args.initialization, sphere_dt=.0002, sphere_refine_dt=.00005, dynamics_dt=.001,
               final_dt=.0002, radius_margin=.015, obstacle="closed aperture complement in each full plane",
               ranking="flight time only among all hard-screen passes", joint_dimension=10))
    print("Preparing three-window scene", flush=True)
    scenario, config, fixed_d = build_three_u_track()
    objective = MultiWindowObjective(scenario, config)
    n_k = len(scenario.windows) + 1
    scene = dict(name=scenario.name, start=scenario.start_state.matrix, goal=scenario.goal_state.matrix,
                 body_half_extents=scenario.body.half_extents, order=[w.name for w in scenario.windows],
                 windows=[dict(name=w.name, center=w.center, normal=w.normal, plane_basis=w.plane_basis,
                               omega=w.omega, phase=w.theta0, thickness=w.thickness, radius=w.rho,
                               physical_polygon=w.physical_polygon, safe_polygon=w.safe_polygon) for w in scenario.windows])
    write_json(output / "scene.json", scene)
    scene_hash = hashlib.sha256(json.dumps(jsonable(scene), sort_keys=True).encode()).hexdigest()
    solve_started = time.perf_counter()
    if args.baseline_json:
        baseline = json.loads(args.baseline_json.read_text())
        if baseline["scene_sha256"] != scene_hash or baseline["config"] != jsonable(asdict(config)):
            raise ValueError("baseline scene/config mismatch")
        center = np.asarray(baseline["x"])
        baseline.update(replayed_from=str(args.baseline_json.resolve()), current_run_solve_seconds=0.)
    else:
        initial = objective.initial_guess()
        if args.initialization == "fixed-wp":
            initial[n_k:] = fixed_d
            def fixed_objective(k):
                value, gradient = objective.value_and_gradient(np.r_[k, fixed_d])
                return value, gradient[:n_k]
            print("Optimizing fixed local waypoints for initialization", flush=True)
            fixed_result = _minimize_togt_lbfgs(fixed_objective, initial[:n_k], config)
            initial = np.r_[fixed_result.x, fixed_d]
        print("Solving original joint SC objective: 4 K scalars + 3 two-dimensional D groups", flush=True)
        calls = 0
        last_print = time.perf_counter()
        def native_objective(x):
            nonlocal calls, last_print
            value, gradient = objective.value_and_gradient(x)
            calls += 1
            now = time.perf_counter()
            if now - last_print > 25:
                print(f"SC evaluations={calls}, J={value:.6g}", flush=True)
                last_print = now
            return value, gradient
        result = _minimize_togt_lbfgs(native_objective, initial, config)
        center = np.asarray(result.x)
        baseline = dict(x=center, initial_x=initial, scene_sha256=scene_hash, config=asdict(config),
                        initialization=args.initialization, objective=float(result.fun), optimizer_success=bool(result.success),
                        optimizer_message=str(result.message), iterations=int(result.nit),
                        solve_seconds=time.perf_counter()-solve_started)
        baseline["current_run_solve_seconds"] = baseline["solve_seconds"]
    forward = objective.forward(center)
    baseline["flight_time"] = float(forward.trajectory.total_time)
    baseline["crossing_times"] = forward.crossing_times
    baseline["dynamics_diagnostic"] = dynamics_check(forward.trajectory, config)
    write_json(output / "baseline.json", baseline)
    np.savez_compressed(output / "baseline_trajectory.npz", x=center, coefficients=forward.trajectory.coefficients, durations=forward.durations)
    print(f"SC baseline T={baseline['flight_time']:.9f}, dynamics={baseline['dynamics_diagnostic']}", flush=True)
    search_started = time.perf_counter()
    with (output / "candidates.jsonl").open("x") as log:
        def record(row):
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
            if row["id"] % 250 == 0:
                print(f"candidate {row['id']}: {row['screen']['reason']}, window={row['screen'].get('window_index')}", flush=True)
        rows, ranked = search(objective, center, scenario, config, sampling, on_record=record)
    search_seconds = time.perf_counter()-search_started
    selected, final_records = None, []
    final_started = time.perf_counter()
    for row in ranked:
        f = objective.forward(row["x"])
        print(f"Final whole-track audit candidate {row['id']}", flush=True)
        def on_window(record, data):
            i = record["window_index"]
            write_json(output / f"candidate_{row['id']}_window_{i}_audit.json", record)
            if data is not None:
                common._EXPERIMENT.save_raw_trajectory(output / f"candidate_{row['id']}_window_{i}", data)
            print(f"final window {i+1}: {record['audit']['trajectory_validation_pass']}", flush=True)
        audit = audit_multi(scenario, f, config, on_window=on_window)
        final_records.append(dict(id=row["id"], audit=audit))
        if audit["trajectory_validation_pass"]:
            selected = row
            np.savez_compressed(output / "selected_trajectory.npz", x=row["x"], coefficients=f.trajectory.coefficients,
                                durations=f.durations, crossing_times=f.crossing_times)
            break
    final_seconds = time.perf_counter()-final_started
    reasons = Counter(row["screen"]["reason"] for row in rows[1:])
    failure_windows = Counter(str(row["screen"].get("window_index")) for row in rows[1:] if not row["screen"]["passed"])
    prefix_pass = [sum(len(row["screen"].get("spheres", [])) > i and all(s["passed"] for s in row["screen"]["spheres"][:i+1]) for row in rows[1:]) for i in range(len(scenario.windows))]
    summary = dict(status="SAMPLED_FEASIBLE" if selected else "NO_FEASIBLE_CANDIDATE_FOUND", baseline=baseline,
                   nominal_screen=rows[0]["screen"], random_candidates=len(rows)-1, search_config=asdict(sampling),
                   screen_feasible=len(ranked), selected=selected, rejection_counts=reasons,
                   first_failure_window=failure_windows, sphere_prefix_pass_counts=prefix_pass,
                   final_audits=final_records, search_seconds=search_seconds, final_audit_seconds=final_seconds,
                   total_seconds=time.perf_counter()-started)
    write_json(output / "result.json", summary)
    report = ["# Random-DK：三窗口开放赛道", "",
              "按 U1 → U2 → U3 穿越；中心为 (0,0,1.8)、(9,1,2.3)、(18,-0.8,1.4) m，平面法向均为 X。",
              "每窗均为零厚度均衡 U、18 rad/s 自旋，初相位分别为 1.1/0.3/2.0 rad。起点 (-4.5,-0.8,1.8)，终点 (22.5,0.8,1.8)，起终速度/加速度/jerk 为零。",
              "完整轨迹采用原始 SC 的 4 段 MINCO，4 个 K 标量与 3 组二维 D 联合扰动（共 10 个标量）。每个窗口的全部接触区间都检查，后续窗口使用累计绝对到达时刻。", "",
              f"- 结果：{summary['status']}；SC T={baseline['flight_time']:.9f} s，优化器停止={baseline['optimizer_success']}。",
              f"- 原始筛选：{rows[0]['screen']['reason']}；随机候选 {len(rows)-1}，全部筛选通过 {len(ranked)}。",
              f"- D 半径比例 {list(sampling.d_scales)}；K 直接扰动比例 {list(sampling.k_scales)}；每档 {sampling.per_scale} 个，种子 {sampling.seed}。",
              f"- 基线来源：{baseline.get('replayed_from', '本次原始 SC 求解')}。",
              f"- 通过前 1/2/3 窗球体筛选的候选数：{prefix_pass}（短路筛选，未检查者不是通过）。",
              f"- 首个失败原因：{dict(reasons)}；首个失败窗口（0 基索引）：{dict(failure_windows)}。",
              f"- SC 本次求解 {baseline['current_run_solve_seconds']:.3f} s；扰动搜索 {search_seconds:.3f} s；最终整机检测 {final_seconds:.3f} s；总计 {summary['total_seconds']:.3f} s。",
              f"- 最终选择：{('id='+str(selected['id'])+', T='+str(selected['flight_time'])+' s') if selected else '无，不返回违规最少者。'}", "",
              "几何、动力学与整机验收均为名义模型密集采样，不是连续域证书。当前预算没有候选不表示全局不可行。", ""]
    (output / "REPORT.md").write_text("\n".join(report), encoding="utf-8")
    print(json.dumps(jsonable({k:summary[k] for k in ('status','random_candidates','screen_feasible','rejection_counts','sphere_prefix_pass_counts','search_seconds')})), flush=True)


if __name__ == "__main__":
    main()
