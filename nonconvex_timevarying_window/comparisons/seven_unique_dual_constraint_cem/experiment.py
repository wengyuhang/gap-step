#!/usr/bin/env python3
"""Run a frozen three-method comparison on the seven-unique-window course."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import polylabel

from nonconvex_timevarying_window.comparisons.curved_rotating_sc_fixed_wp.experiment import (
    FixedMultiWindowObjective,
)
from nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.experiment import solve
from nonconvex_timevarying_window.comparisons.seven_unique_sc_sphere.experiment import (
    SHAPES,
    build_seven_unique_track,
    plot_closed_route,
    scene_record,
)
from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.safety_penalty import (
    SafetyPenaltyConfig,
    integrated_safety_penalty,
)
from nonconvex_timevarying_window.dual_constraint_cem_sc_dynatogt.search import (
    DualCEMConfig,
    dual_constraint_cem,
    resume_dual_constraint_cem,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_seeded import (
    _disk_to_unconstrained,
)
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import sphere_check
from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    PenaltyWeights,
    constraint_extrema,
    integrated_dynamic_penalty,
)


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]


def togt_config(base):
    """Use the released standard TOGT limits and enabled C++ penalty terms."""
    return replace(base, dynamic_limits=DynamicLimits(),
                   penalty_weights=PenaltyWeights(velocity=0.0, collective_thrust=0.0,
                                                  body_rate=1.0, rotor_thrust=1.0))


def fixed_waypoints(scenario):
    local, latent = [], []
    for window in scenario.windows:
        q = np.asarray(polylabel(Polygon(window.safe_polygon), tolerance=1e-7).coords[0])
        disk = np.asarray(window.gate.sc_map.inverse(q), dtype=float)
        local.append(q)
        latent.append(_disk_to_unconstrained(disk))
    return np.asarray(local), np.asarray(latent)


def dynamic_audit(trajectory, config, dt=0.001):
    max_nodes = max(int(np.ceil(float(t) / dt)) + 1 for t in trajectory.durations)
    extrema = constraint_extrema(trajectory, parameters=config.quadrotor,
                                 samples_per_segment=max_nodes)
    limits = config.dynamic_limits
    tests = {
        "velocity": extrema["max_velocity"] <= limits.max_velocity + 1e-9,
        "body_rate_xy": extrema["max_body_rate_xy"] <= limits.max_body_rate_xy + 1e-9,
        "body_rate_z": extrema["max_abs_body_rate_z"] <= limits.max_body_rate_z + 1e-9,
        "collective_thrust": (extrema["min_collective_thrust"] >= limits.min_collective_thrust - 1e-9 and
                              extrema["max_collective_thrust"] <= limits.max_collective_thrust + 1e-9),
        "rotor_thrust": (np.min(extrema["min_rotor_thrust"]) >= limits.min_rotor_thrust - 1e-9 and
                         np.max(extrema["max_rotor_thrust"]) <= limits.max_rotor_thrust + 1e-9),
    }
    return {"passed": bool(all(tests.values())), "per_constraint": tests,
            "extrema": extrema, "samples_per_segment": max_nodes,
            "maximum_step_bound": dt,
            "evidence": "full-flight nominal-model dense sampling; not a continuous certificate"}


def safety_audit(forward, scenario):
    rows = []
    for index, window in enumerate(scenario.windows):
        result = sphere_check(
            forward.trajectory, window, scenario.body.circumscribed_radius,
            obstacle_model="boundary_frame", stop_at_first_violation=False)
        rows.append({"window_index": index, "window_name": window.name, **result})
    order = bool(np.all(np.diff(forward.crossing_times) > 0))
    single = all(len(row["crossings"]) == 1 and
                 abs(row["crossings"][0] - forward.crossing_times[i]) <= 1e-7
                 for i, row in enumerate(rows))
    return {"passed": bool(all(row["passed"] for row in rows)),
            "ordered": order, "exactly_one_crossing_each": single, "per_window": rows,
            "crossing_count_used_for_collision_pass": False,
            "evidence": "all sphere/zero-thickness-frame contact intervals at <=0.2 ms with 0.05 ms refinement; plane-crossing count is diagnostic only; sampled, not certified"}


def soft_metrics(forward, scenario, config, safety_config):
    dynamic = integrated_dynamic_penalty(
        forward.trajectory, parameters=config.quadrotor, limits=config.dynamic_limits,
        weights=config.penalty_weights, samples_per_segment=None, return_breakdown=True)
    safety = integrated_safety_penalty(
        forward.trajectory, scenario.windows, safety_config,
        return_breakdown=True)
    return {"dynamic_integral": float(dynamic.total),
            "dynamic_breakdown": {k: float(getattr(dynamic, k)) for k in
                                  ("velocity", "collective_thrust", "body_rate", "rotor_thrust")},
            "safety_integral": safety["total"], "safety_breakdown": safety["per_window"]}


def method_row(name, result, forward, solve_seconds, calls, scenario, config, safety_config):
    metrics = soft_metrics(forward, scenario, config, safety_config)
    dynamics = dynamic_audit(forward.trajectory, config)
    safety = safety_audit(forward, scenario)
    return {"method": name, "flight_time": float(forward.trajectory.total_time),
            **metrics, "dynamic_audit": dynamics, "safety_audit": safety,
            "dynamics_pass": dynamics["passed"], "safety_pass": safety["passed"],
            "all_hard_pass": bool(dynamics["passed"] and safety["passed"]),
            "optimizer_success": bool(result.success), "optimizer_message": str(result.message),
            "iterations": int(result.nit), "objective_evaluations": calls,
            "native_objective": float(result.fun), "solve_seconds": solve_seconds,
            "decision_vector": np.asarray(result.x), "durations": forward.durations,
            "crossing_times": forward.crossing_times, "local_points": forward.local_points}


def source_manifest():
    roots = [REPO / "nonconvex_timevarying_window" / name for name in (
        "dual_constraint_cem_sc_dynatogt", "sc_dynatogt", "random_dk_sc_dynatogt",
        "rot_sync_sc_togt", "comparisons/seven_unique_dual_constraint_cem",
        "comparisons/seven_unique_sc_sphere", "comparisons/curved_rotating_sc_fixed_wp")]
    files = sorted(p for root in roots for p in root.rglob("*.py"))
    return {str(p.relative_to(REPO)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def write_report(path, rows, status, search):
    lines = ["# 七异形闭合赛道三方法结果", "",
             "| 方法 | 飞行时间 (s) | TOGT 动力学软积分 | 安全软积分 | 完整动力学检测 | 完整安全检测 |",
             "|---|---:|---:|---:|---|---|"]
    for row in rows:
        lines.append(f"| {row['method']} | {row['flight_time']:.9f} | {row['dynamic_integral']:.9g} | "
                     f"{row['safety_integral']:.9g} | {'满足' if row['dynamics_pass'] else '不满足'} | "
                     f"{'满足' if row['safety_pass'] else '不满足'} |")
    lines += ["", f"新算法状态：`{status}`；CEM 候选数：`{search['candidate_count']}`；"
                    f"硬检测候选数：`{search['hard_audited']}`。",
              "动力学硬检测覆盖全程，步长上界 1 ms；安全硬检测覆盖七个窗口的全部球体接触区间。两者都是密集采样数值证据，不是连续域证书。",
              "Fixed-WP 与 SC-DynaTOGT 的安全积分是用统一度量离线计算的，没有加入它们的优化目标。", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--population", type=int, default=128)
    parser.add_argument("--maximum-rounds", type=int, default=12)
    parser.add_argument("--hard-audit-limit", type=int, default=96)
    parser.add_argument("--baseline-result-dir", type=Path,
                        help="Replay frozen Fixed-WP/SC decisions from an earlier run of this protocol")
    parser.add_argument("--candidate-jsonl", type=Path,
                        help="Audit an already generated frozen candidate set without rerunning CEM")
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    start_manifest = source_manifest()
    write_json(output / "code_manifest_start.json", start_manifest)
    started = time.perf_counter()

    scenario, inherited = build_seven_unique_track()
    config = togt_config(inherited)
    safety_config = SafetyPenaltyConfig()
    cem_config = DualCEMConfig(population=args.population,
                               elite=max(4, min(24, args.population // 5)),
                               memory=max(2, min(12, args.population // 10)),
                               maximum_rounds=args.maximum_rounds)
    fixed_local, fixed_d = fixed_waypoints(scenario)
    write_json(output / "scene.json", scene_record(scenario))
    write_json(output / "protocol.json", {
        "methods": ["Fixed-WP", "SC-DynaTOGT", "Dual-Constraint CEM SC-DynaTOGT"],
        "dynamic_config": asdict(config), "safety_penalty": asdict(safety_config),
        "safety_sphere_radius": float(scenario.body.circumscribed_radius),
        "planning_inset_radius": float(scenario.windows[0].rho),
        "cem": asdict(cem_config), "structured_frontend": False,
        "ranking": "Pareto fronts of dynamic and safety soft integrals; within-front augmented objective",
        "return_rule": "minimum flight time among independently hard-audited passes",
        "code_frozen_during_run": True,
        "baseline_replay": None if args.baseline_result_dir is None else str(args.baseline_result_dir.resolve()),
        "candidate_replay": None if args.candidate_jsonl is None else str(args.candidate_jsonl.resolve()),
    })
    free = MultiWindowObjective(scenario, config)

    fixed = FixedMultiWindowObjective(free, fixed_d)
    if args.baseline_result_dir is None:
        print("Solving Fixed-WP", flush=True)
        fixed_result, fixed_forward, fixed_seconds, fixed_calls = solve(
            "Fixed-WP", fixed, fixed.initial_guess())
        print("Solving original SC-DynaTOGT from its native initialization", flush=True)
        sc_result, sc_forward, sc_seconds, sc_calls = solve(
            "SC-DynaTOGT", free, free.initial_guess())
    else:
        source = args.baseline_result_dir.resolve()
        prior_protocol = json.loads((source / "protocol.json").read_text(encoding="utf-8"))
        if prior_protocol["dynamic_config"] != jsonable(asdict(config)):
            raise ValueError("baseline dynamic configuration mismatch")
        prior = json.loads((source / "baseline_checkpoint.json").read_text(encoding="utf-8"))["rows"]
        fixed_old, sc_old = prior
        fixed_x = np.asarray(fixed_old["decision_vector"])
        sc_x = np.asarray(sc_old["decision_vector"])
        fixed_forward, sc_forward = fixed.forward(fixed_x), free.forward(sc_x)
        fixed_result = SimpleNamespace(success=fixed_old["optimizer_success"],
            message=fixed_old["optimizer_message"], nit=fixed_old["iterations"],
            fun=fixed_old["native_objective"], x=fixed_x)
        sc_result = SimpleNamespace(success=sc_old["optimizer_success"],
            message=sc_old["optimizer_message"], nit=sc_old["iterations"],
            fun=sc_old["native_objective"], x=sc_x)
        fixed_seconds, sc_seconds = fixed_old["solve_seconds"], sc_old["solve_seconds"]
        fixed_calls, sc_calls = fixed_old["objective_evaluations"], sc_old["objective_evaluations"]
        print(f"Replaying frozen baselines from {source}", flush=True)
    print("Computing complete baseline audits", flush=True)
    rows = [method_row("Fixed-WP", fixed_result, fixed_forward, fixed_seconds, fixed_calls,
                       scenario, config, safety_config),
            method_row("SC-DynaTOGT", sc_result, sc_forward, sc_seconds, sc_calls,
                       scenario, config, safety_config)]
    write_json(output / "baseline_checkpoint.json", {"rows": rows})
    for row, forward, stem in zip(rows, (fixed_forward, sc_forward), ("fixed_wp", "sc_dynatogt")):
        np.savez_compressed(output / f"{stem}_trajectory.npz", x=row["decision_vector"],
                            coefficients=forward.trajectory.coefficients, durations=forward.durations,
                            crossing_times=forward.crossing_times, local_points=forward.local_points)
        plot_closed_route(scenario, forward.trajectory, output / "figures" / f"{stem}_route.png")

    if args.candidate_jsonl is None:
        print("Running frontend-free dual-constraint CEM", flush=True)
        with (output / "candidates.jsonl").open("x", encoding="utf-8") as log:
            def record(row):
                log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
                log.flush()
                if row["id"] % max(1, args.population // 4) == 0:
                    print(f"candidate={row['id']} round={row['round']} Jdyn={row['dynamic_integral']:.5g} "
                          f"Jsafe={row['safety_integral']:.5g} T={row['flight_time']:.5g}", flush=True)
            candidates, ranked, round_summaries = dual_constraint_cem(
                free, sc_result.x, scenario, config, cem_config, safety_config, on_record=record)
    else:
        source_candidates = args.candidate_jsonl.resolve()
        candidates = [json.loads(line) for line in source_candidates.read_text(encoding="utf-8").splitlines()]
        print(f"Replaying {len(candidates)} frozen candidates from {source_candidates}", flush=True)
        with (output / "candidates.jsonl").open("x", encoding="utf-8") as log:
            for row in candidates:
                log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()

            def record_continuation(row):
                log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
                log.flush()
                print(f"candidate={row['id']} round={row['round']} Jdyn={row['dynamic_integral']:.5g} "
                      f"Jsafe={row['safety_integral']:.5g} T={row['flight_time']:.5g}", flush=True)

            candidates, ranked, round_summaries = resume_dual_constraint_cem(
                free, sc_result.x, scenario, config, candidates, cem_config, safety_config,
                on_record=record_continuation)

    # Fixed epsilons affect Pareto resolution only. Eligibility remains exact
    # zero for both integrals, followed by independent hard verification.
    audit_order = sorted((r for r in ranked if r["both_soft_zero"]),
                         key=lambda r: (r["flight_time"], r["id"]))
    hard_rows = []
    print("Running independent complete audits of ranked CEM candidates", flush=True)
    with (output / "hard_audits.jsonl").open("x", encoding="utf-8") as audit_log:
        for audit_index, candidate in enumerate(audit_order[:args.hard_audit_limit], start=1):
            forward = free.forward(candidate["x"])
            dynamics = dynamic_audit(forward.trajectory, config)
            safety = safety_audit(forward, scenario)
            hard_row = {"audit_index": audit_index, "id": candidate["id"],
                        "flight_time": candidate["flight_time"], "dynamics": dynamics,
                        "safety": safety,
                        "passed": bool(dynamics["passed"] and safety["passed"])}
            hard_rows.append(hard_row)
            audit_log.write(json.dumps(jsonable(hard_row), ensure_ascii=False, allow_nan=False) + "\n")
            audit_log.flush()
            minimum_margin = min(row["minimum_margin"] for row in safety["per_window"])
            print(f"hard-audit {audit_index}: candidate={candidate['id']} T={candidate['flight_time']:.9f} "
                  f"dynamics={dynamics['passed']} safety={safety['passed']} "
                  f"min_margin={minimum_margin:.9g}", flush=True)
            if hard_row["passed"]:
                break
    feasible = [r for r in hard_rows if r["passed"]]
    selected = min(feasible, key=lambda r: (r["flight_time"], r["id"])) if feasible else None
    if selected is not None:
        candidate = next(r for r in candidates if r["id"] == selected["id"])
        forward = free.forward(candidate["x"])
        pseudo = type("CEMResult", (), {"success": True, "message": "selected hard-feasible CEM sample",
                                        "nit": len(round_summaries), "fun": candidate["augmented_objective"],
                                        "x": candidate["x"]})()
        new_row = method_row("Dual-Constraint CEM SC-DynaTOGT", pseudo, forward, 0.0,
                             len(candidates), scenario, config, safety_config)
        new_row["solve_seconds"] = time.perf_counter() - started
        new_row["selected_candidate_id"] = selected["id"]
        rows.append(new_row)
        np.savez_compressed(output / "dual_constraint_cem_trajectory.npz", x=candidate["x"],
                            coefficients=forward.trajectory.coefficients, durations=forward.durations,
                            crossing_times=forward.crossing_times, local_points=forward.local_points)
        plot_closed_route(scenario, forward.trajectory, output / "figures" / "dual_constraint_cem_route.png")
        status = "SAMPLED_HARD_FEASIBLE"
    else:
        status = "NO_HARD_FEASIBLE_CANDIDATE_FOUND"

    end_manifest = source_manifest()
    unchanged = start_manifest == end_manifest
    write_json(output / "code_manifest_end.json", {"unchanged": unchanged, "files": end_manifest})
    if not unchanged:
        raise RuntimeError("source code changed during the formal experiment")
    search_record = {"candidate_count": len(candidates), "hard_audited": len(hard_rows),
                     "rounds": round_summaries, "hard_results": hard_rows,
                     "selected_candidate_id": None if selected is None else selected["id"]}
    result = {"status": status, "shape_sequence": SHAPES, "fixed_local_points": fixed_local,
              "rows": rows, "search": search_record, "total_seconds": time.perf_counter() - started,
              "code_unchanged_during_run": unchanged}
    write_json(output / "result.json", result)
    write_report(output / "REPORT.md", jsonable(rows), status, search_record)
    print(json.dumps(jsonable({"status": status, "rows": rows,
                               "candidate_count": len(candidates), "hard_audited": len(hard_rows)}),
                     ensure_ascii=False), flush=True)
    return 0 if selected is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
