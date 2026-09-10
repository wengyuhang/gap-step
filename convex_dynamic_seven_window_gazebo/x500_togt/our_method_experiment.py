#!/usr/bin/env python3
"""Run nominal TOGT, conditional safety restoration, and dual CEM."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from convex_timevarying_window.togt.experiment import plot_route
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import (
    AUDIT_STEP, X500_FRAME_RADIUS, build_track, dynamic_audit, safety_audit, X500Objective,
)
REPO = Path(__file__).resolve().parents[2]
BODY_RADIUS = X500_FRAME_RADIUS
def scene_record(track):
    return {"name": track.name, "start": track.start, "goal": track.goal,
            "order": list(track.order), "x500_frame_radius": X500_FRAME_RADIUS,
            "window_margin": [w.aperture.margin for w in track.windows]}
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs

from convex_timevarying_window.conditional_dual_constraint_cem.objective import SafetyAugmentedTOGTObjective
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import ConvexSafetyConfig, HandwrittenConvexSafetyIntegral
from convex_timevarying_window.conditional_dual_constraint_cem.search import ConvexDualCEMConfig, conditional_dual_cem


HERE = Path(__file__).resolve().parent


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    return value


def _write_json(path, value):
    path.write_text(json.dumps(_jsonable(value), ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


def _manifest():
    method = REPO / "convex_timevarying_window" / "conditional_dual_constraint_cem"
    files = sorted(HERE.glob("*.py")) + sorted((HERE / "native").glob("*.cpp"))
    files += sorted(method.glob("*.py"))
    files += [REPO / "convex_timevarying_window" / "geometry.py"]
    files += sorted((REPO / "nonconvex_timevarying_window" / "sc_dynatogt").glob("*.py"))
    files += [REPO / "nonconvex_timevarying_window" / "dual_constraint_cem_sc_dynatogt" / "search.py"]
    return {
        str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files
    }


def _solve(objective, initial_x, config, label):
    evaluations = 0
    last_update = time.perf_counter()
    def measured(x):
        nonlocal evaluations, last_update
        evaluations += 1
        answer = objective.scipy_value_and_gradient(x)
        now = time.perf_counter()
        if now-last_update >= 25.0:
            print(f"{label}: eval={evaluations} cost={answer[0]:.9g}", flush=True)
            last_update = now
        return answer
    started = time.perf_counter()
    result = _minimize_togt_lbfgs(measured, initial_x, config)
    elapsed = time.perf_counter()-started
    value, gradient = objective.value_and_gradient(result.x)
    print(f"{label}: {elapsed:.6f}s, eval={evaluations}, cost={value:.9g}", flush=True)
    return result, elapsed, evaluations, value, gradient


def _optimizer_record(result, seconds, evaluations, gradient, invalid_trials):
    return {
        "success": bool(result.success), "status": int(result.status),
        "message": str(result.message), "iterations": int(result.nit),
        "evaluations": int(result.nfev), "measured_evaluations": evaluations,
        "seconds": seconds,
        "gradient_inf_norm": float(np.linalg.norm(gradient, ord=np.inf)),
        "invalid_trial_count": int(invalid_trials),
    }


def _soft_values(base, safety, x):
    forward = base.forward(x)
    base_cost, _ = base.value_and_gradient(x)
    safety_result = safety.evaluate(forward.trajectory, with_gradient=False)
    return forward, {
        "dynamic_integral": max(float(base_cost-forward.trajectory.total_time), 0.0),
        "safety_integral": max(float(safety_result.value), 0.0),
        "safety_per_window": list(safety_result.per_window),
    }


def _write_report(path, result):
    baseline = result["comparison"]["togt"]
    method = result["comparison"]["conditional_dual_constraint_cem"]
    def mark(value):
        if value is None:
            return "无合格轨迹"
        return "满足" if value else "不满足"
    method_time = "—" if method["flight_time_seconds"] is None else f"{method['flight_time_seconds']:.9f}"
    lines = [
        "# 条件式安全恢复 Dual-Constraint CEM：七窗口结果", "",
        f"分支：`{result['branch']['mode']}`；名义 TOGT 安全软积分："
        f"`{result['branch']['nominal_safety_integral']:.12g}`。", "",
        "| 方法 | 飞行时间 (s) | 完整动力学 | 完整安全 |", "|---|---:|---|---|",
        f"| TOGT | {baseline['flight_time_seconds']:.9f} | {mark(baseline['dynamic_passed'])} | {mark(baseline['safety_passed'])} |",
        f"| Conditional Dual-Constraint CEM | {method_time} | {mark(method['dynamic_passed'])} | {mark(method['safety_passed'])} |",
        "", "## 阶段耗时", "",
        "| 阶段 | 时间 (s) |", "|---|---:|",
    ]
    for key, value in result["timing"].items():
        lines.append(f"| {key} | {value:.6f} |")
    lines += [
        "", f"安全优化积分使用外接球半径加 {1000.0 * result['protocol']['planning_extra_safety_margin_m']:.0f} mm "
        "规划余量；完整安全验收只使用真实外接球半径。",
        "动力学和安全最终验收采用最大 1 ms 步长的密集采样，是数值证据，不是连续域认证。", "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--population", type=int, default=64)
    parser.add_argument("--maximum-rounds", type=int, default=50)
    parser.add_argument("--repair-max-iterations", type=int, default=600)
    parser.add_argument("--initial-result", type=Path)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "figures").mkdir()
    run_started = time.perf_counter()
    start_manifest = _manifest()
    _write_json(output / "code_manifest_start.json", start_manifest)

    stage = time.perf_counter()
    track, config = build_track()
    base = X500Objective(track, config)
    initial_x = base.initial_guess()
    if args.initial_result:
        initial_x = np.asarray(json.loads(args.initial_result.read_text())["decision_vector"], dtype=float)
    setup_seconds = time.perf_counter()-stage

    stage = time.perf_counter()
    safety_config = ConvexSafetyConfig(body_radius=BODY_RADIUS)
    safety = HandwrittenConvexSafetyIntegral(
        track.windows, base.head, base.tail, safety_config, base.core
    )
    safety_geometry_seconds = time.perf_counter()-stage
    print(f"setup={setup_seconds:.6f}s safety_geometry={safety_geometry_seconds:.6f}s", flush=True)

    nominal_result, nominal_seconds, nominal_evaluations, nominal_cost, nominal_gradient = _solve(
        base, initial_x, config, "nominal TOGT"
    )
    nominal_forward, nominal_soft = _soft_values(base, safety, nominal_result.x)
    mode = "dynamic_only" if nominal_soft["safety_integral"] == 0.0 else "joint"
    print(f"conditional branch={mode}, nominal safety={nominal_soft['safety_integral']:.12g}", flush=True)

    repair_record = None
    repair_seconds = 0.0
    seed_x = np.asarray(nominal_result.x, dtype=float)
    if mode == "joint":
        augmented = SafetyAugmentedTOGTObjective(base, safety)
        repair_config = replace(config, max_iterations=args.repair_max_iterations)
        repair_result, repair_seconds, repair_evaluations, repair_cost, repair_gradient = _solve(
            augmented, seed_x, repair_config, "joint safety restoration"
        )
        seed_x = np.asarray(repair_result.x, dtype=float)
        _, repair_soft = _soft_values(base, safety, seed_x)
        repair_record = {
            "optimizer": _optimizer_record(
                repair_result, repair_seconds, repair_evaluations, repair_gradient,
                augmented.invalid_trial_count,
            ),
            "objective": repair_cost,
            **repair_soft,
        }

    candidate_path = output / "candidates.jsonl"
    candidate_file = candidate_path.open("w", encoding="utf-8")
    def on_record(row):
        candidate_file.write(json.dumps(_jsonable(row), ensure_ascii=False)+"\n")
        candidate_file.flush()
    cem_config = ConvexDualCEMConfig(
        population=args.population, maximum_rounds=args.maximum_rounds
    )
    stage = time.perf_counter()
    rows, ranked, summaries = conditional_dual_cem(
        base, safety, seed_x, mode, cem_config, on_record=on_record
    )
    cem_seconds = time.perf_counter()-stage
    candidate_file.close()
    double_zero = sorted(
        (row for row in rows if row["both_soft_zero"]),
        key=lambda row: (row["flight_time"], row["id"]),
    )
    print(f"CEM={cem_seconds:.6f}s candidates={len(rows)} double_zero={len(double_zero)}", flush=True)

    stage = time.perf_counter()
    nominal_dynamic = dynamic_audit(nominal_forward.trajectory, config)
    nominal_safety = safety_audit(nominal_forward.trajectory, track)
    baseline_audit_seconds = time.perf_counter()-stage

    selected = None
    audit_attempts = []
    stage = time.perf_counter()
    for row in double_zero:
        forward = base.forward(row["x"])
        dynamic = dynamic_audit(forward.trajectory, config)
        safety_result = None
        if dynamic["passed"]:
            safety_result = safety_audit(forward.trajectory, track)
        attempt = {
            "candidate_id": row["id"], "flight_time_seconds": row["flight_time"],
            "dynamic_passed": dynamic["passed"],
            "safety_passed": None if safety_result is None else safety_result["passed"],
        }
        audit_attempts.append(attempt)
        safety_label = "skipped" if safety_result is None else str(safety_result["passed"])
        print(
            f"audit candidate={row['id']} T={row['flight_time']:.9f}s "
            f"dynamic={dynamic['passed']} safety={safety_label}",
            flush=True,
        )
        if dynamic["passed"] and safety_result is not None and safety_result["passed"]:
            selected = (row, forward, dynamic, safety_result)
            print(f"first accepted candidate={row['id']} T={row['flight_time']:.9f}s", flush=True)
            break
    method_audit_seconds = time.perf_counter()-stage

    method_comparison = {
        "flight_time_seconds": None,
        "dynamic_passed": None,
        "safety_passed": None,
        "candidate_id": None,
    }
    if selected is not None:
        row, forward, dynamic, safety_result = selected
        method_comparison.update(
            flight_time_seconds=row["flight_time"], dynamic_passed=dynamic["passed"],
            safety_passed=safety_result["passed"], candidate_id=row["id"],
        )
        plot_route(output / "figures" / "selected_route.png", forward.trajectory, track, forward.traversal_times)

    timing = {
        "scene_and_objective_setup": setup_seconds,
        "safety_geometry_setup": safety_geometry_seconds,
        "nominal_togt_lbfgs": nominal_seconds,
        "conditional_joint_lbfgs": repair_seconds,
        "cem_search": cem_seconds,
        "baseline_final_audits": baseline_audit_seconds,
        "method_final_audits": method_audit_seconds,
        "total_wall": time.perf_counter()-run_started,
    }
    result = {
        "protocol": {
            "conditional_rule": "activate safety L-BFGS iff nominal safety soft integral is nonzero",
            "safety_gradient": "hand-written local-frame, plane-normal, smooth-max/circle implicit gradient plus released C++ MINCO adjoint",
            "planning_extra_safety_margin_m": safety_config.extra_optimization_margin,
            "final_audit_extra_margin_m": 0.0,
            "x500_model": True, "initial_result": str(args.initial_result.resolve()) if args.initial_result else None,
            "cem": asdict(cem_config),
            "safety": asdict(safety_config),
        },
        "scene": scene_record(track),
        "branch": {"mode": mode, "nominal_safety_integral": nominal_soft["safety_integral"]},
        "nominal": {
            "optimizer": _optimizer_record(
                nominal_result, nominal_seconds, nominal_evaluations,
                nominal_gradient, base.invalid_trial_count,
            ),
            "objective": nominal_cost, **nominal_soft,
            "dynamic_audit": nominal_dynamic, "safety_audit": nominal_safety,
        },
        "restoration": repair_record,
        "cem": {
            "candidate_count_including_seed": len(rows),
            "strict_double_zero_count": len(double_zero),
            "rounds": summaries,
            "hard_audit_attempts": audit_attempts,
        },
        "comparison": {
            "togt": {
                "flight_time_seconds": float(nominal_forward.trajectory.total_time),
                "dynamic_passed": nominal_dynamic["passed"],
                "safety_passed": nominal_safety["passed"],
            },
            "conditional_dual_constraint_cem": method_comparison,
        },
        "timing": timing,
    }
    _write_json(output / "result.json", result)
    _write_json(output / "scene.json", scene_record(track))
    _write_json(output / "protocol.json", result["protocol"])
    _write_json(output / "cem_rounds.json", summaries)
    end_manifest = _manifest()
    _write_json(output / "code_manifest_end.json", end_manifest)
    result["source_frozen"] = start_manifest == end_manifest
    result["timing"]["total_wall"] = time.perf_counter()-run_started
    _write_json(output / "result.json", result)
    _write_report(output / "REPORT.md", result)
    print(f"source_frozen={result['source_frozen']} result={output/'result.json'}", flush=True)
    return 0 if result["source_frozen"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
