#!/usr/bin/env python3
"""Refine an audited Conditional Dual-Constraint CEM solution in flight time."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import (
    ConvexSafetyConfig,
    HandwrittenConvexSafetyIntegral,
)
from convex_timevarying_window.togt.experiment import (
    BODY_RADIUS,
    build_track,
    dynamic_audit,
    plot_route,
    safety_audit,
    scene_record,
)
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective

from .refinement import JointTimeRefinementConfig, refine_joint_feasible_time


DEFAULT_CANDIDATES = Path(
    "convex_timevarying_window/conditional_dual_constraint_cem/results/"
    "formal_cpp_aligned_margin30mm_20260910/candidates.jsonl"
)


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path, value):
    path.write_text(
        json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _load_seed(path, base, track, optimizer_config):
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    candidates = sorted(
        (row for row in rows if row.get("both_soft_zero")),
        key=lambda row: (float(row["flight_time"]), int(row["id"])),
    )
    for row in candidates:
        x = np.asarray(row["x"], dtype=float)
        forward = base.forward(x)
        dynamic = dynamic_audit(forward.trajectory, optimizer_config)
        if not dynamic["passed"]:
            continue
        safety = safety_audit(forward.trajectory, track)
        if safety["passed"]:
            return row, x, dynamic, safety
    raise RuntimeError("candidate file contains no independently audited feasible seed")


def _write_report(path, result):
    improvement = result["seed"]["flight_time"] - result["refined"]["flight_time"]
    percent = 100.0 * improvement / result["seed"]["flight_time"]
    lines = [
        "# 联合可行时间精修结果",
        "",
        "| 阶段 | 飞行时间 (s) | 完整动力学 | 完整安全 |",
        "|---|---:|---|---|",
        f"| 输入可行解 | {result['seed']['flight_time']:.9f} | 满足 | 满足 |",
        f"| 联合时间精修 | {result['refined']['flight_time']:.9f} | 满足 | 满足 |",
        "",
        f"在保持相同 1 ms 完整动力学与整机安全审计的前提下，飞行时间缩短 "
        f"`{improvement:.9f} s` (`{percent:.3f}%`)。",
        "",
        "该结果是密集采样数值证据，不是连续域认证；也不声称全局时间最优。",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seed-candidates", type=Path, default=DEFAULT_CANDIDATES)
    parser.add_argument("--maximum-rounds", type=int, default=3)
    parser.add_argument("--local-max-iterations", type=int, default=300)
    parser.add_argument("--scan-points", type=int, default=41)
    parser.add_argument("--hard-bisection-steps", type=int, default=3)
    args = parser.parse_args(argv)

    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "figures").mkdir()
    started = time.perf_counter()

    track, optimizer_config = build_track()
    base = NativeJointTOGTObjective(track, optimizer_config)
    safety_config = ConvexSafetyConfig(body_radius=BODY_RADIUS)
    safety = HandwrittenConvexSafetyIntegral(
        track.windows, base.head, base.tail, safety_config, base.core
    )
    seed_row, seed_x, seed_dynamic, seed_safety = _load_seed(
        args.seed_candidates.resolve(), base, track, optimizer_config
    )
    print(
        f"seed candidate={seed_row['id']} T={seed_row['flight_time']:.9f}s",
        flush=True,
    )

    config = JointTimeRefinementConfig(
        maximum_rounds=args.maximum_rounds,
        local_max_iterations=args.local_max_iterations,
        scan_points=args.scan_points,
        hard_bisection_steps=args.hard_bisection_steps,
    )
    evaluation_path = output / "refinement_evaluations.jsonl"
    evaluation_file = evaluation_path.open("w", encoding="utf-8")

    def on_evaluation(row):
        evaluation_file.write(json.dumps(_jsonable(row), ensure_ascii=False) + "\n")
        evaluation_file.flush()

    refined = refine_joint_feasible_time(
        base,
        safety,
        seed_x,
        optimizer_config,
        lambda trajectory: dynamic_audit(trajectory, optimizer_config),
        lambda trajectory: safety_audit(trajectory, track),
        config,
        on_evaluation=on_evaluation,
    )
    evaluation_file.close()
    final_forward = base.forward(refined.x)
    plot_route(
        output / "figures" / "refined_route.png",
        final_forward.trajectory,
        track,
        final_forward.traversal_times,
        trajectory_label="joint refined trajectory",
    )

    result = {
        "method": "joint_feasibility_preserving_time_refinement",
        "protocol": {
            "refinement": asdict(config),
            "safety": asdict(safety_config),
            "acceptance": "soft feasibility screen followed by independent 1 ms C++ dynamics and sphere-frame audits",
            "evidence": "dense numerical sampling; not a continuous-domain certificate",
        },
        "scene": scene_record(track),
        "seed": {
            "source": args.seed_candidates.resolve(),
            "candidate_id": seed_row["id"],
            "x": seed_x,
            "flight_time": float(seed_row["flight_time"]),
            "dynamic_audit": seed_dynamic,
            "safety_audit": seed_safety,
        },
        "refined": {
            "x": refined.x,
            "flight_time": refined.flight_time,
            "dynamic_audit": refined.dynamic_audit,
            "safety_audit": refined.safety_audit,
        },
        "rounds": refined.rounds,
        "evaluation_count": len(refined.evaluations),
        "wall_seconds": time.perf_counter() - started,
    }
    _write_json(output / "result.json", result)
    _write_json(output / "scene.json", scene_record(track))
    _write_json(output / "protocol.json", result["protocol"])
    _write_report(output / "REPORT.md", result)
    print(
        f"refined T={refined.flight_time:.9f}s wall={result['wall_seconds']:.3f}s",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
