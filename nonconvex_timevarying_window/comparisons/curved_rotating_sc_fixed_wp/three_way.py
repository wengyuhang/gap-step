#!/usr/bin/env python3
"""Add Feasibility-Guided CEM to the curved-track baseline comparison."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.search import (
    CEMConfig,
    feasible_rank,
    local_cem_search,
    proposal_key,
)
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective, audit_multi
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import screen_candidate
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations

from .experiment import build_curved_track


METHOD = "Feasibility-Guided CEM"


def _evaluate_front(objective, scenario, config, x, scale, index):
    started = time.perf_counter()
    row = {"id": index, "stage": "time_scale_front", "scale": scale, "x": np.asarray(x)}
    try:
        forward = objective.forward(row["x"])
        row["flight_time"] = float(forward.trajectory.total_time)
        row["screen"] = screen_candidate(forward, scenario, config)
    except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
        row["flight_time"] = 1.0e300
        row["screen"] = {"passed": False, "reason": "numerical_failure", "error": str(exc)}
    row["proposal_key"] = proposal_key(row["screen"], row["flight_time"])
    row["screen_seconds"] = time.perf_counter() - started
    return row


def _verify_replay(objective, baseline):
    replayed = {}
    for row in baseline["rows"]:
        x = np.asarray(row["decision_vector"], dtype=float)
        forward = objective.forward(x)
        if abs(forward.trajectory.total_time - row["flight_time"]) > 1e-9:
            raise ValueError(f"baseline replay mismatch for {row['method']}")
        replayed[row["method"]] = (row, forward)
    return replayed


def _report(path, old_rows, new_row):
    rows = [*old_rows, new_row]
    lines = [
        "# 曲线三窗口三方法实验结果",
        "",
        "|方法|飞行时间 (s)|动力学约束|碰撞约束|",
        "|---|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"|{row['method']}|{row['flight_time']:.9f}|"
            f"{'满足' if row['dynamics_pass'] else '不满足'}|"
            f"{'满足' if row['collision_pass'] else '不满足'}|"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-result", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=29)
    parser.add_argument("--population", type=int, default=256)
    parser.add_argument("--maximum-rounds", type=int, default=12)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()

    baseline = json.loads(args.baseline_result.read_text(encoding="utf-8"))
    if baseline["scene"] != "curved_three_window_open_spin":
        raise ValueError("baseline scene mismatch")
    print("Rebuilding the frozen curved scene", flush=True)
    scenario, config = build_curved_track()
    objective = MultiWindowObjective(scenario, config)
    replayed = _verify_replay(objective, baseline)
    old_rows = [replayed[name][0] for name in ("Fixed-WP", "SC-DynaTOGT")]
    for row, audit_name in zip(old_rows, ("fixed_wp_audit.json", "sc_dynatogt_audit.json")):
        audit = json.loads((args.baseline_result.parent / audit_name).read_text(encoding="utf-8"))
        row["dynamics_pass"] = bool(row["intermediate_screen"].get("dynamics", {}).get("passed", False))
        row["collision_pass"] = all(
            item["audit"].get("collision_free", False)
            and item["audit"].get("solid_exterior_violating_samples", 1) == 0
            for item in audit["per_window"]
        )
    sc_x = np.asarray(replayed["SC-DynaTOGT"][0]["decision_vector"], dtype=float)
    sc_forward = replayed["SC-DynaTOGT"][1]
    n_k = len(scenario.windows) + 1

    rows = []
    log = (output / "candidates.jsonl").open("x", encoding="utf-8")
    try:
        print("Scanning the deterministic common-time front", flush=True)
        for index, scale in enumerate(np.linspace(1.0, 1.65, 40)):
            x = np.r_[k_from_durations(sc_forward.durations * scale), sc_x[n_k:]]
            row = _evaluate_front(objective, scenario, config, x, float(scale), index)
            rows.append(row)
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
        front_feasible = feasible_rank(rows)
        if front_feasible:
            center = np.asarray(front_feasible[0]["x"])
        else:
            center = np.asarray(max(rows, key=lambda row: tuple(row["proposal_key"]))["x"])
        cem_config = CEMConfig(
            seed=args.seed,
            population=args.population,
            elite=max(8, args.population // 8),
            memory=max(4, args.population // 16),
            maximum_rounds=args.maximum_rounds,
            post_feasible_rounds=1,
            independent_time_std=0.06,
            common_time_std=0.08,
            angle_std=0.08,
            log_radius_std=0.22,
        )
        print(f"Running full-covariance CEM from T={objective.forward(center).trajectory.total_time:.6f}", flush=True)

        def record(row):
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
            if row["id"] % 256 == 0:
                print(f"candidate {row['id']}: {row['screen']['reason']}", flush=True)

        cem_rows, _, round_summaries, _ = local_cem_search(
            objective,
            center,
            scenario,
            config,
            cem_config,
            start_id=len(rows),
            on_record=record,
        )
        rows.extend(cem_rows)
    finally:
        log.close()

    ranked = feasible_rank(rows)
    selected = None
    audit_records = []
    for row in ranked:
        forward = objective.forward(row["x"])
        print(f"Final cuboid audit of candidate {row['id']} T={row['flight_time']:.6f}", flush=True)
        audit = audit_multi(scenario, forward, config)
        audit_records.append({"id": row["id"], "audit": audit})
        if audit["trajectory_validation_pass"]:
            selected = row
            np.savez_compressed(
                output / "selected_trajectory.npz",
                x=row["x"], coefficients=forward.trajectory.coefficients,
                durations=forward.durations, crossing_times=forward.crossing_times,
                local_points=forward.local_points,
            )
            break

    if selected is None:
        new_row = {
            "method": METHOD,
            "flight_time": float("nan"),
            "intermediate_screen": {"passed": False, "reason": "no_final_candidate"},
            "whole_body_audit_pass": False,
            "all_hard_requirements_pass": False,
            "dynamics_pass": False,
            "collision_pass": False,
        }
    else:
        new_row = {
            "method": METHOD,
            "flight_time": selected["flight_time"],
            "decision_vector": selected["x"],
            "candidate_id": selected["id"],
            "stage": selected["stage"],
            "intermediate_screen": selected["screen"],
            "whole_body_audit_pass": True,
            "all_hard_requirements_pass": True,
            "dynamics_pass": bool(selected["screen"]["dynamics"]["passed"]),
            "collision_pass": all(
                item["audit"].get("collision_free", False)
                and item["audit"].get("solid_exterior_violating_samples", 1) == 0
                for item in audit_records[-1]["audit"]["per_window"]
            ),
        }
    counts = Counter(row["screen"]["reason"] for row in rows if not row["screen"]["passed"])
    result = {
        "scene": scenario.name,
        "baseline_result": str(args.baseline_result.resolve()),
        "protocol": {
            "time_front_scales": [1.0, 1.65, 40],
            "cem": asdict(cem_config),
            "hard_rule": "discard every failed candidate; rank flight time among all-pass candidates only",
        },
        "old_rows": old_rows,
        "new_row": new_row,
        "candidate_count": len(rows),
        "screen_feasible_count": len(ranked),
        "rejection_counts": counts,
        "round_summaries": round_summaries,
        "final_audits": audit_records,
        "total_seconds": time.perf_counter() - started,
        "evidence": "sampled nominal-model validation; not continuous certification",
    }
    write_json(output / "result.json", result)
    _report(output / "REPORT.md", old_rows, new_row)
    print(json.dumps(jsonable(result), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
