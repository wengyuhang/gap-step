#!/usr/bin/env python3
"""Run Feasibility-Guided CEM and assemble the seven-window comparison."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np

from nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.search import (
    CEMConfig, feasible_rank, geometry_complete, local_cem_search, proposal_key,
)
from nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.multi_window import (
    DEFAULT_TEMPLATES, load_template,
)
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import screen_candidate
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations

from .experiment import build_seven_track, collision_audit


METHOD = "Feasibility-Guided CEM"


def evaluate(objective, scenario, config, x, metadata, index):
    started = time.perf_counter()
    row = dict(metadata, id=index, x=np.asarray(x, dtype=float))
    try:
        forward = objective.forward(row["x"])
        row["flight_time"] = float(forward.trajectory.total_time)
        row["screen"] = screen_candidate(forward, scenario, config)
    except (ValueError, RuntimeError, FloatingPointError, OverflowError, np.linalg.LinAlgError) as exc:
        row["flight_time"] = 1e300
        row["screen"] = {"passed": False, "reason": "numerical_failure", "error": str(exc)}
    row["proposal_key"] = proposal_key(row["screen"], row["flight_time"])
    row["screen_seconds"] = time.perf_counter() - started
    return row


def nonzero_polar_seed(x, temporal_dimension):
    """Move exact SC-map centers by a numerically negligible deterministic radius."""
    values = np.asarray(x, dtype=float).copy()
    d = values[temporal_dimension:].reshape(-1, 2)
    for i, block in enumerate(d):
        if np.linalg.norm(block) <= 1e-12:
            angle = 2 * np.pi * (i + 1) / (len(d) + 1)
            block[:] = 1e-6 * np.asarray((np.cos(angle), np.sin(angle)))
    return values


def write_report(path, rows):
    lines = ["# 七窗口混合赛道实验结果", "",
             "|方法|飞行时间 (s)|动力学约束|碰撞约束|",
             "|---|---:|---:|---:|"]
    for row in rows:
        flight_time = "—" if row["flight_time"] is None else f"{row['flight_time']:.9f}"
        lines.append(
            f"|{row['method']}|{flight_time}|"
            f"{'满足' if row['dynamics_pass'] else '不满足'}|"
            f"{'满足' if row['collision_pass'] else '不满足'}|"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def phase_aliases(window, phase, lower, upper):
    values = []
    for cycle in range(-100, 101):
        instant = (phase - window.theta0 + 2 * np.pi * cycle) / window.omega
        if lower <= instant <= upper:
            values.append(float(instant))
    return sorted(values)


def mixed_phase_front(objective, scenario, config, sc_forward, sc_x, templates):
    """Seed both repeated U windows by validated phase/D pairs."""
    n_k = len(scenario.windows) + 1
    base_d = np.asarray(sc_x[n_k:], dtype=float).reshape(-1, 2)
    rows = []
    index = 0
    for scale in np.linspace(1.10, 1.70, 16):
        middle = np.asarray(sc_forward.durations[1:6]) * scale
        final_duration = float(sc_forward.durations[7] * scale)
        nominal_first = float(sc_forward.durations[0] * scale)
        for first_template_index, first_template in enumerate(templates):
            first_times = phase_aliases(
                scenario.windows[0], first_template["phase"],
                max(0.8, nominal_first - 0.55), nominal_first + 0.55,
            )
            for first_time in first_times:
                sixth_arrival = first_time + float(np.sum(middle))
                for last_template_index, last_template in enumerate(templates):
                    last_times = phase_aliases(
                        scenario.windows[6], last_template["phase"],
                        sixth_arrival + 0.60, sixth_arrival + 1.75,
                    )
                    for last_time in last_times:
                        durations = np.r_[first_time, middle, last_time - sixth_arrival, final_duration]
                        d = base_d.copy()
                        d[0] = first_template["d"]
                        d[6] = last_template["d"]
                        x = np.r_[k_from_durations(durations), d.ravel()]
                        row = evaluate(
                            objective, scenario, config, x,
                            {"stage": "mixed_u_phase_front", "scale": float(scale),
                             "first_template": first_template_index,
                             "last_template": last_template_index,
                             "durations": durations}, index,
                        )
                        rows.append(row)
                        index += 1
    return rows


def phase_preserving_slow_front(objective, scenario, config, seed_x):
    """Slow a seven-window geometry seed while preserving both U crossing phases."""
    forward = objective.forward(seed_x)
    durations0 = np.asarray(forward.durations, dtype=float)
    n_k = len(scenario.windows) + 1
    d = np.asarray(seed_x[n_k:], dtype=float)
    safe_phases = [
        (scenario.windows[i].theta0 + scenario.windows[i].omega * forward.crossing_times[i]) % (2 * np.pi)
        for i in (0, 6)
    ]
    rows = []
    for index, scale in enumerate(np.linspace(1.0, 1.9, 181)):
        target_first = float(forward.crossing_times[0] * scale)
        first_options = phase_aliases(
            scenario.windows[0], safe_phases[0],
            max(0.6, target_first - 0.4), target_first + 0.4,
        )
        if not first_options:
            continue
        first_time = min(first_options, key=lambda value: abs(value - target_first))
        middle = durations0[1:6] * scale
        sixth_arrival = first_time + float(np.sum(middle))
        target_last = float(forward.crossing_times[6] * scale)
        last_options = phase_aliases(
            scenario.windows[6], safe_phases[1],
            sixth_arrival + 0.45, sixth_arrival + 2.4,
        )
        if not last_options:
            continue
        last_time = min(last_options, key=lambda value: abs(value - target_last))
        durations = np.r_[first_time, middle, last_time - sixth_arrival, durations0[7] * scale]
        x = np.r_[k_from_durations(durations), d]
        rows.append(evaluate(
            objective, scenario, config, x,
            {"stage": "phase_preserving_slow_front", "scale": float(scale),
             "source_u_phases": safe_phases, "durations": durations}, index,
        ))
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-result", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=37)
    parser.add_argument("--population", type=int, default=256)
    parser.add_argument("--maximum-rounds", type=int, default=12)
    parser.add_argument("--u-template-result", type=Path, nargs=2, default=DEFAULT_TEMPLATES)
    parser.add_argument("--resume-candidates", type=Path)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    baseline = json.loads(args.baseline_result.read_text(encoding="utf-8"))
    if baseline["scene"] != "seven_mixed_open_spin" or not baseline["fixed_collision_requirement_met"]:
        raise ValueError("baseline is not the frozen seven-window Fixed-WP collision case")

    print("Rebuilding frozen seven-window scene", flush=True)
    scenario, config, _, _ = build_seven_track()
    objective = MultiWindowObjective(scenario, config)
    old_rows = baseline["rows"]
    sc_row = next(row for row in old_rows if row["method"] == "SC-DynaTOGT")
    sc_x = np.asarray(sc_row["decision_vector"], dtype=float)
    sc_forward = objective.forward(sc_x)
    if abs(sc_forward.trajectory.total_time - sc_row["flight_time"]) > 1e-9:
        raise ValueError("SC baseline replay mismatch")
    n_k = len(scenario.windows) + 1
    sc_x = nonzero_polar_seed(sc_x, n_k)

    rows = []
    templates = [load_template(path) for path in args.u_template_result]
    with (output / "candidates.jsonl").open("x", encoding="utf-8") as log:
        if args.resume_candidates:
            previous = [json.loads(line) for line in args.resume_candidates.open(encoding="utf-8")]
            dynamics_rows = [row for row in previous if row["screen"]["reason"].startswith("dynamics_")]
            if not dynamics_rows:
                raise ValueError("resume candidates contain no geometry-complete dynamics seed")
            seed_row = min(dynamics_rows,
                           key=lambda row: row["screen"]["dynamics"].get("max_velocity", np.inf))
            print(f"Phase-preserving slow front from candidate {seed_row['id']}", flush=True)
            rows = phase_preserving_slow_front(
                objective, scenario, config, np.asarray(seed_row["x"], dtype=float)
            )
            front_description = {
                "type": "phase-preserving slow front",
                "source": str(args.resume_candidates.resolve()),
                "source_candidate": seed_row["id"],
                "scales": [1.0, 1.9, 181],
            }
        else:
            print("Enumerating validated U phase aliases in windows 1 and 7", flush=True)
            rows = mixed_phase_front(objective, scenario, config, sc_forward, sc_x, templates)
            front_description = {
                "type": "validated U phase aliases for windows 1 and 7",
                "scales": [1.10, 1.70, 16],
            }
        for row in rows:
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
        front_ranked = feasible_rank(rows)
        if front_ranked:
            center = np.asarray(front_ranked[0]["x"])
        else:
            geometric = [row for row in rows if geometry_complete(row, len(scenario.windows))]
            center = np.asarray((min(geometric,
                key=lambda r: r["screen"].get("dynamics", {}).get("max_velocity", np.inf))
                if geometric else max(rows, key=lambda r: tuple(r["proposal_key"]))) ["x"])
        cem_config = CEMConfig(
            seed=args.seed, population=args.population,
            elite=max(8, args.population // 8), memory=max(4, args.population // 16),
            maximum_rounds=args.maximum_rounds, post_feasible_rounds=1,
            independent_time_std=0.07, common_time_std=0.10,
            angle_std=0.08, log_radius_std=0.20,
        )
        print(f"CEM center T={objective.forward(center).trajectory.total_time:.6f}; "
              f"front feasible={len(front_ranked)}", flush=True)

        def record(row):
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
            if row["id"] % 128 == 0:
                print(f"candidate {row['id']}: {row['screen']['reason']}", flush=True)

        cem_rows, _, rounds, _ = local_cem_search(
            objective, center, scenario, config, cem_config,
            start_id=len(rows), on_record=record,
        )
        rows.extend(cem_rows)

    ranked = feasible_rank(rows)
    selected = None
    final_audits = []
    for row in ranked:
        forward = objective.forward(row["x"])
        print(f"Cuboid audit candidate {row['id']}, T={row['flight_time']:.6f}", flush=True)
        audit = collision_audit(scenario, forward, config)
        final_audits.append({"id": row["id"], "flight_time": row["flight_time"], "collision": audit})
        if audit["passed"]:
            selected = row
            np.savez_compressed(output / "selected_trajectory.npz", x=row["x"],
                                coefficients=forward.trajectory.coefficients,
                                durations=forward.durations, crossing_times=forward.crossing_times,
                                local_points=forward.local_points)
            break

    if selected:
        new_row = {"method": METHOD, "flight_time": selected["flight_time"],
                   "decision_vector": selected["x"], "candidate_id": selected["id"],
                   "stage": selected["stage"], "dynamics": selected["screen"]["dynamics"],
                   "dynamics_pass": True, "collision_pass": True,
                   "intermediate_screen": selected["screen"]}
    else:
        new_row = {"method": METHOD, "flight_time": None, "dynamics_pass": False,
                   "collision_pass": False, "status": "NO_FEASIBLE_CANDIDATE_FOUND"}
    comparison_rows = [*old_rows, new_row]
    rejection = Counter(row["screen"]["reason"] for row in rows if not row["screen"]["passed"])
    result = {"scene": scenario.name, "baseline_result": str(args.baseline_result.resolve()),
              "protocol": {"front": front_description,
                           "u_templates": templates, "cem": asdict(cem_config),
                           "selection": "flight time only among all hard-screen passes; cuboid pass required"},
              "rows": comparison_rows, "candidate_count": len(rows),
              "hard_screen_feasible": len(ranked), "rejection_counts": rejection,
              "cem_rounds": rounds, "final_audits": final_audits,
              "total_seconds": time.perf_counter() - started,
              "evidence": "sampled nominal-model validation; not continuous certification"}
    write_json(output / "result.json", result)
    write_report(output / "REPORT.md", comparison_rows)
    print(json.dumps(jsonable(result), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
