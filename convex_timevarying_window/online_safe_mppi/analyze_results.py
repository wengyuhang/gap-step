#!/usr/bin/env python3
"""Create the paired summary and compact figure used by RESULTS.md."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import wilcoxon


def load_runs(folder: Path, prefix: str):
    records = []
    for path in sorted(folder.glob(f"{prefix}_seed*.json"),
                       key=lambda p: int(p.stem.rsplit("seed", 1)[1])):
        record = json.loads(path.read_text())
        trajectory = np.load(path.with_suffix(".npz"))
        record["maximum_jerk_mps3"] = float(
            np.linalg.norm(trajectory["jerk"], axis=1).max())
        record["maximum_snap_mps4"] = float(
            np.linalg.norm(trajectory["snap"], axis=1).max())
        records.append(record)
    return records


def validate_paired_records(proposed, center):
    proposed_seeds=[record["seed"] for record in proposed]
    center_seeds=[record["seed"] for record in center]
    if not proposed or proposed_seeds!=center_seeds:
        raise ValueError("paired seed sets are empty or do not match")


def common_successful_pairs(proposed,center):
    """Return only pairs for which a lap time exists for both methods."""
    validate_paired_records(proposed,center)
    return [(p,c) for p,c in zip(proposed,center) if p["success"] and c["success"]]


def method_summary(records):
    successful=[record for record in records if record["success"]]
    return {
        "runs": len(records),
        "successes": len(successful),
        "mean_flight_time_success_s": (
            float(np.mean([r["flight_time_s"] for r in successful]))
            if successful else None),
        "minimum_dense_clearance_m": min(r["minimum_dense_clearance_m"] for r in records),
        "maximum_speed_mps": max(r["maximum_speed_mps"] for r in records),
        "maximum_acceleration_mps2": max(r["maximum_acceleration_mps2"] for r in records),
        "maximum_jerk_mps3": max(r["maximum_jerk_mps3"] for r in records),
        "maximum_snap_mps4": max(r["maximum_snap_mps4"] for r in records),
        "minimum_rotor_thrust_n": min(r["minimum_rotor_thrust_n"] for r in records),
        "maximum_rotor_thrust_n": max(r["maximum_rotor_thrust_n"] for r in records),
        "maximum_body_rate_rps": max(r["maximum_body_rate_rps"] for r in records),
        "minimum_height_m": min(r["minimum_height_m"] for r in records),
        "worst_mppi_p99_ms": max(r["mppi_latency_ms"]["p99"] for r in records),
        "maximum_mppi_ms": max(r["mppi_latency_ms"]["maximum"] for r in records),
        "maximum_planner_ms": max(r["planner_latency_ms"]["maximum"] for r in records),
        "maximum_complete_cycle_ms": max(r["cycle_latency_ms"]["maximum"] for r in records),
        "complete_cycles_over_100ms": sum(r["cycle_over_100ms"] for r in records),
        "mppi_cycles_over_100ms": sum(r["mppi_over_100ms"] for r in records),
        "no_safe_rollout_cycles": sum(r["no_safe_rollout_cycles"] for r in records),
        "final_shield_interventions": sum(r["final_shield_interventions"] for r in records),
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("proposed_dir", type=Path)
    parser.add_argument("center_dir", type=Path)
    parser.add_argument("outdir", type=Path)
    args = parser.parse_args(argv)
    proposed = load_runs(args.proposed_dir, "proposed")
    center = load_runs(args.center_dir, "center_periodic")
    validate_paired_records(proposed,center)

    common=common_successful_pairs(proposed,center)
    if not common:
        raise ValueError("flight-time comparison has no commonly successful pairs")
    p_common,c_common=zip(*common)
    p_time = np.asarray([r["flight_time_s"] for r in p_common])
    c_time = np.asarray([r["flight_time_s"] for r in c_common])
    difference = p_time - c_time
    rng = np.random.default_rng(20260917)
    bootstrap = np.asarray([
        rng.choice(difference, len(difference), replace=True).mean()
        for _ in range(200_000)
    ])
    excluded=[{
        "seed":p["seed"],"proposed_success":p["success"],
        "center_success":c["success"],
    } for p,c in zip(proposed,center) if not (p["success"] and c["success"])]
    paired = {
        "definition": "proposed minus center; negative is faster",
        "common_successful_pairs":len(common),
        "excluded_from_lap_time":excluded,
        "mean_difference_s": float(difference.mean()),
        "relative_mean_improvement_percent": float(-difference.mean()/c_time.mean()*100.0),
        "median_difference_s": float(np.median(difference)),
        "proposed_wins": int(np.count_nonzero(difference < 0.0)),
        "bootstrap_95_percent_ci_s": [float(x) for x in np.quantile(bootstrap, [0.025, 0.975])],
        "wilcoxon_one_sided_p": float(wilcoxon(
            difference,alternative="less",zero_method="wilcox").pvalue),
    }
    success_table={
        "both_success":sum(p["success"] and c["success"] for p,c in zip(proposed,center)),
        "proposed_only":sum(p["success"] and not c["success"] for p,c in zip(proposed,center)),
        "center_only":sum(not p["success"] and c["success"] for p,c in zip(proposed,center)),
        "both_failed":sum(not p["success"] and not c["success"] for p,c in zip(proposed,center)),
    }
    summary = {
        "proposed": method_summary(proposed),
        "center": method_summary(center),
        "paired_success":success_table,
        "paired": paired,
        "audit": "5 ms dense numerical sphere-to-frame audit; not a continuous certificate",
    }
    args.outdir.mkdir(parents=True, exist_ok=True)
    (args.outdir / "comparison.json").write_text(json.dumps(summary, indent=2) + "\n")

    with (args.outdir / "paired_results.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["seed","proposed_success","center_success",
                         "proposed_s", "center_s", "proposed_minus_center_s",
                         "proposed_clearance_m", "center_clearance_m"])
        for p,c in zip(proposed,center):
            both=p["success"] and c["success"]
            delta=p["flight_time_s"]-c["flight_time_s"] if both else ""
            writer.writerow([p["seed"],p["success"],c["success"],
                             p["flight_time_s"] if p["success"] else "",
                             c["flight_time_s"] if c["success"] else "",delta,
                             p["minimum_dense_clearance_m"], c["minimum_dense_clearance_m"]])

    seeds = np.asarray([r["seed"] for r in p_common])
    fig, axes = plt.subplots(2, 1, figsize=(8.0, 6.2), constrained_layout=True)
    width = 0.38
    axes[0].bar(seeds-width/2, p_time, width, label="joint gate point")
    axes[0].bar(seeds+width/2, c_time, width, label="gate center")
    axes[0].set_ylabel("flight time (s)")
    axes[0].set_xticks(seeds)
    axes[0].legend(frameon=False)
    axes[0].grid(axis="y", alpha=0.25)
    colors = np.where(difference < 0.0, "#2878B5", "#C82423")
    axes[1].bar(seeds, difference, color=colors)
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_xlabel("paired MPPI seed")
    axes[1].set_ylabel("joint - center (s)")
    axes[1].set_xticks(seeds)
    axes[1].grid(axis="y", alpha=0.25)
    fig.suptitle(
        f"Lap time on {len(common)} jointly successful seeds; "
        f"excluded failures: {len(excluded)}",fontsize=11)
    fig.savefig(args.outdir / "comparison.png", dpi=180)


if __name__ == "__main__":
    main()
