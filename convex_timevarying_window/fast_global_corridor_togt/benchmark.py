"""Repeat the native planner and summarize its internal planning latency."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import tempfile
from pathlib import Path

import numpy as np


def parse(text: str):
    values = {}
    for line in text.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=20)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    method = Path(__file__).resolve().parent
    repo = method.parents[1]
    executable = method / "native/build/global_corridor_togt"
    parameter_dir = repo / "复现/TOGT-Planner-reproduction/source/parameters/cpc"
    track = repo / "convex_timevarying_window/togt/native/race_uzh_7g_dynamic_mixed_closed.yaml"
    env = os.environ.copy()
    env.update({
        "TOGT_MOTION_SPEED_SCALE": "2.5",
        "TOGT_INITIAL_SPEED": "10",
        "TOGT_SAFETY_WEIGHT": "10000",
        "TOGT_DYNAMICS_WEIGHT": "1",
        "TOGT_SAFETY_NODES": "8",
        "TOGT_LBFGS_PAST": "8",
        "TOGT_OUTPUT_TIME_SCALE": "1.02",
    })
    records = []
    with tempfile.TemporaryDirectory(prefix="fast_global_safe_togt_") as tmp:
        trajectory = Path(tmp) / "trajectory.csv"
        for index in range(args.repeats):
            run = subprocess.run([
                str(executable), str(parameter_dir), "cpc_setups.yaml",
                str(track), str(trajectory),
            ], env=env, check=True, text=True, capture_output=True)
            values = parse(run.stdout)
            records.append({
                "repeat": index + 1,
                "planning_s": float(values["planning_s"]),
                "optimizer_code": int(values["optimizer_code"]),
                "iterations": int(values["iterations"]),
                "evaluations": int(values["evaluations"]),
                "flight_time_s": float(values["flight_time_s"]),
            })
    times = np.asarray([record["planning_s"] for record in records])
    report = {
        "repeats": args.repeats,
        "all_optimizer_stops_successful": all(record["optimizer_code"] in (0, 1) for record in records),
        "mean_planning_ms": float(1000 * np.mean(times)),
        "median_planning_ms": float(1000 * np.median(times)),
        "p95_planning_ms": float(1000 * np.percentile(times, 95)),
        "maximum_planning_ms": float(1000 * np.max(times)),
        "records": records,
        "timing_scope": "native objective, released C++ L-BFGS, output time dilation, and final MINCO construction; excludes CSV export and independent audits",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "records"}, indent=2))


if __name__ == "__main__":
    main()
