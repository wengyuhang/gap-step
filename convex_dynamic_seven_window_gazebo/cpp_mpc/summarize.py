#!/usr/bin/env python3
"""Summarize one C++ MPC run without restarting Gazebo."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np


MOTION_START_SIM_TIME_S = 30.0


def error_metrics(values: np.ndarray) -> dict:
    if values.size == 0:
        return {key: None for key in ("rms", "mean", "p95", "maximum", "final")} | {"samples": 0}
    return {
        "samples": int(values.size),
        "rms": float(np.sqrt(np.mean(values**2))),
        "mean": float(np.mean(values)),
        "p95": float(np.quantile(values, 0.95)),
        "maximum": float(np.max(values)),
        "final": float(values[-1]),
    }


def contact_summary(path: Path) -> dict:
    content = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
    pairs = re.findall(
        r'collision1\s*\{.*?name:\s*"([^"]+)".*?\}\s*'
        r'collision2\s*\{.*?name:\s*"([^"]+)"',
        content,
        flags=re.S,
    )
    stamps = [
        int(sec) + int(nsec) * 1e-9
        for sec, nsec in re.findall(
            r"stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)", content
        )
    ]
    return {
        "raw_log": path.name,
        "contact_pair_count": len(pairs),
        "first_contact_sim_time_s": min(stamps) if stamps else None,
        "collision_pairs": sorted({f"{a} <-> {b}" for a, b in pairs}),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()

    telemetry = np.genfromtxt(run_dir / "telemetry.csv", delimiter=",", names=True)
    telemetry = np.atleast_1d(telemetry)
    contacts = [contact_summary(path) for path in sorted(run_dir.glob("gate_*_contacts.pbtxt"))]
    first_contact_sim = min(
        (row["first_contact_sim_time_s"] for row in contacts if row["first_contact_sim_time_s"] is not None),
        default=None,
    )
    first_contact_reference = (
        first_contact_sim - MOTION_START_SIM_TIME_S if first_contact_sim is not None else None
    )
    pre_contact = (
        telemetry["reference_time_s"] < first_contact_reference
        if first_contact_reference is not None
        else np.ones(telemetry.size, dtype=bool)
    )

    controller_text = (run_dir / "controller.txt").read_text(encoding="utf-8", errors="replace")
    match = re.search(
        r"calls=(\d+)\s+frequency=([0-9.eE+-]+)\s+mean_solve_ms=([0-9.eE+-]+)\s+max_solve_ms=([0-9.eE+-]+)",
        controller_text,
    )
    if match is None:
        raise RuntimeError("controller.txt does not contain the expected timing summary")

    solve_ms = telemetry["solve_ms"]
    result = {
        "controller": "native C++ online MPC with serialized CasADi functions and MAVLink UDP",
        "motion_start_sim_time_s": MOTION_START_SIM_TIME_S,
        "controller_calls": int(match.group(1)),
        "achieved_frequency_hz": float(match.group(2)),
        "solve_time_ms": {
            "mean": float(np.mean(solve_ms)),
            "p95": float(np.quantile(solve_ms, 0.95)),
            "maximum": float(np.max(solve_ms)),
        },
        "tracking_error_m": error_metrics(telemetry["error_m"]),
        "pre_contact_tracking_error_m": error_metrics(telemetry["error_m"][pre_contact]),
        "first_contact_sim_time_s": first_contact_sim,
        "first_contact_reference_time_s": first_contact_reference,
        "collision_detected": any(row["contact_pair_count"] for row in contacts),
        "contacts": contacts,
    }
    (run_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
