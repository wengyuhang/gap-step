#!/usr/bin/env python3
"""Freeze the current nominal TOGT trajectory into a standalone PX4 reference."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


HERE = Path(__file__).resolve().parent
REPO = HERE.parent
SOURCE = REPO / "convex_timevarying_window/togt/results/seven_convex_togt_margin_1p1_body_diameter_20260910/result.json"
OUT_DIR = HERE / "trajectories"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--time-scale", type=float, default=1.0)
    args = parser.parse_args(argv)
    if args.time_scale < 1.0:
        raise ValueError("time scale must be at least one")
    sys.path.insert(0, str(REPO))
    from convex_timevarying_window.togt.experiment import build_track
    from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective
    from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations

    result = json.loads(SOURCE.read_text(encoding="utf-8"))
    track, config = build_track()
    objective = NativeJointTOGTObjective(track, config)
    decision = np.asarray(result["decision_vector"], dtype=float)
    if args.time_scale != 1.0:
        decision[:8] = k_from_durations(
            np.asarray(result["durations"], dtype=float) * args.time_scale
        )
    forward = objective.forward(decision)
    duration = float(forward.trajectory.total_time)
    count = int(np.ceil(duration / 0.01)) + 1
    times = np.linspace(0.0, duration, count)
    position = np.real(forward.trajectory.evaluate(times, 0))
    velocity = np.real(forward.trajectory.evaluate(times, 1))
    acceleration = np.real(forward.trajectory.evaluate(times, 2))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    suffix = "baseline" if args.time_scale == 1.0 else f"timescale_{args.time_scale:g}".replace(".", "p")
    output = OUT_DIR / f"togt_{suffix}_100hz.npz"
    metadata_path = OUT_DIR / f"togt_{suffix}_100hz.json"
    np.savez_compressed(
        output,
        time=times,
        position_enu=position,
        velocity_enu=velocity,
        acceleration_enu=acceleration,
        traversal_times=np.asarray(forward.traversal_times),
        waypoints_enu=np.asarray(forward.waypoints),
    )
    metadata = {
        "name": f"nominal_togt_margin_1p1_body_diameter_{suffix}",
        "time_scale": args.time_scale,
        "construction": "original D with scaled durations; moving-window waypoints reevaluated at scaled crossing times",
        "source_result": str(SOURCE.relative_to(REPO)),
        "source_result_sha256": sha256(SOURCE),
        "reference_file": output.name,
        "reference_sha256": sha256(output),
        "coordinate_frame": "Gazebo ENU",
        "sample_count": count,
        "nominal_sample_period_s": 0.01,
        "flight_time_s": duration,
        "maximum_speed_mps": float(np.max(np.linalg.norm(velocity, axis=1))),
        "start_enu": position[0].tolist(),
        "goal_enu": position[-1].tolist(),
        "traversal_times_s": forward.traversal_times.tolist(),
    }
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
