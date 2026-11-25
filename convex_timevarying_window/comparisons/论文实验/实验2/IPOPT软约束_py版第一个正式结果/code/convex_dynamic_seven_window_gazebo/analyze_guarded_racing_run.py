#!/usr/bin/env python3
"""Audit ordered gate passages for a guarded PX4 racing run."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from convex_timevarying_window.online_safe_mppi.experiment import Course
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import X500_FRAME_RADIUS


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    result = json.loads((run_dir / "result.json").read_text(encoding="utf-8"))
    telemetry = pd.read_csv(run_dir / "px4_local_position.csv")
    telemetry = telemetry[telemetry.reference_time_s >= 0.0]
    times = telemetry.reference_time_s.to_numpy(dtype=float)
    origin = np.asarray(result["origin_enu_m"], dtype=float)
    positions = np.column_stack((telemetry.actual_e_m, telemetry.actual_n_m,
                                 -telemetry.actual_d_m)) + origin
    course = Course(torch.device("cpu"))
    ordered = []
    search_start = 0
    per_gate_minimum = []
    for gate_index, window in enumerate(course.windows):
        local = []
        plane = []
        clearance = []
        for position, instant in zip(positions, times):
            q, z = window.world_to_local(position, float(instant))
            local.append(q)
            plane.append(z)
            clearance.append(
                np.hypot(float(window.aperture.boundary_distance(q)), z)
                - X500_FRAME_RADIUS)
        local = np.asarray(local)
        plane = np.asarray(plane)
        clearance = np.asarray(clearance)
        minimum_index = int(np.argmin(clearance))
        per_gate_minimum.append({
            "gate": gate_index + 1,
            "minimum_sampled_sphere_clearance_m": float(clearance[minimum_index]),
            "time_s": float(times[minimum_index]),
        })
        selected = None
        for sample in range(search_start, len(times) - 1):
            if plane[sample] * plane[sample + 1] > 0.0:
                continue
            alpha = abs(plane[sample]) / (
                abs(plane[sample]) + abs(plane[sample + 1]) + 1.0e-12)
            q = (1.0 - alpha) * local[sample] + alpha * local[sample + 1]
            if not window.aperture.contains(q):
                continue
            crossing_time = (1.0 - alpha) * times[sample] + alpha * times[sample + 1]
            boundary = float(window.aperture.boundary_distance(q))
            selected = {
                "gate": gate_index + 1,
                "time_s": float(crossing_time),
                "local_xy_m": q.tolist(),
                "sphere_clearance_at_plane_m": boundary - X500_FRAME_RADIUS,
                "inside_physical_aperture": True,
            }
            search_start = sample + 1
            break
        ordered.append(selected)
    contacts_zero = not result["collision_detected"]
    completed = result.get("actual_course_time_s") is not None
    passed = all(item is not None for item in ordered) and contacts_zero and completed
    audit = {
        "passed": passed,
        "ordered_passages_passed": all(item is not None for item in ordered),
        "gazebo_contacts_zero": contacts_zero,
        "strict_goal_completion_passed": completed,
        "actual_course_time_s": result.get("actual_course_time_s"),
        "ordered_passages": ordered,
        "per_gate_minimum_sampled_sphere_clearance": per_gate_minimum,
        "vehicle_sphere_radius_m": X500_FRAME_RADIUS,
        "evidence": (
            "linear interpolation of PX4 telemetry plane sign changes plus "
            "Gazebo contact sensors; sampled numerical evidence, not a continuous certificate"
        ),
    }
    output = run_dir / "racing_analysis.json"
    output.write_text(json.dumps(audit, ensure_ascii=False, indent=2) + "\n",
                      encoding="utf-8")
    print(json.dumps(audit, ensure_ascii=False, indent=2))
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
