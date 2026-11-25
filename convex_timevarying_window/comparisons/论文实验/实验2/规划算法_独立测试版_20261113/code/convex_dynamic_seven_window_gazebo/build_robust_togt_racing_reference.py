#!/usr/bin/env python3
"""Build a time-scaled, gate-synchronised x500 TOGT racing reference.

The correction uses a compact C4 bump around each crossing.  It is exactly
zero outside the local crossing neighbourhood.  W1--W6 map to the live centre;
W7 retains an optimised local offset to keep its approach away from the frame.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.signal import savgol_filter
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from convex_timevarying_window.online_safe_mppi.experiment import Course


def interpolate(times, values, query):
    return np.column_stack([np.interp(query, times, values[:, axis])
                            for axis in range(values.shape[1])])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path,
                        default=HERE / "trajectories/our_method_x500_accepted_100hz.npz")
    parser.add_argument("--output", type=Path,
                        default=HERE / "trajectories/robust_togt_racing_1p25_100hz.npz")
    parser.add_argument("--time-scale", type=float, default=1.25)
    parser.add_argument("--bump-half-width", type=float, default=1.90)
    parser.add_argument("--final-bump-half-width", type=float, default=1.90)
    parser.add_argument("--final-pre-lateral-repair", type=float, default=0.0)
    parser.add_argument("--final-crossing-local-y", type=float, default=0.29)
    parser.add_argument("--repair-lead", type=float, default=0.34)
    parser.add_argument("--repair-half-width", type=float, default=0.80)
    args = parser.parse_args()
    source = np.load(args.source.resolve())
    course = Course(torch.device("cpu"))
    duration = args.time_scale * float(source["time"][-1])
    samples = int(np.ceil(duration / 0.01)) + 1
    time = np.linspace(0.0, duration, samples)
    phase = time / args.time_scale
    base_position = interpolate(source["time"], source["position_enu"], phase)
    correction = np.zeros_like(base_position)
    traversal = args.time_scale * np.asarray(source["traversal_times"], dtype=float)
    corrections = []
    for index, crossing_time in enumerate(traversal):
        half_width = (args.final_bump_half_width if index == 6
                      else args.bump_half_width)
        u = (time - crossing_time) / half_width
        inside = np.abs(u) < 1.0
        bump = np.zeros_like(time)
        bump[inside] = (1.0 - u[inside] * u[inside]) ** 5
        crossing_position = interpolate(
            time, base_position, np.asarray([crossing_time]))[0]
        crossing_center, crossing_rotation, _, _ = course.pose(
            index, float(crossing_time))
        crossing_target = crossing_center.copy()
        if index == 6:
            crossing_target += (args.final_crossing_local_y
                                * crossing_rotation[:, 1])
        crossing_shift = crossing_target - crossing_position
        maximum_warp = 0.0
        for sample_index in np.flatnonzero(inside):
            # A constant displacement preserves the TOGT segment shape and
            # therefore costs much less acceleration than pulling a whole
            # neighbourhood to the instantaneous centre.
            direction = crossing_shift
            warp = bump[sample_index] * direction
            correction[sample_index] += warp
            maximum_warp = max(maximum_warp, float(np.linalg.norm(warp)))
        corrections.append({"gate": index + 1,
                            "crossing_time_s": float(crossing_time),
                            "maximum_warp_m": maximum_warp})
    # On this frozen track the time-scaled W7 approach has one shallow pass by
    # the lower rectangle rail before the intended crossing.  A tiny local C4
    # repair in the live gate's +y direction removes that isolated contact.
    final_crossing = traversal[-1]
    repair_center = final_crossing - args.repair_lead
    repair_half_width = args.repair_half_width
    repair_u = (time - repair_center) / repair_half_width
    repair_inside = np.abs(repair_u) < 1.0
    repair_weight = np.zeros_like(time)
    repair_weight[repair_inside] = (
        1.0 - repair_u[repair_inside] * repair_u[repair_inside]) ** 5
    for sample_index in np.flatnonzero(repair_inside):
        _, live_rotation, _, _ = course.pose(6, float(time[sample_index]))
        correction[sample_index] += (
            args.final_pre_lateral_repair * repair_weight[sample_index]
            * live_rotation[:, 1])
    position = base_position + correction
    sample_step = float(time[1] - time[0])
    # One consistent differentiable fit is preferable to differentiating the
    # source's already-discretized derivatives repeatedly.  A 0.51 s window
    # retains racing curvature while suppressing fourth-derivative grid noise.
    filter_window = 51
    velocity = savgol_filter(position, filter_window, 7, deriv=1,
                             delta=sample_step, axis=0, mode="interp")
    acceleration = savgol_filter(position, filter_window, 7, deriv=2,
                                 delta=sample_step, axis=0, mode="interp")
    jerk = savgol_filter(position, filter_window, 7, deriv=3,
                         delta=sample_step, axis=0, mode="interp")
    snap = savgol_filter(position, filter_window, 7, deriv=4,
                         delta=sample_step, axis=0, mode="interp")
    waypoints = interpolate(time, position, traversal)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, time=time, position_enu=position,
                        velocity_enu=velocity, acceleration_enu=acceleration,
                        jerk_enu=jerk, snap_enu=snap,
                        traversal_times=traversal, waypoints_enu=waypoints)
    result = {
        "source": str(args.source.resolve()), "output": str(args.output.resolve()),
        "time_scale": args.time_scale, "bump_half_width_s": args.bump_half_width,
        "final_bump_half_width_s": args.final_bump_half_width,
        "final_crossing_local_y_m": args.final_crossing_local_y,
        "final_pre_lateral_repair_m": args.final_pre_lateral_repair,
        "flight_time_s": duration,
        "path_length_m": float(np.linalg.norm(np.diff(position, axis=0), axis=1).sum()),
        "maximum_speed_mps": float(np.linalg.norm(velocity, axis=1).max()),
        "maximum_acceleration_mps2": float(np.linalg.norm(acceleration, axis=1).max()),
        "maximum_jerk_mps3": float(np.linalg.norm(jerk, axis=1).max()),
        "maximum_snap_mps4": float(np.linalg.norm(snap, axis=1).max()),
        "corrections": corrections,
    }
    args.output.with_suffix(".json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
