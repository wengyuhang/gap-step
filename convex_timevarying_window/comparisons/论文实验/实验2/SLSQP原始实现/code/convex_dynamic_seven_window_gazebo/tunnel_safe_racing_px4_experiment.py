#!/usr/bin/env python3
"""Dynamic gate-tunnel racing controller for PX4/Gazebo x500.

The TOGT solution contributes its optimized gate-local crossing directions.
Between gates the controller flies at racing speed toward a short approach
point.  Inside a four-metre gate tunnel, normal progress and in-plane error are
controlled separately; after a valid plane crossing the same tunnel is held
until the complete x500 is one metre clear of the frame.

Safety is evaluated by physical Gazebo contacts and an online x500-sphere
margin.  The latter is sampled feedback, not a continuous certificate.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
_TELEOP_SITE = next((HERE / ".runtime/teleop-venv/lib").glob("python*/site-packages"))
sys.path.insert(0, str(_TELEOP_SITE))
from pymavlink import mavutil

from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import (
    CONTAINER, CONTACT_TOPICS, MOTION_START_S, PARTITION,
    ClockReader, StallWatchdog, receive, set_parameter,
)
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    enu_to_ned, parse_contacts, send_gcs_heartbeat, send_setpoint,
)
from convex_timevarying_window.online_safe_mppi.experiment import Course, passage


RESULTS = HERE / "results" / "px4_tunnel_safe_racing"
DEFAULT_REFERENCE = HERE / "trajectories" / "our_method_x500_accepted_100hz.npz"
X500_FRAME_RADIUS = 0.385944720
PARAMETERS = {
    "MPC_XY_VEL_MAX": 14.0, "MPC_XY_CRUISE": 10.0,
    "MPC_Z_VEL_MAX_UP": 7.0, "MPC_Z_VEL_MAX_DN": 7.0,
    "MPC_ACC_HOR": 10.0, "MPC_ACC_HOR_MAX": 12.0,
    "MPC_ACC_UP_MAX": 9.0, "MPC_ACC_DOWN_MAX": 9.0,
    "MPC_JERK_AUTO": 35.0, "MPC_JERK_MAX": 45.0,
    "MPC_TILTMAX_AIR": 42.0,
}


def unit(vector: np.ndarray) -> np.ndarray:
    return vector / max(float(np.linalg.norm(vector)), 1e-9)


def interp(reference, phase: float) -> np.ndarray:
    return np.array([np.interp(phase, reference["time"], reference["position_enu"][:, i])
                     for i in range(3)])


def crossing_locals(reference, course: Course, scale: float) -> list[np.ndarray]:
    output = []
    for index, instant in enumerate(reference["traversal_times"]):
        position = interp(reference, float(instant))
        center, rotation, _, _ = course.pose(index, float(instant))
        local = rotation.T @ (position - center)
        local[:2] *= scale
        local[2] = 0.0
        output.append(local)
    return output


def incoming_sign(course: Course, gate_index: int, instant: float) -> float:
    center, rotation, _, _ = course.pose(gate_index, instant)
    source = course.start if gate_index == 0 else course.pose(gate_index - 1, instant)[0]
    return float(np.sign(np.dot(source - center, rotation[:, 2])) or 1.0)


def all_frame_clearance(course: Course, position: np.ndarray, instant: float) -> float:
    minimum = math.inf
    for window in course.windows:
        planar, normal = window.world_to_local(position, instant)
        boundary = float(window.aperture.boundary_distance(planar))
        minimum = min(minimum, math.hypot(normal, boundary) - X500_FRAME_RADIUS)
    return minimum


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--control-frequency", type=float, default=100.0)
    parser.add_argument("--max-flight-time", type=float, default=55.0)
    parser.add_argument("--race-speed", type=float, default=8.0)
    parser.add_argument("--tunnel-speed", type=float, default=5.0)
    parser.add_argument("--approach-distance", type=float, default=7.0)
    parser.add_argument("--exit-distance", type=float, default=1.05)
    parser.add_argument("--crossing-offset-scale", type=float, default=0.25)
    args = parser.parse_args(argv)
    reference = np.load(args.reference.resolve())
    run_dir = RESULTS / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True)
    for name, value in PARAMETERS.items(): set_parameter(name, value)
    course = Course(torch.device("cpu"))
    local_points = crossing_locals(reference, course, args.crossing_offset_scale)

    link = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=254,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
    if link.wait_heartbeat(timeout=12) is None: raise RuntimeError("PX4 heartbeat not received")
    clock = ClockReader()
    deadline = time.monotonic() + 8.0
    while clock.value is None and time.monotonic() < deadline: time.sleep(0.02)
    if clock.value is None or clock.value >= MOTION_START_S - 12.0:
        clock.close(); raise RuntimeError(f"not enough preparation time: {clock.value}")
    local = None
    deadline = time.monotonic() + 5.0
    while local is None and time.monotonic() < deadline:
        local = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    if local is None: clock.close(); raise RuntimeError("LOCAL_POSITION_NED not received")
    origin_enu = np.array([-16.0 - local.y, 4.0 - local.x, -5.76 + local.z])
    start_ned = enu_to_ned(course.start, position=True, origin_enu=origin_enu)

    contact_processes = []; contact_paths = []
    for index, topic in enumerate(CONTACT_TOPICS, 1):
        path = run_dir / f"gate_{index:02d}_contacts.pbtxt"
        output = path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            ("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
             "gz", "topic", "-e", "-t", topic), stdout=output,
            stderr=subprocess.DEVNULL, text=True)
        contact_processes.append((process, output)); contact_paths.append(path)

    telemetry = (run_dir / "telemetry.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(telemetry)
    writer.writerow(("sim_time_s", "course_time_s", "east_m", "north_m", "up_m",
                     "ve_mps", "vn_mps", "vu_mps", "gate_index", "mode",
                     "signed_distance_m", "lateral_error_m", "aperture_margin_m",
                     "all_frame_clearance_m", "target_e", "target_n", "target_u",
                     "target_speed_mps", "cycle_ms"))
    latest = local; last_heartbeat = 0.0; watchdog = StallWatchdog()
    gate_index = 0; crossed_current = False; tunnel_committed = False
    alignment_streak = 0; crossings = []
    previous_position = course.start.copy(); previous_time = 0.0
    cycle_times = []; margins = []; finished = False; failure = None
    try:
        for _ in range(120):
            send_setpoint(link, start_ned, yaw=0.0); send_gcs_heartbeat(link); time.sleep(0.01)
        link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
        armed = False; last_arm = 0.0; arm_deadline = time.monotonic() + 15.0
        while not armed and time.monotonic() < arm_deadline:
            send_setpoint(link, start_ned, yaw=0.0)
            now_wall = time.monotonic()
            if now_wall - last_arm > 1.0:
                link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 1, 0, 0, 0, 0, 0, 0); last_arm = now_wall
            message = link.recv_match(blocking=False)
            if message is not None:
                if message.get_type() == "HEARTBEAT":
                    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif message.get_type() == "LOCAL_POSITION_NED": latest = message
            time.sleep(0.01)
        if not armed: raise RuntimeError("PX4 did not arm")
        while clock.value is not None and clock.value < MOTION_START_S:
            send_setpoint(link, start_ned, yaw=0.0); latest = receive(link, latest)
            if time.monotonic() - last_heartbeat > 0.4:
                send_gcs_heartbeat(link); last_heartbeat = time.monotonic()
            time.sleep(0.002)
        period = 1.0 / args.control_frequency; next_control = MOTION_START_S
        while clock.value is not None and clock.value <= MOTION_START_S + args.max_flight_time:
            if watchdog.stalled(clock.value): failure = "gazebo_clock_stalled"; break
            latest = receive(link, latest)
            if clock.value + 1e-9 < next_control: time.sleep(0.0004); continue
            started = time.perf_counter(); course_time = clock.value - MOTION_START_S
            position = np.array([latest.y, latest.x, -latest.z]) + origin_enu
            velocity = np.array([latest.vy, latest.vx, -latest.vz])
            mode = "finish"; signed = lateral_error = aperture_margin = math.inf
            if gate_index < len(course.windows):
                center, rotation, center_rate, _ = course.pose(gate_index, course_time)
                sign = incoming_sign(course, gate_index, course_time)
                cross_point = center + rotation @ local_points[gate_index]
                normal = rotation[:, 2]
                signed = sign * float(normal @ (position - center))
                planar_position = (rotation.T @ (position - center))[:2]
                lateral_vector = rotation[:, :2] @ (local_points[gate_index][:2] - planar_position)
                lateral_error = float(np.linalg.norm(lateral_vector))
                lateral_velocity = velocity - normal * float(normal @ velocity)
                aperture_margin = (float(course.windows[gate_index].aperture.boundary_distance(
                    planar_position)) - X500_FRAME_RADIUS)
                if not crossed_current:
                    valid, cross_time, clearance = passage(
                        course, gate_index, previous_position, position,
                        previous_time, course_time, sign)
                    if valid:
                        crossings.append({"gate": gate_index + 1, "time_s": cross_time,
                                          "sphere_clearance_m": clearance
                                          - (X500_FRAME_RADIUS - 0.37942273679894306)})
                        crossed_current = True
                if crossed_current:
                    target = cross_point - args.exit_distance * sign * normal
                    desired_speed = min(4.8, args.tunnel_speed)
                    mode = "departure_tunnel"
                    if signed < -0.90:
                        gate_index += 1; crossed_current = False
                        tunnel_committed = False; alignment_streak = 0
                elif signed > args.approach_distance + 0.35:
                    target = cross_point + args.approach_distance * sign * normal
                    distance = float(np.linalg.norm(target - position))
                    desired_speed = min(args.race_speed, max(2.0, 1.8 * distance))
                    mode = "free_flight"
                else:
                    if signed < -0.05 and not tunnel_committed:
                        # An invalid high-inertia plane crossing is not allowed
                        # to turn into a bypass.  Return to the incoming side,
                        # re-centre, and create a new valid positive-to-negative
                        # crossing event.
                        target = cross_point + 1.25 * sign * normal
                        normal_speed = -1.8
                        mode = "invalid_crossing_recovery"
                    elif not tunnel_committed:
                        # Settle on a two-metre incoming alignment plane.  The
                        # commit latch prevents mode chatter once both lateral
                        # position and lateral speed have converged.
                        target = cross_point + 2.0 * sign * normal
                        normal_speed = 0.0
                        mode = "gate_alignment_hold"
                        if lateral_error < 0.16 and np.linalg.norm(lateral_velocity) < 0.55 \
                                and 0.6 < signed < 3.2:
                            alignment_streak += 1
                        else:
                            alignment_streak = 0
                        if alignment_streak >= 8:
                            tunnel_committed = True
                    else:
                        target = cross_point - args.exit_distance * sign * normal
                        normal_speed = min(4.0, args.tunnel_speed)
                        mode = "gate_tunnel_committed"
                    if mode == "gate_alignment_hold":
                        desired_velocity = center_rate
                    else:
                        tangent_velocity = 1.8 * lateral_vector
                        tangent_velocity *= min(
                            1.0, 2.0 / max(np.linalg.norm(tangent_velocity), 1e-9))
                        desired_velocity = (-sign * normal_speed * normal
                                            + tangent_velocity + center_rate)
                    desired_speed = float(np.linalg.norm(desired_velocity))
            else:
                target = course.goal
                distance = float(np.linalg.norm(target - position))
                desired_speed = min(args.race_speed, max(0.0, 1.6 * distance))
                if distance < 0.55 and np.linalg.norm(velocity) < 1.0:
                    finished = True; break

            if mode not in ("gate_tunnel_committed", "gate_alignment_hold",
                             "invalid_crossing_recovery"):
                desired_velocity = unit(target - position) * desired_speed
            frame_margin = all_frame_clearance(course, position, course_time)
            # Current-state emergency braking never masquerades as a safe
            # nominal command; it is recorded and slows the vehicle in place.
            if frame_margin < 0.06 and mode not in ("gate_tunnel", "departure_tunnel"):
                target = position.copy(); desired_velocity = np.zeros(3); mode = "emergency_hold"
            send_setpoint(link,
                          enu_to_ned(target, position=True, origin_enu=origin_enu),
                          enu_to_ned(desired_velocity, position=False, origin_enu=origin_enu),
                          np.zeros(3), yaw=0.0)
            if time.monotonic() - last_heartbeat > 0.4:
                send_gcs_heartbeat(link); last_heartbeat = time.monotonic()
            cycle_ms = (time.perf_counter() - started) * 1000.0
            writer.writerow((clock.value, course_time, *position, *velocity, gate_index + 1,
                             mode, signed, lateral_error, aperture_margin, frame_margin,
                             *target, np.linalg.norm(desired_velocity), cycle_ms))
            telemetry.flush(); cycle_times.append(cycle_ms); margins.append(frame_margin)
            previous_position = position.copy(); previous_time = course_time
            next_control = max(next_control + period, clock.value + 0.55 * period)
        if not finished and failure is None: failure = "timeout_or_incomplete_order"
        for _ in range(100):
            send_setpoint(link, enu_to_ned(course.goal, position=True, origin_enu=origin_enu), yaw=0.0)
            send_gcs_heartbeat(link); time.sleep(0.01)
    finally:
        telemetry.close(); clock.close()
        for process, output in contact_processes:
            process.terminate()
            try: process.wait(timeout=2)
            except subprocess.TimeoutExpired: process.kill()
            output.close()
    contacts = [parse_contacts(path) for path in contact_paths]
    collision = any(item["contact_pair_count"] for item in contacts)
    success = bool(finished and len(crossings) == 7 and not collision and failure is None)
    times = np.asarray(cycle_times)
    result = {
        "status": "SUCCESS" if success else "FAILED", "failure": failure,
        "controller": "TOGT crossing directions + dynamic normal/lateral gate tunnel",
        "reference_time_s": float(reference["time"][-1]), "flight_time_s": previous_time,
        "finished": finished, "gates_crossed": len(crossings), "crossings": crossings,
        "gazebo_contact_collision": collision, "contacts": contacts,
        "minimum_sampled_x500_sphere_frame_clearance_m": float(min(margins)),
        "online_cycle_ms": {"mean": float(times.mean()), "p95": float(np.percentile(times, 95)),
                            "maximum": float(times.max()),
                            "over_100ms": int(np.count_nonzero(times > 100.0))},
        "config": {"reference": str(args.reference.resolve()),
                   "control_frequency": args.control_frequency,
                   "max_flight_time": args.max_flight_time,
                   "race_speed": args.race_speed,
                   "tunnel_speed": args.tunnel_speed,
                   "approach_distance": args.approach_distance,
                   "exit_distance": args.exit_distance,
                   "crossing_offset_scale": args.crossing_offset_scale},
        "evidence": "PX4 SITL x500 and Gazebo sleeve-frame contacts; not continuous certification",
    }
    (run_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                                          encoding="utf-8")
    print(json.dumps({"run_dir": str(run_dir), **result}, ensure_ascii=False, indent=2))
    return 0 if success else 2


if __name__ == "__main__":
    raise SystemExit(main())
