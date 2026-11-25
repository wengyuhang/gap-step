#!/usr/bin/env python3
"""Phase-governed TOGT racing on PX4/Gazebo x500.

The x500-validated TOGT trajectory supplies the fast nominal P/V/A path.  A
scalar phase governor prevents the reference from crossing the next gate until
the measured vehicle is aligned, while a local SE(3) warp moves the nominal
gate-relative reference onto the gate pose at the current Gazebo time.  Only a
short measured-state safety prediction is evaluated online.

This is empirical PX4/Gazebo collision evidence, not continuous certification.
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
    CONTAINER, CONTACT_TOPICS, MOTION_START_S, PARTITION, WORLD,
    ClockReader, StallWatchdog, docker, receive, set_parameter,
)
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    enu_to_ned, parse_contacts, send_gcs_heartbeat, send_setpoint,
)
from convex_timevarying_window.online_safe_mppi.experiment import Course, passage


RESULTS = HERE / "results" / "px4_phase_safe_racing"
DEFAULT_REFERENCE = HERE / "trajectories" / "our_method_x500_accepted_100hz.npz"
X500_FRAME_RADIUS = 0.385944720
PARAMETERS = {
    "MPC_XY_VEL_MAX": 18.0, "MPC_XY_CRUISE": 12.0,
    "MPC_Z_VEL_MAX_UP": 8.0, "MPC_Z_VEL_MAX_DN": 8.0,
    "MPC_ACC_HOR": 12.0, "MPC_ACC_HOR_MAX": 15.0,
    "MPC_ACC_UP_MAX": 10.0, "MPC_ACC_DOWN_MAX": 10.0,
    "MPC_JERK_AUTO": 40.0, "MPC_JERK_MAX": 50.0,
    "MPC_TILTMAX_AIR": 45.0,
}


def interp(reference, name: str, phase: float) -> np.ndarray:
    source = reference[name]
    t = reference["time"]
    return np.array([np.interp(phase, t, source[:, axis]) for axis in range(3)])


def smoothstep01(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * (3.0 - 2.0 * x)


def warped_position(reference, course: Course, gate_index: int,
                    phase: float, course_time: float,
                    crossing_offset_scale: float = 0.35) -> np.ndarray:
    """Map a near-gate nominal point to the gate's live pose."""
    nominal = interp(reference, "position_enu", phase)
    if gate_index >= len(course.windows):
        return nominal
    nominal_center, nominal_rotation, _, _ = course.pose(gate_index, phase)
    live_center, live_rotation, _, _ = course.pose(gate_index, course_time)
    gate_relative = nominal_rotation.T @ (nominal - nominal_center)
    distance = float(np.linalg.norm(nominal - nominal_center))
    weight = smoothstep01((6.0 - distance) / 4.0)
    # Keep the TOGT-selected corner-cutting direction, but contract its planar
    # offset near the aperture to buy real tracking margin for the x500.
    gate_relative[:2] *= 1.0 - weight * (1.0 - crossing_offset_scale)
    mapped = live_center + live_rotation @ gate_relative
    return (1.0 - weight) * nominal + weight * mapped


def warped_pva(reference, course: Course, gate_index: int, phase: float,
               course_time: float, phase_rate: float,
               crossing_offset_scale: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Finite-difference the live warped reference; the query is analytic and cheap."""
    h = 0.015
    duration = float(reference["time"][-1])
    values = []
    for multiple in (0.0, 1.0, 2.0):
        s = min(duration, phase + multiple * h * phase_rate)
        values.append(warped_position(reference, course, gate_index, s,
                                      course_time + multiple * h,
                                      crossing_offset_scale))
    p0, p1, p2 = values
    velocity = (p1 - p0) / h
    acceleration = (p2 - 2.0 * p1 + p0) / (h * h)
    velocity *= min(1.0, 16.0 / max(np.linalg.norm(velocity), 1e-9))
    acceleration *= min(1.0, 12.0 / max(np.linalg.norm(acceleration), 1e-9))
    return p0, velocity, acceleration


def incoming_sign(course: Course, gate_index: int, instant: float) -> float:
    center, rotation, _, _ = course.pose(gate_index, instant)
    source = course.start if gate_index == 0 else course.pose(gate_index - 1, instant)[0]
    return float(np.sign(np.dot(source - center, rotation[:, 2])) or 1.0)


def frame_clearance(course: Course, position: np.ndarray, instant: float) -> float:
    minimum = math.inf
    for window in course.windows:
        planar, normal = window.world_to_local(position, instant)
        boundary = float(window.aperture.boundary_distance(planar))
        minimum = min(minimum, math.sqrt(normal * normal + boundary * boundary)
                      - X500_FRAME_RADIUS)
    return minimum


def short_prediction_clearance(course: Course, position: np.ndarray,
                               velocity: np.ndarray, instant: float) -> float:
    return min(frame_clearance(course, position + tau * velocity, instant + tau)
               for tau in np.linspace(0.0, 0.35, 8))


def phase_rate_command(phase: float, gate_phase: float,
                       predicted_clearance: float,
                       gate_index: int, gate_count: int) -> tuple[float, str]:
    if predicted_clearance < 0.08:
        return 0.0, "short_horizon_safety_hold"
    if gate_index >= gate_count:
        return 1.0, "finish"
    if phase >= gate_phase + 0.16:
        return 0.0, "gate_phase_hold"
    return 1.0, "free_flight"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--control-frequency", type=float, default=100.0)
    parser.add_argument("--max-flight-time", type=float, default=65.0)
    parser.add_argument("--gate-rate-cap", type=float, default=0.82)
    parser.add_argument("--crossing-offset-scale", type=float, default=0.35)
    args = parser.parse_args(argv)
    if args.control_frequency < 20.0:
        parser.error("control frequency must be at least 20 Hz")
    reference = np.load(args.reference.resolve())
    duration = float(reference["time"][-1])
    traversal_times = np.asarray(reference["traversal_times"], dtype=float)
    run_dir = RESULTS / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True)
    for name, value in PARAMETERS.items():
        set_parameter(name, value)
    course = Course(torch.device("cpu"))

    link = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=254,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
    if link.wait_heartbeat(timeout=12) is None:
        raise RuntimeError("PX4 heartbeat not received")
    clock = ClockReader()
    deadline = time.monotonic() + 8.0
    while clock.value is None and time.monotonic() < deadline:
        time.sleep(0.02)
    if clock.value is None or clock.value >= MOTION_START_S - 12.0:
        clock.close()
        raise RuntimeError(f"not enough preparation time: {clock.value}")
    local = None
    deadline = time.monotonic() + 5.0
    while local is None and time.monotonic() < deadline:
        local = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    if local is None:
        clock.close(); raise RuntimeError("LOCAL_POSITION_NED not received")
    origin_enu = np.array([-16.0 - local.y, 4.0 - local.x, -5.76 + local.z])
    start_ned = enu_to_ned(course.start, position=True, origin_enu=origin_enu)

    contact_processes = []
    contact_paths = []
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
    writer.writerow(("sim_time_s", "course_time_s", "phase_s", "phase_rate",
                     "east_m", "north_m", "up_m", "ve_mps", "vn_mps", "vu_mps",
                     "reference_e", "reference_n", "reference_u", "position_error_m",
                     "gate_index", "signed_distance_m", "lateral_error_m",
                     "predicted_clearance_m",
                     "mode", "cycle_ms"))
    latest = local
    last_heartbeat = 0.0
    gate_index = 0
    phase = 0.0
    crossings = []
    rates = []
    errors = []
    clearances = []
    cycle_times = []
    finished = False
    failure = None
    watchdog = StallWatchdog()
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
                                           0, 1, 0, 0, 0, 0, 0, 0)
                last_arm = now_wall
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
        if clock.value is None: raise RuntimeError("Gazebo clock ended during preparation")

        period = 1.0 / args.control_frequency
        next_control = MOTION_START_S
        previous_time = 0.0
        previous_position = course.start.copy()
        phase_rate = 0.0
        departure_gate = None
        departure_sign = None
        departure_local = None
        while clock.value is not None and clock.value <= MOTION_START_S + args.max_flight_time:
            if watchdog.stalled(clock.value): failure = "gazebo_clock_stalled"; break
            latest = receive(link, latest)
            if clock.value + 1e-9 < next_control:
                time.sleep(0.0004); continue
            started = time.perf_counter()
            course_time = max(0.0, clock.value - MOTION_START_S)
            dt = max(0.0, course_time - previous_time)
            phase = min(duration, phase + phase_rate * dt)
            position = np.array([latest.y, latest.x, -latest.z]) + origin_enu
            velocity = np.array([latest.vy, latest.vx, -latest.vz])

            if gate_index < len(course.windows):
                sign = incoming_sign(course, gate_index, course_time)
                center, rotation, _, _ = course.pose(gate_index, course_time)
                signed = sign * float(rotation[:, 2] @ (position - center))
                crossed, cross_time, crossing_clearance = passage(
                    course, gate_index, previous_position, position,
                    previous_time, course_time, sign)
                if crossed:
                    crossings.append({"gate": gate_index + 1, "time_s": cross_time,
                                      "sphere_clearance_m": crossing_clearance
                                      - (X500_FRAME_RADIUS - 0.37942273679894306)})
                    nominal_cross = interp(reference, "position_enu",
                                           float(traversal_times[gate_index]))
                    nominal_center, nominal_rotation, _, _ = course.pose(
                        gate_index, float(traversal_times[gate_index]))
                    departure_local = nominal_rotation.T @ (nominal_cross - nominal_center)
                    departure_local[:2] *= args.crossing_offset_scale
                    departure_gate = gate_index
                    departure_sign = sign
                    gate_index += 1
                    phase = max(phase, float(traversal_times[gate_index - 1]) + 0.16)
                target_gate_time = (float(traversal_times[gate_index])
                                    if gate_index < len(course.windows) else duration)
            else:
                signed = math.inf; target_gate_time = duration

            nominal_target = warped_position(
                reference, course, gate_index, phase, course_time,
                args.crossing_offset_scale)
            position_error = float(np.linalg.norm(position - nominal_target))
            predicted_clearance = short_prediction_clearance(course, position, velocity, course_time)
            if gate_index < len(course.windows):
                live_center, live_rotation, _, _ = course.pose(gate_index, course_time)
                crossing_phase = float(traversal_times[gate_index])
                nominal_cross = interp(reference, "position_enu", crossing_phase)
                nominal_center, nominal_rotation, _, _ = course.pose(gate_index, crossing_phase)
                local_cross = nominal_rotation.T @ (nominal_cross - nominal_center)
                local_cross[:2] *= args.crossing_offset_scale
                live_cross = live_center + live_rotation @ local_cross
                delta = position - live_cross
                lateral = delta - live_rotation[:, 2] * float(live_rotation[:, 2] @ delta)
                lateral_error = float(np.linalg.norm(lateral))
            else:
                lateral_error = 0.0
            gate_phase = (float(traversal_times[gate_index])
                          if gate_index < len(course.windows) else duration)
            phase_rate, mode = phase_rate_command(
                phase, gate_phase, predicted_clearance,
                gate_index, len(course.windows))
            if gate_index < len(course.windows):
                # The reference may reach 16 cm of nominal phase beyond the
                # gate to keep a non-zero crossing velocity, but cannot run on
                # to the next segment before the measured x500 has crossed.
                phase = min(phase, gate_phase + 0.16)

            if departure_gate is not None:
                old_center, old_rotation, old_center_rate, _ = course.pose(
                    departure_gate, course_time)
                old_signed = departure_sign * float(
                    old_rotation[:, 2] @ (position - old_center))
                if old_signed < -0.85:
                    departure_gate = None
                else:
                    exit_local = departure_local.copy()
                    exit_local[2] = -departure_sign * 1.05
                    target_p = old_center + old_rotation @ exit_local
                    delta_exit = target_p - position
                    target_v = delta_exit / max(np.linalg.norm(delta_exit), 1e-9) * min(
                        4.5, 2.5 * np.linalg.norm(delta_exit))
                    target_v += old_center_rate
                    target_a = np.zeros(3)
                    phase_rate = 0.0
                    mode = "departure_corridor"
            if departure_gate is None:
                target_p, target_v, target_a = warped_pva(
                    reference, course, gate_index, phase, course_time, phase_rate,
                    args.crossing_offset_scale)
            send_setpoint(link,
                          enu_to_ned(target_p, position=True, origin_enu=origin_enu),
                          enu_to_ned(target_v, position=False, origin_enu=origin_enu),
                          enu_to_ned(target_a, position=False, origin_enu=origin_enu), yaw=0.0)
            if time.monotonic() - last_heartbeat > 0.4:
                send_gcs_heartbeat(link); last_heartbeat = time.monotonic()
            cycle_ms = (time.perf_counter() - started) * 1000.0
            writer.writerow((clock.value, course_time, phase, phase_rate,
                             *position, *velocity, *target_p, position_error,
                             gate_index + 1, signed, lateral_error,
                             predicted_clearance, mode, cycle_ms))
            telemetry.flush()
            rates.append(phase_rate); errors.append(position_error)
            clearances.append(predicted_clearance); cycle_times.append(cycle_ms)
            if gate_index == len(course.windows) and phase >= duration - 0.02:
                if np.linalg.norm(position - course.goal) < 0.55 and np.linalg.norm(velocity) < 1.0:
                    finished = True; break
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
    def stats(values):
        a = np.asarray(values, dtype=float)
        return {"mean": float(a.mean()), "p95": float(np.percentile(a, 95)),
                "maximum": float(a.max())}
    result = {
        "status": "SUCCESS" if success else "FAILED", "failure": failure,
        "controller": "x500 TOGT nominal + live gate SE(3) warp + safety phase governor + PX4 PVA",
        "reference": str(args.reference.resolve()), "reference_time_s": duration,
        "finished": finished, "gates_crossed": len(crossings), "crossings": crossings,
        "flight_time_s": previous_time, "slowdown_over_reference_s": previous_time - duration,
        "gazebo_contact_collision": collision, "contacts": contacts,
        "phase_rate": stats(rates), "position_error_m": stats(errors),
        "short_prediction_clearance_m": {"minimum": float(min(clearances))},
        "online_cycle_ms": stats(cycle_times),
        "online_cycle_over_100ms": int(np.count_nonzero(np.asarray(cycle_times) > 100.0)),
        "evidence": "PX4 SITL x500 and Gazebo sleeve-frame contacts; not continuous certification",
    }
    (run_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                                          encoding="utf-8")
    print(json.dumps({"run_dir": str(run_dir), **result}, ensure_ascii=False, indent=2))
    return 0 if success else 2


if __name__ == "__main__":
    raise SystemExit(main())
