#!/usr/bin/env python3
"""Run the online VT-GP-MPPI outer loop on PX4/Gazebo x500 telemetry.

This is deliberately an adapter, not a second planner: gate-point selection,
short-horizon MPPI, event re-planning, and the final one-step verifier come
from ``online_safe_mppi.experiment`` unchanged.  PX4 receives the resulting
short P/V/A command through MAVLink Offboard mode.  Gate motion time is the
Gazebo clock minus the common 30 s preparation epoch.

The run is an empirical PX4/Gazebo experiment.  A no-contact outcome is not a
continuous-domain certificate and a contact is retained as a failure.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import replace
from datetime import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
_TELEOP_SITE = next((HERE / ".runtime/teleop-venv/lib").glob("python*/site-packages"))
sys.path.insert(0, str(_TELEOP_SITE))
from pymavlink import mavutil

from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    StallWatchdog,
    enu_to_ned,
    parse_contacts,
    send_gcs_heartbeat,
    send_setpoint,
)
from convex_timevarying_window.online_safe_mppi.experiment import (
    BODY_RADIUS,
    Config,
    Course,
    GatePlan,
    GatePointPlanner,
    LocalSafeMPPI,
    commitment_slack,
    local_barrier_filter,
    ordered_plane_filter,
    passage,
    plan_point_at,
    select_verified_command,
    workspace_barrier_filter,
)


CONTAINER = "convex_seven_togt_experiment"
PARTITION = "convex_seven_togt_experiment_partition"
WORLD = "convex_seven_dynamic_px4_togt"
MOTION_START_S = 30.0
X500_FRAME_RADIUS = 0.385944720
RESULTS = HERE / "results" / "px4_online_safe_mppi"
CONTACT_TOPICS = tuple(
    f"/world/{WORLD}/model/gate_{index:02d}_W{index}_{shape}/link/frame/sensor/frame_contact/contact"
    for index, shape in enumerate(
        ("rectangle", "circle", "pentagon", "circle", "hexagon", "circle", "rectangle"),
        start=1,
    )
)
PARAMETERS = {
    "MPC_XY_VEL_MAX": 6.0, "MPC_XY_CRUISE": 4.0,
    "MPC_Z_VEL_MAX_UP": 4.0, "MPC_Z_VEL_MAX_DN": 3.0,
    "MPC_ACC_HOR": 6.0, "MPC_ACC_HOR_MAX": 6.0,
    "MPC_ACC_UP_MAX": 5.0, "MPC_ACC_DOWN_MAX": 4.0,
    "MPC_JERK_AUTO": 20.0, "MPC_JERK_MAX": 20.0,
    "MPC_TILTMAX_AIR": 35.0,
}


class ClockReader:
    """Read this adapter's world clock (the imported NMPC reader names another world)."""
    def __init__(self) -> None:
        self.value = None
        self.process = subprocess.Popen(
            ("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
             "gz", "topic", "-e", "-t", f"/world/{WORLD}/clock"),
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self) -> None:
        section = None
        seconds = 0
        assert self.process.stdout is not None
        for raw in self.process.stdout:
            line = raw.strip()
            if line in ("system {", "real {", "sim {"):
                section = line[:-2]
            elif section == "sim" and line.startswith("sec:"):
                seconds = int(line.split(":", 1)[1])
            elif section == "sim" and line.startswith("nsec:"):
                self.value = seconds + int(line.split(":", 1)[1]) * 1e-9
            elif line == "}":
                section = None

    def close(self) -> None:
        self.process.terminate()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()


def docker(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(("docker", "exec", CONTAINER, *args), check=check,
                          text=True, capture_output=True)


def set_parameter(name: str, value: float) -> None:
    result = docker("/opt/px4-gazebo/bin/px4-param", "set", name, str(value), check=False)
    if result.returncode:
        raise RuntimeError(f"PX4 parameter {name} failed: {result.stdout} {result.stderr}")


def receive(link, latest):
    """Drain MAVLink without blocking and retain the newest local state."""
    for _ in range(60):
        message = link.recv_match(blocking=False)
        if message is None:
            break
        if message.get_type() == "LOCAL_POSITION_NED":
            latest = message
    return latest


def state_from_local(local, origin_enu: np.ndarray, acceleration: np.ndarray) -> np.ndarray:
    position = np.array([local.y, local.x, -local.z], dtype=float) + origin_enu
    velocity = np.array([local.vy, local.vx, -local.vz], dtype=float)
    return np.r_[position, velocity, acceleration, np.zeros(3)]


def plan_after_pass(course, cfg, planner, state, now, gate_index, old_plan):
    """Mirror the original controller's short, frame-clearing departure mode."""
    _, rotation, _, _ = course.pose(gate_index - 1, now)
    departure = (plan_point_at(course, old_plan, now)
                 - cfg.departure_distance * old_plan.incoming_sign * rotation[:, 2])
    if gate_index == len(course.windows):
        return GatePlan(gate_index - 1, now, now + 1.0, departure, departure, departure,
                        np.zeros(2), False, old_plan.incoming_sign, "finish", 0.0, math.inf), True
    plan = planner.plan(state, now, gate_index, "gate_pass", math.inf)
    plan.approach_point = departure.copy()
    plan.point = departure.copy()
    plan.exit_point = departure.copy()
    plan.dynamic_target = False
    return plan, True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--cruise-speed", type=float, default=3.5)
    parser.add_argument("--tracking-tube", type=float, default=0.15)
    parser.add_argument("--max-flight-time", type=float, default=90.0)
    parser.add_argument("--control-period", type=float, default=0.05)
    args = parser.parse_args(argv)
    if args.control_period < 0.04:
        parser.error("control period must be at least 0.04 s")
    if args.tracking_tube < 0:
        parser.error("tracking tube must be non-negative")

    run_dir = RESULTS / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True)
    for name, value in PARAMETERS.items():
        set_parameter(name, value)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    # The added term exactly closes the online model's sphere-radius gap to x500.
    cfg = Config(controller_dt=args.control_period, cruise_speed=args.cruise_speed,
                 max_speed=4.0, max_acceleration=4.0, max_jerk=8.0,
                 max_snap=80.0, acceleration_response_time=0.30,
                 safety_margin=args.tracking_tube + X500_FRAME_RADIUS - BODY_RADIUS,
                 max_flight_time=args.max_flight_time)
    course = Course(device)
    planner = GatePointPlanner(course, cfg)
    mppi = LocalSafeMPPI(course, cfg, seed=1000 + args.seed)

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
        raise RuntimeError(f"not enough setup time before moving gates: {clock.value}")
    local = None
    deadline = time.monotonic() + 5.0
    while local is None and time.monotonic() < deadline:
        local = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    if local is None:
        clock.close()
        raise RuntimeError("LOCAL_POSITION_NED not received")
    origin_enu = np.array([-16.0 - local.y, 4.0 - local.x, -5.76 + local.z], dtype=float)
    start_ned = enu_to_ned(course.start, position=True, origin_enu=origin_enu)

    contact_processes = []
    contact_paths = []
    for index, topic in enumerate(CONTACT_TOPICS, 1):
        path = run_dir / f"gate_{index:02d}_contacts.pbtxt"
        stream = path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            ("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
             "gz", "topic", "-e", "-t", topic), stdout=stream,
            stderr=subprocess.DEVNULL, text=True,
        )
        contact_processes.append((process, stream))
        contact_paths.append(path)

    telemetry_path = run_dir / "telemetry.csv"
    stream = telemetry_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(stream)
    writer.writerow(("sim_time_s", "course_time_s", "east_m", "north_m", "up_m",
                     "ve_mps", "vn_mps", "vu_mps", "gate_index", "safe_ratio",
                     "plan_ms", "mppi_ms", "cycle_ms", "command_ax", "command_ay",
                     "command_az", "p_ref_e", "p_ref_n", "p_ref_u", "event"))
    rows = []
    crossings = []
    cycle_ms = []
    barrier_count = ordered_count = workspace_count = shield_count = 0
    failure = None
    finished = False
    latest = local
    acceleration = np.zeros(3)
    last_measured_velocity = np.zeros(3)
    last_measured_time = None
    last_heartbeat = 0.0
    watchdog = StallWatchdog()
    try:
        # PX4 requires an established setpoint stream before entering Offboard.
        for _ in range(120):
            send_setpoint(link, start_ned, yaw=0.0)
            send_gcs_heartbeat(link)
            time.sleep(0.01)
        link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
        armed = False
        arm_deadline = time.monotonic() + 15.0
        last_arm = 0.0
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
                elif message.get_type() == "LOCAL_POSITION_NED":
                    latest = message
            time.sleep(0.01)
        if not armed:
            raise RuntimeError("PX4 did not arm")
        while clock.value is not None and clock.value < MOTION_START_S:
            send_setpoint(link, start_ned, yaw=0.0)
            if time.monotonic() - last_heartbeat > 0.4:
                send_gcs_heartbeat(link); last_heartbeat = time.monotonic()
            latest = receive(link, latest)
            time.sleep(0.002)
        if clock.value is None:
            raise RuntimeError("Gazebo clock ended during preparation")

        # Warm-up has no physical effect and is excluded from reported timing.
        warm_state = state_from_local(latest, origin_enu, acceleration)
        plan = planner.plan(warm_state, 0.0, 0, "initial", math.inf)
        mppi.command(warm_state, 0.0, plan)
        mppi.latencies_ms.clear(); mppi.safe_ratios.clear(); mppi.no_safe_count = 0
        mppi.relevant_gate_counts.clear(); mppi.controls.zero_()
        gate_index = 0
        departure_mode = False
        departure_until = 0.0
        previous_safe_ratio = 1.0
        previous_state = warm_state
        previous_time = 0.0
        next_control = MOTION_START_S

        while clock.value is not None and clock.value <= MOTION_START_S + args.max_flight_time:
            if watchdog.stalled(clock.value):
                failure = "gazebo_clock_stalled"
                break
            latest = receive(link, latest)
            sim_time = clock.value
            if sim_time + 1e-9 < next_control:
                # Keep the last valid command alive between MPPI updates.
                time.sleep(0.0005)
                continue
            course_time = max(0.0, sim_time - MOTION_START_S)
            measured_velocity = np.array([latest.vy, latest.vx, -latest.vz], dtype=float)
            if last_measured_time is not None and course_time > last_measured_time + 1e-4:
                raw_acceleration = (measured_velocity - last_measured_velocity) / (course_time - last_measured_time)
                raw_acceleration *= min(1.0, cfg.max_acceleration / max(np.linalg.norm(raw_acceleration), 1e-9))
                acceleration = 0.75 * acceleration + 0.25 * raw_acceleration
            state = state_from_local(latest, origin_enu, acceleration)
            event = "hold"
            started = time.perf_counter()
            if gate_index < len(course.windows):
                if departure_mode and (course_time >= departure_until or np.linalg.norm(state[:3] - plan.point) < 0.45):
                    plan = planner.plan(state, course_time, gate_index, "departure_clear", math.inf,
                                        locked_local=plan.point_local)
                    departure_mode = False; event = "departure_clear"
                elif not departure_mode:
                    slack = commitment_slack(course, cfg, state, course_time, plan)
                    if course_time - plan.made_at >= cfg.max_plan_age:
                        plan = planner.plan(state, course_time, gate_index, "max_dwell", slack,
                                            locked_local=plan.point_local)
                        event = "max_dwell"
                    elif previous_safe_ratio < 0.05:
                        plan = planner.plan(state, course_time, gate_index, "rollout_viability", slack)
                        event = "rollout_viability"
            command, safe_ratio = mppi.command(state, course_time, plan)
            command, intervened, _, _, _ = local_barrier_filter(course, cfg, state, course_time, command)
            barrier_count += int(intervened)
            command, intervened, _ = ordered_plane_filter(
                course, cfg, state, course_time, command, plan,
                enabled=(gate_index < len(course.windows) and not departure_mode))
            ordered_count += int(intervened)
            command, intervened = workspace_barrier_filter(cfg, state, command)
            workspace_count += int(intervened)
            command, shielded = select_verified_command(
                course, cfg, state, course_time, command, plan,
                departure_mode=(departure_mode or gate_index == len(course.windows)))
            shield_count += int(shielded)
            elapsed = (time.perf_counter() - started) * 1000.0
            cycle_ms.append(elapsed)
            if command is None:
                failure = "no_verified_one_step_control"
                break
            # P/V/A form is the one 50 ms segment which the x500 PX4 position
            # controller should execute; it is recomputed from real telemetry.
            p_ref = state[:3] + state[3:6] * cfg.controller_dt + 0.5 * command * cfg.controller_dt**2
            v_ref = state[3:6] + command * cfg.controller_dt
            send_setpoint(link,
                          enu_to_ned(p_ref, position=True, origin_enu=origin_enu),
                          enu_to_ned(v_ref, position=False, origin_enu=origin_enu),
                          enu_to_ned(command, position=False, origin_enu=origin_enu), yaw=0.0)
            if time.monotonic() - last_heartbeat > 0.4:
                send_gcs_heartbeat(link); last_heartbeat = time.monotonic()
            writer.writerow((sim_time, course_time, *state[:3], *state[3:6], gate_index + 1,
                             safe_ratio, plan.solve_ms, mppi.latencies_ms[-1], elapsed,
                             *command, *p_ref, event))
            stream.flush()
            rows.append((sim_time, course_time, state.copy(), command.copy(), safe_ratio, elapsed))

            if gate_index < len(course.windows):
                crossed, cross_time, clearance = passage(
                    course, gate_index, previous_state[:3], state[:3], previous_time, course_time,
                    plan.incoming_sign)
                if crossed:
                    crossings.append({"gate": gate_index + 1, "time_s": cross_time,
                                      "center_sphere_clearance_m": clearance})
                    gate_index += 1
                    plan, departure_mode = plan_after_pass(course, cfg, planner, state, course_time,
                                                            gate_index, plan)
                    departure_until = course_time + 1.60
                    event = "gate_pass"
            if gate_index == len(course.windows):
                if np.linalg.norm(state[:3] - course.goal) < 0.5 and np.linalg.norm(state[3:6]) < 0.7:
                    finished = True
                    break
                if np.linalg.norm(state[:3] - plan.point) < 0.55:
                    plan = replace(plan, approach_point=course.goal.copy(), point=course.goal.copy(),
                                   exit_point=course.goal.copy(), made_at=course_time)
            previous_safe_ratio = safe_ratio
            previous_state = state
            previous_time = course_time
            last_measured_velocity = measured_velocity
            last_measured_time = course_time
            next_control = max(next_control + cfg.controller_dt, sim_time + 0.60 * cfg.controller_dt)
        if not finished and failure is None:
            failure = "timeout_or_incomplete_order"
        for _ in range(100):
            send_setpoint(link, enu_to_ned(course.goal, position=True, origin_enu=origin_enu), yaw=0.0)
            send_gcs_heartbeat(link)
            time.sleep(0.01)
    finally:
        stream.close()
        clock.close()
        for process, contact_stream in contact_processes:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
            contact_stream.close()

    contacts = [parse_contacts(path) for path in contact_paths]
    contact = any(item["contact_pair_count"] for item in contacts)
    success = bool(finished and len(crossings) == 7 and not contact and failure is None)
    def timing(values):
        a = np.asarray(values, dtype=float)
        return {"samples": int(len(a)), "mean_ms": float(a.mean()) if len(a) else None,
                "p95_ms": float(np.percentile(a, 95)) if len(a) else None,
                "maximum_ms": float(a.max()) if len(a) else None,
                "over_100ms": int(np.count_nonzero(a > 100.0))}
    result = {
        "status": "SUCCESS" if success else "FAILED",
        "controller": "VT-GP-MPPI outer loop + PX4 position/velocity/acceleration Offboard inner loop",
        "course": "seven_convex_periodic_3d_closed", "vehicle": "PX4 Gazebo official x500",
        "motion_time": "Gazebo simulation time - 30 s preparation epoch",
        "config": {"control_period_s": cfg.controller_dt, "rollouts": cfg.rollouts,
                   "horizon_s": cfg.rollout_steps * cfg.controller_dt,
                   "cruise_speed_mps": cfg.cruise_speed, "tracking_tube_m": args.tracking_tube,
                   "x500_radius_m": X500_FRAME_RADIUS},
        "finished": finished, "failure": failure, "gates_crossed": len(crossings),
        "crossings": crossings, "gazebo_contact_collision": contact, "contacts": contacts,
        "mppi_timing": timing(mppi.latencies_ms), "full_outer_cycle_timing": timing(cycle_ms),
        "planner_max_ms": float(max(planner.times_ms)) if planner.times_ms else None,
        "barrier_interventions": barrier_count, "ordered_filter_interventions": ordered_count,
        "workspace_filter_interventions": workspace_count, "final_shield_interventions": shield_count,
        "no_safe_rollout_cycles": mppi.no_safe_count, "telemetry_samples": len(rows),
        "evidence": "PX4 SITL x500 + moving Gazebo sleeve-frame contact sensors; not a continuous safety certificate",
    }
    (run_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"run_dir": str(run_dir), **result}, ensure_ascii=False, indent=2))
    return 0 if success else 2


if __name__ == "__main__":
    raise SystemExit(main())
