#!/usr/bin/env python3
"""Execute the privileged hybrid teacher as direct PX4 CTBR control."""

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
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from closed_loop_deformable_window.fapp_ppo.dynamics import QuadrotorState
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    CONTACT_TOPICS,
    CONTAINER,
    MOTION_START_S,
    PARAMETERS,
    PARTITION,
    ClockReader,
    StallWatchdog,
    docker,
    enu_to_ned,
    parse_contacts,
    send_bodyrate_thrust,
    send_gcs_heartbeat,
    send_setpoint,
)
from convex_dynamic_seven_window_gazebo.togt_nmpc import (
    MAX_TOTAL_THRUST,
    collective_thrust_to_px4,
)
from convex_timevarying_window.privileged_safe_teacher import (
    DirectControlTeacher,
    PrivilegedTeacherEnv,
)
from convex_timevarying_window.privileged_safe_teacher.safety_filter import (
    PredictiveSafetyFilter,
)


RESULTS = HERE / "results" / "px4_privileged_teacher"
C_NED_ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
C_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def measured_state(local, attitude, origin_enu: np.ndarray) -> QuadrotorState:
    position = np.array([local.y, local.x, -local.z]) + origin_enu
    velocity = np.array([local.vy, local.vx, -local.vz])
    rotation_ned_frd = Rotation.from_euler(
        "ZYX", [attitude.yaw, attitude.pitch, attitude.roll]
    ).as_matrix()
    rotation_enu_flu = C_NED_ENU.T @ rotation_ned_frd @ C_FLU_FRD
    body_rate_flu = C_FLU_FRD @ np.array(
        [attitude.rollspeed, attitude.pitchspeed, attitude.yawspeed]
    )
    return QuadrotorState(position, velocity, rotation_enu_flu, body_rate_flu)


def load_teacher(path: Path, device: torch.device) -> DirectControlTeacher:
    payload = torch.load(path, map_location=device, weights_only=False)
    model = DirectControlTeacher(int(payload["observation_dim"])).to(device)
    model.load_state_dict(payload["model_state"])
    model.eval()
    return model


@torch.no_grad()
def infer(model: DirectControlTeacher, observation: np.ndarray, device: torch.device) -> np.ndarray:
    value = torch.as_tensor(observation, dtype=torch.float32, device=device).unsqueeze(0)
    return model(value).squeeze(0).cpu().numpy()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--recovery-fraction", type=float, default=0.70)
    parser.add_argument("--frequency", type=float, default=50.0)
    args = parser.parse_args()
    if not 0.0 <= args.recovery_fraction <= 1.0:
        parser.error("--recovery-fraction must be in [0,1]")
    run_dir = RESULTS / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True)
    for name, value in PARAMETERS.items():
        docker("/opt/px4-gazebo/bin/px4-param", "set", name, str(value))

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = load_teacher(args.checkpoint.resolve(), device)
    env = PrivilegedTeacherEnv(seed=0)
    safety_filter = PredictiveSafetyFilter()
    clock = ClockReader()

    teleop_site = next((HERE / ".runtime/teleop-venv/lib").glob("python*/site-packages"))
    sys.path.insert(0, str(teleop_site))
    from pymavlink import mavutil

    link = mavutil.mavlink_connection("udp:127.0.0.1:14550", source_system=254)
    heartbeat = link.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise RuntimeError("PX4 heartbeat unavailable")
    local = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if local is None:
        raise RuntimeError("LOCAL_POSITION_NED unavailable")
    origin_enu = np.asarray((-16.0 - float(local.y), 4.0 - float(local.x), -5.76 + float(local.z)))
    start_ned = enu_to_ned(env.privilege.start, position=True, origin_enu=origin_enu)
    initial_yaw = math.pi / 2.0

    contact_processes = []
    contact_paths = []
    for index, topic in enumerate(CONTACT_TOPICS, start=1):
        path = run_dir / f"gate_{index:02d}_contacts.pbtxt"
        stream = path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            ("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
             "gz", "topic", "-e", "-t", topic),
            stdout=stream,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        contact_processes.append((process, stream))
        contact_paths.append(path)

    latest_local = local
    latest_attitude = None
    rows = []
    header = (
        "sim_time_s", "control_time_s", "east_m", "north_m", "up_m",
        "ve_mps", "vn_mps", "vu_mps", "route_index", "frame_margin_m",
        "intervened", "filter_source", "inference_ms", "filter_ms",
        "thrust_action", "roll_rate_action", "pitch_rate_action", "yaw_rate_action",
    )
    telemetry = (run_dir / "telemetry.csv").open("w", newline="", encoding="utf-8")
    writer = csv.writer(telemetry)
    writer.writerow(header)
    stalled = False
    collision = False
    finished = False
    last_heartbeat = 0.0
    watchdog = StallWatchdog()
    try:
        for _ in range(120):
            send_setpoint(link, start_ned, yaw=initial_yaw)
            send_gcs_heartbeat(link)
            time.sleep(0.01)
        link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
        armed = False
        arm_deadline = time.monotonic() + 60.0
        last_arm = 0.0
        while not armed and time.monotonic() < arm_deadline:
            send_setpoint(link, start_ned, yaw=initial_yaw)
            now = time.monotonic()
            if now - last_arm > 1.0:
                link.mav.command_long_send(
                    1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                    1, 0, 0, 0, 0, 0, 0,
                )
                last_arm = now
            message = link.recv_match(blocking=False)
            if message is not None:
                if message.get_type() == "HEARTBEAT":
                    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif message.get_type() == "LOCAL_POSITION_NED":
                    latest_local = message
                elif message.get_type() == "ATTITUDE":
                    latest_attitude = message
            time.sleep(0.01)
        if not armed:
            raise RuntimeError("PX4 did not arm")
        while clock.value is not None and clock.value < MOTION_START_S:
            send_setpoint(link, start_ned, yaw=initial_yaw)
            send_gcs_heartbeat(link)
            for _ in range(20):
                message = link.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_type() == "LOCAL_POSITION_NED":
                    latest_local = message
                elif message.get_type() == "ATTITUDE":
                    latest_attitude = message
            time.sleep(0.01)
        if latest_attitude is None:
            latest_attitude = link.recv_match(type="ATTITUDE", blocking=True, timeout=3)
        if latest_attitude is None:
            raise RuntimeError("ATTITUDE unavailable")

        measured = measured_state(latest_local, latest_attitude, origin_enu)
        env.state = measured
        env.time = 0.0
        next_control = MOTION_START_S
        period = 1.0 / args.frequency
        while clock.value is not None and clock.value <= MOTION_START_S + env.maximum_time:
            if watchdog.stalled(clock.value):
                stalled = True
                break
            for _ in range(30):
                message = link.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_type() == "LOCAL_POSITION_NED":
                    latest_local = message
                elif message.get_type() == "ATTITUDE":
                    latest_attitude = message
            if clock.value + 1.0e-9 < next_control:
                time.sleep(0.0005)
                continue
            relative = max(0.0, clock.value - MOTION_START_S)
            measured = measured_state(latest_local, latest_attitude, origin_enu)
            info = env.accept_external_state(measured, relative)
            observation = env.observe()
            inference_start = time.perf_counter()
            learned = infer(model, observation, device)
            inference_ms = 1000.0 * (time.perf_counter() - inference_start)
            recovery = env.expert_action()
            active_recovery_fraction = (
                1.0 if env.route_index == len(env.privilege.gates) else args.recovery_fraction
            )
            proposed = (
                (1.0 - active_recovery_fraction) * learned
                + active_recovery_fraction * recovery
            )
            filter_start = time.perf_counter()
            decision = safety_filter.filter(env, proposed)
            filter_ms = 1000.0 * (time.perf_counter() - filter_start)
            action = decision.action
            specific_thrust = env.dynamics._specific_thrust_from_action(float(action[0]))
            thrust = float(collective_thrust_to_px4(env.quad_config.mass * specific_thrust))
            body_rate_flu = action[1:] * env.quad_config.max_body_rate
            body_rate_frd = C_FLU_FRD @ body_rate_flu
            send_bodyrate_thrust(link, body_rate_frd, thrust)
            row = (
                clock.value, relative, *measured.position, *measured.velocity,
                env.route_index, info.minimum_frame_margin, int(decision.intervened),
                decision.source, inference_ms, filter_ms, *action,
            )
            writer.writerow(row)
            telemetry.flush()
            rows.append(row)
            collision = info.collision
            finished = info.finished
            if collision or finished:
                break
            if time.monotonic() - last_heartbeat > 0.5:
                send_gcs_heartbeat(link)
                last_heartbeat = time.monotonic()
            next_control = max(next_control + period, clock.value + 0.6 * period)
        # Hold the last state briefly instead of issuing another trajectory.
        for _ in range(25):
            send_bodyrate_thrust(link, np.zeros(3), float(collective_thrust_to_px4(env.quad_config.mass * env.quad_config.gravity)))
            time.sleep(0.02)
    finally:
        telemetry.close()
        clock.close()
        for process, stream in contact_processes:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
            stream.close()

    contacts = [parse_contacts(path) for path in contact_paths]
    result = {
        "status": "SUCCESS" if finished and not collision and not any(c["contact_pair_count"] for c in contacts) else "FAILED",
        "checkpoint": str(args.checkpoint.resolve()),
        "controller": "30% learned direct CTBR + 70% sparse planner recovery + predictive whole-body safety filter",
        "time_indexed_trajectory_tracking": False,
        "control_frequency_hz": args.frequency,
        "control_steps": len(rows),
        "route_index": env.route_index,
        "finished": finished,
        "sampled_collision": collision,
        "gazebo_contact_collision": any(c["contact_pair_count"] for c in contacts),
        "gazebo_clock_stalled": stalled,
        "minimum_sampled_frame_margin_m": env.minimum_frame_margin,
        "safety_interventions": sum(int(row[10]) for row in rows),
        "mean_inference_ms": float(np.mean([row[12] for row in rows])) if rows else None,
        "mean_filter_ms": float(np.mean([row[13] for row in rows])) if rows else None,
        "contacts": contacts,
        "evidence": "PX4 SITL official x500 in Gazebo Harmonic with physical frame contact sensors",
    }
    (run_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"run_dir": str(run_dir), **result}, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "SUCCESS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
