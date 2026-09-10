#!/usr/bin/env python3
"""Track the frozen nominal TOGT trajectory through PX4 and record contacts."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import threading
import time

import numpy as np

HERE = Path(__file__).resolve().parent
_TELEOP_SITE = next((HERE / ".runtime/teleop-venv/lib").glob("python*/site-packages"))
sys.path.insert(0, str(_TELEOP_SITE))
from pymavlink import mavutil


CONTAINER = "convex_seven_togt_experiment"
PARTITION = "convex_seven_togt_experiment_partition"
WORLD = "convex_seven_dynamic_px4_togt"
MOTION_START_S = 30.0
DEFAULT_REFERENCE = HERE / "trajectories/togt_baseline_100hz.npz"
RESULTS = HERE / "results" / "px4_togt"
CONTACT_TOPICS = tuple(
    f"/world/{WORLD}/model/gate_{index:02d}_W{index}_{shape}/link/frame/sensor/frame_contact/contact"
    for index, shape in enumerate(
        ("rectangle", "circle", "pentagon", "circle", "hexagon", "circle", "rectangle"),
        start=1,
    )
)
PARAMETERS = {
    "MPC_XY_VEL_MAX": 25.0,
    "MPC_XY_CRUISE": 15.0,
    "MPC_Z_VEL_MAX_UP": 12.0,
    "MPC_Z_VEL_MAX_DN": 12.0,
    "MPC_ACC_HOR": 15.0,
    "MPC_ACC_HOR_MAX": 15.0,
    "MPC_ACC_UP_MAX": 12.0,
    "MPC_ACC_DOWN_MAX": 12.0,
    "MPC_JERK_AUTO": 50.0,
    "MPC_JERK_MAX": 50.0,
    "MPC_TILTMAX_AIR": 45.0,
}


def docker(*args: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(("docker", "exec", CONTAINER, *args), check=check,
                          text=True, capture_output=True)


class ClockReader:
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


def set_parameter(name: str, value: float) -> None:
    result = docker("/opt/px4-gazebo/bin/px4-param", "set", name, str(value), check=False)
    if result.returncode != 0:
        raise RuntimeError(f"failed to set {name}: {result.stdout} {result.stderr}")


def send_gcs_heartbeat(link) -> None:
    link.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0,
    )


def send_setpoint(link, position_ned, velocity_ned=None, acceleration_ned=None,
                  yaw: float = 0.0) -> None:
    if velocity_ned is None or acceleration_ned is None:
        velocity_ned = (0.0, 0.0, 0.0)
        acceleration_ned = (0.0, 0.0, 0.0)
        type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 2048
    else:
        type_mask = 2048
    link.mav.set_position_target_local_ned_send(
        0, 1, 1, mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
        *map(float, position_ned), *map(float, velocity_ned),
        *map(float, acceleration_ned), float(yaw), 0.0,
    )


def interpolate(reference, t: float):
    source_t = reference["time"]
    values = []
    for name in ("position_enu", "velocity_enu", "acceleration_enu"):
        source = reference[name]
        values.append(np.asarray([np.interp(t, source_t, source[:, axis]) for axis in range(3)]))
    return values


def enu_to_ned(vector: np.ndarray, *, position: bool, origin_enu: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=float)
    if position:
        value = value - origin_enu
    return np.asarray((value[1], value[0], -value[2]))


def parse_contacts(path: Path) -> dict:
    content = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
    pairs = re.findall(
        r'collision1\s*\{.*?name:\s*"([^"]+)".*?\}\s*'
        r'collision2\s*\{.*?name:\s*"([^"]+)"',
        content,
        flags=re.S,
    )
    return {
        "raw_log": path.name,
        "contact_pair_count": len(pairs),
        "collision_pairs": sorted({f"{first} <-> {second}" for first, second in pairs}),
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    args = parser.parse_args(argv)
    reference_path = args.reference.resolve()
    run_dir = RESULTS / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True)
    reference = np.load(reference_path)
    reference_duration = float(reference["time"][-1])
    for name, value in PARAMETERS.items():
        set_parameter(name, value)

    link = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=254,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER)
    heartbeat = link.wait_heartbeat(timeout=12)
    if heartbeat is None:
        raise RuntimeError("PX4 heartbeat not received on UDP 14550")

    clock = ClockReader()
    deadline = time.monotonic() + 8.0
    while clock.value is None and time.monotonic() < deadline:
        time.sleep(0.02)
    if clock.value is None or clock.value >= MOTION_START_S - 12.0:
        clock.close()
        raise RuntimeError(f"insufficient preparation time before t={MOTION_START_S}: {clock.value}")

    # The PX4 local origin coincides with the spawn point on the z=-6 m floor.
    # Using the live local altitude accounts for the x500_base 0.24 m include pose.
    local = None
    deadline = time.monotonic() + 5.0
    while local is None and time.monotonic() < deadline:
        message = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
        if message is not None:
            local = message
    if local is None:
        clock.close()
        raise RuntimeError("LOCAL_POSITION_NED was not received")
    origin_enu = np.asarray((-16.0 - float(local.y), 4.0 - float(local.x),
                             -5.76 + float(local.z)))
    start_enu = np.asarray(reference["position_enu"][0])
    start_ned = enu_to_ned(start_enu, position=True, origin_enu=origin_enu)

    contact_processes = []
    contact_paths = []
    for index, topic in enumerate(CONTACT_TOPICS, start=1):
        path = run_dir / f"gate_{index:02d}_contacts.pbtxt"
        stream = path.open("w", encoding="utf-8")
        process = subprocess.Popen(
            ("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
             "gz", "topic", "-e", "-t", topic), stdout=stream,
            stderr=subprocess.DEVNULL, text=True,
        )
        contact_processes.append((process, stream))
        contact_paths.append(path)

    rows = []
    latest_local = local
    last_heartbeat = 0.0
    next_send = time.monotonic()
    try:
        # Stream the start setpoint before switching to Offboard, as PX4 requires.
        for _ in range(120):
            send_setpoint(link, start_ned)
            send_gcs_heartbeat(link)
            time.sleep(0.01)
        link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6 << 16)
        # PX4 can report a commander before its health checks are ready. Keep
        # streaming the initial setpoint and retry arming until the heartbeat
        # confirms the armed bit, instead of silently waiting on the ground.
        armed = False
        last_arm_request = 0.0
        arm_deadline = time.monotonic() + 10.0
        while not armed and time.monotonic() < arm_deadline:
            now = time.monotonic()
            send_setpoint(link, start_ned)
            send_gcs_heartbeat(link)
            if now - last_arm_request >= 1.0:
                link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 1, 0, 0, 0, 0, 0, 0)
                last_arm_request = now
            message = link.recv_match(blocking=False)
            if message is not None:
                if message.get_type() == "HEARTBEAT":
                    armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                elif message.get_type() == "LOCAL_POSITION_NED":
                    latest_local = message
            time.sleep(0.01)
        if not armed:
            raise RuntimeError("PX4 did not confirm armed state within 10 seconds")

        # Hold the exact trajectory initial state until the common simulation epoch.
        while clock.value is not None and clock.value < MOTION_START_S:
            now = time.monotonic()
            if now >= next_send:
                send_setpoint(link, start_ned)
                next_send += 0.01
            if now - last_heartbeat >= 0.5:
                send_gcs_heartbeat(link)
                last_heartbeat = now
            message = link.recv_match(blocking=False)
            if message is not None and message.get_type() == "LOCAL_POSITION_NED":
                latest_local = message
                rows.append((clock.value, -1.0, message.x, message.y, message.z,
                             message.vx, message.vy, message.vz, math.nan, math.nan, math.nan))
            time.sleep(0.001)

        if clock.value is None:
            raise RuntimeError("Gazebo clock stream ended before trajectory start")
        preparation_error = float(np.linalg.norm(
            np.asarray((latest_local.x, latest_local.y, latest_local.z)) - start_ned
        ))
        preparation_speed = float(np.linalg.norm(
            np.asarray((latest_local.vx, latest_local.vy, latest_local.vz))
        ))
        if preparation_error > 0.75 or preparation_speed > 1.0:
            raise RuntimeError(
                f"x500 did not settle at TOGT start: error={preparation_error:.3f} m, "
                f"speed={preparation_speed:.3f} m/s"
            )
        while clock.value <= MOTION_START_S + reference_duration:
            relative_t = max(0.0, clock.value - MOTION_START_S)
            position_enu, velocity_enu, acceleration_enu = interpolate(reference, relative_t)
            position_ned = enu_to_ned(position_enu, position=True, origin_enu=origin_enu)
            velocity_ned = enu_to_ned(velocity_enu, position=False, origin_enu=origin_enu)
            acceleration_ned = enu_to_ned(acceleration_enu, position=False, origin_enu=origin_enu)
            # The TOGT flatness model uses ConstAngle yaw(0). Keep execution
            # on the identical yaw branch used for thrust/body-rate auditing.
            yaw = 0.0
            now = time.monotonic()
            if now >= next_send:
                send_setpoint(link, position_ned, velocity_ned, acceleration_ned, yaw)
                next_send = now + 0.008
            if now - last_heartbeat >= 0.5:
                send_gcs_heartbeat(link)
                last_heartbeat = now
            for _ in range(20):
                message = link.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_type() == "LOCAL_POSITION_NED":
                    latest_local = message
                    error = np.asarray((message.x, message.y, message.z)) - position_ned
                    rows.append((clock.value, relative_t,
                                 message.x, message.y, message.z,
                                 message.vx, message.vy, message.vz,
                                 position_ned[0], position_ned[1], position_ned[2],
                                 float(np.linalg.norm(error))))
            time.sleep(0.001)

        goal_ned = start_ned
        hold_end = time.monotonic() + 2.0
        while time.monotonic() < hold_end:
            send_setpoint(link, goal_ned)
            send_gcs_heartbeat(link)
            time.sleep(0.01)
    finally:
        for process, stream in contact_processes:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
            stream.close()
        clock.close()

    csv_path = run_dir / "px4_local_position.csv"
    header = ("sim_time_s", "reference_time_s", "actual_n_m", "actual_e_m", "actual_d_m",
              "actual_vn_mps", "actual_ve_mps", "actual_vd_mps", "reference_n_m",
              "reference_e_m", "reference_d_m", "position_error_m")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for row in rows:
            if len(row) == len(header):
                writer.writerow(row)

    flight_rows = [row for row in rows if len(row) == len(header) and row[1] >= 0.0]
    errors = [row[-1] for row in flight_rows if math.isfinite(row[-1])]
    contacts = [parse_contacts(path) for path in contact_paths]
    status = docker("/opt/px4-gazebo/bin/px4-commander", "status", check=False).stdout
    result = {
        "passed_execution": bool(flight_rows),
        "reference": str(reference_path.relative_to(HERE)),
        "reference_flight_time_s": reference_duration,
        "motion_start_sim_time_s": MOTION_START_S,
        "origin_enu_m": origin_enu.tolist(),
        "preparation_error_m": preparation_error,
        "preparation_speed_mps": preparation_speed,
        "px4_parameters": PARAMETERS,
        "yaw_reference_rad": 0.0,
        "telemetry_samples": len(flight_rows),
        "tracking_error_m": {
            "rms": float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            "maximum": float(np.max(errors)) if errors else None,
            "final": float(errors[-1]) if errors else None,
        },
        "contacts": contacts,
        "collision_detected": any(item["contact_pair_count"] > 0 for item in contacts),
        "commander_status": status.strip().splitlines(),
        "evidence": "PX4 SITL x500 dynamics and collision shapes in Gazebo Harmonic",
    }
    (run_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"run_dir": str(run_dir), **result}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
