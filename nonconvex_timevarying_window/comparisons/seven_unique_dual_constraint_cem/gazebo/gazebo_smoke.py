#!/usr/bin/env python3
"""Start Gazebo Harmonic in Docker and verify the seven spinning joints."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import time


HERE = Path(__file__).resolve().parent
IMAGE = "ghcr.io/j-rivero/gazebo:harmonic-full"
CONTAINER = "seven_unique_gz_smoke"
DEFAULT_WORLD = "seven_unique_high_fidelity.sdf"
OMEGAS = (5.0, -4.0, 4.5, -5.0, 5.5, -15.0, 18.0)


def run(*args, check=True, timeout=30):
    return subprocess.run(args, check=check, text=True, capture_output=True, timeout=timeout)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", choices=(DEFAULT_WORLD, "seven_unique_race_preview.sdf",
                                             "seven_unique_physics.sdf"),
                        default=DEFAULT_WORLD)
    args = parser.parse_args()
    world_name = args.world
    run("docker", "rm", "-f", CONTAINER, check=False)
    try:
        simulator_args = ["gz", "sim", "-s", "-r"]
        simulator_args.extend(("-v", "3", world_name))
        started = run(
            "docker", "run", "-d", "--name", CONTAINER, "--network", "host",
            "-v", f"{HERE}:/workspace:ro", "-w", "/workspace", IMAGE,
            *simulator_args,
        )
        if not started.stdout.strip():
            raise RuntimeError("Gazebo container did not return an id")
        deadline = time.monotonic() + 45.0
        topics = ""
        while time.monotonic() < deadline:
            listing = run("docker", "exec", CONTAINER, "gz", "topic", "-l", check=False)
            topics = listing.stdout
            if "/seven_unique/gate_07/joint_state" in topics:
                break
            status = run("docker", "inspect", "-f", "{{.State.Running}}", CONTAINER, check=False)
            if status.stdout.strip() != "true":
                raise RuntimeError("Gazebo server exited before publishing joint states")
            time.sleep(1.0)
        else:
            raise RuntimeError("Gazebo joint-state topics did not appear within 45 seconds")

        states = []
        for index, omega in enumerate(OMEGAS, start=1):
            topic = f"/seven_unique/gate_{index:02d}/joint_state"
            require_topic = topic in topics.splitlines()
            if not require_topic:
                raise RuntimeError(f"missing topic {topic}")
            sample = run("docker", "exec", CONTAINER, "gz", "topic", "-e", "-n", "1",
                         "-t", topic, timeout=15)
            velocities = [float(value) for value in re.findall(r"velocity:\s*([-+0-9.eE]+)", sample.stdout)]
            if not velocities:
                raise RuntimeError(f"no velocity in {topic}: {sample.stdout}")
            measured = velocities[-1]
            if abs(measured - omega) > 0.02:
                raise RuntimeError(f"{topic} velocity {measured} differs from {omega}")
            states.append({"gate": index, "target_rad_s": omega, "measured_rad_s": measured})

        logs = run("docker", "logs", CONTAINER, check=False).stdout
        fatal = [line for line in logs.splitlines()
                 if re.search(r"\b(fatal|segmentation fault|failed to load)\b", line, re.I)]
        if fatal:
            raise RuntimeError("Gazebo fatal log lines: " + " | ".join(fatal[-5:]))
        result = {"passed": True, "image": IMAGE, "world": world_name,
                  "joint_states": states, "fatal_log_lines": fatal}
        output = ({DEFAULT_WORLD: "gazebo_smoke.json",
                   "seven_unique_race_preview.sdf": "gazebo_smoke_preview.json",
                   "seven_unique_physics.sdf": "gazebo_smoke_physics.json"}[world_name])
        (HERE / output).write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        run("docker", "rm", "-f", CONTAINER, check=False)


if __name__ == "__main__":
    main()
