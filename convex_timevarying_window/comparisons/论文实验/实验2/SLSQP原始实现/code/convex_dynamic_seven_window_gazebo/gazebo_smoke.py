#!/usr/bin/env python3
"""Start an isolated Harmonic server and verify all gates and plugin motion."""

from __future__ import annotations

import json
from pathlib import Path
import re
import subprocess
import time


HERE = Path(__file__).resolve().parent
IMAGE = "ghcr.io/j-rivero/gazebo:harmonic-full"
CONTAINER = "convex_seven_dynamic_smoke"
PARTITION = "convex_seven_dynamic_smoke_partition"
WORLD = "convex_seven_dynamic_physics.sdf"


def run(*args, check=True, timeout=30):
    return subprocess.run(args, check=check, text=True, capture_output=True, timeout=timeout)


def model_pose(name: str) -> list[float]:
    result = run("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
                 "gz", "model", "-m", name, "-p")
    match = re.search(r"Pose.*?\n\s*\[([^]]+)\]\s*\n\s*\[([^]]+)\]", result.stdout, re.S)
    if not match:
        raise RuntimeError(f"could not parse {name} pose: {result.stdout}")
    return [float(value) for group in match.groups() for value in group.split()]


def main() -> None:
    run("docker", "rm", "-f", CONTAINER, check=False)
    try:
        run("docker", "run", "-d", "--name", CONTAINER, "--network", "host",
            "-v", f"{HERE}:/workspace:ro", "-w", "/workspace",
            "-e", f"GZ_PARTITION={PARTITION}",
            "-e", "GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/build", IMAGE,
            "gz", "sim", "-s", "-r", "-v", "3", WORLD)
        names = []
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline:
            listing = run("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
                          "gz", "model", "--list", check=False)
            names = re.findall(r"gate_\d\d_[A-Za-z0-9_]+", listing.stdout)
            if len(set(names)) == 7:
                break
            time.sleep(0.5)
        if len(set(names)) != 7:
            raise RuntimeError(f"seven gates did not appear: {listing.stdout}")
        name = "gate_01_W1_rectangle"
        first = model_pose(name)
        time.sleep(0.8)
        second = model_pose(name)
        maximum_change = max(abs(a - b) for a, b in zip(first, second))
        if maximum_change < 1e-3:
            raise RuntimeError("motion plugin did not change the gate pose")
        logs = run("docker", "logs", CONTAINER, check=False).stdout
        fatal = [line for line in logs.splitlines()
                 if re.search(r"failed to load.*PeriodicGateMotion|segmentation fault|fatal", line, re.I)]
        if fatal:
            raise RuntimeError("Gazebo fatal log lines: " + " | ".join(fatal[-5:]))
        result = {"passed": True, "world": WORLD, "gate_count": 7,
                  "motion_probe": name, "maximum_pose_change": maximum_change,
                  "fatal_log_lines": fatal}
        (HERE / "gazebo_smoke.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        run("docker", "rm", "-f", CONTAINER, check=False)


if __name__ == "__main__":
    main()

