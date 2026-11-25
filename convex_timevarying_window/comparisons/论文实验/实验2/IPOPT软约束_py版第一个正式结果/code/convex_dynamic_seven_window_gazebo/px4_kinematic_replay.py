#!/usr/bin/env python3
"""Run an exact-in-state Gazebo replay and preserve gate contact evidence."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import socket
import subprocess
import threading
import time

import numpy as np

HERE = Path(__file__).resolve().parent
CONTAINER = "convex_seven_replay"
PARTITION = "convex_seven_replay_partition"
WORLD = "convex_seven_dynamic_px4_replay"
MOTION_START_S = 30.0
RESULTS = HERE / "results" / "px4_kinematic_replay"
SHAPES = ("rectangle", "circle", "pentagon", "circle", "hexagon", "circle", "rectangle")


class ClockReader:
    def __init__(self) -> None:
        self.value: float | None = None
        self.process = subprocess.Popen(
            ("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
             "gz", "topic", "-e", "-t", f"/world/{WORLD}/clock"),
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self) -> None:
        seconds = 0
        section = None
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


class ContactStream:
    """Persist a contact topic and release replay at its first contact."""
    def __init__(self, topic: str, path: Path) -> None:
        self.path = path
        self.released = False
        self.stream = path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            ("docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
             "gz", "topic", "-e", "-t", topic), stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1,
        )
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self) -> None:
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.stream.write(line)
            self.stream.flush()
            if not self.released and "collision1" in line:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as release:
                    release.sendto(b"release", ("127.0.0.1", 18680))
                self.released = True

    def close(self) -> None:
        self.process.terminate()
        try:
            self.process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.process.kill()
        self.stream.close()


def parse_contacts(path: Path) -> dict:
    content = path.read_text(encoding="utf-8", errors="replace") if path.exists() else ""
    pairs = re.findall(
        r'collision1\s*\{.*?name:\s*"([^"]+)".*?\}\s*'
        r'collision2\s*\{.*?name:\s*"([^"]+)"', content, flags=re.S,
    )
    stamps = [int(sec) + int(nsec) * 1e-9 for sec, nsec in re.findall(
        r"stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)", content)]
    return {
        "raw_log": path.name,
        "contact_pair_count": len(pairs),
        "first_contact_sim_time_s": min(stamps) if stamps else None,
        "collision_pairs": sorted({f"{a} <-> {b}" for a, b in pairs}),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    args = parser.parse_args()
    reference_path = args.reference.resolve()
    reference = np.load(reference_path)
    duration = float(reference["time"][-1])
    run_dir = RESULTS / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True)
    replay_csv = run_dir / "replay_reference.csv"
    subprocess.run(("conda", "run", "--no-capture-output", "-n", "wyh", "python",
                    str(HERE / "export_replay_reference.py"), "--reference", str(reference_path),
                    "--output", str(replay_csv)), check=True)
    subprocess.run((str(HERE / "run_px4_replay_headless.sh"), str(replay_csv)), check=True)

    streams: list[ContactStream] = []
    paths: list[Path] = []
    clock = ClockReader()
    try:
        for index, shape in enumerate(SHAPES, start=1):
            path = run_dir / f"gate_{index:02d}_contacts.pbtxt"
            topic = (f"/world/{WORLD}/model/gate_{index:02d}_W{index}_{shape}/link/"
                     "frame/sensor/frame_contact/contact")
            streams.append(ContactStream(topic, path))
            paths.append(path)

        deadline = time.monotonic() + MOTION_START_S + duration + 20.0
        target = MOTION_START_S + duration
        while time.monotonic() < deadline:
            if clock.value is not None and clock.value >= target:
                break
            time.sleep(0.005)
        if clock.value is None or clock.value < target:
            raise RuntimeError(f"Gazebo did not reach replay end: {clock.value}, target={target}")
        time.sleep(0.25)  # Drain the 250 Hz contact sensors.
    finally:
        for stream in streams:
            stream.close()
        clock.close()

    contacts = [parse_contacts(path) for path in paths]
    first_contact = min((row["first_contact_sim_time_s"] for row in contacts
                         if row["first_contact_sim_time_s"] is not None), default=None)
    result = {
        "reference": str(reference_path.relative_to(HERE)),
        "reference_flight_time_s": duration,
        "motion_start_sim_time_s": MOTION_START_S,
        "replay_contract": (
            "Before the first sensor contact, the Gazebo plugin overwrites x500_0 pose, "
            "world linear velocity, and body angular velocity at every 4 ms physics step. "
            "On the first contact it permanently stops overwriting them, leaving gravity and "
            "Gazebo collision response active."
        ),
        "tracking_error_before_first_contact_m": 0.0,
        "release_sent_on_contact": any(stream.released for stream in streams),
        "first_contact_sim_time_s": first_contact,
        "first_contact_reference_time_s": first_contact - MOTION_START_S if first_contact else None,
        "collision_detected": any(row["contact_pair_count"] for row in contacts),
        "contacts": contacts,
        "evidence": "kinematic pre-contact replay with Gazebo x500 collision geometry and dynamic gate collision meshes",
    }
    (run_dir / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"run_dir": str(run_dir), **result}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
