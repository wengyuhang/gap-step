"""Independent released-TOGT dynamics audit for an exported trajectory."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    data = np.genfromtxt(args.trajectory, delimiter=",", names=True)
    pvajs = np.stack([
        np.column_stack([data["px"], data["py"], data["pz"]]),
        np.column_stack([data["vx"], data["vy"], data["vz"]]),
        np.column_stack([data["ax"], data["ay"], data["az"]]),
        np.column_stack([data["jx"], data["jy"], data["jz"]]),
        np.column_stack([data["sx"], data["sy"], data["sz"]]),
    ], axis=1)
    native = AnalyticTOGTCore().sample_dynamics(pvajs)
    regular = native["regular_branch"]
    rotor = native["rotor_thrusts"][regular]
    report = {
        "passed": bool(
            np.max(native["speed"]) <= 60.0
            and np.max(np.linalg.norm(native["body_rate"][:, :2], axis=1)) <= 10.0
            and np.max(np.abs(native["body_rate"][:, 2][regular])) <= 10.0
            and np.min(native["collective_thrust"]) >= 1.0
            and np.max(native["collective_thrust"]) <= 20.0
            and np.min(rotor) >= 0.25
            and np.max(rotor) <= 5.0
        ),
        "sample_count": int(len(data)),
        "maximum_sample_step_s": float(np.max(np.diff(data["time"]))),
        "max_speed_mps": float(np.max(native["speed"])),
        "max_tilt_rad": float(np.max(native["tilt"])),
        "max_body_rate_xy_radps": float(np.max(np.linalg.norm(native["body_rate"][:, :2], axis=1))),
        "max_abs_body_rate_z_radps": float(np.max(np.abs(native["body_rate"][:, 2][regular]))),
        "min_collective_thrust_n": float(np.min(native["collective_thrust"])),
        "max_collective_thrust_n": float(np.max(native["collective_thrust"])),
        "min_rotor_thrust_n": float(np.min(rotor)),
        "max_rotor_thrust_n": float(np.max(rotor)),
        "evidence": "released TOGT C++ QuadManifold evaluated on a maximum 1 ms grid; not a continuous-time certificate",
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
