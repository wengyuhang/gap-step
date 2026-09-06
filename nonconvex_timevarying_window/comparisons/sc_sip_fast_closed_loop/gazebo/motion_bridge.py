#!/usr/bin/env python3
"""Drive Gazebo gate poses from the frozen SC/SIP MotionProfile equations."""
from __future__ import annotations

import math
import subprocess
import sys
import time

import numpy as np

SERVER = "sc_sip_gz_server"
START = np.array((0.0, -18.0, 4.0))
CENTERS = np.array(((12., -10., 8.), (-13., 9., 3.), (4., 14., 10.),
                    (-12., -12., 7.), (14., 6., 4.), (-2., 0., 13.)))
ORDER = (3, 4, 1, 5, 0, 2)
NAMES = ("L_polygon", "circle_arc", "bezier_notch", "bspline_wave", "arc_capsule", "bezier_diamond")
# translation amplitude, RPY amplitude, scale amplitude, translation/rotation/scale periods, phase
MOTION = (
    ((1.40,1.10,0.90),(0.72,0.58,0.82),.55,2.60,2.25,2.90,.25),
    ((1.75,.95,1.15),(.60,.78,.68),.48,2.30,2.70,2.45,1.05),
    ((1.10,1.80,1.00),(.82,.66,.74),.60,2.80,2.15,3.10,1.70),
    ((1.85,1.25,.85),(.68,.84,.62),.52,2.20,2.50,2.35,2.30),
    ((1.30,1.65,1.20),(.76,.70,.88),.57,2.55,2.05,2.75,2.85),
    ((1.60,1.45,1.30),(.86,.64,.80),.50,2.40,2.35,2.20,3.40),
)

def initial_rpy() -> list[np.ndarray]:
    out: list[np.ndarray | None] = [None] * 6
    previous = START.copy()
    for i in ORDER:
        normal = CENTERS[i] - previous
        normal /= np.linalg.norm(normal)
        out[i] = np.array((0., math.acos(float(np.clip(normal[2], -1., 1.))), math.atan2(normal[1], normal[0])))
        previous = CENTERS[i]
    return [item for item in out if item is not None]

ANGLES = initial_rpy()

def quaternion(rpy: np.ndarray) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy / 2.0
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return (cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)

def pose(i: int, t: float) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    trans, rot, scale, tp, rp, sp, phase = MOTION[i]
    phase3 = phase + np.array((0., .7, 1.4))
    position = CENTERS[i] + np.asarray(trans) * np.sin(2*math.pi*t/tp + phase3)
    angles = ANGLES[i] + np.asarray(rot) * np.sin(2*math.pi*t/rp + phase3)
    return position, quaternion(angles)

def send(i: int, t: float) -> None:
    pos, q = pose(i, t)
    # A model-pose update moves the complete collision frame.  gz.msgs.Pose
    # has no scale field; scale is intentionally not approximated here.
    request = (f'name: "gate_{i}_{NAMES[i]}" position {{ x: {pos[0]:.8f} y: {pos[1]:.8f} z: {pos[2]:.8f} }} '
               f'orientation {{ w: {q[0]:.8f} x: {q[1]:.8f} y: {q[2]:.8f} z: {q[3]:.8f} }}')
    subprocess.run(["docker", "exec", SERVER, "gz", "service", "-s", "/world/wide_scrambled_fast_closed_loop/set_pose",
                    "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "200", "--req", request],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)

def main() -> None:
    started = time.monotonic()
    print("Driving six SC/SIP gates at 20 Hz. Press Ctrl-C to stop.")
    try:
        while True:
            now = time.monotonic() - started
            for i in range(6): send(i, now)
            time.sleep(.05)
    except KeyboardInterrupt:
        print("Gate motion stopped.")

if __name__ == "__main__":
    main()
