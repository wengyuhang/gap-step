#!/usr/bin/env python3
"""Export the frozen six-window SC/SIP course as a Gazebo SDF world.

The gate outlines are collision-enabled 0.12 m tubular frames.  Geometry and
initial poses intentionally mirror scenario.py; dynamics are applied by
motion_bridge.py at runtime, so this exporter never changes the benchmark.
"""
from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
from nonconvex_timevarying_window.comparisons.sc_sip_fast_closed_loop.scenario import _primitive_boundaries

OUT = Path(__file__).with_name("wide_scrambled_fast_closed_loop.sdf")
START = np.array((0.0, -18.0, 4.0))
CENTERS = np.array(((12., -10., 8.), (-13., 9., 3.), (4., 14., 10.),
                    (-12., -12., 7.), (14., 6., 4.), (-2., 0., 13.)))
ORDER = (3, 4, 1, 5, 0, 2)
COLORS = ("0.10 0.55 1.0 1", "0.95 0.30 0.25 1", "0.30 0.90 0.45 1",
          "0.95 0.75 0.10 1", "0.75 0.25 0.95 1", "0.10 0.85 0.85 1")

def rpy(previous: np.ndarray, center: np.ndarray) -> tuple[float, float, float]:
    normal = center - previous
    normal /= np.linalg.norm(normal)
    return (0.0, math.acos(float(np.clip(normal[2], -1., 1.))), math.atan2(normal[1], normal[0]))

def samples(segments: tuple) -> np.ndarray:
    values: list[np.ndarray] = []
    for segment in segments:
        count = 1 if segment.is_straight() else 12
        values.extend(segment.evaluate(float(u)) for u in np.linspace(0., 1., count, endpoint=False))
    return np.asarray(values)

def cylinder(i: int, a: np.ndarray, b: np.ndarray, color: str) -> str:
    d = b - a
    length = float(np.linalg.norm(d))
    yaw = math.atan2(float(d[1]), float(d[0]))
    mid = (a + b) / 2
    # Cylinder's native axis is z; pitch it into the gate's local xy plane.
    return f'''<link name="edge_{i}"><pose>{mid[0]:.6f} {mid[1]:.6f} 0 0 1.570796 {yaw:.6f}</pose>
      <collision name="collision"><geometry><cylinder><radius>0.06</radius><length>{length:.6f}</length></cylinder></geometry></collision>
      <visual name="visual"><geometry><cylinder><radius>0.06</radius><length>{length:.6f}</length></cylinder></geometry><material><diffuse>{color}</diffuse><emissive>{color}</emissive></material></visual></link>'''

def gate(index: int, name: str, boundary: tuple, pose: tuple[float, ...], color: str) -> str:
    pts = samples(boundary)
    edges = "\n".join(cylinder(k, pts[k], pts[(k + 1) % len(pts)], color) for k in range(len(pts)))
    p = " ".join(f"{value:.6f}" for value in pose)
    return f'<model name="gate_{index}_{name}"><static>true</static><pose>{p}</pose>{edges}</model>'

def main() -> None:
    boundaries = _primitive_boundaries()
    angles: list[tuple[float, float, float] | None] = [None] * len(CENTERS)
    previous = START.copy()
    for index in ORDER:
        angles[index] = rpy(previous, CENTERS[index])
        previous = CENTERS[index]
    gate_xml = "\n".join(
        gate(i, name, boundary, (*CENTERS[i], *(angles[i] or (0., 0., 0.))), COLORS[i])
        for i, (name, boundary) in enumerate(boundaries)
    )
    order_text = " → ".join(f"W{i}" for i in ORDER)
    OUT.write_text(f'''<?xml version="1.0" ?>
<sdf version="1.9"><world name="wide_scrambled_fast_closed_loop">
  <physics name="1ms" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <light type="directional" name="sun"><pose>0 0 20 0 0 0</pose><cast_shadows>true</cast_shadows><diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.2 -0.9</direction></light>
  <model name="ground"><static>true</static><pose>0 0 -6 0 0 0</pose><link name="link"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry></collision><visual name="v"><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry><material><diffuse>0.08 0.10 0.13 1</diffuse></material></visual></link></model>
  <model name="start_finish"><static>true</static><pose>0 -18 4 0 0 0</pose><link name="link"><visual name="v"><geometry><sphere><radius>0.45</radius></sphere></geometry><material><emissive>1 1 1 1</emissive></material></visual></link></model>
  {gate_xml}
</world></sdf>''', encoding="utf-8")
    print(f"wrote {OUT} (traversal: {order_text})")

if __name__ == "__main__":
    main()
