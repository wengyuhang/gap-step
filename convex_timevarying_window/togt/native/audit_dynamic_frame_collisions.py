#!/usr/bin/env python3
"""Dense conservative-sphere audit against moving zero-thickness gate frames."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml

BODY_HALF_EXTENTS = np.array([0.26504, 0.26504, 0.05890])
BODY_RADIUS = float(np.linalg.norm(BODY_HALF_EXTENTS))


def rotation_rpy_deg(values):
    x, y, z = np.deg2rad(np.asarray(values, dtype=float))
    cx, sx = np.cos(x), np.sin(x); cy, sy = np.cos(y), np.sin(y); cz, sz = np.cos(z), np.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx],
                     [-sy, cy*sx, cy*cx]])


def local_vertices(shape, safe):
    kind = shape["type"]
    if kind == "TrianglePrisma":
        margin = shape.get("margin", 0.0) if safe else 0.0
        hw = .5*(shape["width"]-margin); hh = .5*(shape["height"]-margin)
        return np.array([[-hh, hw, 0.0], [hh, 0.0, 0.0], [-hh, -hw, 0.0]])
    if kind == "RectanglePrisma":
        mw = shape.get("marginW", 0.0) if safe else 0.0
        mh = shape.get("marginH", 0.0) if safe else 0.0
        hw = .5*(shape["width"]-mw); hh = .5*(shape["height"]-mh)
        return np.array([[-hh, hw, 0.0], [-hh, -hw, 0.0], [hh, -hw, 0.0], [hh, hw, 0.0]])
    if kind == "PentagonPrisma":
        radius = shape["radius"]-(shape.get("margin", 0.0) if safe else 0.0)
        angles = np.deg2rad([144.0, 72.0, 0.0, -72.0, -144.0])
        return np.column_stack((radius*np.cos(angles), radius*np.sin(angles), np.zeros(5)))
    if kind == "HexagonPrisma":
        side = shape["side"]-(shape.get("margin", 0.0) if safe else 0.0)
        h = .5*side*np.tan(np.pi/3.0)
        return np.array([[-h, .5*side, 0.0], [0.0, side, 0.0], [h, .5*side, 0.0],
                         [h, -.5*side, 0.0], [0.0, -side, 0.0], [-h, -.5*side, 0.0]])
    raise ValueError(f"unsupported gate type {kind}")


def dynamic_boundary(shape, index, instant, speed_scale):
    base = rotation_rpy_deg(shape["rpy"])
    yaml_position = np.asarray(shape["position"], dtype=float)
    safe_static = yaml_position + local_vertices(shape, True) @ base.T
    physical_static = yaml_position + local_vertices(shape, False) @ base.T
    pivot = safe_static.mean(axis=0)  # ShapeBase::position after Polyhedron::initialize().
    tangent, normal = base[:, 0], base[:, 2]
    amplitude = .45+.05*((3*index) % 6)
    phase = -1.3+.37*index
    translation = tangent*amplitude*np.sin(phase+2*np.pi*instant*speed_scale/(9.5+.47*(index % 11)))
    angle = np.deg2rad(15+4*(index % 6))*np.sin(phase+.73+2*np.pi*instant*speed_scale/(8+.43*((3*index) % 13)))
    skew = np.array([[0.0, -normal[2], normal[1]], [normal[2], 0.0, -normal[0]], [-normal[1], normal[0], 0.0]])
    dynamic_rotation = np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)
    return pivot+translation+(physical_static-pivot)@dynamic_rotation.T


def point_segment_distances(point, vertices):
    starts = vertices
    ends = np.roll(vertices, -1, axis=0)
    delta = ends-starts
    alpha = np.clip(np.einsum("ij,ij->i", point-starts, delta)/np.einsum("ij,ij->i", delta, delta), 0.0, 1.0)
    closest = starts+alpha[:, None]*delta
    return np.linalg.norm(closest-point, axis=1)


def body_rotation(acceleration):
    z_body = acceleration+np.array([0.0, 0.0, 9.8066])
    z_body /= np.linalg.norm(z_body)
    denominator = np.sqrt(2.0*(1.0+z_body[2]))
    quaternion = np.array([0.5*denominator, -z_body[1]/denominator, z_body[0]/denominator, 0.0])
    w, x, y, z = quaternion
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def segment_intersects_body_box(start, end, center, body_world_rotation):
    start_local = body_world_rotation.T@(start-center)
    delta = body_world_rotation.T@(end-start)
    lower, upper = 0.0, 1.0
    for axis, half_extent in enumerate(BODY_HALF_EXTENTS):
        if abs(delta[axis]) < 1e-14:
            if abs(start_local[axis]) > half_extent:
                return False
            continue
        first = (-half_extent-start_local[axis])/delta[axis]
        second = (half_extent-start_local[axis])/delta[axis]
        enter, leave = min(first, second), max(first, second)
        lower, upper = max(lower, enter), min(upper, leave)
        if lower > upper:
            return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("track", type=Path)
    parser.add_argument("output_json", type=Path)
    parser.add_argument("collision_csv", type=Path)
    parser.add_argument("--speed-scale", type=float, required=True)
    args = parser.parse_args()
    if not args.speed_scale > 0.0:
        raise ValueError("speed scale must be positive")
    scene = yaml.safe_load(args.track.read_text())
    names = scene["orders"]
    gates = [scene[name] for name in names]
    data = np.genfromtxt(args.trajectory, delimiter=",", names=True)
    times = np.asarray(data["time"])
    positions = np.column_stack((data["px"], data["py"], data["pz"]))
    accelerations = np.column_stack((data["ax"], data["ay"], data["az"]))
    attitudes = [body_rotation(acceleration) for acceleration in accelerations]
    collision_rows = []
    exact_rows = []
    per_gate = []
    global_worst = None
    for gate_index, (name, gate) in enumerate(zip(names, gates)):
        margins = np.empty(times.size)
        exact_count = 0
        for sample_index, (instant, position, attitude) in enumerate(zip(times, positions, attitudes)):
            boundary = dynamic_boundary(gate, gate_index, float(instant), args.speed_scale)
            distance = float(point_segment_distances(position, boundary).min())
            margin = distance-BODY_RADIUS
            margins[sample_index] = margin
            if margin < 0.0:
                collision_rows.append((sample_index, float(instant), gate_index+1, name, *position, distance, margin))
            exact_collision = any(segment_intersects_body_box(start, end, position, attitude)
                                  for start, end in zip(boundary, np.roll(boundary, -1, axis=0)))
            if exact_collision:
                exact_count += 1
                exact_rows.append((sample_index, float(instant), gate_index+1, name, *position))
        worst_index = int(np.argmin(margins))
        record = {"gate_index": gate_index+1, "gate_name": name,
                  "collision_sample_count": int(np.count_nonzero(margins < 0.0)),
                  "exact_cuboid_collision_sample_count": exact_count,
                  "minimum_margin_m": float(margins[worst_index]),
                  "minimum_margin_time_s": float(times[worst_index]),
                  "minimum_margin_position_m": positions[worst_index].tolist()}
        per_gate.append(record)
        if global_worst is None or record["minimum_margin_m"] < global_worst["minimum_margin_m"]:
            global_worst = record
    args.collision_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.collision_csv.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sample_index", "time_s", "gate_index", "gate_name", "px", "py", "pz", "center_to_frame_distance_m", "sphere_margin_m"])
        writer.writerows(collision_rows)
    exact_path = args.collision_csv.with_name(args.collision_csv.stem+"_exact_cuboid.csv")
    with exact_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sample_index", "time_s", "gate_index", "gate_name", "px", "py", "pz"])
        writer.writerows(exact_rows)
    report = {"passed": not exact_rows, "whole_body_cuboid_passed": not exact_rows,
              "exact_cuboid_collision_sample_count": len(exact_rows),
              "exact_cuboid_collision_time_sample_count": len({row[0] for row in exact_rows}),
              "exact_cuboid_collision_csv": str(exact_path),
              "conservative_sphere_passed": not collision_rows, "collision_sample_count": len(collision_rows),
              "collision_time_sample_count": len({row[0] for row in collision_rows}),
              "trajectory_sample_count": int(times.size),
              "maximum_sample_step_s": float(np.diff(times).max()),
              "body_model": "oriented cuboid from TOGT constant-yaw differential-flatness attitude; circumscribed sphere also reported",
              "body_half_extents_m": BODY_HALF_EXTENTS.tolist(), "body_radius_m": BODY_RADIUS,
              "obstacle_model": "moving finite zero-thickness physical polygon frame",
              "motion_speed_scale": args.speed_scale, "minimum_margin": global_worst,
              "per_gate": per_gate,
              "evidence": "dense 1 ms sampled exact-at-samples segment-versus-oriented-box test; not a continuous-time certificate"}
    args.output_json.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
