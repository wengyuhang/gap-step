#!/usr/bin/env python3
"""Generate standalone Gazebo Harmonic and PX4 worlds for the frozen course."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import trimesh

from course import HERE, load_course, local_boundary, pose_at


MESH_DIR = HERE / "meshes"
TEXTURE_DIR = HERE / "materials" / "textures"
PHYSICS_WORLD = HERE / "convex_seven_dynamic_physics.sdf"
PX4_WORLD = HERE / "convex_seven_dynamic_px4.sdf"
PX4_TOGT_WORLD = HERE / "convex_seven_dynamic_px4_togt.sdf"
PX4_NMPC_WORLD = HERE / "convex_seven_dynamic_px4_togt_nmpc.sdf"
PX4_REPLAY_WORLD = HERE / "convex_seven_dynamic_px4_replay.sdf"
MANIFEST = HERE / "manifest.json"
FRAME_RADIUS = 0.020
FRAME_SECTIONS = 12
SLEEVE_RADIUS = 0.085
SLEEVE_OFFSET = SLEEVE_RADIUS - FRAME_RADIUS
PHYSICS_STEP = 0.001
PX4_STEP = 0.004
GROUND_Z = -6.0
PX4_SPAWN = (-16.0, 4.0, -6.0, 0.0, 0.0, 0.0)
TOGT_MOTION_START = 30.0
# Three real-time iterations take about 14 ms on this host. Slow wall-clock
# execution leaves enough CPU time for every 10 ms simulation-time control
# update; physics step, trajectory timebase, and window motion stay unchanged.
NMPC_REAL_TIME_FACTOR = 1.0
COLORS = (
    (0.10, 0.58, 1.00, 1.0),
    (1.00, 0.30, 0.16, 1.0),
    (0.92, 0.18, 0.62, 1.0),
    (0.98, 0.76, 0.08, 1.0),
    (0.18, 0.90, 0.48, 1.0),
    (0.12, 0.88, 0.94, 1.0),
    (0.67, 0.35, 1.00, 1.0),
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tube_mesh(points: np.ndarray, radius: float, sections: int) -> trimesh.Trimesh:
    values = np.column_stack((np.asarray(points, dtype=float), np.zeros(len(points))))
    vertices: list[np.ndarray] = []
    faces: list[tuple[int, int, int]] = []
    angles = 2.0 * math.pi * np.arange(sections) / sections
    cosine, sine = np.cos(angles), np.sin(angles)
    for a, b in zip(values, np.roll(values, -1, axis=0)):
        delta = b - a
        length = float(np.linalg.norm(delta))
        if length <= 1e-10:
            continue
        tangent = delta / length
        reference = np.asarray((0.0, 0.0, 1.0))
        if abs(float(tangent @ reference)) > 0.95:
            reference = np.asarray((1.0, 0.0, 0.0))
        u = np.cross(tangent, reference)
        u /= np.linalg.norm(u)
        v = np.cross(tangent, u)
        ring_a = a + radius * (cosine[:, None] * u + sine[:, None] * v)
        ring_b = b + radius * (cosine[:, None] * u + sine[:, None] * v)
        offset = len(vertices)
        vertices.extend(ring_a)
        vertices.extend(ring_b)
        vertices.extend((a, b))
        center_a, center_b = offset + 2 * sections, offset + 2 * sections + 1
        for index in range(sections):
            following = (index + 1) % sections
            ai, aj = offset + index, offset + following
            bi, bj = offset + sections + index, offset + sections + following
            faces.extend(((ai, bi, bj), (ai, bj, aj),
                          (center_a, aj, ai), (center_b, bi, bj)))
    mesh = trimesh.Trimesh(np.asarray(vertices), np.asarray(faces), process=False)
    mesh.remove_unreferenced_vertices()
    return mesh


def outward_offset_polygon(points: np.ndarray, distance: float) -> np.ndarray:
    """Miter-offset a convex outline outwards without reducing its opening."""
    values = np.asarray(points, dtype=float)
    area2 = float(np.sum(values[:, 0] * np.roll(values[:, 1], -1)
                         - np.roll(values[:, 0], -1) * values[:, 1]))
    orientation = 1.0 if area2 > 0.0 else -1.0
    edges = np.roll(values, -1, axis=0) - values
    tangents = edges / np.linalg.norm(edges, axis=1)[:, None]
    normals = orientation * np.column_stack((tangents[:, 1], -tangents[:, 0]))
    shifted = np.empty_like(values)
    for index in range(len(values)):
        direction = normals[index - 1] + normals[index]
        length = float(np.linalg.norm(direction))
        direction = normals[index] if length < 1e-10 else direction / length
        denominator = max(0.5, float(direction @ normals[index]))
        shifted[index] = values[index] + direction * distance / denominator
    return shifted


def sleeve_boundary(window: dict, boundary: np.ndarray) -> np.ndarray:
    if window["aperture_kind"] == "circle":
        radius = float(window["radius"])
        return boundary * ((radius + SLEEVE_OFFSET) / radius)
    return outward_offset_polygon(boundary, SLEEVE_OFFSET)


def text(values) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def gate_xml(index: int, window: dict, core_mesh_name: str, sleeve_mesh_name: str,
             *, motion_start_time: float = 0.0) -> str:
    position, rpy = pose_at(window, 0.0)
    motion = window["motion"]
    color = text(COLORS[index])
    return f'''  <model name="gate_{index + 1:02d}_{window['name']}">
    <static>true</static><pose>{text((*position, *rpy))}</pose>
    <link name="frame">
      <collision name="frame_collision"><geometry><mesh><uri>meshes/{sleeve_mesh_name}</uri></mesh></geometry>
        <surface><contact><collide_bitmask>0xffff</collide_bitmask></contact>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
      </collision>
      <visual name="foam_sleeve"><geometry><mesh><uri>meshes/{sleeve_mesh_name}</uri></mesh></geometry>
        <material><ambient>.025 .030 .040 1</ambient><diffuse>.055 .065 .085 1</diffuse>
          <specular>.14 .14 .16 1</specular></material>
      </visual>
      <visual name="led_inner_edge"><geometry><mesh><uri>meshes/{core_mesh_name}</uri></mesh></geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse>
          <emissive>{color}</emissive><specular>.75 .75 .75 1</specular></material>
      </visual>
      <sensor name="frame_contact" type="contact"><always_on>true</always_on><update_rate>250</update_rate>
        <topic>/convex_seven/gate_{index + 1:02d}/contacts</topic>
        <contact><collision>frame_collision</collision></contact>
      </sensor>
    </link>
    <plugin filename="libPeriodicGateMotion.so" name="convex_dynamic_course::PeriodicGateMotion">
      <center0>{text(window['center0'])}</center0>
      <base_rpy>{text(window['angles0_rpy_rad'])}</base_rpy>
      <translation_amplitude>{text(motion['translation_amplitude'])}</translation_amplitude>
      <rotation_amplitude>{text(motion['rotation_amplitude'])}</rotation_amplitude>
      <translation_period>{float(motion['translation_period']):.12g}</translation_period>
      <rotation_period>{float(motion['rotation_period']):.12g}</rotation_period>
      <phase>{float(motion['phase']):.12g}</phase>
      <motion_start_time>{motion_start_time:.12g}</motion_start_time>
    </plugin>
  </model>'''


def visual_box(name: str, pose, size, color: str) -> str:
    return f'''  <model name="{name}"><static>true</static><pose>{text(pose)}</pose><link name="link">
    <visual name="visual"><geometry><box><size>{text(size)}</size></box></geometry>
      <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
  </link></model>'''


def indoor_lab_xml() -> str:
    pieces = [
        visual_box("back_wall", (0, 23, 5, 0, 0, 0), (58, .22, 22), ".70 .72 .74 1"),
        visual_box("left_wall", (-29, 0, 5, 0, 0, 0), (.22, 46, 22), ".70 .72 .74 1"),
        visual_box("right_wall", (29, 0, 5, 0, 0, 0), (.22, 46, 22), ".70 .72 .74 1"),
        visual_box("lab_ceiling", (0, 0, 16.1, 0, 0, 0), (58, 46, .20), ".56 .59 .62 1"),
    ]
    for row, y in enumerate((-12.0, 0.0, 12.0)):
        for column, x in enumerate((-20.0, -7.0, 7.0, 20.0)):
            pieces.append(visual_box(
                f"ceiling_panel_{row}_{column}", (x, y, 15.96, 0, 0, 0),
                (6.0, 1.0, .025), ".98 .98 .96 1"))
    return "\n".join(pieces)


def world_xml(course: dict, gates: str, *, px4: bool,
              name: str | None = None, real_time_factor: float = 1.0,
              trajectory_replay: bool = False) -> str:
    if name is None:
        name = "convex_seven_dynamic_px4" if px4 else "convex_seven_dynamic_physics"
    step = PX4_STEP if px4 else PHYSICS_STEP
    systems = "" if px4 else '''
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'''
    if trajectory_replay:
        # This direct-Gazebo replay world must expose its x500_0 model and
        # contact streams to the world-level replay plugin.
        systems += '''
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'''
    environment = '''
  <gravity>0 0 -9.8</gravity>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
  <atmosphere type="adiabatic"/>
  <spherical_coordinates><surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.397971057728974</latitude_deg>
    <longitude_deg>8.546163739800146</longitude_deg><elevation>0</elevation>
    <heading_deg>0</heading_deg></spherical_coordinates>''' if px4 else "\n  <gravity>0 0 -9.8</gravity>"
    start = course["start"]
    decorations = indoor_lab_xml()
    replay_plugin = '''
  <plugin filename="libTrajectoryReplay.so" name="convex_dynamic_course::TrajectoryReplay">
    <target_model>x500_0</target_model>
    <motion_start_time>30</motion_start_time>
    <reference_environment_variable>REPLAY_REFERENCE</reference_environment_variable>
    <release_udp_port>18680</release_udp_port>
    </plugin>''' if trajectory_replay else ""
    replay_vehicle = '''
  <include>
    <uri>model://x500</uri><name>x500_0</name>
    <pose>-16 4 3.2 0 0 0</pose>
  </include>''' if trajectory_replay else ""
    return f'''<?xml version="1.0"?>
<sdf version="1.9"><world name="{name}">
  <physics name="course_physics" type="dart"><max_step_size>{step}</max_step_size><real_time_factor>{real_time_factor:g}</real_time_factor></physics>{systems}{environment}
{replay_plugin}
  <scene><ambient>0.65 0.67 0.70 1</ambient><background>0.74 0.77 0.81 1</background><shadows>true</shadows><grid>false</grid></scene>
  <light type="directional" name="sun"><pose>0 0 35 0 0 0</pose><cast_shadows>true</cast_shadows>
    <diffuse>0.95 0.95 0.92 1</diffuse><direction>-0.35 0.25 -0.9</direction></light>
  <model name="ground"><static>true</static><pose>0 0 {GROUND_Z} 0 0 0</pose><link name="ground_link">
    <collision name="ground_collision"><geometry><plane><normal>0 0 1</normal><size>80 70</size></plane></geometry></collision>
    <visual name="ground_visual"><geometry><plane><normal>0 0 1</normal><size>80 70</size></plane></geometry>
      <material><ambient>.72 .74 .77 1</ambient><diffuse>.84 .85 .86 1</diffuse>
        <pbr><metal><albedo_map>materials/textures/indoor_lab_floor.png</albedo_map>
          <roughness>.92</roughness><metalness>.01</metalness></metal></pbr></material></visual>
  </link></model>
  <model name="start_finish_marker"><static>true</static><pose>{text((*start, 0, 0, 0))}</pose><link name="marker">
    <visual name="ring"><geometry><cylinder><radius>.65</radius><length>.035</length></cylinder></geometry>
      <material><diffuse>.05 1 .30 1</diffuse><emissive>.03 .65 .18 1</emissive></material></visual>
  </link></model>
{decorations}
{gates}
{replay_vehicle}
</world></sdf>'''


def main() -> None:
    course = load_course()
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    records = []
    models = []
    delayed_models = []
    for index, window in enumerate(course["windows"]):
        boundary = local_boundary(window)
        core_mesh_name = f"gate_{index + 1:02d}_{window['shape']}_core.stl"
        sleeve_mesh_name = f"gate_{index + 1:02d}_{window['shape']}_sleeve.stl"
        core_mesh_path = MESH_DIR / core_mesh_name
        sleeve_mesh_path = MESH_DIR / sleeve_mesh_name
        tube_mesh(boundary, FRAME_RADIUS, FRAME_SECTIONS).export(core_mesh_path)
        tube_mesh(sleeve_boundary(window, boundary), SLEEVE_RADIUS, FRAME_SECTIONS).export(sleeve_mesh_path)
        models.append(gate_xml(index, window, core_mesh_name, sleeve_mesh_name))
        delayed_models.append(gate_xml(
            index, window, core_mesh_name, sleeve_mesh_name,
            motion_start_time=TOGT_MOTION_START,
        ))
        records.append({
            "index": index + 1,
            "name": window["name"],
            "shape": window["shape"],
            "core_mesh": str(core_mesh_path.relative_to(HERE)),
            "core_mesh_sha256": sha256(core_mesh_path),
            "sleeve_mesh": str(sleeve_mesh_path.relative_to(HERE)),
            "sleeve_mesh_sha256": sha256(sleeve_mesh_path),
            "boundary_vertex_count": len(boundary),
            "motion": window["motion"],
        })
    gate_text = "\n".join(models)
    PHYSICS_WORLD.write_text(world_xml(course, gate_text, px4=False), encoding="utf-8")
    PX4_WORLD.write_text(world_xml(course, gate_text, px4=True), encoding="utf-8")
    PX4_TOGT_WORLD.write_text(world_xml(
        course, "\n".join(delayed_models), px4=True,
        name="convex_seven_dynamic_px4_togt",
    ), encoding="utf-8")
    PX4_NMPC_WORLD.write_text(world_xml(
        course, "\n".join(delayed_models), px4=True,
        name="convex_seven_dynamic_px4_togt_nmpc",
        real_time_factor=NMPC_REAL_TIME_FACTOR,
    ), encoding="utf-8")
    PX4_REPLAY_WORLD.write_text(world_xml(
        course, "\n".join(delayed_models), px4=True,
        name="convex_seven_dynamic_px4_replay",
        real_time_factor=1.0,
        trajectory_replay=True,
    ), encoding="utf-8")
    manifest = {
        "course": course["name"],
        "source": "course_spec.json",
        "source_sha256": sha256(HERE / "course_spec.json"),
        "gazebo": "Harmonic / SDF 1.9",
        "plugin": "libPeriodicGateMotion.so",
        "motion_timebase": "Gazebo simulation time",
        "physics_world": PHYSICS_WORLD.name,
        "px4_world": PX4_WORLD.name,
        "px4_togt_world": PX4_TOGT_WORLD.name,
        "px4_togt_nmpc_world": PX4_NMPC_WORLD.name,
        "px4_replay_world": PX4_REPLAY_WORLD.name,
        "togt_motion_start_time_s": TOGT_MOTION_START,
        "nmpc_real_time_factor": NMPC_REAL_TIME_FACTOR,
        "physics_step_s": PHYSICS_STEP,
        "px4_step_s": PX4_STEP,
        "px4_vehicle": "x500",
        "px4_spawn_pose": PX4_SPAWN,
        "ground_z_m": GROUND_Z,
        "frame_radius_m": FRAME_RADIUS,
        "sleeve_radius_m": SLEEVE_RADIUS,
        "sleeve_outward_offset_m": SLEEVE_OFFSET,
        "gate_count": len(records),
        "gates": records,
    }
    MANIFEST.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {PHYSICS_WORLD.name}, {PX4_WORLD.name}, {len(records)} gate meshes")


if __name__ == "__main__":
    main()
