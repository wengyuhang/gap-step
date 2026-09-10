#!/usr/bin/env python3
"""Export the accepted seven-window course as a Gazebo Harmonic world."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image, ImageDraw
from shapely.geometry import Polygon


try:
    from .course_spec import GATES, MESH_CHORD_TOLERANCE_M, SHAPES, START
except ImportError:  # direct script execution
    from course_spec import GATES, MESH_CHORD_TOLERANCE_M, SHAPES, START


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]


WORLD = HERE / "seven_unique_high_fidelity.sdf"
PREVIEW_WORLD = HERE / "seven_unique_race_preview.sdf"
PHYSICS_WORLD = HERE / "seven_unique_physics.sdf"
PX4_WORLD = HERE / "seven_unique_px4_manual.sdf"
COURSE_MANIFEST = HERE / "course_manifest.json"
MESH_DIR = HERE / "meshes"
TEXTURE_DIR = HERE / "materials" / "textures"
RESULT = (
    HERE.parent / "results" / "formal_final_post5_sphere_only_20260909" / "result.json"
)
RESULT_TRAJECTORY = RESULT.parent / "dual_constraint_cem_trajectory.npz"
REPLAY_BODY_HALF_EXTENTS = np.asarray((0.26504, 0.26504, 0.0589))
REPLAY_BODY_SPHERE_RADIUS = float(np.linalg.norm(REPLAY_BODY_HALF_EXTENTS))
REPLAY_PLANNING_SPHERE_RADIUS = REPLAY_BODY_SPHERE_RADIUS + 0.015
FRAME_RADIUS = 0.010
FRAME_SECTIONS = 12
SLEEVE_RADIUS = 0.085
SLEEVE_SECTIONS = 10
PREVIEW_BOUNDARY_TOLERANCE = 0.006
PREVIEW_PHYSICS_STEP = 0.004
PX4_PHYSICS_STEP = 0.004
PX4_SPAWN_POSE = (-16.0, 4.0, -6.0, 0.0, 0.0, 0.0)
PATH_RADIUS = 0.022
PATH_SAMPLES = 720
COLORS = (
    (0.10, 0.58, 1.00, 1.0),
    (1.00, 0.30, 0.16, 1.0),
    (0.92, 0.18, 0.62, 1.0),
    (0.98, 0.76, 0.08, 1.0),
    (0.18, 0.90, 0.48, 1.0),
    (0.12, 0.88, 0.94, 1.0),
    (0.67, 0.35, 1.00, 1.0),
)
LAB_GATE_COLOR = (0.78, 0.83, 0.90, 1.0)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tube_mesh(points, radius: float, sections: int, *, closed: bool) -> trimesh.Trimesh:
    """Create capped cylindrical segments centered exactly on a polyline."""
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] not in (2, 3):
        raise ValueError("points must have shape (N,2) or (N,3)")
    if values.shape[1] == 2:
        values = np.column_stack((values, np.zeros(len(values))))
    pairs = list(zip(values[:-1], values[1:]))
    if closed:
        pairs.append((values[-1], values[0]))
    vertices: list[np.ndarray] = []
    faces: list[tuple[int, int, int]] = []
    angles = 2.0 * math.pi * np.arange(sections) / sections
    cosine, sine = np.cos(angles), np.sin(angles)
    for a, b in pairs:
        delta = b - a
        length = float(np.linalg.norm(delta))
        if length <= 1e-10:
            continue
        tangent = delta / length
        reference = np.array((0.0, 0.0, 1.0))
        if abs(float(tangent @ reference)) > 0.95:
            reference = np.array((1.0, 0.0, 0.0))
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
        for i in range(sections):
            j = (i + 1) % sections
            ai, aj = offset + i, offset + j
            bi, bj = offset + sections + i, offset + sections + j
            faces.extend(((ai, bi, bj), (ai, bj, aj),
                          (center_a, aj, ai), (center_b, bi, bj)))
    mesh = trimesh.Trimesh(vertices=np.asarray(vertices), faces=np.asarray(faces), process=False)
    mesh.remove_unreferenced_vertices()
    return mesh


def outward_offset_polygon(points, distance: float) -> np.ndarray:
    """Miter-offset a closed polygon toward its exterior for a visual gate sleeve."""
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 2 or len(values) < 3:
        raise ValueError("closed planar polygon expected")
    area2 = float(np.sum(values[:, 0] * np.roll(values[:, 1], -1)
                         - np.roll(values[:, 0], -1) * values[:, 1]))
    orientation = 1.0 if area2 > 0.0 else -1.0
    edges = np.roll(values, -1, axis=0) - values
    lengths = np.linalg.norm(edges, axis=1)
    tangents = edges / lengths[:, None]
    normals = orientation * np.column_stack((tangents[:, 1], -tangents[:, 0]))
    shifted = np.empty_like(values)
    for index in range(len(values)):
        previous = normals[index - 1]
        current = normals[index]
        direction = previous + current
        norm = float(np.linalg.norm(direction))
        if norm < 1e-8:
            direction = current
        else:
            direction /= norm
        denominator = max(0.5, float(direction @ current))
        shifted[index] = values[index] + direction * min(distance / denominator, 2.0 * distance)
    return shifted


def simplified_visual_polygon(points, tolerance: float) -> tuple[np.ndarray, float]:
    """Simplify only the rendered outline and report its boundary Hausdorff error."""
    source = Polygon(np.asarray(points, dtype=float))
    simplified = source.simplify(tolerance, preserve_topology=True)
    values = np.asarray(simplified.exterior.coords[:-1], dtype=float)
    error = float(source.boundary.hausdorff_distance(simplified.boundary))
    return values, error


def create_floor_texture() -> Path:
    """Create a deterministic rubberized arena floor texture."""
    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    texture = TEXTURE_DIR / "arena_floor.png"
    rng = np.random.default_rng(20260910)
    noise = rng.normal(0.0, 5.0, (1024, 1024, 1))
    base = np.array([31.0, 38.0, 47.0])[None, None, :]
    pixels = np.clip(base + noise, 12, 70).astype(np.uint8)
    image = Image.fromarray(pixels, mode="RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    for value in range(0, 1024, 128):
        draw.line((value, 0, value, 1024), fill=(95, 108, 122, 42), width=2)
        draw.line((0, value, 1024, value), fill=(95, 108, 122, 42), width=2)
    draw.rectangle((18, 18, 1005, 1005), outline=(20, 196, 255, 105), width=10)
    image.save(texture)
    return texture


def create_lab_floor_texture() -> Path:
    """Create the light grey gridded floor used by the indoor test hall."""
    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    texture = TEXTURE_DIR / "indoor_lab_floor.png"
    rng = np.random.default_rng(20260911)
    noise = rng.normal(0.0, 1.8, (1024, 1024, 1))
    base = np.array([218.0, 222.0, 225.0])[None, None, :]
    pixels = np.clip(base + noise, 198, 236).astype(np.uint8)
    image = Image.fromarray(pixels, mode="RGB")
    draw = ImageDraw.Draw(image, "RGBA")
    for value in range(0, 1024, 64):
        width = 2 if value % 256 else 4
        shade = (128, 137, 145, 90 if width == 2 else 125)
        draw.line((value, 0, value, 1024), fill=shade, width=width)
        draw.line((0, value, 1024, value), fill=shade, width=width)
    image.save(texture)
    return texture


def rgba_text(color) -> str:
    return " ".join(f"{value:.3f}" for value in color)


def mesh_geometry(uri: str) -> str:
    return f"<geometry><mesh><uri>{uri}</uri></mesh></geometry>"


def gate_model(index, window, angles, collision_mesh_name, visual_core_name,
               sleeve_mesh_name, color, *, include_collision: bool) -> str:
    pose = " ".join(f"{x:.12g}" for x in (*window.center, *angles))
    frame_pose = f"0 0 0 0 0 {window.theta0:.12g}"
    material = rgba_text(color)
    collision_xml = f'''<collision name="frame_collision">{mesh_geometry('meshes/' + collision_mesh_name)}
          <surface><contact><collide_bitmask>0xffff</collide_bitmask></contact>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction>
          <bounce><restitution_coefficient>.16</restitution_coefficient><threshold>.05</threshold></bounce></surface>
        </collision>''' if include_collision else ""
    return f'''<model name="gate_{index + 1:02d}_{window.name}">
      <pose>{pose}</pose>
      <self_collide>false</self_collide>
      <link name="base">
        <gravity>false</gravity>
        <inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>
      </link>
      <link name="frame"><pose relative_to="base">{frame_pose}</pose><gravity>false</gravity>
        <inertial><mass>8</mass><inertia><ixx>40</ixx><iyy>40</iyy><izz>40</izz></inertia></inertial>
        {collision_xml}
        <visual name="foam_sleeve">{mesh_geometry('meshes/' + sleeve_mesh_name)}
          <material><ambient>0.025 0.030 0.040 1</ambient><diffuse>0.055 0.065 0.085 1</diffuse>
          <specular>0.14 0.14 0.16 1</specular></material>
        </visual>
        <visual name="led_inner_edge">{mesh_geometry('meshes/' + visual_core_name)}
          <material><ambient>{material}</ambient><diffuse>{material}</diffuse>
          <specular>0.75 0.75 0.75 1</specular><emissive>{material}</emissive></material>
        </visual>
      </link>
      <joint name="world_fixed" type="fixed"><parent>world</parent><child>base</child></joint>
      <joint name="spin" type="revolute"><parent>base</parent><child>frame</child>
        <axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper>
        <velocity>100</velocity><effort>1e9</effort></limit><dynamics><damping>0</damping></dynamics></axis>
      </joint>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>spin</joint_name><initial_velocity>{window.omega:.12g}</initial_velocity>
      </plugin>
      <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
        <joint_name>spin</joint_name><topic>/seven_unique/gate_{index + 1:02d}/joint_state</topic>
      </plugin>
    </model>'''


def visual_box(name: str, pose, size, color, *, emissive=None) -> str:
    pose_text = " ".join(f"{float(value):.6g}" for value in pose)
    size_text = " ".join(f"{float(value):.6g}" for value in size)
    diffuse = rgba_text(color)
    glow = rgba_text(emissive if emissive is not None else (0, 0, 0, 1))
    return f'''<model name="{name}"><static>true</static><pose>{pose_text}</pose><link name="visual_only">
      <visual name="visual"><geometry><box><size>{size_text}</size></box></geometry>
        <material><ambient>{diffuse}</ambient><diffuse>{diffuse}</diffuse><emissive>{glow}</emissive></material>
      </visual></link></model>'''


def race_venue_xml(gates) -> str:
    """Visual-only arena dressing; it intentionally adds no collision geometry."""
    pieces = []
    dark = (0.045, 0.055, 0.075, 1.0)
    cyan = (0.05, 0.72, 1.0, 1.0)
    magenta = (0.95, 0.10, 0.52, 1.0)
    # Low crash barriers and grandstand tiers around the 64 x 52 m competition floor.
    pieces.extend((
        visual_box("north_barrier", (0, 26, -5.35, 0, 0, 0), (64, .35, 1.3), dark),
        visual_box("south_barrier", (0, -26, -5.35, 0, 0, 0), (64, .35, 1.3), dark),
        visual_box("east_barrier", (32, 0, -5.35, 0, 0, 0), (.35, 52, 1.3), dark),
        visual_box("west_barrier", (-32, 0, -5.35, 0, 0, 0), (.35, 52, 1.3), dark),
    ))
    for side_index, y in enumerate((-29.0, 29.0)):
        for tier in range(4):
            pieces.append(visual_box(
                f"stand_{side_index}_{tier}",
                (0, y + math.copysign(0.8 * tier, y), -5.55 + 0.55 * tier, 0, 0, 0),
                (52, 1.4, 0.5), (0.075 + .012 * tier, 0.085, 0.115, 1.0)))
    # Ceiling truss and colored arena fascia.
    pieces.extend((
        visual_box("truss_north", (0, 23.5, 15.5, 0, 0, 0), (60, .18, .18), dark),
        visual_box("truss_south", (0, -23.5, 15.5, 0, 0, 0), (60, .18, .18), dark),
        visual_box("truss_east", (29.5, 0, 15.5, 0, 0, 0), (.18, 47, .18), dark),
        visual_box("truss_west", (-29.5, 0, 15.5, 0, 0, 0), (.18, 47, .18), dark),
        visual_box("north_led", (0, 25.78, -4.9, 0, 0, 0), (60, .04, .09), cyan, emissive=cyan),
        visual_box("south_led", (0, -25.78, -4.9, 0, 0, 0), (60, .04, .09), magenta, emissive=magenta),
        visual_box("start_arch_left", (START[0], START[1] - 2.1, -.2, 0, 0, 0), (.18, .18, 11.6), dark),
        visual_box("start_arch_right", (START[0], START[1] + 2.1, -.2, 0, 0, 0), (.18, .18, 11.6), dark),
        visual_box("start_arch_header", (START[0], START[1], 5.6, 0, 0, 0), (.20, 4.4, .35), dark),
        visual_box("start_arch_cyan", (START[0] - .105, START[1] - 1.0, 5.6, 0, 0, 0), (.04, 1.9, .13), cyan, emissive=cyan),
        visual_box("start_arch_magenta", (START[0] - .105, START[1] + 1.0, 5.6, 0, 0, 0), (.04, 1.9, .13), magenta, emissive=magenta),
    ))
    # Two slim towers and a header identify every checkpoint while leaving the race geometry unchanged.
    for index, (window, color) in enumerate(zip(gates, COLORS), start=1):
        cx, cy, cz = map(float, window.center)
        sweep_radius = float(np.max(np.linalg.norm(window.boundary, axis=1))) + SLEEVE_RADIUS
        pedestal_top = cz - sweep_radius - 0.25
        height = max(0.25, pedestal_top + 6.0)
        pieces.append(visual_box(f"gate_{index:02d}_tower",
                                 (cx, cy, -6 + height / 2, 0, 0, 0),
                                 (.09, .09, height), color, emissive=tuple(.22 * v for v in color[:3]) + (1,)))
        pieces.append(visual_box(f"gate_{index:02d}_marker",
                                 (cx, cy, -5.88, 0, 0, 0),
                                 (1.15, 1.15, .05), color, emissive=color))
    return "\n".join(pieces)


def indoor_lab_xml(gates) -> str:
    """Neutral indoor flight-test hall inspired by laboratory gate courses."""
    pieces = []
    wall = (0.70, 0.72, 0.74, 1.0)
    wall_dark = (0.56, 0.59, 0.62, 1.0)
    light = (0.98, 0.98, 0.96, 1.0)
    # Open-front room: back and side walls give the same uncluttered lab depth
    # as the reference while keeping the overview camera outside the room.
    pieces.extend((
        visual_box("lab_back_wall", (0, 20.0, 4.5, 0, 0, 0), (54, .22, 21), wall),
        visual_box("lab_left_wall", (-27.0, 0, 4.5, 0, 0, 0), (.22, 40, 21), wall),
        visual_box("lab_right_wall", (27.0, 0, 4.5, 0, 0, 0), (.22, 40, 21), wall),
        visual_box("lab_ceiling", (0, 0, 15.1, 0, 0, 0), (54, 40, .20), wall_dark),
        visual_box("lab_back_skirt", (0, 19.84, -5.55, 0, 0, 0), (54, .08, .70), wall_dark),
        visual_box("lab_left_skirt", (-26.84, 0, -5.55, 0, 0, 0), (.08, 40, .70), wall_dark),
        visual_box("lab_right_skirt", (26.84, 0, -5.55, 0, 0, 0), (.08, 40, .70), wall_dark),
    ))
    for row, y in enumerate((-10.0, 0.0, 10.0)):
        for column, x in enumerate((-18.0, -6.0, 6.0, 18.0)):
            pieces.append(visual_box(
                f"ceiling_panel_{row}_{column}", (x, y, 14.96, 0, 0, 0),
                (5.6, 1.0, .025), light, emissive=light))
    return "\n".join(pieces)


def remove_model(xml: str, name: str) -> str:
    marker = f'<model name="{name}">'
    if marker not in xml:
        return xml
    start = xml.index(marker)
    end = xml.index('</model>', start) + len('</model>')
    return xml[:start] + xml[end:]


def quadrotor_model(half_extents: np.ndarray) -> str:
    size = 2.0 * np.asarray(half_extents)
    x, y, z = START
    rotor_xy = 0.205
    visuals = [
        '<visual name="body"><geometry><box><size>0.16 0.11 0.065</size></box></geometry><material><diffuse>0.08 0.10 0.14 1</diffuse><specular>0.8 0.8 0.8 1</specular></material></visual>',
    ]
    for i, yaw in enumerate((math.pi / 4, -math.pi / 4)):
        visuals.append(f'<visual name="arm_{i}"><pose>0 0 0 0 0 {yaw}</pose><geometry><box><size>0.54 0.018 0.018</size></box></geometry><material><diffuse>0.18 0.20 0.24 1</diffuse></material></visual>')
    for i, (rx, ry) in enumerate(((rotor_xy, rotor_xy), (-rotor_xy, rotor_xy),
                                  (-rotor_xy, -rotor_xy), (rotor_xy, -rotor_xy))):
        visuals.append(f'<visual name="rotor_{i}"><pose>{rx} {ry} 0.018 0 0 0</pose><geometry><cylinder><radius>0.058</radius><length>0.006</length></cylinder></geometry><material><diffuse>0.12 0.12 0.12 0.72</diffuse></material></visual>')
    return f'''<model name="accepted_quadrotor"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>
      <link name="body"><collision name="audit_body_box"><geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry></collision>
      {''.join(visuals)}</link></model>'''


def evaluate_saved_trajectory(path: Path, sample_count: int) -> tuple[float, np.ndarray]:
    """Evaluate a retained replay polynomial without importing planning code."""
    payload = np.load(path)
    coefficients = np.asarray(payload["coefficients"], dtype=float)
    durations = np.asarray(payload["durations"], dtype=float)
    total_time = float(np.sum(durations))
    times = np.linspace(0.0, total_time, sample_count)
    cumulative = np.r_[0.0, np.cumsum(durations)]
    points = np.empty((sample_count, 3), dtype=float)
    for output_index, instant in enumerate(times):
        segment = min(
            int(np.searchsorted(cumulative[1:], instant, side="right")),
            len(durations) - 1,
        )
        local = instant - cumulative[segment]
        basis = local ** np.arange(coefficients.shape[1])
        points[output_index] = basis @ coefficients[segment]
    return total_time, points


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--with-replay",
        action="store_true",
        help="also rebuild the separate accepted-algorithm replay worlds",
    )
    args = parser.parse_args(argv)

    MESH_DIR.mkdir(parents=True, exist_ok=True)
    floor_texture = create_floor_texture()
    lab_floor_texture = create_lab_floor_texture()
    gate_xml = []
    preview_gate_xml = []
    physics_gate_xml = []
    gate_records = []
    for index, (shape, window, color) in enumerate(
        zip(SHAPES, GATES, COLORS)
    ):
        angles = window.base_rpy
        collision_name = f"gate_{index + 1:02d}_{shape}_collision.stl"
        visual_core_name = f"gate_{index + 1:02d}_{shape}_visual_core.stl"
        sleeve_name = f"gate_{index + 1:02d}_{shape}_sleeve.stl"
        collision_path = MESH_DIR / collision_name
        visual_core_path = MESH_DIR / visual_core_name
        sleeve_path = MESH_DIR / sleeve_name
        collision_mesh = tube_mesh(window.boundary, FRAME_RADIUS, FRAME_SECTIONS, closed=True)
        visual_polygon, visual_error = simplified_visual_polygon(
            window.boundary, PREVIEW_BOUNDARY_TOLERANCE)
        visual_core_mesh = tube_mesh(visual_polygon, FRAME_RADIUS, 8, closed=True)
        sleeve_centerline = outward_offset_polygon(
            visual_polygon, SLEEVE_RADIUS - FRAME_RADIUS)
        sleeve_mesh = tube_mesh(sleeve_centerline, SLEEVE_RADIUS, SLEEVE_SECTIONS, closed=True)
        collision_mesh.export(collision_path, file_type="stl")
        visual_core_mesh.export(visual_core_path, file_type="stl")
        sleeve_mesh.export(sleeve_path, file_type="stl")
        gate_xml.append(gate_model(index, window, angles, collision_name, visual_core_name,
                                   sleeve_name, color, include_collision=True))
        preview_gate_xml.append(gate_model(index, window, angles, collision_name, visual_core_name,
                                           sleeve_name, color, include_collision=False))
        physics_gate_xml.append(gate_model(index, window, angles, sleeve_name, visual_core_name,
                                           sleeve_name, LAB_GATE_COLOR, include_collision=True))
        edge_lengths = np.linalg.norm(
            np.roll(window.boundary, -1, axis=0) - window.boundary, axis=1)
        gate_records.append({
            "index": index + 1, "name": window.name, "shape": shape,
            "center": window.center.tolist(), "base_rpy": list(map(float, angles)),
            "theta0": float(window.theta0), "omega": float(window.omega),
            "boundary_points": int(len(window.boundary)),
            "maximum_boundary_chord_m": float(np.max(edge_lengths)),
            "collision_mesh": f"meshes/{collision_name}",
            "collision_mesh_sha256": sha256(collision_path),
            "collision_mesh_vertices": int(len(collision_mesh.vertices)),
            "collision_mesh_triangles": int(len(collision_mesh.faces)),
            "collision_mesh_watertight": bool(collision_mesh.is_watertight),
            "visual_sleeve_mesh": f"meshes/{sleeve_name}",
            "visual_sleeve_mesh_sha256": sha256(sleeve_path),
            "visual_sleeve_mesh_vertices": int(len(sleeve_mesh.vertices)),
            "visual_sleeve_mesh_triangles": int(len(sleeve_mesh.faces)),
            "visual_sleeve_radius_m": SLEEVE_RADIUS,
            "visual_sleeve_outward_offset_m": SLEEVE_RADIUS - FRAME_RADIUS,
            "visual_core_mesh": f"meshes/{visual_core_name}",
            "visual_core_mesh_sha256": sha256(visual_core_path),
            "visual_boundary_points": int(len(visual_polygon)),
            "visual_boundary_hausdorff_error_m": visual_error,
        })

    replay_time = None
    accepted = None
    path_mesh = None
    path_file = MESH_DIR / "accepted_trajectory.stl"
    path_uri = "meshes/accepted_trajectory.stl"
    if args.with_replay:
        result = json.loads(RESULT.read_text(encoding="utf-8"))
        accepted = next(
            row for row in result["rows"]
            if row["method"].startswith("Dual-Constraint")
        )
        replay_time, path_points = evaluate_saved_trajectory(
            RESULT_TRAJECTORY, PATH_SAMPLES
        )
        path_mesh = tube_mesh(path_points, PATH_RADIUS, 10, closed=False)
        path_mesh.export(path_file, file_type="stl")

    floor_route_points = np.vstack((np.asarray(START, dtype=float),
                                    np.asarray([window.center for window in GATES]),
                                    np.asarray(START, dtype=float)))
    floor_route_points[:, 2] = -5.91
    floor_route_mesh = tube_mesh(floor_route_points, 0.10, 8, closed=False)
    floor_route_file = MESH_DIR / "floor_racing_line.stl"
    floor_route_mesh.export(floor_route_file, file_type="stl")

    gate_text = "\n".join(gate_xml)
    preview_gate_text = "\n".join(preview_gate_xml)
    physics_gate_text = "\n".join(physics_gate_xml)
    quad = quadrotor_model(REPLAY_BODY_HALF_EXTENTS) if args.with_replay else ""
    replay_visual = f'''<model name="accepted_trajectory"><static>true</static><link name="path"><visual name="path_visual">{mesh_geometry(path_uri)}
    <material><ambient>0.10 0.95 0.88 0.78</ambient><diffuse>0.10 0.95 0.88 0.78</diffuse><emissive>0.05 0.55 0.50 0.78</emissive></material>
  </visual></link></model>
  {quad}''' if args.with_replay else ""
    venue = race_venue_xml(GATES)
    lab_venue = indoor_lab_xml(GATES)
    world_text = f'''<?xml version="1.0" ?>
<sdf version="1.9"><world name="seven_unique_high_fidelity">
  <physics name="one_millisecond" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
  <scene><ambient>0.18 0.20 0.25 1</ambient><background>0.025 0.035 0.060 1</background><shadows>true</shadows><grid>true</grid></scene>
  <gui fullscreen="0">
    <plugin filename="MinimalScene" name="Seven-window 3D view">
      <gz-gui><title>Seven-window course</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui>
      <engine>ogre2</engine><scene>scene</scene><ambient_light>0.28 0.31 0.38</ambient_light>
      <background_color>0.025 0.035 0.060</background_color>
      <camera_pose>0 -55 42 0 0.56 1.57079632679</camera_pose>
      <camera_clip><near>0.1</near><far>500</far></camera_clip><anti_aliasing>0</anti_aliasing>
    </plugin>
    <plugin filename="GzSceneManager" name="Scene Manager">
      <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui>
    </plugin>
    <plugin filename="InteractiveViewControl" name="Interactive view control">
      <gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui>
    </plugin>
  </gui>
  <light type="directional" name="sun"><pose>0 0 40 0 0 0</pose><cast_shadows>true</cast_shadows>
    <diffuse>0.92 0.90 0.86 1</diffuse><specular>0.35 0.35 0.35 1</specular><direction>-0.35 0.22 -0.91</direction>
  </light>
  <light type="point" name="fill"><pose>3 1 22 0 0 0</pose><cast_shadows>false</cast_shadows>
    <diffuse>0.30 0.38 0.55 1</diffuse><attenuation><range>80</range><constant>0.4</constant><linear>0.015</linear><quadratic>0.002</quadratic></attenuation>
  </light>
  <light type="spot" name="flood_nw"><pose>-24 20 15 0 .72 -.68</pose><cast_shadows>true</cast_shadows><diffuse>.62 .72 1 1</diffuse>
    <attenuation><range>80</range><constant>.3</constant><linear>.01</linear><quadratic>.001</quadratic></attenuation><spot><inner_angle>.4</inner_angle><outer_angle>1.15</outer_angle><falloff>.7</falloff></spot></light>
  <light type="spot" name="flood_se"><pose>24 -20 15 0 .72 2.46</pose><cast_shadows>true</cast_shadows><diffuse>1 .55 .72 1</diffuse>
    <attenuation><range>80</range><constant>.3</constant><linear>.01</linear><quadratic>.001</quadratic></attenuation><spot><inner_angle>.4</inner_angle><outer_angle>1.15</outer_angle><falloff>.7</falloff></spot></light>
  <model name="ground"><static>true</static><pose>0 0 -6 0 0 0</pose><link name="ground_link">
    <collision name="ground_collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
    <visual name="ground_visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
      <material><ambient>0.055 0.070 0.095 1</ambient><diffuse>0.075 0.095 0.125 1</diffuse><specular>0.18 0.18 0.18 1</specular>
        <pbr><metal><albedo_map>materials/textures/arena_floor.png</albedo_map><roughness>0.92</roughness><metalness>0.02</metalness></metal></pbr></material>
    </visual></link></model>
  <model name="floor_racing_line"><static>true</static><link name="route"><visual name="route_visual">{mesh_geometry('meshes/floor_racing_line.stl')}
    <material><ambient>.04 .48 .68 .9</ambient><diffuse>.04 .64 .90 .9</diffuse><emissive>.02 .20 .31 .9</emissive></material>
  </visual></link></model>
  <model name="start_finish_beacon"><static>true</static><pose>{START[0]} {START[1]} -5.96 0 0 0</pose><link name="beacon">
    <visual name="pad"><geometry><cylinder><radius>0.85</radius><length>0.08</length></cylinder></geometry><material><diffuse>0.08 0.95 0.32 1</diffuse><emissive>0.04 0.45 0.12 1</emissive></material></visual>
    <visual name="mast"><pose>0 0 4.56 0 0 0</pose><geometry><cylinder><radius>0.025</radius><length>9.04</length></cylinder></geometry><material><emissive>0.1 1 0.35 1</emissive></material></visual>
  </link></model>
  {replay_visual}
  {venue}
  {gate_text}
</world></sdf>'''
    if args.with_replay:
        WORLD.write_text(world_text, encoding="utf-8")
    preview_text = world_text.replace(
        'world name="seven_unique_high_fidelity"',
        'world name="seven_unique_race_preview"', 1).replace(
        'physics name="one_millisecond"',
        'physics name="preview_four_millisecond"', 1).replace(
        "<max_step_size>0.001</max_step_size>",
        f"<max_step_size>{PREVIEW_PHYSICS_STEP}</max_step_size>", 1).replace(
        gate_text, preview_gate_text, 1)
    if args.with_replay:
        PREVIEW_WORLD.write_text(preview_text, encoding="utf-8")
    physics_text = world_text.replace(
        'world name="seven_unique_high_fidelity"',
        'world name="seven_unique_physics"', 1).replace(
        'physics name="one_millisecond"',
        'physics name="physical_one_millisecond"', 1).replace(
        venue, lab_venue, 1).replace(
        gate_text, physics_gate_text, 1)
    physics_text = physics_text.replace(
        '<camera_pose>0 -55 42 0 0.56 1.57079632679</camera_pose>',
        '<camera_pose>0 -43 9.0 0 0.16 1.57079632679</camera_pose>', 1).replace(
        '<ambient>0.18 0.20 0.25 1</ambient><background>0.025 0.035 0.060 1</background>',
        '<ambient>0.64 0.66 0.68 1</ambient><background>0.73 0.75 0.77 1</background>', 1).replace(
        '<shadows>true</shadows><grid>true</grid>',
        '<shadows>false</shadows><grid>false</grid>', 1).replace(
        '<ambient_light>0.28 0.31 0.38</ambient_light>',
        '<ambient_light>0.72 0.73 0.74</ambient_light>', 1).replace(
        '<background_color>0.025 0.035 0.060</background_color>',
        '<background_color>0.73 0.75 0.77</background_color>', 1).replace(
        'materials/textures/arena_floor.png', 'materials/textures/indoor_lab_floor.png', 1).replace(
        '<ambient>0.055 0.070 0.095 1</ambient><diffuse>0.075 0.095 0.125 1</diffuse><specular>0.18 0.18 0.18 1</specular>',
        '<ambient>0.72 0.74 0.76 1</ambient><diffuse>0.84 0.85 0.86 1</diffuse><specular>0.08 0.08 0.08 1</specular>', 1)
    light_start = physics_text.index('  <light type="directional" name="sun">')
    light_end = physics_text.index('  <model name="ground">', light_start)
    physics_text = physics_text[:light_start] + physics_text[light_end:]
    for model_name in (
        "accepted_quadrotor",
        "accepted_trajectory",
        "floor_racing_line",
        "start_finish_beacon",
    ):
        physics_text = remove_model(physics_text, model_name)
    gui_start = physics_text.index('  <gui fullscreen="0">')
    gui_end = physics_text.index('  </gui>', gui_start) + len('  </gui>')
    physics_text = physics_text[:gui_start] + physics_text[gui_end:]
    PHYSICS_WORLD.write_text(physics_text, encoding="utf-8")

    # PX4's Gazebo image loads the simulation systems (including sensors,
    # magnetometer and NavSat) through its server.config.  Keep those systems
    # out of the world itself so they are instantiated exactly once.
    px4_text = physics_text.replace(
        'world name="seven_unique_physics"',
        'world name="seven_unique_px4_manual"', 1).replace(
        'physics name="physical_one_millisecond"',
        'physics name="px4_manual_four_millisecond"', 1).replace(
        '<max_step_size>0.001</max_step_size>',
        f'<max_step_size>{PX4_PHYSICS_STEP}</max_step_size>', 1)
    for filename in (
        "gz-sim-physics-system",
        "gz-sim-user-commands-system",
        "gz-sim-scene-broadcaster-system",
        "gz-sim-contact-system",
    ):
        start = px4_text.index(f'  <plugin filename="{filename}"')
        end = px4_text.index('/>', start) + len('/>')
        px4_text = px4_text[:start] + px4_text[end:]
    px4_environment = '''
  <gravity>0 0 -9.8</gravity>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
  <atmosphere type="adiabatic"/>
  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.397971057728974</latitude_deg>
    <longitude_deg>8.546163739800146</longitude_deg>
    <elevation>0</elevation>
    <heading_deg>0</heading_deg>
  </spherical_coordinates>'''
    physics_end = px4_text.index('</physics>') + len('</physics>')
    px4_text = px4_text[:physics_end] + px4_environment + px4_text[physics_end:]
    PX4_WORLD.write_text(px4_text, encoding="utf-8")

    manifest = {
        "sdf_version": "1.9",
        "gazebo_release": "Harmonic",
        "physics_world": PHYSICS_WORLD.name,
        "px4_manual_world": PX4_WORLD.name,
        "course_spec": "course_spec.py",
        "algorithm_inputs": [],
        "curve_mesh_chord_tolerance_m": MESH_CHORD_TOLERANCE_M,
        "visual_boundary_tolerance_m": PREVIEW_BOUNDARY_TOLERANCE,
        "physics_step_s": 0.001, "ground_height_m": -6.0,
        "px4_physics_step_s": PX4_PHYSICS_STEP,
        "px4_spawn_pose": list(PX4_SPAWN_POSE),
        "px4_vehicle_model": "x500",
        "frame_centerline_radius_m": FRAME_RADIUS,
        "frame_sections": FRAME_SECTIONS,
        "visual_sleeve_radius_m": SLEEVE_RADIUS,
        "visual_sleeve_outward_offset_m": SLEEVE_RADIUS - FRAME_RADIUS,
        "arena_decorations_are_visual_only": True,
        "floor_texture": {"path": str(floor_texture.relative_to(HERE)),
                          "sha256": sha256(floor_texture)},
        "lab_floor_texture": {"path": str(lab_floor_texture.relative_to(HERE)),
                              "sha256": sha256(lab_floor_texture)},
        "floor_racing_line": {"mesh": "meshes/floor_racing_line.stl",
                              "mesh_sha256": sha256(floor_route_file),
                              "mesh_triangles": int(len(floor_route_mesh.faces)),
                              "collision": False},
        "gates": gate_records,
        "coordinate_convention": "mesh xy is the original local aperture plane; model base is frozen RPY; child yaw is theta0+omega*t",
    }
    COURSE_MANIFEST.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if args.with_replay:
        assert accepted is not None and replay_time is not None and path_mesh is not None
        hard_margin = min(
            row["minimum_margin"] for row in accepted["safety_audit"]["per_window"]
        )
        replay_manifest = {
            **manifest,
            "world": WORLD.name,
            "preview_world": PREVIEW_WORLD.name,
            "preview_physics_step_s": PREVIEW_PHYSICS_STEP,
            "preview_gate_collisions": False,
            "source_result": str(RESULT.relative_to(REPO)),
            "source_result_sha256": sha256(RESULT),
            "accepted_candidate_id": int(accepted["selected_candidate_id"]),
            "accepted_flight_time_s": replay_time,
            "body_half_extents_m": REPLAY_BODY_HALF_EXTENTS.tolist(),
            "body_sphere_radius_m": REPLAY_BODY_SPHERE_RADIUS,
            "planning_sphere_radius_m": REPLAY_PLANNING_SPHERE_RADIUS,
            "hard_audit_minimum_centerline_margin_m": float(hard_margin),
            "remaining_margin_after_gazebo_frame_radius_m": float(
                hard_margin - FRAME_RADIUS - MESH_CHORD_TOLERANCE_M
            ),
            "trajectory": {
                "samples": PATH_SAMPLES,
                "mesh": path_uri,
                "mesh_sha256": sha256(path_file),
                "mesh_vertices": int(len(path_mesh.vertices)),
                "mesh_triangles": int(len(path_mesh.faces)),
            },
        }
        (HERE / "manifest.json").write_text(
            json.dumps(replay_manifest, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"wrote {WORLD}")
        print(f"wrote {PREVIEW_WORLD}")
        print(
            f"replay trajectory_samples={PATH_SAMPLES} "
            f"conservative_remaining_margin="
            f"{hard_margin - FRAME_RADIUS - MESH_CHORD_TOLERANCE_M:.9f} m"
        )
    print(f"wrote {PHYSICS_WORLD}")
    print(f"wrote {PX4_WORLD}")
    print(f"wrote {COURSE_MANIFEST}")
    print("gates=7 source=course_spec.py algorithm_inputs=none")


if __name__ == "__main__":
    main()
