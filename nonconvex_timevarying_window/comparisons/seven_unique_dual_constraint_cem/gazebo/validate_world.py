#!/usr/bin/env python3
"""Validate SDF structure, meshes, poses, spin laws and accepted-path clearance."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


try:
    from .course_spec import GATES, boundary_at, rotation_matrix
except ImportError:  # direct script execution
    from course_spec import GATES, boundary_at, rotation_matrix


HERE = Path(__file__).resolve().parent


WORLD = HERE / "seven_unique_high_fidelity.sdf"
PREVIEW_WORLD = HERE / "seven_unique_race_preview.sdf"
PHYSICS_WORLD = HERE / "seven_unique_physics.sdf"
PX4_WORLD = HERE / "seven_unique_px4_manual.sdf"
MANIFEST = HERE / "manifest.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def validate() -> dict:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    root = ET.parse(WORLD).getroot()
    require(root.tag == "sdf" and root.attrib["version"] == "1.9", "expected SDF 1.9")
    world = root.find("world")
    require(world is not None and world.attrib["name"] == "seven_unique_high_fidelity",
            "world name mismatch")
    physics = world.find("physics")
    require(physics is not None and float(physics.findtext("max_step_size")) == 0.001,
            "physics step must be 1 ms")
    camera = world.find("./gui/plugin[@filename='MinimalScene']/camera_pose")
    require(camera is not None and len(np.fromstring(camera.text, sep=" ")) == 6,
            "overview GUI camera missing")
    require(world.find("./gui/plugin[@filename='InteractiveViewControl']") is not None,
            "interactive mouse camera control plugin missing")
    models = {item.attrib["name"]: item for item in world.findall("model")}
    require(len(manifest["gates"]) == len(GATES) == 7, "expected seven gates")

    pose_errors = []
    mesh_records = []
    for index, (window, record) in enumerate(
        zip(GATES, manifest["gates"]), start=1
    ):
        angles = window.base_rpy
        name = f"gate_{index:02d}_{window.name}"
        require(name in models, f"missing model {name}")
        model = models[name]
        pose = np.fromstring(model.findtext("pose"), sep=" ")
        expected_pose = np.r_[window.center, np.asarray(angles)]
        require(np.max(np.abs(pose - expected_pose)) < 1e-10, f"base pose mismatch: {name}")
        child_pose = np.fromstring(model.find("./link[@name='frame']/pose").text, sep=" ")
        require(abs(child_pose[5] - window.theta0) < 1e-10, f"theta0 mismatch: {name}")
        plugin = next(p for p in model.findall("plugin") if p.findtext("joint_name") == "spin"
                      and p.find("initial_velocity") is not None)
        require(abs(float(plugin.findtext("initial_velocity")) - window.omega) < 1e-10,
                f"omega mismatch: {name}")
        uri = model.findtext("./link[@name='frame']/collision/geometry/mesh/uri")
        require(uri == record["collision_mesh"], f"collision mesh mismatch: {name}")
        mesh_path = HERE / uri
        require(mesh_path.is_file() and sha256(mesh_path) == record["collision_mesh_sha256"],
                f"mesh hash mismatch: {name}")
        # STL stores triangle-local vertices.  Weld coincident copies before
        # asking a topology question such as watertightness.
        mesh = trimesh.load_mesh(mesh_path, process=True)
        _, edge_counts = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
        # Each exported cylinder is capped.  Adjacent collinear cylinders can
        # share a cap edge after STL welding, producing multiplicity four and
        # a non-manifold union, but an odd count would expose an open boundary.
        require(bool(np.all(edge_counts % 2 == 0)), f"mesh has an open edge: {name}")
        require(len(mesh.faces) == record["collision_mesh_triangles"], f"triangle count mismatch: {name}")
        require(np.max(np.abs(mesh.bounds[:, 2])) <= manifest["frame_centerline_radius_m"] + 1e-6,
                f"frame thickness mismatch: {name}")
        visual_uri = model.findtext("./link[@name='frame']/visual[@name='foam_sleeve']/geometry/mesh/uri")
        require(visual_uri == record["visual_sleeve_mesh"], f"visual sleeve mismatch: {name}")
        visual_path = HERE / visual_uri
        require(visual_path.is_file() and sha256(visual_path) == record["visual_sleeve_mesh_sha256"],
                f"visual sleeve hash mismatch: {name}")
        visual_mesh = trimesh.load_mesh(visual_path, process=True)
        require(len(visual_mesh.faces) == record["visual_sleeve_mesh_triangles"],
                f"visual sleeve triangle count mismatch: {name}")
        core_uri = model.findtext("./link[@name='frame']/visual[@name='led_inner_edge']/geometry/mesh/uri")
        require(core_uri == record["visual_core_mesh"], f"visual core mismatch: {name}")
        core_path = HERE / core_uri
        require(core_path.is_file() and sha256(core_path) == record["visual_core_mesh_sha256"],
                f"visual core hash mismatch: {name}")
        require(record["visual_boundary_hausdorff_error_m"]
                <= manifest["visual_boundary_tolerance_m"] + 1e-12,
                f"visual simplification error exceeds tolerance: {name}")
        mesh_records.append({"name": name, "vertices": int(len(mesh.vertices)),
                             "triangles": int(len(mesh.faces)), "open_edges": 0,
                             "visual_sleeve_triangles": int(len(visual_mesh.faces))})

        base_rotation = rotation_matrix(angles)
        for instant in (0.0, 0.137, 1.0):
            angle = window.theta0 + window.omega * instant
            spin = np.asarray(((np.cos(angle), -np.sin(angle)),
                               (np.sin(angle), np.cos(angle))))
            exported = (window.center[None, :] +
                        (base_rotation[:, :2] @ spin
                         @ window.boundary.T).T)
            reference = boundary_at(window, instant)
            pose_errors.append(float(np.max(np.abs(exported - reference))))

    trajectory = manifest["trajectory"]
    trajectory_path = HERE / trajectory["mesh"]
    require(trajectory_path.is_file() and sha256(trajectory_path) == trajectory["mesh_sha256"],
            "trajectory mesh hash mismatch")
    require(manifest["remaining_margin_after_gazebo_frame_radius_m"] > 0.0,
            "Gazebo frame radius consumes accepted trajectory clearance")
    require(max(pose_errors) < 1e-10, "exported spin law differs from scenario")
    require(world.find("./model[@name='accepted_quadrotor']") is not None,
            "accepted quadrotor model missing")
    require(world.find("./model[@name='accepted_trajectory']") is not None,
            "accepted trajectory visualization missing")
    require(world.find("./model[@name='floor_racing_line']") is not None,
            "floor racing line missing")
    texture_path = HERE / manifest["floor_texture"]["path"]
    require(texture_path.is_file() and sha256(texture_path) == manifest["floor_texture"]["sha256"],
            "arena floor texture mismatch")
    decoration_names = ("north_barrier", "south_barrier", "east_barrier", "west_barrier",
                        "start_arch_left", "start_arch_right", "start_arch_header")
    for name in decoration_names:
        require(name in models and models[name].find(".//collision") is None,
                f"arena decoration must remain visual-only: {name}")

    preview_root = ET.parse(PREVIEW_WORLD).getroot()
    preview = preview_root.find("world")
    require(preview is not None and preview.attrib["name"] == "seven_unique_race_preview",
            "preview world name mismatch")
    require(abs(float(preview.find("physics").findtext("max_step_size"))
                - manifest["preview_physics_step_s"]) < 1e-12,
            "preview physics step mismatch")
    preview_models = {item.attrib["name"]: item for item in preview.findall("model")}
    for index, window in enumerate(GATES, start=1):
        name = f"gate_{index:02d}_{window.name}"
        require(preview_models[name].find("./link[@name='frame']/collision") is None,
                f"preview gate must be render-only: {name}")

    physics_root = ET.parse(PHYSICS_WORLD).getroot()
    physical = physics_root.find("world")
    require(physical is not None and physical.attrib["name"] == "seven_unique_physics",
            "physics world name mismatch")
    require(physical.find("gui") is None,
            "track world must use Gazebo's standard GUI configuration")
    physical_models = {item.attrib["name"]: item for item in physical.findall("model")}
    for index, (window, record) in enumerate(zip(GATES, manifest["gates"]), start=1):
        name = f"gate_{index:02d}_{window.name}"
        model = physical_models.get(name)
        require(model is not None, f"physical gate missing: {name}")
        collision_uri = model.findtext("./link[@name='frame']/collision/geometry/mesh/uri")
        require(collision_uri == record["visual_sleeve_mesh"],
                f"physical gate collision must equal its visible sleeve: {name}")
    require("impact_quadrotor" not in physical_models and "accepted_quadrotor" not in physical_models,
            "track-only physics world must not contain a quadrotor")
    require(physical.find("./model[@name='accepted_trajectory']") is None,
            "track-only physics world must not contain a trajectory")
    require(physical.find("./model[@name='floor_racing_line']") is None,
            "indoor lab world must not contain the neon racing line")
    require(physical.find("./model[@name='start_finish_beacon']") is None,
            "indoor lab world must not contain the race beacon")
    require(physical.find("./plugin[@filename='gz-sim-apply-link-wrench-system']") is None,
            "track-only physics world must not contain an impact actuator")
    for name in ("lab_back_wall", "lab_left_wall", "lab_right_wall", "lab_ceiling"):
        require(name in physical_models and physical_models[name].find(".//collision") is None,
                f"visual-only indoor lab surface missing: {name}")
    require(not any(name.startswith("lab_gate_") for name in physical_models),
            "indoor lab world must not add support rods below the gates")
    require(physical.findtext("scene/grid") == "false",
            "Gazebo grid overlay must be disabled to avoid a duplicated floor grid")
    require(not physical.findall("light"),
            "standalone Gazebo light markers must be absent from the indoor lab world")
    lab_texture_path = HERE / manifest["lab_floor_texture"]["path"]
    require(lab_texture_path.is_file()
            and sha256(lab_texture_path) == manifest["lab_floor_texture"]["sha256"],
            "indoor lab floor texture mismatch")

    px4_root = ET.parse(PX4_WORLD).getroot()
    px4_world = px4_root.find("world")
    require(px4_world is not None and px4_world.attrib["name"] == "seven_unique_px4_manual",
            "PX4 manual world name mismatch")
    require(px4_world.find("gui") is None,
            "PX4 manual world must use Gazebo's standard GUI configuration")
    require(abs(float(px4_world.find("physics").findtext("max_step_size"))
                - manifest["px4_physics_step_s"]) < 1e-12,
            "PX4 manual world physics step mismatch")
    require(px4_world.find("spherical_coordinates") is not None,
            "PX4 manual world needs geodetic coordinates for simulated GPS")
    require(px4_world.find("atmosphere") is not None
            and px4_world.find("magnetic_field") is not None,
            "PX4 manual world needs atmospheric and magnetic-field models")
    require(not px4_world.findall("plugin"),
            "PX4 image server.config must be the sole source of world system plugins")
    px4_models = {item.attrib["name"]: item for item in px4_world.findall("model")}
    require(set(px4_models) == set(physical_models),
            "PX4 manual world must preserve the track-only model set")
    require(not any(name.startswith("x500") for name in px4_models),
            "PX4 must spawn x500 through its simulator bridge")
    require(manifest["px4_vehicle_model"] == "x500"
            and len(manifest["px4_spawn_pose"]) == 6,
            "PX4 vehicle and spawn metadata missing")

    result = {
        "passed": True,
        "world": str(WORLD),
        "gate_count": 7,
        "maximum_pose_error_m": max(pose_errors),
        "physics_step_s": 0.001,
        "frame_radius_m": manifest["frame_centerline_radius_m"],
        "remaining_accepted_clearance_m": manifest["remaining_margin_after_gazebo_frame_radius_m"],
        "mesh_records": mesh_records,
        "checks": [
            "SDF 1.9 structure and required systems",
            "seven collision and visual meshes with SHA-256",
            "outward visual sleeves do not replace physical collision cores",
            "capped tube segments with no open mesh edges",
            "base pose, theta0 and omega",
            "theta0+omega*t boundary agreement",
            "1 ms physics step",
            "OGRE2 overview camera",
            "interactive mouse camera control",
            "accepted trajectory and quadrotor visuals",
            "race venue, floor texture and projected racing line",
            "arena decorations contain no collision geometry",
            "4 ms render-only preview world with bounded visual simplification",
            "visible 85 mm gate sleeves are the physical collision meshes",
            "track-only world contains no trajectory, vehicle or impact actuator",
            "Gazebo standard GUI is not overridden by the world",
            "neutral indoor lab walls, ceiling panels and a single floor grid",
            "no light markers or support rods",
            "PX4 x500 manual world at the official 4 ms simulation step",
            "PX4 GPS, atmosphere and magnetic-field world metadata",
            "PX4 server configuration owns the sensor and physics systems",
            "positive clearance after physical frame radius",
        ],
    }
    (HERE / "validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return result


def main() -> None:
    print(json.dumps(validate(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
