#!/usr/bin/env python3
"""Validate generated meshes, world structure, motion parameters and PX4 setup."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

from course import HERE, load_course, pose_at


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def validate_world(path: Path, course: dict, *, px4: bool,
                   expected_name: str | None = None,
                   motion_start_time: float = 0.0) -> None:
    world = ET.parse(path).getroot().find("world")
    require(world is not None, f"missing world in {path.name}")
    if expected_name is None:
        expected_name = "convex_seven_dynamic_px4" if px4 else "convex_seven_dynamic_physics"
    require(world.attrib["name"] == expected_name, f"world name mismatch: {path.name}")
    expected_step = 0.004 if px4 else 0.001
    require(abs(float(world.find("physics").findtext("max_step_size")) - expected_step) < 1e-12,
            f"physics step mismatch: {path.name}")
    if px4:
        require(world.find("spherical_coordinates") is not None, "PX4 GPS datum missing")
        require(not world.findall("plugin"), "PX4 world systems must come from server.config")
    else:
        require(len(world.findall("plugin")) == 4, "physics world system count mismatch")
    models = {model.attrib["name"]: model for model in world.findall("model")}
    for index, window in enumerate(course["windows"], start=1):
        name = f"gate_{index:02d}_{window['name']}"
        model = models.get(name)
        require(model is not None, f"missing {name}")
        expected_position, expected_rpy = pose_at(window, 0.0)
        actual_pose = np.fromstring(model.findtext("pose"), sep=" ")
        require(np.max(np.abs(actual_pose - np.r_[expected_position, expected_rpy])) < 1e-9,
                f"initial pose mismatch: {name}")
        plugin = model.find("plugin")
        require(plugin is not None and plugin.attrib["filename"] == "libPeriodicGateMotion.so",
                f"motion plugin missing: {name}")
        require(abs(float(plugin.findtext("motion_start_time")) - motion_start_time) < 1e-10,
                f"motion start mismatch: {name}")
        motion = window["motion"]
        for key in ("translation_amplitude", "rotation_amplitude"):
            require(np.max(np.abs(np.fromstring(plugin.findtext(key), sep=" ")
                                  - np.asarray(motion[key]))) < 1e-10,
                    f"{key} mismatch: {name}")
        for key in ("translation_period", "rotation_period", "phase"):
            require(abs(float(plugin.findtext(key)) - float(motion[key])) < 1e-10,
                    f"{key} mismatch: {name}")
        uri = model.findtext("./link/collision/geometry/mesh/uri")
        require((HERE / uri).is_file(), f"missing mesh: {uri}")


def main() -> None:
    course = load_course()
    manifest = json.loads((HERE / "manifest.json").read_text(encoding="utf-8"))
    require(manifest["source_sha256"] == sha256(HERE / "course_spec.json"),
            "course specification hash mismatch")
    require(manifest["gate_count"] == len(course["windows"]) == 7, "expected seven gates")
    for record in manifest["gates"]:
        for kind in ("core", "sleeve"):
            mesh_path = HERE / record[f"{kind}_mesh"]
            require(sha256(mesh_path) == record[f"{kind}_mesh_sha256"],
                    f"{kind} mesh hash mismatch")
            mesh = trimesh.load_mesh(mesh_path, process=False)
            require(len(mesh.vertices) > 0 and len(mesh.faces) > 0,
                    f"empty {kind} gate mesh")
    validate_world(HERE / manifest["physics_world"], course, px4=False)
    validate_world(HERE / manifest["px4_world"], course, px4=True)
    validate_world(
        HERE / manifest["px4_togt_world"], course, px4=True,
        expected_name="convex_seven_dynamic_px4_togt",
        motion_start_time=float(manifest["togt_motion_start_time_s"]),
    )
    require((HERE / "build/libPeriodicGateMotion.so").is_file(), "motion plugin is not built")
    result = {
        "passed": True,
        "course": course["name"],
        "gate_count": 7,
        "physics_world": manifest["physics_world"],
        "px4_world": manifest["px4_world"],
        "motion_timebase": manifest["motion_timebase"],
        "px4_vehicle": manifest["px4_vehicle"],
    }
    (HERE / "validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
