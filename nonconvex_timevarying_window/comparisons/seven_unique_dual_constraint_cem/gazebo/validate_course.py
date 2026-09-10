#!/usr/bin/env python3
"""Validate only the algorithm-independent Gazebo course assets."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

try:
    from .course_spec import GATES, MESH_CHORD_TOLERANCE_M
except ImportError:  # direct script execution
    from course_spec import GATES, MESH_CHORD_TOLERANCE_M


HERE = Path(__file__).resolve().parent
MANIFEST = HERE / "course_manifest.json"
PHYSICS_WORLD = HERE / "seven_unique_physics.sdf"
PX4_WORLD = HERE / "seven_unique_px4_manual.sdf"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def validate() -> dict:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    require(manifest["course_spec"] == "course_spec.py", "course source is not explicit")
    require(manifest["algorithm_inputs"] == [], "course depends on algorithm inputs")
    require(
        manifest["curve_mesh_chord_tolerance_m"] == MESH_CHORD_TOLERANCE_M,
        "curve mesh tolerance mismatch",
    )
    require(len(manifest["gates"]) == len(GATES) == 7, "expected seven gates")

    physical = ET.parse(PHYSICS_WORLD).getroot().find("world")
    require(
        physical is not None and physical.attrib["name"] == "seven_unique_physics",
        "physics world name mismatch",
    )
    require(physical.find("gui") is None, "course must use the standard Gazebo GUI")
    require(
        abs(float(physical.find("physics").findtext("max_step_size")) - 0.001) < 1e-12,
        "course physics step mismatch",
    )
    models = {model.attrib["name"]: model for model in physical.findall("model")}
    for index, (gate, record) in enumerate(zip(GATES, manifest["gates"]), start=1):
        name = f"gate_{index:02d}_{gate.name}"
        model = models.get(name)
        require(model is not None, f"missing course gate {name}")
        pose = np.fromstring(model.findtext("pose"), sep=" ")
        require(
            np.max(np.abs(pose - np.r_[gate.center, gate.base_rpy])) < 1e-10,
            f"pose mismatch: {name}",
        )
        frame_pose = np.fromstring(
            model.find("./link[@name='frame']/pose").text, sep=" "
        )
        require(abs(frame_pose[5] - gate.theta0) < 1e-10, f"phase mismatch: {name}")
        controller = next(
            plugin for plugin in model.findall("plugin")
            if plugin.findtext("joint_name") == "spin"
        )
        require(
            abs(float(controller.findtext("initial_velocity")) - gate.omega) < 1e-10,
            f"angular velocity mismatch: {name}",
        )
        uri = model.findtext(
            "./link[@name='frame']/collision/geometry/mesh/uri"
        )
        require(uri == record["visual_sleeve_mesh"], f"collision URI mismatch: {name}")
        mesh = HERE / uri
        require(mesh.is_file(), f"missing collision mesh: {name}")
        require(
            sha256(mesh) == record["visual_sleeve_mesh_sha256"],
            f"collision mesh hash mismatch: {name}",
        )

    require("accepted_trajectory" not in models, "course contains an algorithm trajectory")
    require("accepted_quadrotor" not in models, "course contains a replay vehicle")

    px4 = ET.parse(PX4_WORLD).getroot().find("world")
    require(
        px4 is not None and px4.attrib["name"] == "seven_unique_px4_manual",
        "PX4 course world name mismatch",
    )
    require(
        abs(float(px4.find("physics").findtext("max_step_size")) - 0.004) < 1e-12,
        "PX4 physics step mismatch",
    )
    require(px4.find("spherical_coordinates") is not None, "PX4 GPS datum missing")
    require(not px4.findall("plugin"), "PX4 systems must come from server.config")
    require(
        {model.attrib["name"] for model in px4.findall("model")} == set(models),
        "PX4 world changed the course model set",
    )

    result = {
        "passed": True,
        "source": "course_spec.py",
        "algorithm_inputs": [],
        "gate_count": len(GATES),
        "curve_mesh_chord_tolerance_m": MESH_CHORD_TOLERANCE_M,
        "physics_world": PHYSICS_WORLD.name,
        "px4_world": PX4_WORLD.name,
    }
    (HERE / "course_validation.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return result


if __name__ == "__main__":
    print(json.dumps(validate(), ensure_ascii=False, indent=2))
