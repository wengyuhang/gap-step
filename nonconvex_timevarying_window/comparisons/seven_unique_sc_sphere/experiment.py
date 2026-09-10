#!/usr/bin/env python3
"""Solve SC-DynaTOGT on seven distinct windows and audit sphere collisions."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from functools import lru_cache
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from nonconvex_timevarying_window.comparisons.seven_mixed_reference_sc_fixed_cem.experiment import (
    _frozen_u_context,
    _mixed_gate_catalog,
)
from nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.experiment import solve
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import sphere_check
from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import RotatingWindow, basis_from_normal
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import RotSyncScenario, preprocess_shape_catalog
from nonconvex_timevarying_window.sc_dynatogt.environment import rotation_and_derivative
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessedGate
from nonconvex_timevarying_window.sc_dynatogt.visualization import plot_route_overview


HERE = Path(__file__).resolve().parent
PREPROCESSED_ROOT = HERE / "preprocessed_gates"
SHAPES = ("L", "U", "star", "limacon", "wavy", "line_bezier", "balanced_U")
CENTERS = (
    (-2.42, -3.52, 6.48),
    (20.24, 14.52, 1.80),
    (20.24, -8.80, 2.16),
    (-9.90, -13.20, 6.30),
    (10.45, -1.98, 2.16),
    (-6.16, 14.96, 2.16),
    (2.00, 8.00, 5.50),
)
ANGLES_RPY = (
    (0.0, -np.pi / 2.0, 0.0),
    (0.0, -np.pi / 2.0, -np.deg2rad(20.0)),
    (0.0, -np.pi / 2.0, -np.deg2rad(130.0)),
    (0.0, -np.pi / 2.0, np.pi),
    (0.0, -np.pi / 2.0, np.deg2rad(70.0)),
    (0.0, -np.pi / 2.0, np.deg2rad(200.0)),
    (0.0, -np.pi / 2.0, np.deg2rad(145.0)),
)
PHASES = (0.40, -0.70, 0.55, -0.35, 0.90, -1.10, 1.25)
OMEGAS = (5.0, -4.0, 4.5, -5.0, 5.5, -15.0, 18.0)
START = (-16.0, 4.0, 3.2)
GOAL = START


@lru_cache(maxsize=1)
def _extra_gates(rho: float):
    """Load the standard L/U artifacts, creating each SC map only once."""

    requested = {"L": 256, "U": 64}
    gates = {}
    for name, vertex_count in requested.items():
        directory = PREPROCESSED_ROOT / name
        if (directory / "manifest.json").is_file():
            gate = PreprocessedGate.load(directory)
        else:
            gate = preprocess_shape_catalog(
                rho=rho,
                vertex_count=vertex_count,
                quadrature_order=64,
                shape_names=(name,),
            )[name]
            directory.mkdir(parents=True, exist_ok=True)
            gate.save(directory)
        if abs(gate.safe_region.distance - rho) > 1.0e-12:
            raise ValueError(f"cached {name} clearance does not match rho")
        gates[name] = gate
    return gates


@lru_cache(maxsize=1)
def build_seven_unique_track():
    config, balanced, rho, body = _frozen_u_context()
    catalog = _mixed_gate_catalog(rho=rho, vertex_count=256, quadrature_order=64)
    catalog.update(_extra_gates(rho))
    gates = tuple(
        balanced if name == "balanced_U" else catalog[name]
        for name in SHAPES
    )
    windows = []
    for i, (shape, gate, center, angles, phase, omega) in enumerate(
        zip(SHAPES, gates, CENTERS, ANGLES_RPY, PHASES, OMEGAS)
    ):
        rotation, _ = rotation_and_derivative(angles, np.zeros(3))
        windows.append(RotatingWindow(
            name=f"W{i + 1}_{shape}",
            gate=gate,
            center=center,
            plane_basis=rotation[:, :2],
            normal=rotation[:, 2],
            theta0=phase,
            omega=omega,
            thickness=0.0,
            rho=rho,
        ))
    scenario = RotSyncScenario(
        "seven_unique_shape_irregular_closed_spin",
        BoundaryState(np.asarray(START, dtype=float)),
        BoundaryState(np.asarray(GOAL, dtype=float)),
        tuple(windows),
        "Seven dispersed, distinct fixed-plane apertures with in-plane spin on a closed route.",
        body,
        "seven-unique-irregular-closed-sphere-collision",
        ("paper-irregular closed layout", "seven distinct shapes", "spinning-window sphere audit"),
    )
    return scenario, config


def warm_start_vector(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    row = next(row for row in payload["rows"] if row["method"] == "SC-DynaTOGT")
    return np.asarray(row["decision_vector"], dtype=float)


def scene_record(scenario) -> dict:
    route_points = np.asarray((START, *CENTERS, GOAL), dtype=float)
    return {
        "name": scenario.name,
        "shape_sequence": SHAPES,
        "start": scenario.start_state.matrix,
        "goal": scenario.goal_state.matrix,
        "closed_loop": bool(np.array_equal(scenario.start_state.matrix, scenario.goal_state.matrix)),
        "route_leg_distances": np.linalg.norm(np.diff(route_points, axis=0), axis=1),
        "sphere_obstacle_model": "finite_zero_thickness_boundary_frame",
        "body_half_extents": scenario.body.half_extents,
        "windows": [
            {
                "name": window.name,
                "center": window.center,
                "normal": window.normal,
                "plane_basis": window.plane_basis,
                "theta0": window.theta0,
                "omega": window.omega,
                "thickness": window.thickness,
                "rho": window.rho,
                "physical_polygon": window.physical_polygon,
                "safe_polygon": window.safe_polygon,
            }
            for window in scenario.windows
        ],
    }


def audit_spheres(forward, scenario) -> list[dict]:
    records = []
    for index, window in enumerate(scenario.windows):
        result = sphere_check(
            forward.trajectory,
            window,
            window.rho,
            obstacle_model="boundary_frame",
            stop_at_first_violation=False,
        )
        records.append({
            "window_index": index,
            "window_name": window.name,
            **result,
        })
    return records


def write_report(path: Path, result: dict) -> None:
    lines = [
        "# 七种形状窗口：SC-DynaTOGT 球体碰撞结果",
        "",
        f"飞行时间：`{result['flight_time']:.9f} s`。",
        f"球体碰撞窗口数：`{result['collision_window_count']}/7`。",
        "障碍物：零厚度有限物理门框边界（不是开口外整张无限实体平面）。",
        "",
        "| 窗口 | 形状 | 角速度 (rad/s) | 最小裕度 (m) | 球体检查 |",
        "|---:|---|---:|---:|---|",
    ]
    for record, shape, omega in zip(result["sphere_audit"], SHAPES, OMEGAS):
        state = "通过" if record["passed"] else "碰撞"
        lines.append(
            f"| W{record['window_index'] + 1} | {shape} | {omega:.1f} | "
            f"{record['minimum_margin']:.9f} | {state} |"
        )
    lines.extend((
        "",
        "检查覆盖球体与窗口平面接触的全部时间区间；结果是密集采样数值证据，不是连续时间证书。",
        f"优化器状态：`{result['optimizer_message']}`。",
    ))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def plot_closed_route(scenario, trajectory, output: Path) -> None:
    """Adapt the fixed-plane spinning windows to the common route renderer."""

    class DisplayWindow:
        def __init__(self, source):
            self.source = source

        def physical_boundary_at(self, instant):
            return self.source.boundary_at(float(instant))

        def polygon_at(self, instant):
            return self.source.boundary_at(float(instant), safe=True)

    track = SimpleNamespace(
        start=scenario.start_state.position,
        goal=scenario.goal_state.position,
        windows=tuple(DisplayWindow(window) for window in scenario.windows),
        order=tuple(range(len(scenario.windows))),
    )
    plot_route_overview(
        track,
        trajectory,
        output,
        num_samples=601,
        method_label="SC-DynaTOGT",
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--warm-start", type=Path)
    args = parser.parse_args(argv)

    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    scenario, config = build_seven_unique_track()
    write_json(output / "scene.json", scene_record(scenario))
    objective = MultiWindowObjective(scenario, config)
    initial = objective.initial_guess() if args.warm_start is None else warm_start_vector(args.warm_start)

    print("Solving original SC-DynaTOGT on seven distinct windows", flush=True)
    optimizer, forward, solve_seconds, calls = solve(
        "SC-DynaTOGT-seven-unique", objective, initial)
    print("Auditing all seven sphere crossing intervals", flush=True)
    audit = audit_spheres(forward, scenario)
    collisions = [record for record in audit if not record["passed"]]
    result = {
        "scene": scenario.name,
        "method": "SC-DynaTOGT",
        "shape_sequence": SHAPES,
        "optimizer_success": bool(optimizer.success),
        "optimizer_message": str(optimizer.message),
        "iterations": int(optimizer.nit),
        "objective_evaluations": calls,
        "objective": float(optimizer.fun),
        "flight_time": float(forward.trajectory.total_time),
        "solve_seconds": solve_seconds,
        "total_seconds": time.perf_counter() - started,
        "decision_vector": np.asarray(optimizer.x),
        "durations": np.asarray(forward.durations),
        "crossing_times": np.asarray(forward.crossing_times),
        "local_points": np.asarray(forward.local_points),
        "sphere_audit": audit,
        "collision_window_count": len(collisions),
        "collision_windows": [record["window_name"] for record in collisions],
        "requirement_at_least_two_collisions": len(collisions) >= 2,
        "evidence": "sampled sphere crossing-interval check; not continuous certification",
        "config": asdict(config),
    }
    write_json(output / "result.json", result)
    np.savez_compressed(
        output / "sc_dynatogt_trajectory.npz",
        x=np.asarray(optimizer.x),
        coefficients=forward.trajectory.coefficients,
        durations=forward.durations,
        crossing_times=forward.crossing_times,
        local_points=forward.local_points,
    )
    plot_closed_route(scenario, forward.trajectory, output / "figures" / "route_overview.png")
    write_report(output / "REPORT.md", jsonable(result))
    print(json.dumps(jsonable(result), ensure_ascii=False), flush=True)
    return 0 if len(collisions) >= 2 else 2


if __name__ == "__main__":
    raise SystemExit(main())
