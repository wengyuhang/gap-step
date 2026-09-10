#!/usr/bin/env python3
"""Solve and audit a seven-window convex periodic 3-D course with TOGT."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from convex_timevarying_window.geometry import ConvexAperture, PeriodicConvexWindow
from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
    constraint_extrema,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig, _minimize_togt_lbfgs
from convex_timevarying_window.native_objective import NativeJointTOGTObjective


HERE = Path(__file__).resolve().parent
REPO = HERE.parent
START = np.array([-16.0, 4.0, 3.2])
GOAL = START.copy()
BODY_HALF_EXTENTS = np.array([0.26504, 0.26504, 0.0589])
BODY_RADIUS = float(np.linalg.norm(BODY_HALF_EXTENTS))
WINDOW_MARGIN_FACTOR = 1.1
WINDOW_MARGIN = WINDOW_MARGIN_FACTOR * 2.0 * BODY_RADIUS
AUDIT_STEP = 0.001

CENTERS = np.array([
    [-2.42, -3.52, 6.48],
    [20.24, 14.52, 1.80],
    [20.24, -8.80, 2.16],
    [-9.90, -13.20, 6.30],
    [10.45, -1.98, 2.16],
    [-6.16, 14.96, 2.16],
    [2.00, 8.00, 5.50],
])
ANGLES_RPY = np.array([
    [0.0, -np.pi / 2.0, 0.0],
    [0.0, -np.pi / 2.0, -np.deg2rad(20.0)],
    [0.0, -np.pi / 2.0, -np.deg2rad(130.0)],
    [0.0, -np.pi / 2.0, np.pi],
    [0.0, -np.pi / 2.0, np.deg2rad(70.0)],
    [0.0, -np.pi / 2.0, np.deg2rad(200.0)],
    [0.0, -np.pi / 2.0, np.deg2rad(145.0)],
])
SHAPE_NAMES = (
    "rectangle", "circle", "pentagon", "circle",
    "hexagon", "circle", "rectangle",
)


def _apertures() -> tuple[ConvexAperture, ...]:
    def regular(count, physical_radius):
        angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
        direction = np.column_stack((np.cos(angle), np.sin(angle)))
        return ConvexAperture(
            "polygon", margin=WINDOW_MARGIN,
            physical_vertices=physical_radius*direction,
            # Released Pentagon/Hexagon constructors subtract margin from
            # their circumradius/side length before Polyhedron::toP.
            safe_vertices=(physical_radius-WINDOW_MARGIN)*direction,
        )
    def rectangle(half_width, half_height):
        signs=np.array([[-1.,-1.],[1.,-1.],[1.,1.],[-1.,1.]])
        return ConvexAperture(
            "polygon", margin=WINDOW_MARGIN,
            physical_vertices=signs*np.array([half_width,half_height]),
            # Released Rectangle subtracts margin from the full width and
            # height, hence margin/2 on each side.
            safe_vertices=signs*np.array([
                half_width-WINDOW_MARGIN/2,
                half_height-WINDOW_MARGIN/2,
            ]),
        )
    return (
        rectangle(1.45,1.15),
        ConvexAperture("circle",radius=1.35,margin=WINDOW_MARGIN),
        regular(5,1.50),
        ConvexAperture("circle",radius=1.25,margin=WINDOW_MARGIN),
        regular(6,1.45),
        ConvexAperture("circle",radius=1.40,margin=WINDOW_MARGIN),
        rectangle(1.35,1.08),
    )


def build_track() -> tuple[SCWindowTrack, OptimizationConfig]:
    """Construct the frozen course and the released TOGT standard settings."""
    apertures = _apertures()
    phases = (0.20, -0.60, 0.90, -0.30, 1.10, -1.00, 0.70)
    translation_amplitudes = np.array([
        [0.18, 0.13, 0.10], [0.14, 0.20, 0.12], [0.20, 0.12, 0.15],
        [0.16, 0.18, 0.11], [0.13, 0.16, 0.18], [0.19, 0.15, 0.13],
        [0.15, 0.19, 0.16],
    ])
    rotation_amplitudes = np.deg2rad(np.array([
        [5.0, 7.0, 10.0], [7.0, 5.0, 9.0], [6.0, 8.0, 7.0],
        [8.0, 6.0, 10.0], [5.0, 9.0, 8.0], [7.0, 8.0, 6.0],
        [9.0, 5.0, 7.0],
    ]))
    windows = []
    for index in range(7):
        motion = MotionProfile(
            translation_amplitude=translation_amplitudes[index],
            rotation_amplitude=rotation_amplitudes[index],
            scale_amplitude=0.0,
            translation_period=10.0 + 0.8 * index,
            rotation_period=8.5 + 0.7 * index,
            scale_period=9.0,
            phase=phases[index],
            scale_enabled=False,
        )
        windows.append(PeriodicConvexWindow(
            name=f"W{index + 1}_{SHAPE_NAMES[index]}",
            aperture=apertures[index],
            center0=CENTERS[index],
            angles0_rpy=ANGLES_RPY[index],
            motion=motion,
        ))
    track = SCWindowTrack(
        name="seven_convex_periodic_3d_closed",
        start=START,
        goal=GOAL,
        windows=tuple(windows),
        order=tuple(range(7)),
    )
    config = OptimizationConfig(
        initial_speed=1.0,
        minimum_initial_duration=0.20,
        max_iterations=0,
        max_line_search_steps=64,
        memory_size=256,
        past_iterations=32,
        function_tolerance=1.0e-5,
        gradient_tolerance=0.0,
        samples_per_segment=None,
        include_window_time_gradient=True,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        penalty_weights=PenaltyWeights(
            velocity=0.0, collective_thrust=0.0, body_rate=1.0, rotor_thrust=1.0
        ),
        dynamic_limits=DynamicLimits(),
    )
    return track, config


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _source_manifest() -> dict[str, str]:
    files = sorted(HERE.glob("*.py"))
    files += sorted((HERE / "native").glob("*.cpp"))
    files += sorted((REPO / "nonconvex_timevarying_window" / "sc_dynatogt").glob("*.py"))
    released = REPO / "复现" / "TOGT-Planner-reproduction" / "source"
    files += sorted(released.glob("src/**/*.cpp"))
    files += sorted(released.glob("include/**/*.hpp"))
    files += sorted((released / "parameters/standard").glob("*.yaml"))
    return {
        str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files
    }


def scene_record(track: SCWindowTrack) -> dict[str, Any]:
    route = np.vstack((START, CENTERS, GOAL))
    windows = []
    for name, window in zip(SHAPE_NAMES, track.windows):
        windows.append({
            "name": window.name,
            "shape": name,
            "aperture_kind": window.aperture.kind,
            "togt_spatial_variable_dimension": window.aperture.dimension,
            "margin": window.aperture.margin,
            "radius": window.aperture.radius,
            "mapping": "TOGT Ball" if window.aperture.kind == "circle" else "TOGT Polyhedron squared barycentric",
            "polygon_vertices_local": (
                None if window.aperture.kind == "circle"
                else window.aperture.boundary_points(4)
            ),
            "center0": window.center0,
            "angles0_rpy_rad": window.angles0_rpy,
            "motion": asdict(window.motion),
        })
    return {
        "name": track.name,
        "start": START,
        "goal": GOAL,
        "closed_route": True,
        "order": list(track.order),
        "route_leg_distances": np.linalg.norm(np.diff(route, axis=0), axis=1),
        "body_half_extents": BODY_HALF_EXTENTS,
        "body_circumscribed_radius": BODY_RADIUS,
        "windows": windows,
    }


def dynamic_audit(trajectory, config: OptimizationConfig, maximum_step=AUDIT_STEP) -> dict[str, Any]:
    nodes = max(int(np.ceil(float(duration) / maximum_step)) + 1 for duration in trajectory.durations)
    extrema = constraint_extrema(
        trajectory, parameters=config.quadrotor, samples_per_segment=nodes
    )
    limits = config.dynamic_limits
    tests = {
        "velocity": extrema["max_velocity"] <= limits.max_velocity + 1e-9,
        "body_rate_xy": extrema["max_body_rate_xy"] <= limits.max_body_rate_xy + 1e-9,
        "body_rate_z": extrema["max_abs_body_rate_z"] <= limits.max_body_rate_z + 1e-9,
        "collective_thrust": (
            extrema["min_collective_thrust"] >= limits.min_collective_thrust - 1e-9
            and extrema["max_collective_thrust"] <= limits.max_collective_thrust + 1e-9
        ),
        "rotor_thrust": (
            np.min(extrema["min_rotor_thrust"]) >= limits.min_rotor_thrust - 1e-9
            and np.max(extrema["max_rotor_thrust"]) <= limits.max_rotor_thrust + 1e-9
        ),
    }
    return {
        "passed": bool(all(tests.values())),
        "per_constraint": tests,
        "extrema": extrema,
        "samples_per_segment": nodes,
        "maximum_step_bound_seconds": maximum_step,
        "evidence": "full-flight dense sampling of the TOGT nominal model; not a continuous certificate",
    }


def safety_audit(trajectory, track: SCWindowTrack, maximum_step=AUDIT_STEP) -> dict[str, Any]:
    count = int(np.ceil(trajectory.total_time / maximum_step)) + 1
    times = np.linspace(0.0, trajectory.total_time, count)
    positions = np.real(trajectory.evaluate(times, 0))
    per_window = []
    for index, window in enumerate(track.windows):
        margins = np.empty(count)
        for sample, (instant, position) in enumerate(zip(times, positions)):
            local, plane_distance = window.world_to_local(position, float(instant))
            edge_distance = float(window.aperture.boundary_distance(local))
            margins[sample] = np.hypot(edge_distance, plane_distance) - BODY_RADIUS
        argmin = int(np.argmin(margins))
        per_window.append({
            "window_index": index,
            "window_name": window.name,
            "passed": bool(margins[argmin] >= -1e-9),
            "minimum_margin": float(margins[argmin]),
            "minimum_margin_time": float(times[argmin]),
        })

    crossing_rows = []
    crossing_times = np.cumsum(trajectory.durations)[:-1]
    for crossing_index, (window_index, instant) in enumerate(zip(track.order, crossing_times)):
        local, plane_distance = track.windows[window_index].world_to_local(
            trajectory.evaluate(float(instant), 0), float(instant)
        )
        crossing_rows.append({
            "crossing_index": crossing_index,
            "window_index": window_index,
            "time": float(instant),
            "plane_error": abs(float(plane_distance)),
            "inside_aperture": track.windows[window_index].aperture.contains(local),
        })
    crossing_pass = all(
        row["plane_error"] <= 1e-7 and row["inside_aperture"] for row in crossing_rows
    )
    return {
        "passed": bool(all(row["passed"] for row in per_window) and crossing_pass),
        "sphere_frame_clearance_passed": bool(all(row["passed"] for row in per_window)),
        "prescribed_crossings_passed": bool(crossing_pass),
        "per_window": per_window,
        "crossings": crossing_rows,
        "sample_count": count,
        "maximum_step_bound_seconds": maximum_step,
        "obstacle_model": "finite zero-thickness polygon/circle boundary frame",
        "evidence": "full-flight dense sphere-distance sampling; not a continuous certificate",
    }


def plot_route(path: Path, trajectory, track: SCWindowTrack, crossing_times: np.ndarray) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = np.linspace(0.0, trajectory.total_time, 2401)
    points = np.real(trajectory.evaluate(times, 0))
    figure = plt.figure(figsize=(9, 7))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(points[:, 0], points[:, 1], points[:, 2], color="tab:blue", linewidth=1.7, label="TOGT trajectory")
    for i, (window, instant) in enumerate(zip(track.windows, crossing_times)):
        boundary = window.boundary_at(float(instant), circle_samples=160)
        boundary = np.vstack((boundary, boundary[0]))
        axis.plot(boundary[:, 0], boundary[:, 1], boundary[:, 2], linewidth=1.4)
        center, *_ = window.state_at(float(instant))
        axis.text(*center, f" W{i + 1}", fontsize=8)
    axis.scatter(*START, color="black", s=35, label="start = goal")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.set_title("Seven convex periodic 3-D windows")
    axis.legend(loc="best")
    axis.set_box_aspect((1.0, 0.9, 0.45))
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def write_report(path: Path, result: dict[str, Any]) -> None:
    dynamics = result["dynamic_audit"]
    safety = result["safety_audit"]
    lines = [
        "# 七窗口凸时变赛道：TOGT 实验结果", "",
        f"飞行时间：`{result['flight_time_seconds']:.9f} s`。",
        f"建图/目标初始化：`{result['timing']['preprocessing_seconds']:.6f} s`；"
        f"TOGT L-BFGS：`{result['timing']['lbfgs_seconds']:.6f} s`；"
        f"动力学验收：`{result['timing']['dynamic_audit_seconds']:.6f} s`；"
        f"安全验收：`{result['timing']['safety_audit_seconds']:.6f} s`。", "",
        "| 项目 | 结果 |", "|---|---|",
        f"| 优化器 | {'收敛' if result['optimizer']['success'] else '未收敛'} |",
        f"| 完整动力学检测 | {'满足' if dynamics['passed'] else '不满足'} |",
        f"| 完整安全检测 | {'满足' if safety['passed'] else '不满足'} |",
        "", "动力学和安全验收均采用最大 1 ms 步长的全轨迹密集采样，属于数值证据，不是连续域认证。",
        "优化目标不含安全项，严格采用 TOGT 发布的 standard 目标配置。", "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "figures").mkdir()
    started = time.perf_counter()
    manifest_start = _source_manifest()
    _write_json(output / "code_manifest_start.json", manifest_start)

    stage = time.perf_counter()
    track, config = build_track()
    objective = NativeJointTOGTObjective(track, config)
    initial_x = objective.initial_guess()
    initial_cost, _ = objective.value_and_gradient(initial_x)
    preprocessing_seconds = time.perf_counter() - stage
    _write_json(output / "scene.json", scene_record(track))
    _write_json(output / "protocol.json", {
        "objective": "TOGT released standard objective",
        "gradient_backend": "released TOGT C++ hand-written analytic derivatives and MINCO adjoint",
        "singular_attitude_and_todo_semantics": "preserved from released QuadManifold source",
        "optimization_config": asdict(config),
        "audit_step_seconds": AUDIT_STEP,
        "source_manifest": "code_manifest_start.json and code_manifest_end.json",
    })

    evaluations = 0
    last_update = time.perf_counter()
    def measured(values):
        nonlocal evaluations, last_update
        evaluations += 1
        answer = objective.scipy_value_and_gradient(values)
        now = time.perf_counter()
        if now - last_update >= 25.0:
            print(f"L-BFGS evaluations={evaluations} cost={answer[0]:.9g}", flush=True)
            last_update = now
        return answer

    print(f"preprocessing={preprocessing_seconds:.6f}s initial_cost={initial_cost:.9g}", flush=True)
    stage = time.perf_counter()
    scipy_result = _minimize_togt_lbfgs(measured, initial_x, config)
    lbfgs_seconds = time.perf_counter() - stage
    final_cost, final_gradient = objective.value_and_gradient(scipy_result.x)
    final = objective.forward(scipy_result.x)
    print(f"L-BFGS done in {lbfgs_seconds:.6f}s, T={final.trajectory.total_time:.9f}s", flush=True)

    stage = time.perf_counter()
    dynamic = dynamic_audit(final.trajectory, config)
    dynamic_seconds = time.perf_counter() - stage
    print(f"dynamic audit: {'PASS' if dynamic['passed'] else 'FAIL'} ({dynamic_seconds:.6f}s)", flush=True)
    stage = time.perf_counter()
    safety = safety_audit(final.trajectory, track)
    safety_seconds = time.perf_counter() - stage
    print(f"safety audit: {'PASS' if safety['passed'] else 'FAIL'} ({safety_seconds:.6f}s)", flush=True)

    timing = {
        "preprocessing_seconds": preprocessing_seconds,
        "lbfgs_seconds": lbfgs_seconds,
        "dynamic_audit_seconds": dynamic_seconds,
        "safety_audit_seconds": safety_seconds,
        "total_wall_seconds": time.perf_counter() - started,
    }
    result = {
        "flight_time_seconds": final.trajectory.total_time,
        "objective": final_cost,
        "dynamic_soft_integral": final_cost - final.trajectory.total_time,
        "dynamic_soft_breakdown": None,
        "optimizer": {
            "success": bool(scipy_result.success), "status": int(scipy_result.status),
            "message": str(scipy_result.message), "iterations": int(scipy_result.nit),
            "evaluations": int(scipy_result.nfev), "measured_evaluations": evaluations,
            "gradient_inf_norm": float(np.linalg.norm(final_gradient, ord=np.inf)),
            "invalid_trial_count": objective.invalid_trial_count,
        },
        "durations": final.durations,
        "traversal_times": final.traversal_times,
        "waypoints": final.waypoints,
        "local_points": final.local_points,
        "decision_vector": np.asarray(scipy_result.x),
        "dynamic_audit": dynamic,
        "safety_audit": safety,
        "timing": timing,
    }
    _write_json(output / "result.json", result)
    plot_route(output / "figures" / "route_overview.png", final.trajectory, track, final.traversal_times)
    manifest_end = _source_manifest()
    _write_json(output / "code_manifest_end.json", manifest_end)
    result["source_frozen"] = manifest_start == manifest_end
    result["timing"]["total_wall_seconds"] = time.perf_counter() - started
    _write_json(output / "result.json", result)
    write_report(output / "REPORT.md", result)
    print(f"source_frozen={result['source_frozen']} result={output / 'result.json'}", flush=True)
    return 0 if scipy_result.success else 2


if __name__ == "__main__":
    raise SystemExit(main())
