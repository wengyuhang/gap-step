"""Run SC-DynaTOGT and SIP-DynaTOGT on one identical fast closed loop."""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
)
from nonconvex_timevarying_window.sc_dynatogt.collision import (
    point_to_oriented_cuboid_distance_squared,
)
from nonconvex_timevarying_window.sc_dynatogt.experiments import (
    _designated_crossings_valid,
    _sampled_dynamic_limits_satisfied,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    OptimizationConfig,
    optimize_track,
)
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.visualization import (
    export_dynamic_window_gif,
    export_trajectory_csv,
    plot_crossing_grid,
    plot_preprocessing,
    plot_route_overview,
    plot_scale_profile,
)
from nonconvex_timevarying_window.sip_dynatogt.certificate import certify
from nonconvex_timevarying_window.sip_dynatogt.constraints import point_flatness
from nonconvex_timevarying_window.sip_dynatogt.io import save_run
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateResult,
    CertificateStatus,
    PolynomialTrajectory,
    SIPConfig,
    SIPProblem,
    problem_to_dict,
)
from nonconvex_timevarying_window.sip_dynatogt.solver import solve

from .scenario import build_fast_closed_loop_scenario
from .visualization import plot_clearance_comparison, plot_contact_timeline, plot_time_comparison


DEFAULT_OUTPUT = Path(__file__).resolve().parent / "results" / "wide_scrambled_curves_v5"


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return "Infinity" if value > 0.0 else "-Infinity" if value < 0.0 else "NaN"
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def _as_minco(track, durations: np.ndarray, waypoints: np.ndarray) -> MincoSnap:
    return MincoSnap(
        BoundaryState(track.start),
        BoundaryState(track.goal),
        np.asarray(waypoints, dtype=float),
        np.asarray(durations, dtype=float),
    )


def _loose_geometry_config(config: SIPConfig) -> SIPConfig:
    return replace(
        config,
        dynamic_limits=DynamicLimits(
            max_velocity=1.0e6,
            min_collective_thrust=-1.0e6,
            max_collective_thrust=np.inf,
            max_body_rate_xy=1.0e6,
            max_body_rate_z=1.0e6,
            min_rotor_thrust=-1.0e6,
            max_rotor_thrust=1.0e6,
        ),
    )


def _collision_label(report: CertificateResult) -> str:
    if report.status is CertificateStatus.CERTIFIED_FEASIBLE:
        return "NO_COLLISION_CERTIFIED"
    if report.status is CertificateStatus.VIOLATED and any(
        witness.kind == "safety" for witness in report.witnesses
    ):
        # A positive safety residual proves that the requested nonzero
        # clearance is violated.  It does not by itself prove zero geometric
        # distance (physical intersection); that requires a separate direct
        # point-in-cuboid witness.
        return "CLEARANCE_VIOLATION_PROVED"
    return "UNRESOLVED"


def _locate(trajectory: PolynomialTrajectory, time: float) -> tuple[int, float]:
    cumulative = np.concatenate(([0.0], np.cumsum(trajectory.durations)))
    segment = min(
        int(np.searchsorted(cumulative[1:], float(time), side="right")),
        trajectory.num_segments - 1,
    )
    tau = (float(time) - cumulative[segment]) / trajectory.durations[segment]
    return segment, float(np.clip(tau, 0.0, 1.0))


def _sampled_clearance_profile(
    problem: SIPProblem,
    trajectory: PolynomialTrajectory,
    config: SIPConfig,
    *,
    num_times: int = 1001,
    boundary_samples: int = 31,
) -> dict[str, Any]:
    times = np.unique(
        np.concatenate(
            (
                np.linspace(0.0, trajectory.total_time, num_times),
                np.cumsum(trajectory.durations),
            )
        )
    )
    parameters = np.linspace(0.0, 1.0, boundary_samples)
    boundary_points = [
        [
            np.asarray([segment.evaluate(float(u)) for u in parameters], dtype=float)
            for segment in window.boundary
        ]
        for window in problem.windows
    ]
    distances = np.full(len(times), np.inf)
    nearest_window = np.full(len(times), -1, dtype=int)
    nearest_boundary = np.full(len(times), -1, dtype=int)
    for time_index, instant in enumerate(times):
        segment_index, tau = _locate(trajectory, float(instant))
        flat = point_flatness(trajectory, segment_index, tau, config)
        for window_index, window in enumerate(problem.windows):
            center, rotation, scale = window.state_at(float(instant))
            for boundary_index, local in enumerate(boundary_points[window_index]):
                world = center + (
                    rotation @ np.column_stack((scale * local, np.zeros(len(local)))).T
                ).T
                candidate = np.sqrt(
                    point_to_oriented_cuboid_distance_squared(
                        world, flat.position, flat.rotation, config.body
                    )
                )
                value = float(np.min(candidate))
                if value < distances[time_index]:
                    distances[time_index] = value
                    nearest_window[time_index] = window_index
                    nearest_boundary[time_index] = boundary_index
    best = int(np.argmin(distances))
    return {
        "time": times,
        "distance": distances,
        "nearest_window": nearest_window,
        "nearest_boundary": nearest_boundary,
        "minimum": float(distances[best]),
        "minimum_time": float(times[best]),
        "minimum_window": int(nearest_window[best]),
        "minimum_boundary": int(nearest_boundary[best]),
        "num_times": int(len(times)),
        "boundary_samples": int(boundary_samples),
    }


def _sampled_dynamics(
    trajectory: PolynomialTrajectory,
    config: SIPConfig,
    *,
    num_times: int = 4001,
) -> dict[str, Any]:
    times = np.unique(
        np.concatenate(
            (
                np.linspace(0.0, trajectory.total_time, num_times),
                np.cumsum(trajectory.durations),
            )
        )
    )
    speed = []
    body_xy = []
    body_z = []
    collective = []
    rotors = []
    force_norm = []
    heading_cross = []
    for instant in times:
        segment, tau = _locate(trajectory, float(instant))
        flat = point_flatness(trajectory, segment, tau, config)
        speed.append(float(np.linalg.norm(flat.velocity)))
        body_xy.append(float(np.linalg.norm(flat.body_rate[:2])))
        body_z.append(abs(float(flat.body_rate[2])))
        collective.append(float(flat.collective_thrust))
        rotors.append(np.asarray(flat.rotor_thrusts, dtype=float))
        force_norm.append(math.sqrt(float(flat.specific_force_norm2)))
        heading_cross.append(math.sqrt(float(flat.heading_cross_norm2)))
    rotor_values = np.asarray(rotors)
    return {
        "sample_count": len(times),
        "max_velocity": max(speed),
        "max_body_rate_xy": max(body_xy),
        "max_abs_body_rate_z": max(body_z),
        "min_collective_thrust": min(collective),
        "max_collective_thrust": max(collective),
        "min_rotor_thrust": rotor_values.min(axis=0),
        "max_rotor_thrust": rotor_values.max(axis=0),
        "min_specific_force_norm": min(force_norm),
        "min_heading_cross_norm": min(heading_cross),
    }


def _profile_csv(path: Path, profile: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time", "sampled_min_distance", "nearest_window", "nearest_boundary"))
        writer.writerows(
            zip(
                profile["time"], profile["distance"],
                profile["nearest_window"], profile["nearest_boundary"],
            )
        )


def _report_markdown(summary: dict[str, Any]) -> str:
    sc = summary["sc_dynatogt"]
    sip = summary["sip_dynatogt"]
    return f"""# SC-DynaTOGT vs SIP-DynaTOGT：快速闭环实验

## 赛道

- 起点与终点：`{summary['scenario']['start']}`（同一点）
- 窗口存储编号：`{' -> '.join(summary['scenario']['window_names'])}`
- 指定穿越顺序：`{summary['scenario']['order']}`
- 六个窗口均含周期平移、完整 RPY 旋转和均匀缩放，所有周期不超过 3.1 s。
- 初始中心空间跨度：`{summary['scenario']['center_span_xyz']} m`。
- SIP 保留 {summary['scenario']['sip_exact_boundary_segment_count']} 个原始连续边界段：`{summary['scenario']['sip_exact_boundary_type_counts']}`；SC 稠密点数仅用于映射：`{summary['scenario']['sc_dense_boundary_point_counts']}`。
- 两者使用同一长方体，半尺寸为 `{summary['scenario']['body_half_extents']} m`。
- SC 映射使用长方体外接球半径加净裕度的保守中心内缩，要求 `{summary['scenario']['required_world_clearance']:.6f} m`。
- SC/SIP 最终碰撞判定都使用姿态相关的精确长方体距离和 `{summary['scenario']['sip_clearance']:.3f} m` 净距离。

## 结果

| 指标 | SC-DynaTOGT | SIP-DynaTOGT |
|---|---:|---:|
| 总飞行时间 | {sc['total_time']:.9f} s | {sip['total_time']:.9f} s |
| 本阶段求解墙钟时间 | {sc['solve_wall_seconds']:.3f} s | {sip['solve_wall_seconds']:.3f} s |
| 计入初值生成的端到端时间 | {sc['solve_wall_seconds']:.3f} s | {sip['end_to_end_wall_seconds']:.3f} s |
| 优化器成功 | {sc['optimizer_success']} | {sip['optimizer_success']} |
| 全部硬约束状态 | {sc['full_certificate']['status']} | {sip['full_certificate']['status']} |
| 连续净距状态 | {sc['collision_status']} | {sip['collision_status']} |
| 采样最小整机距离 | {sc['sampled_minimum_clearance']:.6f} m | {sip['sampled_minimum_clearance']:.6f} m |

连续域结论以 Arb 证书为准；采样最小距离只用于定位和绘图。
"""


def run_comparison(
    output: str | Path,
    *,
    make_gif: bool = True,
    sip_initialization: str = "independent",
    max_cells: int = 2_000_000,
) -> dict[str, Any]:
    if sip_initialization not in {"independent", "sc_warm_start"}:
        raise ValueError("sip_initialization must be independent or sc_warm_start")
    root = Path(output).expanduser()
    root.mkdir(parents=True, exist_ok=True)

    print("[1/8] preprocessing exact curves for SC mapping", flush=True)
    preprocessing_started = perf_counter()
    scenario = build_fast_closed_loop_scenario()
    preprocessing_seconds = perf_counter() - preprocessing_started
    track = scenario.track
    problem = SIPProblem.from_track(track, boundaries=scenario.sip_boundaries)

    for index, gate in enumerate(scenario.preprocessed_gates):
        gate_root = root / "scenario" / "preprocessing" / f"{index:02d}_{gate.name}"
        gate.save(gate_root)
        plot_preprocessing(gate, gate_root / "preprocessing.png", samples_per_line=40)

    limits = DynamicLimits(
        max_velocity=60.0,
        max_body_rate_xy=10.0,
        max_body_rate_z=10.0,
        min_rotor_thrust=0.25,
        max_rotor_thrust=5.0,
    )
    sc_config = OptimizationConfig(
        initial_speed=20.0,
        minimum_initial_duration=0.30,
        max_iterations=400,
        samples_per_segment=12,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        penalty_weights=PenaltyWeights(
            velocity=0.0, collective_thrust=0.0, body_rate=1.0, rotor_thrust=1.0
        ),
        dynamic_limits=limits,
    )
    print("[2/8] solving SC-DynaTOGT", flush=True)
    sc_started = perf_counter()
    sc_result = optimize_track(track, config=sc_config)
    sc_solve_seconds = perf_counter() - sc_started

    sip_config = SIPConfig(
        body=scenario.body,
        clearance=scenario.net_clearance,
        dynamic_limits=limits,
        dynamic_guard_fraction=0.005,
        initial_speed=20.0,
        minimum_initial_duration=0.30,
        separator_grid_size=9,
        max_exchange_iterations=32,
        max_witnesses_per_iteration=8,
        slsqp_max_iterations=240,
        precision_bits=(128,),
        max_cells=max_cells,
        max_depth=26,
    )
    print(
        f"[3/8] solving SIP-DynaTOGT ({sum(len(x) for x in scenario.sip_boundaries)} exact boundary segments)",
        flush=True,
    )
    sip_started = perf_counter()
    sip_result = solve(
        problem,
        sip_config,
        initial_x=sc_result.x if sip_initialization == "sc_warm_start" else None,
        progress=lambda record: print(
            f"  SIP round {record.iteration + 1:02d}: "
            f"T={record.total_time:.6f}s, active={record.active_witnesses}, "
            f"status={record.certificate_status.value}, cells={record.certificate_cells}",
            flush=True,
        ),
    )
    sip_solve_seconds = perf_counter() - sip_started

    sc_poly = PolynomialTrajectory.from_minco(sc_result.trajectory)
    print("[4/8] replaying full hard constraints on SC trajectory", flush=True)
    sc_full_started = perf_counter()
    sc_full = certify(problem, sc_poly, sip_config)
    sc_full_seconds = perf_counter() - sc_full_started
    print("[5/8] certifying SC whole-body collision status", flush=True)
    sc_geometry_started = perf_counter()
    sc_geometry = certify(problem, sc_poly, _loose_geometry_config(sip_config))
    sc_geometry_seconds = perf_counter() - sc_geometry_started

    print("[6/8] deriving/replaying SIP whole-body certificate", flush=True)
    if sip_result.certificate.status is CertificateStatus.CERTIFIED_FEASIBLE:
        # The full certificate already proves every geometry cell.  Do not
        # repeat the expensive subdivision solely to discard dynamic bounds.
        sip_geometry = sip_result.certificate
        sip_geometry_seconds = 0.0
    else:
        sip_geometry_started = perf_counter()
        sip_geometry = certify(
            problem, sip_result.trajectory, _loose_geometry_config(sip_config)
        )
        sip_geometry_seconds = perf_counter() - sip_geometry_started

    print("[7/8] computing dense diagnostics and figures", flush=True)
    sc_profile = _sampled_clearance_profile(problem, sc_poly, sip_config)
    sip_profile = _sampled_clearance_profile(problem, sip_result.trajectory, sip_config)
    sc_dynamics = _sampled_dynamics(sc_poly, sip_config)
    sip_dynamics = _sampled_dynamics(sip_result.trajectory, sip_config)
    _profile_csv(root / "sc_dynatogt" / "data" / "clearance_profile.csv", sc_profile)
    _profile_csv(root / "sip_dynatogt" / "data" / "clearance_profile.csv", sip_profile)

    sip_minco = _as_minco(track, sip_result.durations, sip_result.waypoints)
    for method_root, trajectory_or_result in (
        (root / "sc_dynatogt", sc_result),
        (root / "sip_dynatogt", sip_minco),
    ):
        plot_route_overview(track, trajectory_or_result, method_root / "figures" / "route_overview.png")
        plot_crossing_grid(track, trajectory_or_result, method_root / "figures" / "crossings_grid.png", columns=2)
        plot_scale_profile(track, trajectory_or_result, method_root / "figures" / "scale_profile.png")
        export_trajectory_csv(trajectory_or_result, method_root / "data" / "trajectory.csv", num_samples=1001)
        if make_gif:
            export_dynamic_window_gif(
                track, trajectory_or_result, method_root / "media" / "dynamic_windows.gif",
                num_frames=72,
            )
    plot_clearance_comparison(
        sc_profile, sip_profile, sip_config.clearance,
        root / "comparison" / "clearance_comparison.png",
    )
    plot_contact_timeline(
        sc_profile, sip_profile, sip_config.clearance,
        root / "comparison" / "contact_timeline.png",
    )
    plot_time_comparison(
        sc_result.trajectory, sip_minco,
        root / "comparison" / "flight_time_comparison.png",
    )

    print("[8/8] writing portable result artifacts", flush=True)
    _write_json(root / "sc_dynatogt" / "result.json", sc_result.to_dict())
    save_run(root / "sip_dynatogt" / "run", problem, sip_config, sip_result)
    np.savez_compressed(
        root / "sc_dynatogt" / "candidate.npz",
        durations=sc_result.durations,
        coefficients=sc_result.trajectory.coefficients,
        x=sc_result.x,
        traversal_times=sc_result.traversal_times,
        waypoints=sc_result.waypoints,
    )

    scenario_data = {
        "name": track.name,
        "start": track.start,
        "goal": track.goal,
        "closed_loop": bool(np.array_equal(track.start, track.goal)),
        "order": track.order,
        "window_names": [window.name for window in track.windows],
        "traversal_window_names": [track.windows[index].name for index in track.order],
        "center_span_xyz": np.ptp(
            np.asarray([window.center0 for window in track.windows]), axis=0
        ),
        "body_half_extents": scenario.body.half_extents,
        "body_circumscribed_radius": scenario.body.circumscribed_radius,
        "required_world_clearance": scenario.preprocessed_gates[0].config.offset_distance
        * track.windows[0].motion.minimum_scale,
        "configured_world_clearance": track.windows[0].required_world_clearance,
        "sip_clearance": sip_config.clearance,
        "sip_exact_boundary_segment_count": sum(
            len(boundary) for boundary in scenario.sip_boundaries
        ),
        "sip_exact_boundary_type_counts": {
            kind: sum(
                type(segment).__name__ == kind
                for boundary in scenario.sip_boundaries
                for segment in boundary
            )
            for kind in ("Line", "CircularArc", "Bezier", "BSpline")
        },
        "sc_dense_boundary_point_counts": [
            len(window.physical_boundary) for window in track.windows
        ],
        "windows": [
            {
                "name": window.name,
                "center0": window.center0,
                "angles0_rpy": window.angles0,
                "translation_amplitude": window.motion.translation_amplitude,
                "rotation_amplitude_rpy": window.motion.rotation_amplitude,
                "scale_amplitude": window.motion.scale_amplitude,
                "translation_period": window.motion.translation_period,
                "rotation_period": window.motion.rotation_period,
                "scale_period": window.motion.scale_period,
                "phase": window.motion.phase,
                "scale_range": [window.motion.minimum_scale, window.motion.maximum_scale],
                "reference_local_clearance": window.reference_local_clearance,
                "guaranteed_world_clearance": window.required_world_clearance,
                "physical_boundary": window.physical_boundary,
            }
            for window in track.windows
        ],
        "sip_problem": problem_to_dict(problem),
    }
    # Use the requested value, not a reconstructed first-gate product, in reports.
    scenario_data["required_world_clearance"] = float(
        track.windows[0].required_world_clearance
    )
    _write_json(root / "scenario" / "scenario.json", scenario_data)

    summary = {
        "scenario": scenario_data,
        "common_preprocessing_wall_seconds": preprocessing_seconds,
        "sip_initialization": sip_initialization,
        "sc_dynatogt": {
            "total_time": sc_result.total_time,
            "durations": sc_result.durations,
            "traversal_times": sc_result.traversal_times,
            "waypoints": sc_result.waypoints,
            "optimizer_success": sc_result.success,
            "optimizer_iterations": sc_result.iterations,
            "solve_wall_seconds": sc_solve_seconds,
            "designated_crossings_legal": _designated_crossings_valid(track, sc_result),
            "sampled_dynamic_limits_satisfied": _sampled_dynamic_limits_satisfied(sc_result, sc_config),
            "sampled_dynamics": sc_dynamics,
            "sampled_minimum_clearance": sc_profile["minimum"],
            "sampled_minimum_clearance_time": sc_profile["minimum_time"],
            "sampled_minimum_clearance_window": sc_profile["minimum_window"],
            "sampled_minimum_clearance_boundary": sc_profile["minimum_boundary"],
            "full_certificate": sc_full.to_dict(),
            "full_certificate_wall_seconds": sc_full_seconds,
            "geometry_certificate": sc_geometry.to_dict(),
            "geometry_certificate_wall_seconds": sc_geometry_seconds,
            "collision_status": _collision_label(sc_geometry),
        },
        "sip_dynatogt": {
            "total_time": sip_result.total_time,
            "durations": sip_result.durations,
            "traversal_times": sip_result.traversal_times,
            "waypoints": sip_result.waypoints,
            "optimizer_success": sip_result.optimizer_success,
            "optimizer_iterations": sip_result.optimizer_iterations,
            "solve_wall_seconds": sip_solve_seconds,
            "end_to_end_wall_seconds": (
                sc_solve_seconds + sip_solve_seconds
                if sip_initialization == "sc_warm_start"
                else sip_solve_seconds
            ),
            "certificate_exchange_rounds": len(sip_result.history),
            "sampled_dynamics": sip_dynamics,
            "sampled_minimum_clearance": sip_profile["minimum"],
            "sampled_minimum_clearance_time": sip_profile["minimum_time"],
            "sampled_minimum_clearance_window": sip_profile["minimum_window"],
            "sampled_minimum_clearance_boundary": sip_profile["minimum_boundary"],
            "full_certificate": sip_result.certificate.to_dict(),
            "geometry_certificate": sip_geometry.to_dict(),
            "geometry_certificate_wall_seconds": sip_geometry_seconds,
            "collision_status": _collision_label(sip_geometry),
        },
    }
    _write_json(root / "summary.json", summary)
    with (root / "comparison" / "metrics.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "algorithm", "total_time", "solve_wall_seconds", "end_to_end_wall_seconds", "optimizer_success",
                "full_certificate", "collision_status", "sampled_minimum_clearance",
            ),
        )
        writer.writeheader()
        for name, values in (("sc_dynatogt", summary["sc_dynatogt"]), ("sip_dynatogt", summary["sip_dynatogt"])):
            writer.writerow(
                {
                    "algorithm": name,
                    "total_time": values["total_time"],
                    "solve_wall_seconds": values["solve_wall_seconds"],
                    "end_to_end_wall_seconds": values.get(
                        "end_to_end_wall_seconds", values["solve_wall_seconds"]
                    ),
                    "optimizer_success": values["optimizer_success"],
                    "full_certificate": values["full_certificate"]["status"],
                    "collision_status": values["collision_status"],
                    "sampled_minimum_clearance": values["sampled_minimum_clearance"],
                }
            )
    (root / "EXPERIMENT_REPORT.md").write_text(
        _report_markdown(summary), encoding="utf-8"
    )
    return _jsonable(summary)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--sip-initialization",
        choices=("independent", "sc_warm_start"),
        default="independent",
    )
    parser.add_argument("--max-cells", type=int, default=2_000_000)
    parser.add_argument("--no-gif", action="store_true")
    args = parser.parse_args(argv)
    summary = run_comparison(
        args.outdir,
        make_gif=not args.no_gif,
        sip_initialization=args.sip_initialization,
        max_cells=args.max_cells,
    )
    print(
        json.dumps(
            {
                "outdir": str(args.outdir),
                "sc_total_time": summary["sc_dynatogt"]["total_time"],
                "sip_total_time": summary["sip_dynatogt"]["total_time"],
                "sc_collision": summary["sc_dynatogt"]["collision_status"],
                "sip_collision": summary["sip_dynatogt"]["collision_status"],
                "sip_certificate": summary["sip_dynatogt"]["full_certificate"]["status"],
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["main", "run_comparison"]
