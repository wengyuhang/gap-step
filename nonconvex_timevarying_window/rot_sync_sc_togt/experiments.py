"""Formal closed-track experiment runner with complete research exports."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, is_dataclass
import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from shapely.geometry import Point, Polygon
from nonconvex_timevarying_window.sc_dynatogt.dynamics import QuadrotorParameters

from .collision import sample_collision_report
from .optimizer import RotSyncOptimizationConfig, RotSyncOptimizationResult, optimize_track
from .scenarios import (
    DEFAULT_RHO,
    REALISTIC_RHO,
    REALISTIC_SHAPE_SCALES,
    RotSyncScenario,
    build_formal_scenarios,
    build_multi_scenarios,
    build_realistic_extreme_scenario,
    build_smoke_scenario,
    preprocess_shape_catalog,
    scenario_difficulty_metrics,
)
from .visualization import (
    export_animation,
    export_trajectory_csv,
    plot_sync_closeups,
    plot_trajectory,
)


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def _scenario_config(
    scenario: RotSyncScenario,
    config: RotSyncOptimizationConfig,
    *,
    collision_samples: int,
) -> dict[str, Any]:
    return {
        "method": "rot_sync_sc_togt",
        "scenario": scenario.name,
        "description": scenario.description,
        "difficulty": scenario.difficulty,
        "design_basis": scenario.design_basis,
        "difficulty_metrics": scenario_difficulty_metrics(scenario),
        "closed": scenario.closed,
        "start_state_pvaj": scenario.start_state.matrix,
        "goal_state_pvaj": scenario.goal_state.matrix,
        "decision_vector": "[K_free(0..N), K_sync(0..N-1), d_0..d_(N-1)]",
        "objective": "T_total + lambda_s * integral_snap_squared + lambda_d * P_dyn",
        "optimization": config,
        "cuboid_body": {
            "half_extents": scenario.body.half_extents,
            "full_dimensions": 2.0 * np.asarray(scenario.body.half_extents),
            "circumscribed_radius_rho": scenario.body.circumscribed_radius,
        },
        "collision_audit": {
            "samples": collision_samples,
            "model": "oriented cuboid against rotating aperture boundary extruded through thickness",
        },
        "windows": [
            {
                "name": window.name,
                "center": window.center,
                "plane_basis": window.plane_basis,
                "normal": window.normal,
                "theta0": window.theta0,
                "omega": window.omega,
                "thickness": window.thickness,
                "rho": window.rho,
                "D": window.clearance_distance,
                "safe_polygon": window.safe_polygon,
                "physical_polygon": window.physical_polygon,
            }
            for window in scenario.windows
        ],
    }


def _sampled_dynamic_validation(
    result: RotSyncOptimizationResult,
    config: RotSyncOptimizationConfig,
) -> dict[str, Any]:
    extrema = result.extrema
    limits = config.dynamic_limits
    tolerance = 1.0e-9
    checks = {
        "velocity": bool(extrema["max_velocity"] <= limits.max_velocity + tolerance),
        "collective_thrust": bool(
            extrema["min_collective_thrust"] >= limits.min_collective_thrust - tolerance
            and extrema["max_collective_thrust"] <= limits.max_collective_thrust + tolerance
        ),
        "body_rate_xy": bool(
            extrema["max_body_rate_xy"] <= limits.max_body_rate_xy + tolerance
        ),
        "body_rate_z": bool(
            extrema["max_abs_body_rate_z"] <= limits.max_body_rate_z + tolerance
        ),
        "rotor_thrust": bool(
            np.min(extrema["min_rotor_thrust"]) >= limits.min_rotor_thrust - tolerance
            and np.max(extrema["max_rotor_thrust"]) <= limits.max_rotor_thrust + tolerance
        ),
    }
    return {"sampled_dynamic_limits_satisfied": bool(all(checks.values())), "checks": checks}


def validate_solution(
    scenario: RotSyncScenario,
    result: RotSyncOptimizationResult,
    config: RotSyncOptimizationConfig,
) -> dict[str, Any]:
    local_lock_errors, safe_margins = [], []
    for index, (window, sync) in enumerate(zip(scenario.windows, result.forward.trajectory.sync_segments)):
        local_times = np.linspace(0.0, sync.duration, 41)
        positions = sync.evaluate(local_times)
        recovered = []
        for tau, position in zip(local_times, positions):
            absolute = sync.entry_time + float(tau)
            z = -window.clearance_distance + 2.0 * window.clearance_distance * tau / sync.duration
            in_plane = position - window.center - window.normal * z
            recovered.append(window.rotated_basis(absolute).T @ in_plane)
        recovered_array = np.asarray(recovered)
        local_lock_errors.append(float(np.max(np.linalg.norm(recovered_array - sync.local_point, axis=1))))
        polygon = Polygon(window.safe_polygon)
        point = Point(sync.local_point)
        if not polygon.covers(point):
            safe_margins.append(-float(point.distance(polygon)))
        else:
            safe_margins.append(float(point.distance(polygon.boundary)))
    endpoint_state_error = float(
        np.max(
            np.abs(
                np.stack(
                    [result.forward.trajectory.evaluate(result.total_time, order) for order in range(4)]
                )
                - scenario.goal_state.matrix
            )
        )
    )
    return {
        "all_q_in_safe_opening": bool(all(margin >= -1.0e-10 for margin in safe_margins)),
        "minimum_safe_margin": float(min(safe_margins)),
        "safe_margins": safe_margins,
        "maximum_sync_local_lock_error": float(max(local_lock_errors)),
        "sync_local_lock_errors": local_lock_errors,
        "maximum_c3_interface_jump": result.max_c3_jump,
        "c3_continuous": bool(result.max_c3_jump <= 1.0e-8),
        "closed_endpoint_state": scenario.closed,
        "endpoint_state_error": endpoint_state_error,
        **_sampled_dynamic_validation(result, config),
    }


def run_scenario(
    scenario: RotSyncScenario,
    output: str | Path,
    *,
    config: RotSyncOptimizationConfig | None = None,
    make_animation: bool = True,
    animation_frames: int = 100,
    collision_samples: int = 2001,
) -> tuple[RotSyncOptimizationResult, dict[str, Any]]:
    settings = RotSyncOptimizationConfig() if config is None else config
    root = Path(output)
    root.mkdir(parents=True, exist_ok=True)
    _write_json(
        root / "config.json",
        _scenario_config(scenario, settings, collision_samples=collision_samples),
    )
    for window in scenario.windows:
        window.gate.save(root / "preprocessed" / window.name)
    result = optimize_track(scenario, config=settings)
    validation = validate_solution(scenario, result, settings)
    collision = sample_collision_report(
        scenario,
        result.forward.trajectory,
        samples=collision_samples,
        parameters=settings.quadrotor,
    )
    collision_values = collision.to_dict()
    trajectory_validation_pass = bool(
        validation["all_q_in_safe_opening"]
        and validation["c3_continuous"]
        and validation["closed_endpoint_state"]
        and validation["endpoint_state_error"] <= 1.0e-8
        and validation["sampled_dynamic_limits_satisfied"]
        and not collision.any_collision
    )
    validation = {
        **validation,
        "collision_free": not collision.any_collision,
        "trajectory_validation_pass": trajectory_validation_pass,
        "formal_acceptance_pass": bool(result.success and trajectory_validation_pass),
        "collision": collision_values,
    }
    _write_json(
        root / "result.json",
        {**result.to_dict(), "validation": validation, "collision": collision_values},
    )
    export_trajectory_csv(scenario, result, root / "trajectory.csv")
    plot_trajectory(
        scenario,
        result,
        root / "trajectory_3d.png",
        collision_report=collision,
    )
    plot_sync_closeups(scenario, result, root / "sync_closeups.png")
    if make_animation:
        export_animation(
            scenario,
            result,
            root / "rotation_sync.gif",
            frames=animation_frames,
            collision_report=collision,
        )
    return result, validation


def _summary_row(
    scenario: RotSyncScenario,
    result: RotSyncOptimizationResult,
    validation: dict[str, Any],
) -> dict[str, Any]:
    metrics = scenario_difficulty_metrics(scenario)
    return {
        "scenario": result.scenario_name,
        "difficulty": scenario.difficulty,
        "gate_count": metrics["gate_count"],
        "shape_sequence": " -> ".join(metrics["shape_sequence"]),
        "unique_shape_count": metrics["unique_shape_count"],
        "nominal_route_length": metrics["nominal_route_length"],
        "altitude_range": metrics["altitude_range"],
        "maximum_turn_angle_deg": metrics["maximum_turn_angle_deg"],
        "maximum_abs_omega": metrics["maximum_abs_omega"],
        "closed": scenario.closed,
        "optimizer_success": result.success,
        "trajectory_validation_pass": validation["trajectory_validation_pass"],
        "formal_acceptance_pass": validation["formal_acceptance_pass"],
        "total_time": result.total_time,
        "solve_time": result.solve_time,
        "selected_q": json.dumps(result.forward.local_points.tolist(), ensure_ascii=False),
        "max_velocity": result.extrema["max_velocity"],
        "max_acceleration": result.max_acceleration,
        "minimum_safe_margin": validation["minimum_safe_margin"],
        "maximum_sync_local_lock_error": validation["maximum_sync_local_lock_error"],
        "maximum_c3_interface_jump": result.max_c3_jump,
        "sampled_dynamic_limits_satisfied": validation["sampled_dynamic_limits_satisfied"],
        "cuboid_collision_rate": validation["collision"]["sampled_collision_rate"],
        "cuboid_colliding_samples": validation["collision"]["colliding_sample_count"],
    }


def _write_summary(path: Path, rows: Iterable[dict[str, Any]]) -> Path:
    records = list(rows)
    fields = list(records[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Rotation-synchronised SC/MINCO experiments")
    parser.add_argument(
        "--suite",
        choices=("smoke", "multi", "formal", "realistic_extreme"),
        default="formal",
    )
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path("nonconvex_timevarying_window/rot_sync_sc_togt/results"),
    )
    parser.add_argument("--max-iterations", type=int)
    parser.add_argument("--samples-per-segment", type=int)
    parser.add_argument("--dynamics-weight", type=float)
    parser.add_argument("--vertex-count", type=int)
    parser.add_argument("--quadrature-order", type=int)
    parser.add_argument("--animation-frames", type=int)
    parser.add_argument("--collision-samples", type=int)
    parser.add_argument("--no-animation", action="store_true")
    args = parser.parse_args(argv)

    production = args.suite in {"formal", "realistic_extreme"}
    realistic = args.suite == "realistic_extreme"
    vertex_count = args.vertex_count or (256 if production else 64)
    quadrature_order = args.quadrature_order or (64 if production else 32)
    max_iterations = args.max_iterations or (120 if production else 40)
    samples_per_segment = args.samples_per_segment or (11 if production else 7)
    dynamics_weight = (
        args.dynamics_weight
        if args.dynamics_weight is not None
        else (0.1 if production else 0.002)
    )
    animation_frames = args.animation_frames or (220 if realistic else (140 if production else 100))
    collision_samples = args.collision_samples or (5001 if production else 2001)
    requested_shapes = {
        "smoke": ("L",),
        "multi": ("L", "U", "star"),
        "formal": None,
        "realistic_extreme": None,
    }[args.suite]
    catalog = preprocess_shape_catalog(
        rho=REALISTIC_RHO if realistic else DEFAULT_RHO,
        vertex_count=vertex_count,
        quadrature_order=quadrature_order,
        shape_names=requested_shapes,
        shape_scales=REALISTIC_SHAPE_SCALES if realistic else None,
    )
    scenarios: list[RotSyncScenario] = []
    if args.suite == "smoke":
        scenarios.append(build_smoke_scenario(catalog))
    elif args.suite == "multi":
        scenarios.extend(build_multi_scenarios(catalog))
    elif realistic:
        scenarios.append(build_realistic_extreme_scenario(catalog))
    else:
        scenarios.extend(build_formal_scenarios(catalog))
    config = RotSyncOptimizationConfig(
        max_iterations=max_iterations,
        samples_per_segment=samples_per_segment,
        dynamics_weight=dynamics_weight,
        quadrotor=(
            QuadrotorParameters(
                mass=1.2,
                inertia=np.asarray((0.012, 0.012, 0.022)),
                arm_length=0.159,
            )
            if realistic
            else QuadrotorParameters()
        ),
    )
    rows = []
    for scenario in scenarios:
        result, validation = run_scenario(
            scenario,
            args.outdir / scenario.name,
            config=config,
            make_animation=not args.no_animation,
            animation_frames=animation_frames,
            collision_samples=collision_samples,
        )
        rows.append(_summary_row(scenario, result, validation))
        print(json.dumps(rows[-1], ensure_ascii=False))
    _write_summary(args.outdir / "summary.csv", rows)
    _write_json(args.outdir / "summary.json", {"suite": args.suite, "runs": rows})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["main", "run_scenario", "validate_solution"]
