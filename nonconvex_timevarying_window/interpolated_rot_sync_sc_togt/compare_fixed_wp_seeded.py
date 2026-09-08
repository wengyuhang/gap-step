#!/usr/bin/env python3
"""Zero-thickness Fixed-WP comparison with a trajectory-derived SC warm start."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
from scipy.optimize import brentq
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt.dynamics import PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations

from . import compare_fixed_wp_counterexample as common


HERE = Path(__file__).resolve().parent
FIXED_METHOD = "Fixed-WP"
NEW_METHOD = "Interpolated-RotSync"


class CodeTOGTFixedWaypointObjective(
    common._CodeTOGTObjectiveMixin, common._EXPERIMENT.FixedWaypointObjective
):
    """The ordinary two-piece fixed-waypoint MINCO with C++ objective values."""

    def __init__(self, scenario, config) -> None:
        common._EXPERIMENT.FixedWaypointObjective.__init__(
            self, scenario, config, collision_weight=0.0
        )


def _zero_thickness_problem():
    weights = json.loads(
        (common.ICRA_ROOT / "focused_results" / "frozen_weights.json").read_text(
            encoding="utf-8"
        )
    )
    config = replace(
        common._EXPERIMENT.make_config(weights, 1),
        smoothness_weight=0.0,
        dynamics_weight=1.0,
        penalty_weights=PenaltyWeights(
            velocity=0.0,
            collective_thrust=0.0,
            body_rate=1.0,
            rotor_thrust=1.0,
        ),
    )
    geometry = common._EXPERIMENT.prepare_geometry(
        "U",
        common.RATIO,
        vertex_count=96,
        quadrature_order=96,
        canonical_axis=np.asarray((0.0, 1.0)),
        source_boundary=common._balanced_u_source(common._EXPERIMENT),
    )
    name = (
        common._EXPERIMENT.scenario_name(
            "U", common.RATIO, common.OMEGA, common.PHASE
        )
        + "_zero_thickness_fixed_wp_seeded"
    )
    finite = common._EXPERIMENT.build_scenario(
        geometry, common.OMEGA, common.PHASE, name=name
    )
    scenario = replace(
        finite,
        windows=(replace(finite.windows[0], thickness=0.0),),
        description=(
            "Zero-thickness Fixed-WP comparison with sphere-tangent SC warm start"
        ),
    )
    return weights, config, geometry, scenario


def _tangent_root(trajectory, window, target: float, left: float, right: float):
    def residual(instant: float) -> float:
        position = np.asarray(trajectory.evaluate(instant), dtype=float)
        return float((position - window.center) @ window.normal - target)

    left_value, right_value = residual(left), residual(right)
    if left_value == 0.0:
        return float(left)
    if right_value == 0.0:
        return float(right)
    if left_value * right_value > 0.0:
        raise RuntimeError(
            f"Fixed-WP trajectory does not bracket signed distance {target:g}"
        )
    return float(brentq(residual, left, right, xtol=1.0e-13))


def _disk_to_unconstrained(disk) -> np.ndarray:
    value = np.asarray(disk, dtype=float)
    norm_squared = float(value @ value)
    if norm_squared >= 1.0:
        raise RuntimeError("SC inverse did not return a point in the open disk")
    return value / np.sqrt(1.0 - norm_squared)


def fixed_wp_derived_initial_guess(fixed_forward, scenario):
    """Infer tangent times and two SC inputs from the optimized Fixed-WP path."""

    trajectory = fixed_forward.trajectory
    window = scenario.windows[0]
    crossing = float(fixed_forward.crossing_times[0])
    total = float(trajectory.total_time)
    radius = float(window.rho)
    entry_time = _tangent_root(trajectory, window, -radius, 0.0, crossing)
    exit_time = _tangent_root(trajectory, window, radius, crossing, total)
    durations = np.asarray(
        (entry_time, exit_time - entry_time, total - exit_time), dtype=float
    )
    if np.any(durations <= 0.0):
        raise RuntimeError("Fixed-WP tangencies did not define three positive durations")

    polygon = Polygon(window.safe_polygon)
    local_points = []
    disk_points = []
    latent_points = []
    margins = []
    inverse_errors = []
    for instant in (entry_time, exit_time):
        position = np.asarray(trajectory.evaluate(instant), dtype=float)
        local = window.rotated_basis(instant).T @ (position - window.center)
        point = Point(local)
        if not polygon.covers(point):
            raise RuntimeError(
                "Fixed-WP sphere tangency lies outside the safe inset opening"
            )
        disk = np.asarray(window.gate.sc_map.inverse(local), dtype=float)
        latent = _disk_to_unconstrained(disk)
        reconstructed = window.local_point(latent)
        local_points.append(local)
        disk_points.append(disk)
        latent_points.append(latent)
        margins.append(float(point.distance(polygon.boundary)))
        inverse_errors.append(float(np.linalg.norm(reconstructed - local)))

    latent = np.asarray(latent_points)
    initial = np.concatenate(
        (k_from_durations(durations[[0, 2]]), k_from_durations(durations[1:2]), latent.reshape(-1))
    )
    diagnostics = {
        "source_fixed_wp_total_time": total,
        "source_fixed_wp_crossing_time": crossing,
        "sphere_radius": radius,
        "entry_time": entry_time,
        "exit_time": exit_time,
        "durations": durations,
        "local_entry_exit": np.asarray(local_points),
        "disk_entry_exit": np.asarray(disk_points),
        "latent_entry_exit": latent,
        "safe_boundary_margins": np.asarray(margins),
        "inverse_roundtrip_errors": np.asarray(inverse_errors),
    }
    return initial, diagnostics


def _write_report(path: Path, rows, seed) -> None:
    fixed, interpolated = rows
    lines = [
        "# 零厚度 Fixed-WP 反推初值对比",
        "",
        "对照为两段普通 MINCO Fixed-WP；窗口厚度为零。",
        "先优化 Fixed-WP，再从其轨迹反求规划球与平面在 -rho/+rho 的相切时刻和局部点，通过 SC 逆映射生成双输入初值。",
        "",
        f"- 相切时刻：{seed['entry_time']:.12g} s / {seed['exit_time']:.12g} s",
        f"- 反推三段时间：{np.asarray(seed['durations']).tolist()}",
        f"- 安全边界余量：{np.asarray(seed['safe_boundary_margins']).tolist()} m",
        f"- SC 逆映射回代误差：{np.asarray(seed['inverse_roundtrip_errors']).tolist()} m",
        f"- 新方法初值：T={seed['initial_total_time']:.12g} s，J={seed['initial_objective']:.12g}",
        "",
        "|method|solver stop|trajectory pass|T|J|dynamic penalty|collision samples|TOGT-code dynamic violations|C3 jump|",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"|{row['method']}|{row['optimizer_success']}|{row['trajectory_pass']}|"
            f"{row['flight_time']:.12g}|{row['objective']:.12g}|"
            f"{row['dynamic_penalty']:.12g}|{row['colliding_samples']}/{row['audit_samples']}|"
            f"{row['togt_code_dynamic_violation_samples']}|{row['maximum_c3_interface_jump']:.3e}|"
        )
    lines.extend(
        (
            "",
            "Fixed-WP 与 Interpolated-RotSync 的轨迹结构不同：后者额外强制球从入口相切到出口相切的整段都按 SC 输入插值和窗口旋转运动。因此反推初值不是 Fixed-WP 轨迹的精确嵌入。",
            "碰撞与动力学是最大 1 ms 网格的独立采样审计，不是连续域证明。",
            "",
        )
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--outdir",
        type=Path,
        default=HERE / "results" / "zero_thickness_fixed_wp_seeded",
    )
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)

    weights, config, geometry, scenario = _zero_thickness_problem()
    fixed = CodeTOGTFixedWaypointObjective(scenario, config)
    fixed_row, fixed_payload = common._run_method(
        common._EXPERIMENT,
        objective=fixed,
        scenario=scenario,
        geometry=geometry,
        method=FIXED_METHOD,
        config=config,
        output=output / FIXED_METHOD,
    )
    print(json.dumps(common._EXPERIMENT._jsonable(fixed_row), ensure_ascii=False), flush=True)

    fixed_solution = np.asarray(fixed_payload["decision_vector"], dtype=float)
    fixed_forward = fixed.forward(fixed_solution)
    initial, seed = fixed_wp_derived_initial_guess(fixed_forward, scenario)
    interpolated = common.CodeTOGTInterpolatedObjective(scenario, config)
    initial_forward = interpolated.forward(initial)
    initial_cost = interpolated.cost_breakdown(initial_forward)
    seed["initial_total_time"] = initial_forward.trajectory.total_time
    seed["initial_objective"] = initial_cost.weighted_total
    seed["initial_dynamic_penalty"] = initial_cost.dynamic_penalty

    interpolated_row, _ = common._run_method(
        common._EXPERIMENT,
        objective=interpolated,
        scenario=scenario,
        geometry=geometry,
        method=NEW_METHOD,
        config=config,
        output=output / NEW_METHOD,
        initial_x=initial,
    )
    print(
        json.dumps(common._EXPERIMENT._jsonable(interpolated_row), ensure_ascii=False),
        flush=True,
    )
    rows = [fixed_row, interpolated_row]
    common._write_comparison_csv(output / "comparison.csv", rows)
    common._EXPERIMENT.write_json(
        output / "comparison.json",
        {
            "scenario": {
                "name": scenario.name,
                "shape": "balanced_U",
                "size_ratio": common.RATIO,
                "omega": common.OMEGA,
                "phase": common.PHASE,
                "gate_thickness": 0.0,
                "sphere_radius": scenario.windows[0].rho,
            },
            "protocol": {
                "objective": "released TOGT C++ value-side objective",
                "collision_objective_weight": 0.0,
                "wall_clock_budget_seconds_each": None,
                "max_iterations": 0,
                "fixed_wp_derived_initialization": seed,
                "frozen_weights_file_reference": weights,
            },
            "rows": rows,
        },
    )
    _write_report(output / "REPORT.md", rows, seed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "CodeTOGTFixedWaypointObjective",
    "fixed_wp_derived_initial_guess",
    "main",
]
