#!/usr/bin/env python3
"""Compare Interpolated-RotSync with its fixed-SC-input RotSync subset.

The geometry and protocol are the selected counterexample from ICRA experiment
03: balanced U, size ratio 1.9, omega 4.5 rad/s and phase 1.1 rad.  The window
is replaced by a zero-thickness plane.  Sync entry and exit are where the
planning sphere is tangent to that plane, at signed distances -rho and +rho.
Both methods use the released TOGT C++ objective: time plus adaptive-grid,
trapezoidal, smoothed-L1 dynamic penalties, with no energy, velocity, or
collision penalty.  L-BFGS has no wall-clock or iteration cap.  Collision and
all retained dynamic limits are checked independently on a <=1 ms audit grid.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
import sys
import time

import numpy as np
from shapely.geometry import Polygon

from .optimizer import InterpolatedRotSyncObjective
from .togt_code_penalty import (
    instantaneous_togt_code_penalty,
    integrated_togt_code_penalty,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import RotSyncObjective
from nonconvex_timevarying_window.sc_dynatogt.dynamics import PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    OptimizationConfig as TOGTLBFGSConfig,
    _minimize_togt_lbfgs,
)
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    k_from_durations,
)


HERE = Path(__file__).resolve().parent
REPOSITORY_ROOT = HERE.parents[1]
ICRA_ROOT = (
    REPOSITORY_ROOT
    / "nonconvex_timevarying_window"
    / "rot_sync_sc_togt"
    / "icra_experiments"
    / "03_sync_single"
)
REFERENCE_ROOT = ICRA_ROOT / "fixed_wp_u_search" / "balanced_candidate_comparison"

RATIO = 1.9
OMEGA = 4.5
PHASE = 1.1
OLD_METHOD = "RotSync"
NEW_METHOD = "Interpolated-RotSync"


def _load_icra_experiment() -> ModuleType:
    path = ICRA_ROOT / "run_experiment.py"
    spec = importlib.util.spec_from_file_location("icra_sync_single_experiment", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load ICRA comparison adapter from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _balanced_u_source(experiment: ModuleType):
    vertices = np.asarray(
        (
            (-2.5, -2.5),
            (2.5, -2.5),
            (2.5, 2.5),
            (0.5, 2.5),
            (0.5, -0.5),
            (-0.5, -0.5),
            (-0.5, 2.5),
            (-2.5, 2.5),
        ),
        dtype=float,
    )
    return experiment.DenseBoundary(
        vertices, vertices.copy(), tuple(range(len(vertices)))
    )


_EXPERIMENT = _load_icra_experiment()


class _CodeTOGTObjectiveMixin:
    """Released C++ objective: time plus smoothed-L1 dynamic penalty."""

    def code_dynamic_breakdown(self, forward):
        return integrated_togt_code_penalty(
            forward.trajectory,
            parameters=self.config.quadrotor,
            limits=self.config.dynamic_limits,
            weights=self.config.penalty_weights,
            return_breakdown=True,
        )

    def cost_breakdown(self, forward):
        trajectory = forward.trajectory
        dynamic = self.code_dynamic_breakdown(forward)
        total = float(trajectory.total_time)
        smoothness = float(trajectory.snap_energy())
        dynamic_total = float(np.real(dynamic.total))
        return _EXPERIMENT.CostBreakdown(
            total,
            smoothness,
            dynamic_total,
            0.0,
            total + dynamic_total,
        )

    def value(self, x):
        self.cost_evaluations += 1
        value = self.cost_breakdown(self.forward(x)).weighted_total
        if not np.isfinite(value):
            raise FloatingPointError("objective became non-finite")
        return value

    def _safe_value(self, x):
        try:
            return self.value(x)
        except (
            ValueError,
            FloatingPointError,
            OverflowError,
            np.linalg.LinAlgError,
            _EXPERIMENT.SCMappingError,
        ):
            self.invalid_trial_count += 1
            clipped = np.clip(x, -1.0e5, 1.0e5)
            return float(1.0e24 * (1.0 + 1.0e-12 * (clipped @ clipped)))

    def value_and_gradient(self, x):
        values = np.asarray(x, dtype=float)
        self.split(values)
        base = self._safe_value(values)
        gradient = np.empty_like(values)
        for index in range(len(values)):
            step = self.config.finite_difference_step * max(
                1.0, abs(float(values[index]))
            )
            plus, minus = values.copy(), values.copy()
            plus[index] += step
            minus[index] -= step
            gradient[index] = (
                self._safe_value(plus) - self._safe_value(minus)
            ) / (2.0 * step)
        return base, gradient


def _minimum_radius_latent(scenario) -> np.ndarray:
    """Return the deterministic interior initializer shared by both methods."""

    window = scenario.windows[0]
    angles = np.linspace(-np.pi, np.pi, 16384, endpoint=False)
    disk_radius = 0.999
    disk = np.column_stack(
        (disk_radius * np.cos(angles), disk_radius * np.sin(angles))
    )
    local = window.gate.sc_map.evaluate_many(disk, batch_size=4096)
    selected = disk[int(np.argmin(np.linalg.norm(local, axis=1)))]
    return selected / np.sqrt(1.0 - selected @ selected)


def _common_sync_initial_guess(scenario) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    free = k_from_durations(np.asarray((3.3, 3.0)))
    sync = k_from_durations(np.asarray((0.4,)))
    return free, sync, _minimum_radius_latent(scenario)


class CodeTOGTRotSyncObjective(_CodeTOGTObjectiveMixin, RotSyncObjective):
    """Original fixed-SC-input Sync parameterization with C++ objective values."""

    def initial_guess(self):
        free, sync, latent = _common_sync_initial_guess(self.scenario)
        return np.concatenate((free, sync, latent))

    def forward(self, x):
        result = super().forward(x)
        return SimpleNamespace(
            **result.__dict__, crossing_local_index=0, method=OLD_METHOD
        )


class CodeTOGTInterpolatedObjective(
    _CodeTOGTObjectiveMixin, InterpolatedRotSyncObjective
):
    """Two-input parameterization with the released-code objective."""

    def initial_guess(self):
        """Use an interior, low-rotation-radius point for this fast U gate.

        The SC normalization center lies in the U's lower bar and produces a
        very large centripetal-acceleration residual at 4.5 rad/s.  A fixed
        radius strictly inside the disk supplies a deterministic low-radius
        initialization without changing either the decision variables or the
        L-BFGS solver.  Entry and exit start equal, so this remains in the
        original RotSync subset at initialization.
        """

        free, sync, latent = _common_sync_initial_guess(self.scenario)
        return np.concatenate(
            (
                free,
                sync,
                latent,
                latent,
            )
        )

    def forward(self, x):
        result = super().forward(x)
        return SimpleNamespace(
            **result.__dict__, crossing_local_index=0, method=NEW_METHOD
        )


def _unlimited_lbfgs_settings() -> TOGTLBFGSConfig:
    return TOGTLBFGSConfig(
        max_iterations=0,
        max_line_search_steps=64,
        memory_size=256,
        past_iterations=32,
        function_tolerance=1.0e-5,
        gradient_tolerance=0.0,
        samples_per_segment=None,
    )


def _solve_without_budget(objective, initial_x=None):
    started = time.perf_counter()
    initial = np.asarray(
        objective.initial_guess() if initial_x is None else initial_x,
        dtype=float,
    )
    objective.split(initial)
    result = _minimize_togt_lbfgs(
        objective.value_and_gradient,
        initial,
        _unlimited_lbfgs_settings(),
    )
    elapsed = time.perf_counter() - started
    x = np.asarray(result.x, dtype=float)
    forward = objective.forward(x)
    cost = objective.cost_breakdown(forward)
    return _EXPERIMENT.SolveRecord(
        bool(result.success),
        int(result.status),
        str(result.message),
        int(result.nit),
        int(result.nfev),
        elapsed,
        None,
        False,
        x,
        forward,
        cost,
        int(objective.invalid_trial_count),
    )


def _run_method(
    experiment: ModuleType,
    *,
    objective,
    scenario,
    geometry,
    method: str,
    config,
    output: Path,
    initial_x=None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    solve = _solve_without_budget(objective, initial_x)
    audit, data = experiment.audit_solution(
        scenario, solve.forward, config, dt=0.001
    )
    code_dynamic_audit = _audit_togt_code_dynamics(data, config)
    row = experiment.result_row(scenario, geometry, method, solve, audit)
    violation = experiment.dynamic_violation_summary(audit, config.dynamic_limits)
    row["max_dynamic_relative_violation"] = violation["maximum_relative_violation"]
    row["dynamic_margin_class"] = violation["classification"]
    row["maximum_c3_interface_jump"] = audit["maximum_c3_interface_jump"]
    row["collision_free"] = audit["collision_free"]
    row["sampled_dynamic_limits_satisfied"] = audit[
        "sampled_dynamic_limits_satisfied"
    ]
    row["togt_code_sampled_dynamic_limits_satisfied"] = code_dynamic_audit[
        "sampled_dynamic_limits_satisfied"
    ]
    row["togt_code_dynamic_violation_samples"] = code_dynamic_audit[
        "violation_samples"
    ]
    row["togt_code_max_body_rate_penalty"] = code_dynamic_audit[
        "maximum_instantaneous_smoothed_l1"
    ]["body_rate"]
    row["togt_code_max_rotor_thrust_penalty"] = code_dynamic_audit[
        "maximum_instantaneous_smoothed_l1"
    ]["rotor_thrust"]
    payload: dict[str, Any] = {
        "row": row,
        "decision_vector": solve.x,
        "selected_local_points": solve.forward.local_points,
        "crossing_times": solve.forward.crossing_times,
        "durations": solve.forward.trajectory.durations,
        "cost_breakdown": solve.cost,
        "optimizer": {
            "success": solve.optimizer_success,
            "status": solve.status,
            "message": solve.message,
            "iterations": solve.iterations,
            "evaluations": solve.evaluations,
            "solve_seconds": solve.solve_seconds,
            "budget_seconds": solve.budget_seconds,
            "timed_out": solve.timed_out,
            "invalid_trials": solve.invalid_trials,
        },
        "audit": audit,
        "dynamic_violation_summary": violation,
        "togt_code_dense_dynamic_audit": code_dynamic_audit,
        "togt_code_dynamic_penalty_components": objective.code_dynamic_breakdown(
            solve.forward
        ),
    }
    if hasattr(solve.forward, "latent_entry_points"):
        payload.update(
            {
                "entry_latent_points": solve.forward.latent_entry_points,
                "exit_latent_points": solve.forward.latent_exit_points,
                "entry_local_points": solve.forward.local_entry_points,
                "exit_local_points": solve.forward.local_exit_points,
            }
        )
    if hasattr(solve.forward, "latent_control_points"):
        payload.update(
            {
                "latent_control_points": solve.forward.latent_control_points,
                "normal_shape_parameters": solve.forward.normal_shape_parameters,
                "normal_control_points": solve.forward.normal_control_points,
            }
        )
    experiment.write_json(output / "result.json", payload)
    experiment.save_raw_trajectory(output, data)
    return row, payload


def _audit_togt_code_dynamics(data, config) -> dict[str, Any]:
    """Check the C++ penalty residuals independently on the <=1 ms grid."""

    components = []
    singular = []
    for velocity, acceleration, jerk, snap in zip(
        data["velocity"],
        data["acceleration"],
        data["jerk"],
        data["snap"],
    ):
        penalty = instantaneous_togt_code_penalty(
            velocity,
            acceleration,
            jerk,
            snap,
            parameters=config.quadrotor,
            limits=config.dynamic_limits,
            weights=config.penalty_weights,
        )
        components.append(
            (
                penalty.velocity,
                penalty.collective_thrust,
                penalty.body_rate,
                penalty.rotor_thrust,
            )
        )
        specific_force = acceleration + np.asarray(
            (0.0, 0.0, config.quadrotor.gravity)
        )
        body_z = specific_force / np.linalg.norm(specific_force)
        singular.append(abs(float(body_z[2] + 1.0)) <= 1.0e-3)
    values = np.asarray(components, dtype=float)
    point_totals = np.sum(values, axis=1)
    violations = point_totals > 0.0
    return {
        "evidence_level": "sampled_numerical_validation_not_continuous_time_proof",
        "audit_samples": int(len(data["time"])),
        "audit_dt_max": float(np.max(np.diff(data["time"]))),
        "sampled_dynamic_limits_satisfied": bool(not np.any(violations)),
        "violation_samples": int(np.count_nonzero(violations)),
        "first_violation_time": (
            None
            if not np.any(violations)
            else float(data["time"][int(np.flatnonzero(violations)[0])])
        ),
        "maximum_instantaneous_smoothed_l1": {
            "velocity": float(np.max(values[:, 0])),
            "singular_collective_thrust": float(np.max(values[:, 1])),
            "body_rate": float(np.max(values[:, 2])),
            "rotor_thrust": float(np.max(values[:, 3])),
            "total": float(np.max(point_totals)),
        },
        "robust_singularity_branch_samples": int(np.count_nonzero(singular)),
    }


def _write_comparison_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = (
        "method",
        "trajectory_pass",
        "optimizer_success",
        "objective",
        "time_cost",
        "smoothness_cost",
        "dynamic_penalty",
        "collision_penalty",
        "collision_free",
        "sampled_dynamic_limits_satisfied",
        "togt_code_sampled_dynamic_limits_satisfied",
        "togt_code_dynamic_violation_samples",
        "togt_code_max_body_rate_penalty",
        "togt_code_max_rotor_thrust_penalty",
        "failure_category",
        "failure_reasons",
        "flight_time",
        "solve_seconds",
        "colliding_samples",
        "audit_samples",
        "audit_dt_max",
        "max_velocity",
        "max_body_rate_xy",
        "max_body_rate_z",
        "min_rotor_thrust",
        "max_rotor_thrust",
        "max_dynamic_relative_violation",
        "dynamic_margin_class",
        "maximum_c3_interface_jump",
        "minimum_frame_clearance",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def _write_report(
    path: Path,
    rows: list[dict[str, Any]],
    config,
    *,
    sphere_radius: float,
    subset_derivative_errors: dict[str, float],
    subset_objective_error: float,
    nested_objective_error: float,
) -> None:
    limits = config.dynamic_limits
    lines = [
        "# 零厚度窗口：双 SC 输入与固定 SC 输入 RotSync 对比",
        "",
        "场景为既有均衡 U 几何：尺寸比 1.9、角速度 4.5 rad/s、初相位 1.1 rad，窗口厚度为零。",
        f"Sync 入口/出口是规划球包络与窗口平面相切时刻，rho={sphere_radius:.12g} m，球心法向坐标分别为 -rho/+rho。",
        "对照为固定一组 SC 输入的原 RotSync。"
        f"运行前 0–4 阶导数最大误差 {max(subset_derivative_errors.values()):.3e}，"
        f"目标误差 {subset_objective_error:.3e}；最终 RotSync 解嵌入误差 {nested_objective_error:.3e}。",
        "两种方法采用 TOGT 配套 C++ 目标：J = T + 动态 8–32 区间梯形积分的 smoothedL1 动力学惩罚。",
        "snap、速度和碰撞都不进入目标；优化无墙钟和迭代上限，动力学与碰撞仍独立审计。",
        "",
        f"动力学限制：速度不超过 {limits.max_velocity:g} m/s，XY/Z 机体角速率分别不超过 "
        f"{limits.max_body_rate_xy:g}/{limits.max_body_rate_z:g} rad/s，单旋翼推力范围 "
        f"[{limits.min_rotor_thrust:g}, {limits.max_rotor_thrust:g}] N。",
        "",
        "|method|J|T|raw snap (diagnostic only)|TOGT-code dynamic penalty|collision objective term|",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "|{method}|{objective:.9g}|{time_cost:.9g}|{smoothness_cost:.9g}|"
            "{dynamic_penalty:.9g}|0 (not evaluated)|".format(**row)
        )
    lines.append("")
    lines.append(
        "双输入方法以“入口=出口”的 RotSync 最终解热启动，"
        "因此两者的 T 与 J 不会出现由不同初始化导致的虚假劣化。"
    )
    lines.extend(
        [
            "",
            "|方法|轨迹验收|求解器正常停止|无碰撞|TOGT-code 1ms 动力学|原审计动力学|碰撞点/审计点|最大速度|最大 XY 角速率|最大 Z 角速率|旋翼推力范围|C3 跳变|",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in rows:
        lines.append(
            "|{method}|{trajectory_pass}|{optimizer_success}|{collision_free}|"
            "{togt_code_sampled_dynamic_limits_satisfied}|"
            "{sampled_dynamic_limits_satisfied}|{colliding_samples}/{audit_samples}|"
            "{max_velocity:.6g}|{max_body_rate_xy:.6g}|{max_body_rate_z:.6g}|"
            "[{min_rotor_thrust:.6g}, {max_rotor_thrust:.6g}]|{maximum_c3_interface_jump:.3e}|".format(
                **row
            )
        )
    lines.extend(
        (
            "",
            "`trajectory_pass` 由独立最大 1 ms 网格及临界时刻细化决定，",
            "与 `optimizer_success` 分开。采样无碰撞不是连续时间安全证明。",
            "TOGT-code 动力学列在同一 1 ms 网格上使用 C++ tilt-yaw/robust-singularity 残差；",
            "后续角速率和旋翼极值列保留原项目姿态恢复审计，两者不混为同一模型。",
            "",
        )
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--outdir",
        type=Path,
        default=HERE / "results" / "zero_thickness_sync_comparison",
    )
    args = parser.parse_args(argv)

    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    weights = json.loads(
        (ICRA_ROOT / "focused_results" / "frozen_weights.json").read_text(
            encoding="utf-8"
        )
    )
    config = replace(
        _EXPERIMENT.make_config(weights, 1),
        smoothness_weight=0.0,
        dynamics_weight=1.0,
        penalty_weights=PenaltyWeights(
            velocity=0.0,
            collective_thrust=0.0,
            body_rate=1.0,
            rotor_thrust=1.0,
        ),
    )
    geometry = _EXPERIMENT.prepare_geometry(
        "U",
        RATIO,
        vertex_count=96,
        quadrature_order=96,
        canonical_axis=np.asarray((0.0, 1.0)),
        source_boundary=_balanced_u_source(_EXPERIMENT),
    )
    scenario_name = (
        _EXPERIMENT.scenario_name("U", RATIO, OMEGA, PHASE) + "_zero_thickness"
    )
    finite_thickness_scenario = _EXPERIMENT.build_scenario(
        geometry, OMEGA, PHASE, name=scenario_name
    )
    scenario = replace(
        finite_thickness_scenario,
        windows=(replace(finite_thickness_scenario.windows[0], thickness=0.0),),
        description="Zero-thickness plane with sphere-tangent Sync endpoints",
    )

    reference = json.loads(
        (REFERENCE_ROOT / "search_protocol.json").read_text(encoding="utf-8")
    )
    expected_safe = Polygon(reference["geometries"][0]["safe_vertices"])
    actual_safe = Polygon(geometry.gate.safe_region.vertices)
    difference_area = float(expected_safe.symmetric_difference(actual_safe).area)
    if difference_area > 1.0e-12:
        raise RuntimeError(
            f"reconstructed safe geometry differs from retained case: {difference_area:g}"
        )

    fixed = CodeTOGTRotSyncObjective(scenario, config)
    interpolated = CodeTOGTInterpolatedObjective(scenario, config)

    fixed_initial = fixed.initial_guess()
    interpolated_initial = interpolated.initial_guess()
    embedded_initial = np.concatenate(
        (fixed_initial[:3], fixed_initial[3:], fixed_initial[3:])
    )
    if not np.array_equal(interpolated_initial, embedded_initial):
        raise RuntimeError("the two methods do not share the same subset initializer")
    fixed_forward = fixed.forward(fixed_initial)
    interpolated_forward = interpolated.forward(interpolated_initial)
    check_times = np.linspace(0.0, fixed_forward.trajectory.total_time, 257)
    subset_derivative_errors = {
        str(order): float(
            np.max(
                np.abs(
                    fixed_forward.trajectory.evaluate(check_times, order)
                    - interpolated_forward.trajectory.evaluate(check_times, order)
                )
            )
        )
        for order in range(5)
    }
    subset_objective_error = abs(
        fixed.cost_breakdown(fixed_forward).weighted_total
        - interpolated.cost_breakdown(interpolated_forward).weighted_total
    )
    if max(subset_derivative_errors.values()) > 1.0e-9 or subset_objective_error > 1.0e-9:
        raise RuntimeError("fixed-input subset equivalence check failed")
    fixed_row, fixed_payload = _run_method(
        _EXPERIMENT,
        objective=fixed,
        scenario=scenario,
        geometry=geometry,
        method=OLD_METHOD,
        config=config,
        output=output / OLD_METHOD,
    )
    print(json.dumps(_EXPERIMENT._jsonable(fixed_row), ensure_ascii=False), flush=True)

    fixed_solution = np.asarray(fixed_payload["decision_vector"], dtype=float)
    nested_initial = np.concatenate(
        (fixed_solution[:3], fixed_solution[3:], fixed_solution[3:])
    )
    nested_objective_error = abs(
        interpolated.value(nested_initial) - fixed.value(fixed_solution)
    )
    if nested_objective_error > 1.0e-9:
        raise RuntimeError("optimized RotSync solution did not embed exactly")
    interpolated_row, _ = _run_method(
        _EXPERIMENT,
        objective=interpolated,
        scenario=scenario,
        geometry=geometry,
        method=NEW_METHOD,
        config=config,
        output=output / NEW_METHOD,
        initial_x=nested_initial,
    )
    print(
        json.dumps(_EXPERIMENT._jsonable(interpolated_row), ensure_ascii=False),
        flush=True,
    )
    rows = [fixed_row, interpolated_row]

    _write_comparison_csv(output / "comparison.csv", rows)
    _EXPERIMENT.write_json(
        output / "comparison.json",
        {
            "scenario": {
                "name": scenario_name,
                "shape": "balanced_U",
                "size_ratio": RATIO,
                "omega": OMEGA,
                "phase": PHASE,
                "gate_thickness": scenario.windows[0].thickness,
                "body_half_extents": _EXPERIMENT.BODY.half_extents,
                "planning_envelope_radius": _EXPERIMENT.PLANNING_RHO,
                "sync_entry_signed_distance": -scenario.windows[0].rho,
                "sync_exit_signed_distance": scenario.windows[0].rho,
                "safe_geometry_symmetric_difference_area": difference_area,
            },
            "protocol": {
                "objective": {
                    "formula": "released TOGT C++: T_total + trapezoidal integral of smoothedL1 residual penalties",
                    "snap_weight": 0.0,
                    "velocity_penalty_weight": 0.0,
                    "dynamic_penalty_weight": 1.0,
                    "collision_weight": 0.0,
                    "quadrature": "dynamicConstCheck: clamp(int(T_i/0.05), 8, 32) intervals with trapezoidal endpoint weights",
                    "smoothed_l1_mu": 0.01,
                    "robust_singularity_threshold": 0.001,
                },
                "retained_constraint_component_weights": config.penalty_weights,
                "frozen_weights_file_reference": weights,
                "wall_clock_budget_seconds_each": None,
                "max_iterations": 0,
                "lbfgs": {
                    "memory_size": 256,
                    "past_iterations": 32,
                    "max_line_search_steps": 64,
                    "relative_cost_tolerance": 1.0e-5,
                    "gradient_tolerance": 0.0,
                },
                "dynamic_limits": config.dynamic_limits,
                "audit": "<=1ms plus critical-time refinement; sampled numerical validation",
                "reference": str(REFERENCE_ROOT),
                "interpolated_initialization": {
                    "free_durations": [3.3, 3.0],
                    "sync_duration": 0.4,
                    "disk_radius": 0.999,
                    "disk_angle_grid_size": 16384,
                    "selection": "minimum local rotation radius",
                    "entry_equals_exit": True,
                },
                "fixed_input_subset_check": {
                    "derivative_max_abs_errors_0_to_4": subset_derivative_errors,
                    "objective_abs_error": subset_objective_error,
                    "optimized_rot_sync_embedding_objective_abs_error": nested_objective_error,
                    "interpolated_warm_start": "optimized RotSync with entry equal to exit",
                },
            },
            "rows": rows,
        },
    )
    _write_report(
        output / "REPORT.md",
        rows,
        config,
        sphere_radius=scenario.windows[0].rho,
        subset_derivative_errors=subset_derivative_errors,
        subset_objective_error=subset_objective_error,
        nested_objective_error=nested_objective_error,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "CodeTOGTRotSyncObjective",
    "CodeTOGTInterpolatedObjective",
    "main",
]
