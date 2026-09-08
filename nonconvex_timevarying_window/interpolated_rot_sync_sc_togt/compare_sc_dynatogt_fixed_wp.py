#!/usr/bin/env python3
"""Compare fixed and free SC waypoints with the same two-piece MINCO model."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from shapely.geometry import Point, Polygon
from shapely.ops import polylabel

from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    ObjectiveWeights,
    PenaltyWeights,
    integrated_dynamic_penalty,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    JointTOGTObjective,
    OptimizationConfig,
)
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import B_with_jacobian

from . import compare_fixed_wp_counterexample as common
from .compare_fixed_wp_seeded import _disk_to_unconstrained


HERE = Path(__file__).resolve().parent
FIXED_METHOD = "Fixed-WP"
FREE_METHOD = "SC-DynaTOGT"
_J = np.asarray(((0.0, -1.0), (1.0, 0.0)))


class _RotatingSCWindowAdapter:
    """Expose the exact RotSync constant-spin window to JointTOGTObjective."""

    def __init__(self, window) -> None:
        self.window = window

    def _local_point_and_jacobian(self, d):
        disk, disk_jacobian = B_with_jacobian(np.asarray(d, dtype=float))
        complex_disk = complex(float(disk[0]), float(disk[1]))
        local = np.asarray(self.window.gate.sc_map.evaluate(complex_disk), dtype=float)
        sc_jacobian = np.asarray(
            self.window.gate.sc_map.jacobian(complex_disk), dtype=float
        )
        return local, sc_jacobian @ disk_jacobian

    def point_and_jacobians(self, d, t):
        local, local_jacobian = self._local_point_and_jacobian(d)
        basis = self.window.rotated_basis(float(t))
        basis_dot = basis @ _J * self.window.omega
        point = self.window.center + basis @ local
        return point, local, basis @ local_jacobian, basis_dot @ local

    def to_point(self, d, traversal_time):
        return self.point_and_jacobians(d, traversal_time)[0]


class _ObjectiveAdapter:
    """Supply the shared comparison/audit interface around SC-DynaTOGT."""

    method = ""

    def __init__(self, track, config) -> None:
        self.joint = JointTOGTObjective(track, config)
        self.config = config

    @property
    def invalid_trial_count(self):
        return self.joint.invalid_trial_count

    def _wrap_forward(self, forward):
        return SimpleNamespace(
            trajectory=forward.trajectory,
            crossing_times=forward.traversal_times,
            local_points=forward.local_points,
            crossing_local_index=0,
            durations=forward.durations,
            method=self.method,
        )

    def code_dynamic_breakdown(self, forward):
        return common.integrated_togt_code_penalty(
            forward.trajectory,
            parameters=self.config.quadrotor,
            limits=self.config.dynamic_limits,
            weights=self.config.penalty_weights,
            return_breakdown=True,
        )

    def cost_breakdown(self, forward):
        trajectory = forward.trajectory
        dynamic = float(
            np.real(
                integrated_dynamic_penalty(
                    trajectory,
                    parameters=self.config.quadrotor,
                    limits=self.config.dynamic_limits,
                    weights=self.config.penalty_weights,
                    samples_per_segment=self.config.samples_per_segment,
                )
            )
        )
        total = float(trajectory.total_time)
        smoothness = float(trajectory.snap_energy())
        return common._EXPERIMENT.CostBreakdown(
            total,
            smoothness,
            dynamic,
            0.0,
            total + dynamic,
        )


class FixedWaypointObjective(_ObjectiveAdapter):
    """Only the two temporal variables are free; the SC waypoint is frozen."""

    method = FIXED_METHOD
    dimension = 2

    def __init__(self, track, config, fixed_d) -> None:
        super().__init__(track, config)
        self.fixed_d = np.asarray(fixed_d, dtype=float).reshape(1, 2)

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (2,) or not np.all(np.isfinite(values)):
            raise ValueError("Fixed-WP requires two finite temporal variables")
        return values

    def _full_x(self, x):
        return np.concatenate((self.split(x), self.fixed_d.reshape(-1)))

    def initial_guess(self):
        return self.joint.initial_guess(self.fixed_d)[:2]

    def forward(self, x):
        return self._wrap_forward(self.joint.forward(self._full_x(x)))

    def value_and_gradient(self, x):
        value, gradient = self.joint.scipy_value_and_gradient(self._full_x(x))
        return value, gradient[:2]


class FreeSCWaypointObjective(_ObjectiveAdapter):
    """The native SC-DynaTOGT one-window decision vector [K0,K1,dx,dy]."""

    method = FREE_METHOD
    dimension = 4

    def split(self, x):
        values = np.asarray(x, dtype=float)
        if values.shape != (4,) or not np.all(np.isfinite(values)):
            raise ValueError("SC-DynaTOGT requires four finite variables")
        return self.joint.split(values)

    def initial_guess(self):
        return self.joint.initial_guess()

    def forward(self, x):
        return self._wrap_forward(self.joint.forward(np.asarray(x, dtype=float)))

    def value_and_gradient(self, x):
        return self.joint.scipy_value_and_gradient(np.asarray(x, dtype=float))


def _build_problem():
    weights = json.loads(
        (common.ICRA_ROOT / "focused_results" / "frozen_weights.json").read_text(
            encoding="utf-8"
        )
    )
    rot_config = common._EXPERIMENT.make_config(weights, 1)
    config = OptimizationConfig(
        initial_speed=rot_config.initial_speed,
        minimum_initial_duration=rot_config.minimum_initial_free_duration,
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
            velocity=0.0,
            collective_thrust=0.0,
            body_rate=1.0,
            rotor_thrust=1.0,
        ),
        dynamic_limits=rot_config.dynamic_limits,
        quadrotor=rot_config.quadrotor,
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
        + "_zero_thickness_sc_dynatogt"
    )
    finite = common._EXPERIMENT.build_scenario(
        geometry, common.OMEGA, common.PHASE, name=name
    )
    scenario = replace(
        finite,
        windows=(replace(finite.windows[0], thickness=0.0),),
        description="Zero-thickness Fixed-WP versus free SC waypoint",
    )
    track = SCWindowTrack(
        name=name,
        start=scenario.start_state.position,
        goal=scenario.goal_state.position,
        windows=(_RotatingSCWindowAdapter(scenario.windows[0]),),
        order=(0,),
    )
    fixed_local = np.asarray(
        polylabel(Polygon(scenario.windows[0].safe_polygon), tolerance=1.0e-7).coords[0],
        dtype=float,
    )
    fixed_disk = np.asarray(
        scenario.windows[0].gate.sc_map.inverse(fixed_local), dtype=float
    )
    fixed_d = _disk_to_unconstrained(fixed_disk)
    return weights, config, geometry, scenario, track, fixed_local, fixed_d


def _write_report(path, rows, embedding, fixed_local, fixed_d) -> None:
    lines = [
        "# SC-DynaTOGT 自由选点与 Fixed-WP 对比",
        "",
        "两者共享零厚度旋转 U 形窗口、两段 degree-7 MINCO、TOGT 目标和 L-BFGS；唯一区别是 SC 穿越点是否可优化。",
        f"Fixed-WP 局部点为 {np.asarray(fixed_local).tolist()}，其 SC 无约束变量为 {np.asarray(fixed_d).tolist()}。",
        f"运行前嵌入检查：目标误差 {embedding['objective_abs_error']:.3e}，0–4 阶轨迹最大误差 {max(embedding['derivative_max_abs_errors'].values()):.3e}。",
        "",
        "|method|solver stop|trajectory pass|T|J|dynamic penalty|local point|collision samples|sampled dynamics|C3 jump|",
        "|---|---:|---:|---:|---:|---:|---|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"|{row['method']}|{row['optimizer_success']}|{row['trajectory_pass']}|"
            f"{row['flight_time']:.12g}|{row['objective']:.12g}|{row['dynamic_penalty']:.12g}|"
            f"{row['optimized_local_point']}|{row['colliding_samples']}/{row['audit_samples']}|"
            f"{row['sampled_dynamic_limits_satisfied']}|{row['maximum_c3_interface_jump']:.3e}|"
        )
    lines.extend(
        (
            "",
            "SC-DynaTOGT 从 Fixed-WP 最终解的精确嵌入热启动，因此这一对比满足可行域包含关系。",
            "碰撞和动力学是求解后最大 1 ms 网格的独立采样审计，不是连续域证明。",
            "",
        )
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--outdir",
        type=Path,
        default=HERE / "results" / "zero_thickness_sc_dynatogt_fixed_wp",
    )
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)

    weights, config, geometry, scenario, track, fixed_local, fixed_d = _build_problem()
    fixed = FixedWaypointObjective(track, config, fixed_d)
    fixed_row, fixed_payload = common._run_method(
        common._EXPERIMENT,
        objective=fixed,
        scenario=scenario,
        geometry=geometry,
        method=FIXED_METHOD,
        config=config,
        output=output / FIXED_METHOD,
    )
    fixed_x = np.asarray(fixed_payload["decision_vector"], dtype=float)
    free_initial = np.concatenate((fixed_x, fixed_d))
    free = FreeSCWaypointObjective(track, config)

    fixed_forward = fixed.forward(fixed_x)
    free_forward = free.forward(free_initial)
    grid = np.linspace(0.0, fixed_forward.trajectory.total_time, 257)
    derivative_errors = {
        str(order): float(
            np.max(
                np.abs(
                    fixed_forward.trajectory.evaluate(grid, order)
                    - free_forward.trajectory.evaluate(grid, order)
                )
            )
        )
        for order in range(5)
    }
    objective_error = abs(
        fixed.cost_breakdown(fixed_forward).weighted_total
        - free.cost_breakdown(free_forward).weighted_total
    )
    if max(derivative_errors.values()) > 1.0e-10 or objective_error > 1.0e-10:
        raise RuntimeError("Fixed-WP embedding into SC-DynaTOGT is not exact")

    free_row, free_payload = common._run_method(
        common._EXPERIMENT,
        objective=free,
        scenario=scenario,
        geometry=geometry,
        method=FREE_METHOD,
        config=config,
        output=output / FREE_METHOD,
        initial_x=free_initial,
    )
    polygon = Polygon(scenario.windows[0].safe_polygon)
    for row, payload, objective in (
        (fixed_row, fixed_payload, fixed),
        (free_row, free_payload, free),
    ):
        local = np.asarray(payload["selected_local_points"][0], dtype=float)
        point = Point(local)
        decision = np.asarray(payload["decision_vector"], dtype=float)
        _, gradient = objective.value_and_gradient(decision)
        row["optimized_local_point"] = local
        row["objective_gradient_inf_norm"] = float(
            np.linalg.norm(gradient, ord=np.inf)
        )
        row["safe_polygon_outside_distance"] = float(point.distance(polygon))
        row["safe_with_1nm_tolerance"] = bool(
            polygon.buffer(1.0e-9).covers(point)
        )
    rows = [fixed_row, free_row]
    for row in rows:
        print(json.dumps(common._EXPERIMENT._jsonable(row), ensure_ascii=False), flush=True)

    embedding = {
        "objective_abs_error": objective_error,
        "derivative_max_abs_errors": derivative_errors,
        "free_initial_x": free_initial,
    }
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
            },
            "protocol": {
                "model": "same two-piece MINCO; fixed versus free SC waypoint",
                "objective": "native SC-DynaTOGT objective and analytic gradient",
                "collision_objective_weight": 0.0,
                "wall_clock_budget_seconds_each": None,
                "max_iterations": 0,
                "fixed_solution_embedding": embedding,
                "frozen_weights_file_reference": weights,
            },
            "rows": rows,
        },
    )
    _write_report(output / "REPORT.md", rows, embedding, fixed_local, fixed_d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["FixedWaypointObjective", "FreeSCWaypointObjective", "main"]
