#!/usr/bin/env python3
"""Compare original SC-DynaTOGT with Fixed-WP on a curved three-window track."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
from types import SimpleNamespace

import matplotlib.pyplot as plt
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import polylabel

from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import compare_fixed_wp_counterexample as common
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_seeded import _disk_to_unconstrained
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective, audit_multi
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import screen_candidate
from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import RotatingWindow, basis_from_normal
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import DEFAULT_BODY, RotSyncScenario, preprocess_shape_catalog
from nonconvex_timevarying_window.sc_dynatogt.dynamics import ObjectiveWeights, PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig, _minimize_togt_lbfgs


HERE = Path(__file__).resolve().parent
METHODS = ("Fixed-WP", "SC-DynaTOGT")
SHAPES = ("limacon", "wavy", "line_bezier")
CENTERS = ((0.0, 0.0, 1.8), (3.5, 0.85, 2.15), (7.0, -0.70, 1.55))
PHASES = (0.35, -0.55, 0.80)
OMEGAS = (1.50, -2.00, 2.50)
START = (-2.1, -0.55, 1.75)
GOAL = (9.1, 0.55, 1.80)


def make_config() -> OptimizationConfig:
    """Rebuild the native SC-DynaTOGT configuration without preprocessing a U gate."""
    weights = json.loads(
        (common.ICRA_ROOT / "focused_results" / "frozen_weights.json").read_text(encoding="utf-8")
    )
    base = common._EXPERIMENT.make_config(weights, 1)
    return OptimizationConfig(
        initial_speed=base.initial_speed,
        minimum_initial_duration=base.minimum_initial_free_duration,
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
        dynamic_limits=base.dynamic_limits,
        quadrotor=base.quadrotor,
    )


def build_curved_track(
    *, vertex_count: int = 256, quadrature_order: int = 64
):
    """Create a fixed-plane, self-spinning open track with three curved apertures."""
    rho = DEFAULT_BODY.circumscribed_radius + 0.015
    catalog = preprocess_shape_catalog(
        rho=rho,
        vertex_count=vertex_count,
        quadrature_order=quadrature_order,
        shape_names=SHAPES,
    )
    basis, normal = basis_from_normal((1.0, 0.0, 0.0))
    windows = tuple(
        RotatingWindow(
            name=shape,
            gate=catalog[shape],
            center=center,
            plane_basis=basis,
            normal=normal,
            theta0=phase,
            omega=omega,
            thickness=0.0,
            rho=rho,
        )
        for shape, center, phase, omega in zip(SHAPES, CENTERS, PHASES, OMEGAS)
    )
    scenario = RotSyncScenario(
        name="curved_three_window_open_spin",
        start_state=BoundaryState(np.asarray(START, dtype=float)),
        goal_state=BoundaryState(np.asarray(GOAL, dtype=float)),
        windows=windows,
        description=(
            "Open track through a limacon, a five-lobe analytic wavy curve, and a "
            "line/cubic-Bezier mixed boundary; centers and planes fixed, apertures spin in-plane."
        ),
        body=DEFAULT_BODY,
        difficulty="curved-multi-window",
        design_basis=("new curve-bearing track", "fixed planes", "constant in-plane spin"),
    )
    return scenario, make_config()


def fixed_waypoints(scenario):
    """Return deepest safe local points and their unconstrained SC coordinates."""
    local_points, d_values = [], []
    for window in scenario.windows:
        point = np.asarray(
            polylabel(Polygon(window.safe_polygon), tolerance=1.0e-7).coords[0], dtype=float
        )
        disk = np.asarray(window.gate.sc_map.inverse(point), dtype=float)
        local_points.append(point)
        d_values.append(_disk_to_unconstrained(disk))
    return np.asarray(local_points), np.asarray(d_values)


class FixedMultiWindowObjective:
    """Freeze all SC points and optimize only the four temporal K variables."""

    def __init__(self, free: MultiWindowObjective, fixed_d: np.ndarray):
        self.free = free
        self.config = free.config
        self.fixed_d = np.asarray(fixed_d, dtype=float).reshape(-1, 2)
        self.n_k = self.fixed_d.shape[0] + 1
        self.dimension = self.n_k

    def full_x(self, k):
        k = np.asarray(k, dtype=float)
        if k.shape != (self.n_k,) or not np.all(np.isfinite(k)):
            raise ValueError(f"Fixed-WP requires {self.n_k} finite K variables")
        return np.r_[k, self.fixed_d.ravel()]

    def initial_guess(self):
        return self.free.joint.initial_guess(self.fixed_d)[: self.n_k]

    def value_and_gradient(self, k):
        value, gradient = self.free.value_and_gradient(self.full_x(k))
        return value, gradient[: self.n_k]

    def forward(self, k):
        result = self.free.forward(self.full_x(k))
        result.method = "Fixed-WP"
        return result


def _solve(name, objective, initial):
    calls = 0
    last = time.perf_counter()

    def fun(x):
        nonlocal calls, last
        value, gradient = objective.value_and_gradient(x)
        calls += 1
        now = time.perf_counter()
        if now - last >= 25.0:
            print(f"{name}: evaluations={calls}, J={value:.8g}", flush=True)
            last = now
        return value, gradient

    begin = time.perf_counter()
    result = _minimize_togt_lbfgs(fun, np.asarray(initial, dtype=float), objective.config)
    return result, objective.forward(result.x), time.perf_counter() - begin, calls


def embedding_errors(fixed_forward, free_forward):
    grid = np.linspace(0.0, fixed_forward.trajectory.total_time, 257)
    derivative = {
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
    return {
        "flight_time_abs_error": abs(
            fixed_forward.trajectory.total_time - free_forward.trajectory.total_time
        ),
        "derivative_max_abs_errors": derivative,
    }


def _scene_record(scenario):
    return {
        "name": scenario.name,
        "description": scenario.description,
        "start": scenario.start_state.matrix,
        "goal": scenario.goal_state.matrix,
        "body_half_extents": scenario.body.half_extents,
        "boundary_sources": {
            "limacon": "analytic r(theta)=2.1+0.72 cos(theta)",
            "wavy": "analytic r(theta)=2.15+0.35 cos(5 theta)",
            "line_bezier": "three line segments and two cubic Bezier segments",
        },
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


def _row(name, optimizer, forward, solve_seconds, calls, screen, final_audit, x):
    audit_pass = bool(final_audit and final_audit["trajectory_validation_pass"])
    per_window = [] if final_audit is None else final_audit["per_window"]
    clearances = [
        item["audit"].get("minimum_frame_clearance") for item in per_window
    ]
    return {
        "method": name,
        "optimizer_success": bool(optimizer.success),
        "optimizer_message": str(optimizer.message),
        "optimizer_iterations": int(optimizer.nit),
        "objective": float(optimizer.fun),
        "solve_seconds": solve_seconds,
        "objective_evaluations": calls,
        "flight_time": float(forward.trajectory.total_time),
        "durations": forward.durations,
        "crossing_times": forward.crossing_times,
        "local_points": forward.local_points,
        "decision_vector": x,
        "intermediate_screen": screen,
        "whole_body_audit_pass": audit_pass,
        "minimum_frame_clearance_per_window": clearances,
        "all_hard_requirements_pass": bool(screen["passed"] and audit_pass),
    }


def _plot(output, scenario, forwards, fixed_local):
    fig = plt.figure(figsize=(13, 7))
    axis = fig.add_subplot(1, 2, 1, projection="3d")
    colors = {"Fixed-WP": "#d95f02", "SC-DynaTOGT": "#1b6ca8"}
    for name, forward in forwards.items():
        t = np.linspace(0.0, forward.trajectory.total_time, 1200)
        p = forward.trajectory.evaluate(t)
        axis.plot(p[:, 0], p[:, 1], p[:, 2], color=colors[name], label=name, linewidth=2)
        q = np.asarray([w.world_point(lp, ct) for w, lp, ct in zip(scenario.windows, forward.local_points, forward.crossing_times)])
        axis.scatter(q[:, 0], q[:, 1], q[:, 2], color=colors[name], s=34)
    centers = np.asarray([w.center for w in scenario.windows])
    axis.scatter(centers[:, 0], centers[:, 1], centers[:, 2], marker="x", color="black", s=45)
    axis.set_xlabel("x [m]"); axis.set_ylabel("y [m]"); axis.set_zlabel("z [m]")
    axis.set_title("Curved rotating three-window track")
    axis.legend()

    panels = fig.add_subplot(1, 2, 2)
    panels.axis("off")
    inset_positions = ((0.56, 0.57, 0.18, 0.30), (0.76, 0.57, 0.18, 0.30), (0.66, 0.16, 0.18, 0.30))
    for i, (window, position) in enumerate(zip(scenario.windows, inset_positions)):
        ax = fig.add_axes(position)
        physical = np.vstack((window.physical_polygon, window.physical_polygon[0]))
        safe = np.vstack((window.safe_polygon, window.safe_polygon[0]))
        ax.plot(physical[:, 0], physical[:, 1], color="#222222", linewidth=1.5, label="physical")
        ax.plot(safe[:, 0], safe[:, 1], color="#6a9f58", linewidth=1.2, label="safe inset")
        ax.scatter(fixed_local[i, 0], fixed_local[i, 1], color=colors["Fixed-WP"], marker="s", s=24)
        for name, forward in forwards.items():
            point = forward.local_points[i]
            ax.scatter(point[0], point[1], color=colors[name], s=24)
        ax.set_aspect("equal"); ax.set_title(window.name, fontsize=9); ax.grid(alpha=0.18)
    fig.savefig(output / "comparison.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def _write_report(output, rows, embedding, fixed_local):
    fixed, sc = rows
    if fixed["all_hard_requirements_pass"] and sc["all_hard_requirements_pass"]:
        change = 100.0 * (fixed["flight_time"] - sc["flight_time"]) / fixed["flight_time"]
        conclusion = f"两种方法均通过全部硬要求；SC-DynaTOGT 相对 Fixed-WP 缩短 {change:.3f}%。"
    else:
        conclusion = "至少一种方法未通过全部硬要求，因此不把更短时间解释为合格性能提升。"
    lines = [
        "# 曲线边界三窗口：SC-DynaTOGT vs Fixed-WP",
        "",
        "新赛道依次使用利马松、五瓣波浪曲线和直线–三次 Bézier 混合边界。三扇窗中心和平面固定，只绕法向匀速自旋；窗口厚度为零。",
        "Fixed-WP 固定在各安全内缩区的最大内切圆中心，仅优化 4 个 K 时间变量。原始 SC-DynaTOGT 从 Fixed-WP 最终解精确热启动，再开放 3 组二维 SC 变量。",
        "",
        "|方法|优化器停止|飞行时间 (s)|求解时间 (s)|穿越区间球筛查|最终整机审计|全部硬要求|",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"|{row['method']}|{row['optimizer_success']}|{row['flight_time']:.9f}|"
            f"{row['solve_seconds']:.3f}|{row['intermediate_screen']['passed']}|"
            f"{row['whole_body_audit_pass']}|{row['all_hard_requirements_pass']}|"
        )
    lines.extend([
        "",
        conclusion,
        f"Fixed-WP 局部点：`{json.dumps(jsonable(fixed_local), ensure_ascii=False)}`。",
        f"嵌入检查的 0–4 阶最大轨迹误差为 `{max(embedding['derivative_max_abs_errors'].values()):.3e}`。",
        "中间筛查只检查每扇窗的球体穿越接触区间，并做全程动力学采样；最终整机审计使用真实姿态长方体。几何与动力学结论均为名义模型密集采样证据，不是连续域证书。",
        "",
    ])
    (output / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--vertex-count", type=int, default=256)
    parser.add_argument("--quadrature-order", type=int, default=64)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    begin = time.perf_counter()

    print("Preprocessing limacon, wavy and line-Bezier boundaries", flush=True)
    scenario, config = build_curved_track(
        vertex_count=args.vertex_count, quadrature_order=args.quadrature_order
    )
    fixed_local, fixed_d = fixed_waypoints(scenario)
    free = MultiWindowObjective(scenario, config)
    fixed = FixedMultiWindowObjective(free, fixed_d)
    write_json(output / "scene.json", _scene_record(scenario))
    write_json(output / "protocol.json", {
        "methods": METHODS,
        "model": "same four-segment degree-7 MINCO and native SC-DynaTOGT objective",
        "fixed_waypoint_rule": "polylabel center of each safe inset polygon",
        "free_initialization": "exact embedding of final Fixed-WP solution",
        "ranking": "compare flight time only when all hard requirements pass",
        "intermediate": "sphere checks only over each window contact interval; full-flight sampled dynamics",
        "final": "oriented cuboid audit of each final trajectory",
        "sphere_dt": 0.0002,
        "sphere_refine_dt": 0.00005,
        "dynamics_dt": 0.001,
        "cuboid_dt": 0.0002,
        "vertex_count": args.vertex_count,
        "quadrature_order": args.quadrature_order,
        "config": asdict(config),
    })

    print("Optimizing Fixed-WP (4 K variables)", flush=True)
    fixed_result, fixed_forward, fixed_seconds, fixed_calls = _solve(
        "Fixed-WP", fixed, fixed.initial_guess()
    )
    free_initial = fixed.full_x(fixed_result.x)
    embedded = free.forward(free_initial)
    embedding = embedding_errors(fixed_forward, embedded)
    if max(embedding["derivative_max_abs_errors"].values()) > 1e-10:
        raise RuntimeError("Fixed-WP embedding into SC-DynaTOGT is not exact")

    print("Optimizing original SC-DynaTOGT (4 K + 3 x 2 D variables)", flush=True)
    sc_result, sc_forward, sc_seconds, sc_calls = _solve(
        "SC-DynaTOGT", free, free_initial
    )
    sc_forward.method = "SC-DynaTOGT"
    forwards = {"Fixed-WP": fixed_forward, "SC-DynaTOGT": sc_forward}
    optimizers = {"Fixed-WP": fixed_result, "SC-DynaTOGT": sc_result}
    solve_meta = {
        "Fixed-WP": (fixed_seconds, fixed_calls, fixed.full_x(fixed_result.x)),
        "SC-DynaTOGT": (sc_seconds, sc_calls, np.asarray(sc_result.x)),
    }

    rows = []
    for name in METHODS:
        forward = forwards[name]
        print(f"Screening final {name} trajectory", flush=True)
        screen = screen_candidate(forward, scenario, config)
        print(f"Final oriented-cuboid audit for {name}", flush=True)
        audit = audit_multi(scenario, forward, config)
        seconds, calls, decision = solve_meta[name]
        rows.append(_row(name, optimizers[name], forward, seconds, calls, screen, audit, decision))
        np.savez_compressed(
            output / ("fixed_wp_trajectory.npz" if name == "Fixed-WP" else "sc_dynatogt_trajectory.npz"),
            x=decision,
            coefficients=forward.trajectory.coefficients,
            durations=forward.durations,
            crossing_times=forward.crossing_times,
            local_points=forward.local_points,
        )
        write_json(output / ("fixed_wp_audit.json" if name == "Fixed-WP" else "sc_dynatogt_audit.json"), audit)

    result = {
        "status": "COMPARABLE_ALL_PASS" if all(r["all_hard_requirements_pass"] for r in rows) else "HARD_REQUIREMENT_FAILURE",
        "scene": scenario.name,
        "fixed_local_points": fixed_local,
        "fixed_d": fixed_d,
        "embedding": embedding,
        "rows": rows,
        "total_seconds": time.perf_counter() - begin,
        "evidence": "sampled nominal-model validation; not continuous certification",
    }
    if result["status"] == "COMPARABLE_ALL_PASS":
        result["sc_time_reduction_percent"] = 100.0 * (
            rows[0]["flight_time"] - rows[1]["flight_time"]
        ) / rows[0]["flight_time"]
    write_json(output / "result.json", result)
    _write_report(output, rows, embedding, fixed_local)
    _plot(output, scenario, forwards, fixed_local)
    print(json.dumps(jsonable(result), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
