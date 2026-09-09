#!/usr/bin/env python3
"""Build and compare Fixed-WP and original SC-DynaTOGT on a mixed seven-window track."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import polylabel

from nonconvex_timevarying_window.comparisons.curved_rotating_sc_fixed_wp.experiment import (
    FixedMultiWindowObjective,
    embedding_errors,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_seeded import _disk_to_unconstrained
from nonconvex_timevarying_window.phase_governed_sc_tracking.experiment import _build_counterexample
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import final_audit, jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import dynamics_check, screen_candidate
from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import RotatingWindow, basis_from_normal
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import RotSyncScenario, preprocess_shape_catalog
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs


SHAPES = ("balanced_U", "limacon", "star", "wavy", "L", "line_bezier", "balanced_U")
CENTERS = (
    (0.0, 0.0, 1.8),
    (4.0, 0.8, 2.0),
    (8.0, -0.7, 1.6),
    (12.0, 0.9, 2.2),
    (16.0, -0.8, 1.5),
    (20.0, 0.6, 2.0),
    (24.0, -0.5, 1.8),
)
PHASES = (1.10, -0.35, 0.55, -0.70, 0.90, -1.05, 2.00)
OMEGAS = (18.0, -1.7, 2.1, -2.4, 2.7, -3.0, -15.0)
START = (-4.5, -0.8, 1.8)
GOAL = (28.5, 0.7, 1.8)


def build_seven_track(*, vertex_count=256, quadrature_order=64):
    _, config, _, counterexample, _, _, _ = _build_counterexample()
    balanced = counterexample.windows[0].gate
    rho = counterexample.windows[0].rho
    catalog = preprocess_shape_catalog(
        rho=rho,
        vertex_count=vertex_count,
        quadrature_order=quadrature_order,
        shape_names=("limacon", "wavy", "L", "line_bezier"),
    )
    # The star's ten exact corners need no dense curve resampling; at 256
    # redundant edge samples its SC prevertex residual is worse than the
    # accepted 64-vertex representation, so retain the existing formal setup.
    catalog.update(preprocess_shape_catalog(
        rho=rho, vertex_count=64, quadrature_order=quadrature_order,
        shape_names=("star",),
    ))
    gates = (balanced, catalog["limacon"], catalog["star"], catalog["wavy"],
             catalog["L"], catalog["line_bezier"], balanced)
    basis, normal = basis_from_normal((1.0, 0.0, 0.0))
    windows = tuple(
        RotatingWindow(
            name=f"W{i + 1}_{shape}", gate=gate, center=center,
            plane_basis=basis, normal=normal, theta0=phase, omega=omega,
            thickness=0.0, rho=rho,
        )
        for i, (shape, gate, center, phase, omega) in enumerate(
            zip(SHAPES, gates, CENTERS, PHASES, OMEGAS)
        )
    )
    scenario = RotSyncScenario(
        "seven_mixed_open_spin",
        BoundaryState(np.asarray(START, dtype=float)),
        BoundaryState(np.asarray(GOAL, dtype=float)),
        windows,
        "Seven fixed parallel planes with mixed non-convex/curved apertures and in-plane spin.",
        counterexample.body,
        "seven-window-mixed",
        ("balanced-U collision stressor", "mixed polygonal and curved boundaries"),
    )
    fixed_local, fixed_d = [], []
    for window in windows:
        q = np.asarray(polylabel(Polygon(window.safe_polygon), tolerance=1e-7).coords[0], dtype=float)
        disk = np.asarray(window.gate.sc_map.inverse(q), dtype=float)
        fixed_local.append(q)
        fixed_d.append(_disk_to_unconstrained(disk))
    return scenario, config, np.asarray(fixed_local), np.asarray(fixed_d)


def scene_record(scenario):
    return {
        "name": scenario.name,
        "shape_sequence": SHAPES,
        "start": scenario.start_state.matrix,
        "goal": scenario.goal_state.matrix,
        "body_half_extents": scenario.body.half_extents,
        "windows": [
            {"name": w.name, "center": w.center, "normal": w.normal,
             "plane_basis": w.plane_basis, "theta0": w.theta0, "omega": w.omega,
             "thickness": w.thickness, "rho": w.rho,
             "physical_polygon": w.physical_polygon, "safe_polygon": w.safe_polygon}
            for w in scenario.windows
        ],
    }


def solve(name, objective, initial):
    calls = 0
    last = time.perf_counter()
    def fun(x):
        nonlocal calls, last
        value, gradient = objective.value_and_gradient(x)
        calls += 1
        now = time.perf_counter()
        if now - last > 25:
            print(f"{name}: evaluations={calls}, J={value:.7g}", flush=True)
            last = now
        return value, gradient
    started = time.perf_counter()
    result = _minimize_togt_lbfgs(fun, initial, objective.config)
    return result, objective.forward(result.x), time.perf_counter() - started, calls


def collision_audit(scenario, forward, config, *, stop_at_first_failure=True):
    records = []
    for i, window in enumerate(scenario.windows):
        sub_scenario = replace(scenario, windows=(window,))
        sub_forward = SimpleNamespace(
            trajectory=forward.trajectory,
            crossing_times=np.asarray(forward.crossing_times)[i:i+1],
            local_points=np.asarray(forward.local_points)[i:i+1],
            crossing_local_index=0,
            durations=forward.durations,
        )
        audit, _ = final_audit(sub_scenario, sub_forward, config)
        collision_pass = bool(
            audit.get("collision_free", False)
            and audit.get("solid_exterior_violating_samples", 1) == 0
        )
        records.append({"window_index": i, "window_name": window.name,
                        "collision_pass": collision_pass, "audit": audit})
        if stop_at_first_failure and not collision_pass:
            break
    return {"passed": len(records) == len(scenario.windows) and all(r["collision_pass"] for r in records),
            "checked_windows": len(records), "per_window": records}


def method_row(name, result, forward, seconds, calls, scenario, config, collision):
    dynamics = dynamics_check(forward.trajectory, config)
    screen = screen_candidate(forward, scenario, config)
    return {
        "method": name, "optimizer_success": bool(result.success),
        "optimizer_message": str(result.message), "iterations": int(result.nit),
        "objective": float(result.fun), "solve_seconds": seconds,
        "objective_evaluations": calls, "flight_time": float(forward.trajectory.total_time),
        "durations": forward.durations, "crossing_times": forward.crossing_times,
        "local_points": forward.local_points, "decision_vector": np.asarray(result.x),
        "dynamics": dynamics, "intermediate_screen": screen,
        "collision": collision, "dynamics_pass": bool(dynamics["passed"]),
        "collision_pass": bool(collision["passed"]),
        "all_hard_requirements_pass": bool(dynamics["passed"] and collision["passed"] and screen["passed"]),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    print("Preprocessing seven mixed windows", flush=True)
    scenario, config, fixed_local, fixed_d = build_seven_track()
    write_json(output / "scene.json", scene_record(scenario))
    write_json(output / "protocol.json", {
        "methods": ["Fixed-WP", "SC-DynaTOGT"], "config": asdict(config),
        "fixed_waypoint_rule": "polylabel of every safe inset polygon",
        "sc_initialization": "exact embedding of final Fixed-WP solution",
        "collision_audit": "oriented cuboid, <=0.2 ms; stop after first failing window",
    })
    free = MultiWindowObjective(scenario, config)
    fixed = FixedMultiWindowObjective(free, fixed_d)
    print("Solving Fixed-WP: 8 K variables", flush=True)
    fixed_result, fixed_forward, fixed_seconds, fixed_calls = solve(
        "Fixed-WP", fixed, fixed.initial_guess()
    )
    free_initial = fixed.full_x(fixed_result.x)
    errors = embedding_errors(fixed_forward, free.forward(free_initial))
    if max(errors["derivative_max_abs_errors"].values()) > 1e-10:
        raise RuntimeError("Fixed-WP embedding is not exact")
    print("Solving original SC-DynaTOGT: 8 K + 7 x 2 D", flush=True)
    sc_result, sc_forward, sc_seconds, sc_calls = solve(
        "SC-DynaTOGT", free, free_initial
    )
    print("Auditing Fixed-WP collision", flush=True)
    fixed_collision = collision_audit(scenario, fixed_forward, config)
    print("Auditing SC-DynaTOGT collision", flush=True)
    sc_collision = collision_audit(scenario, sc_forward, config)
    rows = [
        method_row("Fixed-WP", fixed_result, fixed_forward, fixed_seconds, fixed_calls,
                   scenario, config, fixed_collision),
        method_row("SC-DynaTOGT", sc_result, sc_forward, sc_seconds, sc_calls,
                   scenario, config, sc_collision),
    ]
    for row, forward in zip(rows, (fixed_forward, sc_forward)):
        stem = "fixed_wp" if row["method"] == "Fixed-WP" else "sc_dynatogt"
        np.savez_compressed(output / f"{stem}_trajectory.npz", x=row["decision_vector"],
                            coefficients=forward.trajectory.coefficients,
                            durations=forward.durations, crossing_times=forward.crossing_times,
                            local_points=forward.local_points)
        write_json(output / f"{stem}_collision_audit.json", row["collision"])
    result = {"scene": scenario.name, "fixed_local_points": fixed_local,
              "fixed_d": fixed_d, "embedding": errors, "rows": rows,
              "fixed_collision_requirement_met": not rows[0]["collision_pass"],
              "total_seconds": time.perf_counter() - started,
              "evidence": "sampled nominal-model validation; not continuous certification"}
    write_json(output / "result.json", result)
    print(json.dumps(jsonable(result), ensure_ascii=False), flush=True)
    return 0 if result["fixed_collision_requirement_met"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
