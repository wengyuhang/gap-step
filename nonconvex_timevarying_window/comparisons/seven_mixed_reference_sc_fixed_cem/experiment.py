#!/usr/bin/env python3
"""Frozen seven-window mixed-track construction and three-method comparison."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
from functools import lru_cache
import json
from pathlib import Path
import time

import numpy as np
from shapely.geometry import Polygon
from shapely.ops import polylabel

from nonconvex_timevarying_window.comparisons.curved_rotating_sc_fixed_wp.experiment import (
    FixedMultiWindowObjective,
    embedding_errors,
)
from nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.experiment import (
    collision_audit,
    method_row,
    solve,
)
from nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.cem import (
    nonzero_polar_seed,
)
from nonconvex_timevarying_window.feasibility_guided_cem_sc_dynatogt.search import (
    CEMConfig,
    feasible_rank,
    local_cem_search,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt.compare_fixed_wp_seeded import (
    _disk_to_unconstrained,
)
from nonconvex_timevarying_window.interpolated_rot_sync_sc_togt import (
    compare_fixed_wp_counterexample as comparison_common,
)
from nonconvex_timevarying_window.phase_governed_sc_tracking.experiment import _build_counterexample
from nonconvex_timevarying_window.random_dk_sc_dynatogt.experiment import jsonable, write_json
from nonconvex_timevarying_window.random_dk_sc_dynatogt.multi_window import MultiWindowObjective
from nonconvex_timevarying_window.random_dk_sc_dynatogt.safety import screen_candidate
from nonconvex_timevarying_window.rot_sync_sc_togt.geometry import RotatingWindow, basis_from_normal
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import RotSyncScenario, preprocess_shape_catalog
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessedGate
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.dynamics import ObjectiveWeights, PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig, _minimize_togt_lbfgs
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations


HERE = Path(__file__).resolve().parent
REFERENCE_RESULT = (
    HERE.parent.parent / "feasibility_guided_cem_sc_dynatogt" /
    "results" / "three_u_seed11" / "result.json"
)
PREPROCESSED_ROOT = HERE / "preprocessed_gates"
SHAPES = ("balanced_U", "limacon", "star", "balanced_U", "wavy", "line_bezier", "balanced_U")
CENTERS = (
    (0.0, 0.0, 1.8),
    (3.0, 0.8517106914106894, 1.7414877454276039),
    (6.0, 1.4750058289874957, 1.4434041829317883),
    (9.0, 1.0, 2.3),
    (12.0, 1.8002041999304057, 1.2988931221507263),
    (15.0, 1.8795553123871587, 0.6606545464645571),
    (18.0, -0.8, 1.4),
)
PHASES = (1.1, -0.4, 0.5, 0.3, -0.7, 1.0, 2.0)
OMEGAS = (18.0, -0.8, 0.9, 18.0, -0.7, 0.8, 18.0)
START = (-4.5, -0.8, 1.8)
GOAL = (22.5, 0.8, 1.8)
REFERENCE_CROSSINGS = np.asarray((
    2.6727429530689184,
    3.2558516694545947,
    3.7634449849178946,
    4.235024690958269,
    4.684467208605317,
    5.124440505770465,
    5.585578258120989,
))
REFERENCE_TOTAL_TIME = 7.390546627144163


@lru_cache(maxsize=1)
def _frozen_optimization_config():
    """Recreate the original comparison config without rebuilding its gate."""
    experiment = comparison_common._EXPERIMENT
    weights = json.loads(
        (comparison_common.ICRA_ROOT / "focused_results" / "frozen_weights.json").read_text(
            encoding="utf-8"))
    rot_config = experiment.make_config(weights, 1)
    return OptimizationConfig(
        initial_speed=rot_config.initial_speed,
        minimum_initial_duration=rot_config.minimum_initial_free_duration,
        max_iterations=0, max_line_search_steps=64, memory_size=256,
        past_iterations=32, function_tolerance=1.0e-5, gradient_tolerance=0.0,
        samples_per_segment=None, include_window_time_gradient=True,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        penalty_weights=PenaltyWeights(
            velocity=0.0, collective_thrust=0.0, body_rate=1.0, rotor_thrust=1.0),
        dynamic_limits=rot_config.dynamic_limits, quadrotor=rot_config.quadrotor,
    )


@lru_cache(maxsize=1)
def _frozen_u_context():
    """Load the balanced-U artifact; use the legacy builder only to create it once."""
    directory = PREPROCESSED_ROOT / "balanced_U"
    if (directory / "manifest.json").is_file():
        experiment = comparison_common._EXPERIMENT
        return (_frozen_optimization_config(), PreprocessedGate.load(directory),
                experiment.PLANNING_RHO, experiment.BODY)
    _, config, _, counterexample, _, _, _ = _build_counterexample()
    gate = counterexample.windows[0].gate
    gate.save(directory)
    return config, gate, counterexample.windows[0].rho, counterexample.body


@lru_cache(maxsize=4)
def _mixed_gate_catalog(*, rho, vertex_count, quadrature_order):
    """Load the frozen offline gates, creating their portable artifacts once."""
    counts = {"limacon": vertex_count, "wavy": vertex_count,
              "line_bezier": vertex_count, "star": 64}
    frozen = vertex_count == 256 and quadrature_order == 64
    directories = {name: PREPROCESSED_ROOT / name for name in counts}
    if frozen and all((directory / "manifest.json").is_file()
                      for directory in directories.values()):
        catalog = {name: PreprocessedGate.load(directory)
                   for name, directory in directories.items()}
        if any(abs(gate.safe_region.distance - rho) > 1e-12 for gate in catalog.values()):
            raise ValueError("cached gate clearance does not match the frozen aircraft envelope")
        return catalog

    catalog = preprocess_shape_catalog(
        rho=rho, vertex_count=vertex_count, quadrature_order=quadrature_order,
        shape_names=("limacon", "wavy", "line_bezier"),
    )
    catalog.update(preprocess_shape_catalog(
        rho=rho, vertex_count=64, quadrature_order=quadrature_order,
        shape_names=("star",),
    ))
    if frozen:
        for name, gate in catalog.items():
            gate.save(directories[name])
    return catalog


def build_seven_track(*, vertex_count=256, quadrature_order=64):
    config, balanced, rho, body = _frozen_u_context()
    catalog = _mixed_gate_catalog(
        rho=rho, vertex_count=vertex_count, quadrature_order=quadrature_order)
    gates = (balanced, catalog["limacon"], catalog["star"], balanced,
             catalog["wavy"], catalog["line_bezier"], balanced)
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
        "seven_mixed_reference_open_spin",
        BoundaryState(np.asarray(START, dtype=float)),
        BoundaryState(np.asarray(GOAL, dtype=float)),
        windows,
        "Three fast balanced-U windows with four intervening mixed curved/polygonal spinning apertures.",
        body,
        "seven-window-mixed-reference",
        ("balanced-U collision stressors", "curved and polygonal intervening apertures"),
    )
    fixed_local, fixed_d = [], []
    for window in windows:
        q = np.asarray(polylabel(Polygon(window.safe_polygon), tolerance=1e-7).coords[0], dtype=float)
        disk = np.asarray(window.gate.sc_map.inverse(q), dtype=float)
        fixed_local.append(q)
        fixed_d.append(_disk_to_unconstrained(disk))
    return scenario, config, np.asarray(fixed_local), np.asarray(fixed_d)


def lifted_reference_seed(scenario, fixed_d):
    """Insert four waypoints into the frozen sampled-feasible three-U trajectory."""
    source = json.loads(REFERENCE_RESULT.read_text(encoding="utf-8"))["selected"]
    source_d = np.asarray(source["x"], dtype=float)[4:].reshape(3, 2)
    d = np.asarray(fixed_d, dtype=float).copy()
    d[[0, 3, 6]] = source_d
    durations = np.diff(np.r_[0.0, REFERENCE_CROSSINGS, REFERENCE_TOTAL_TIME])
    return np.r_[k_from_durations(durations), d.ravel()]


def scene_record(scenario):
    return {
        "name": scenario.name, "shape_sequence": SHAPES,
        "start": scenario.start_state.matrix, "goal": scenario.goal_state.matrix,
        "body_half_extents": scenario.body.half_extents,
        "windows": [
            {"name": w.name, "center": w.center, "normal": w.normal,
             "plane_basis": w.plane_basis, "theta0": w.theta0, "omega": w.omega,
             "thickness": w.thickness, "rho": w.rho,
             "physical_polygon": w.physical_polygon, "safe_polygon": w.safe_polygon}
            for w in scenario.windows
        ],
    }


def evaluate_seed(objective, scenario, config, x):
    forward = objective.forward(x)
    return {
        "stage": "lifted_three_u_reference", "id": 0, "x": np.asarray(x),
        "flight_time": float(forward.trajectory.total_time),
        "screen": screen_candidate(forward, scenario, config),
    }


def write_report(path, rows):
    lines = ["# 七窗口混合赛道实验结果", "",
             "|方法|飞行时间 (s)|动力学约束|碰撞约束|",
             "|---|---:|---:|---:|"]
    for row in rows:
        lines.append(f"|{row['method']}|{row['flight_time']:.9f}|"
                     f"{'满足' if row['dynamics_pass'] else '不满足'}|"
                     f"{'满足' if row['collision_pass'] else '不满足'}|")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--population", type=int, default=128)
    parser.add_argument("--maximum-rounds", type=int, default=2)
    args = parser.parse_args(argv)
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    print("Preprocessing frozen seven-window mixed-reference scene", flush=True)
    scenario, config, fixed_local, fixed_d = build_seven_track()
    write_json(output / "scene.json", scene_record(scenario))
    objective = MultiWindowObjective(scenario, config)

    print("Solving Fixed-WP", flush=True)
    fixed = FixedMultiWindowObjective(objective, fixed_d)
    fixed_result, fixed_forward, fixed_seconds, fixed_calls = solve(
        "Fixed-WP", fixed, fixed.initial_guess())
    free_initial = fixed.full_x(fixed_result.x)
    errors = embedding_errors(fixed_forward, objective.forward(free_initial))
    print("Solving original SC-DynaTOGT from exact Fixed-WP embedding", flush=True)
    sc_result, sc_forward, sc_seconds, sc_calls = solve(
        "SC-DynaTOGT", objective, free_initial)

    print("Auditing Fixed-WP and original SC-DynaTOGT", flush=True)
    old_rows = [
        method_row("Fixed-WP", fixed_result, fixed_forward, fixed_seconds, fixed_calls,
                   scenario, config, collision_audit(scenario, fixed_forward, config)),
        method_row("SC-DynaTOGT", sc_result, sc_forward, sc_seconds, sc_calls,
                   scenario, config, collision_audit(scenario, sc_forward, config)),
    ]
    # Persist the expensive legacy-method solve/audit stage before starting CEM
    # and the full seven-window final audit.  This is also useful provenance for
    # long unattended runs.
    write_json(output / "old_methods_checkpoint.json", {
        "scene": scenario.name, "rows": old_rows, "embedding": errors,
        "fixed_collision_requirement_met": not old_rows[0]["collision_pass"],
    })
    for row, forward, stem in ((old_rows[0], fixed_forward, "fixed_wp"),
                               (old_rows[1], sc_forward, "sc_dynatogt")):
        np.savez_compressed(output / f"{stem}_trajectory.npz", x=row["decision_vector"],
                            coefficients=forward.trajectory.coefficients,
                            durations=forward.durations,
                            crossing_times=forward.crossing_times,
                            local_points=forward.local_points)

    seed_x = nonzero_polar_seed(lifted_reference_seed(scenario, fixed_d), len(scenario.windows) + 1)
    seed = evaluate_seed(objective, scenario, config, seed_x)
    if not seed["screen"]["passed"]:
        raise RuntimeError(f"lifted reference failed intermediate screen: {seed['screen']['reason']}")
    rows = [seed]
    cem_config = CEMConfig(
        seed=37, population=args.population, elite=max(8, args.population // 8),
        memory=max(4, args.population // 16), maximum_rounds=args.maximum_rounds,
        post_feasible_rounds=1, independent_time_std=0.025, common_time_std=0.025,
        angle_std=0.025, log_radius_std=0.08,
    )
    with (output / "candidates.jsonl").open("x", encoding="utf-8") as log:
        log.write(json.dumps(jsonable(seed), ensure_ascii=False, allow_nan=False) + "\n")
        def record(row):
            log.write(json.dumps(jsonable(row), ensure_ascii=False, allow_nan=False) + "\n")
            log.flush()
        cem_rows, _, rounds, _ = local_cem_search(
            objective, seed_x, scenario, config, cem_config, start_id=1, on_record=record)
        rows.extend(cem_rows)

    ranked = feasible_rank(rows)
    selected = None
    final_audits = []
    for candidate in ranked:
        forward = objective.forward(candidate["x"])
        print(f"Auditing new candidate {candidate['id']} T={candidate['flight_time']:.9f}", flush=True)
        audit = collision_audit(scenario, forward, config)
        final_audits.append({"id": candidate["id"], "flight_time": candidate["flight_time"],
                             "collision": audit})
        write_json(output / f"candidate_{candidate['id']}_final_audit.json", final_audits[-1])
        if audit["passed"]:
            selected = candidate
            np.savez_compressed(output / "selected_trajectory.npz", x=candidate["x"],
                                coefficients=forward.trajectory.coefficients,
                                durations=forward.durations,
                                crossing_times=forward.crossing_times,
                                local_points=forward.local_points)
            break
    if selected is None:
        raise RuntimeError("no new-algorithm candidate passed the final cuboid audit")
    new_row = {
        "method": "Feasibility-Guided CEM", "flight_time": selected["flight_time"],
        "decision_vector": selected["x"], "candidate_id": selected["id"],
        "dynamics": selected["screen"]["dynamics"], "dynamics_pass": True,
        "collision_pass": True, "intermediate_screen": selected["screen"],
    }
    comparison = [*old_rows, new_row]
    if old_rows[0]["collision_pass"]:
        raise RuntimeError("designed Fixed-WP collision failure was not reproduced")
    result = {
        "scene": scenario.name, "rows": comparison,
        "fixed_collision_requirement_met": True,
        "lifted_reference": str(REFERENCE_RESULT.resolve()),
        "lifted_seed": seed, "hard_screen_feasible": len(ranked),
        "candidate_count": len(rows), "cem_rounds": rounds,
        "final_audits": final_audits, "embedding": errors,
        "protocol": {"config": asdict(config), "cem": asdict(cem_config),
                     "selection": "flight time only among hard-screen passes; final cuboid pass required"},
        "total_seconds": time.perf_counter() - started,
        "evidence": "sampled nominal-model validation; not continuous certification",
    }
    write_json(output / "result.json", result)
    write_report(output / "REPORT.md", comparison)
    print(json.dumps(jsonable(result), ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
