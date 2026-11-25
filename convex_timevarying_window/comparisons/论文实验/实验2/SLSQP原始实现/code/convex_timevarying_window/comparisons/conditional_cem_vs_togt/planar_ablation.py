#!/usr/bin/env python3
"""Planar translation--rotation ablation for TOGT and Conditional CEM.

The two deliberately separated factors are (A) motion magnitude with five
windows fixed and (B) window count with motion magnitude fixed.  All apertures
have the same normal-size physical rectangle, lie at the same height, and only
translate horizontally and yaw about the vertical axis.  A missing solution
after the generous search budget is a *budgeted no-solution result*, never a
claim that the instance is physically infeasible.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, replace
import csv
import json
from pathlib import Path
import time

import numpy as np

from convex_timevarying_window.comparisons.conditional_cem_vs_togt.benchmark import (
    _audit_metrics, _flat_row, _jsonable, _solve_togt, _soft_values, optimization_config,
)
from convex_timevarying_window.conditional_dual_constraint_cem.objective import SafetyAugmentedTOGTObjective
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import ConvexSafetyConfig, HandwrittenConvexSafetyIntegral
from convex_timevarying_window.conditional_dual_constraint_cem.search import ConvexDualCEMConfig, conditional_dual_cem
from convex_timevarying_window.geometry import ConvexAperture, PeriodicConvexWindow
from convex_timevarying_window.togt.experiment import BODY_RADIUS, WINDOW_MARGIN
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs


# Multipliers apply jointly to a 0.35 m in-plane translation and 12 degree yaw.
MOTION_LEVELS = (("static", 0.0), ("low", 0.50), ("medium", 1.0), ("high", 1.50), ("very_high", 2.0))
# A logarithmic-like progression exposes both few-window and dozens-of-window
# behaviour without falsely treating adjacent, near-identical counts as
# independent difficulty levels.
COUNT_LEVELS = (3, 5, 8, 12, 16, 24, 32)
FIXED_COUNT = 5
FIXED_MOTION = 1.0
BASE_TRANSLATION_M = 0.35
BASE_YAW_RAD = np.deg2rad(12.0)
WINDOW_HALF_WIDTH_M = 1.65
WINDOW_HALF_HEIGHT_M = 1.30
COURSE_LENGTH_M = 36.0
WINDOW_Z_M = 3.2


@dataclass(frozen=True)
class PlanarCourse:
    course_id: str
    study: str
    factor_label: str
    window_count: int
    motion_scale: float
    repeat: int
    phases: tuple[float, ...]


def generate_courses(repeats: int, seed: int) -> tuple[PlanarCourse, ...]:
    """Generate matched phase realisations for each independent factor sweep."""
    rng = np.random.default_rng(seed)
    result: list[PlanarCourse] = []
    for label, scale in MOTION_LEVELS:
        for repeat in range(repeats):
            phases = tuple(float(x) for x in rng.uniform(-np.pi, np.pi, FIXED_COUNT))
            result.append(PlanarCourse(f"motion_{label}_{repeat:02d}", "motion", label,
                                       FIXED_COUNT, scale, repeat, phases))
    for count in COUNT_LEVELS:
        for repeat in range(repeats):
            phases = tuple(float(x) for x in rng.uniform(-np.pi, np.pi, count))
            result.append(PlanarCourse(f"count_{count}_{repeat:02d}", "count", str(count),
                                       count, FIXED_MOTION, repeat, phases))
    return tuple(result)


def _aperture() -> ConvexAperture:
    signs = np.array(([-1., -1.], [1., -1.], [1., 1.], [-1., 1.]))
    safe = np.array((WINDOW_HALF_WIDTH_M - WINDOW_MARGIN / 2.0,
                     WINDOW_HALF_HEIGHT_M - WINDOW_MARGIN / 2.0))
    return ConvexAperture("polygon", margin=WINDOW_MARGIN,
                          physical_vertices=signs * np.array((WINDOW_HALF_WIDTH_M, WINDOW_HALF_HEIGHT_M)),
                          safe_vertices=signs * safe)


def build_course(spec: PlanarCourse):
    """Parallel, vertical windows; motion is y-translation plus vertical yaw only."""
    windows = []
    for index in range(spec.window_count):
        phase = spec.phases[index]
        motion = MotionProfile(
            translation_amplitude=np.array((0.0, BASE_TRANSLATION_M * spec.motion_scale, 0.0)),
            rotation_amplitude=np.array((0.0, 0.0, BASE_YAW_RAD * spec.motion_scale)),
            scale_amplitude=0.0, translation_period=10.0, rotation_period=10.0,
            scale_period=10.0, phase=phase, scale_enabled=False,
        )
        windows.append(PeriodicConvexWindow(
            name=f"W{index + 1}_rectangle", aperture=_aperture(),
            # Hold start--goal distance fixed in the count sweep.  Otherwise
            # a longer route would be confounded with the number of windows.
            center0=np.array((COURSE_LENGTH_M * (index + 1) / (spec.window_count + 1), 0.0, WINDOW_Z_M)),
            angles0_rpy=np.array((0.0, -np.pi / 2.0, 0.0)), motion=motion,
        ))
    return SCWindowTrack(
        name=f"planar_{spec.course_id}", start=np.array((0.0, 0.0, WINDOW_Z_M)),
        goal=np.array((COURSE_LENGTH_M, 0.0, WINDOW_Z_M)),
        windows=tuple(windows), order=tuple(range(spec.window_count)),
    ), optimization_config()


def build_distributed_closed_hardest32():
    """TOGT-style, dispersed indoor 32-window closed stress course.

    Rather than a synthetic ring, the route makes four alternating passes
    across a rectangular flight hall, with wide lane separation and a return
    leg outside the gate field. This creates the spatially dispersed,
    heterogeneous layout of a real multi-gate course while retaining a planar
    model and a precisely identical start/goal state position.
    """
    x_lanes = np.array((-18., -13., -8., -3., 2., 7., 12., 18.))
    y_lanes = np.array((-12., -4., 4., 12.))
    centers_xy = []
    headings = []
    for row, y in enumerate(y_lanes):
        xs = x_lanes if row % 2 == 0 else x_lanes[::-1]
        heading = 0.0 if row % 2 == 0 else np.pi
        for x in xs:
            centers_xy.append((x, y)); headings.append(heading)
    count = len(centers_xy)
    centers = np.column_stack((np.asarray(centers_xy), np.full(count, WINDOW_Z_M)))
    # The final gate is at (-18, 12); an outer x=-24 return leg closes without
    # cutting through the four gate lanes.
    start = np.array((-23.0, -12.0, WINDOW_Z_M))
    rng = np.random.default_rng(20260932)
    windows = []
    for index, heading in enumerate(headings):
        phase = float(rng.uniform(-np.pi, np.pi))
        windows.append(PeriodicConvexWindow(
            name=f"W{index + 1}_distributed_rectangle", aperture=_aperture(),
            center0=centers[index],
            # Gate normal follows the local lane direction; sign is irrelevant
            # for a two-sided zero-thickness aperture.
            angles0_rpy=np.array((0.0, -np.pi / 2.0, heading)),
            motion=MotionProfile(
                translation_amplitude=np.array((0.70, 0.70, 0.0)),
                rotation_amplitude=np.array((0.0, 0.0, np.deg2rad(24.0))),
                scale_amplitude=0.0, translation_period=6.0, rotation_period=6.0,
                scale_period=6.0, phase=phase, scale_enabled=False,
            ),
        ))
    return SCWindowTrack(
        name="distributed_closed_32_high_motion", start=start, goal=start.copy(),
        windows=tuple(windows), order=tuple(range(count)),
    ), optimization_config()


def evaluate_course(spec: PlanarCourse, population: int, rounds: int, repair_iterations: int,
                    cem_wall_seconds: float, seed: int):
    track, config = build_course(spec)
    base = NativeJointTOGTObjective(track, config)
    nominal, nominal_evals, nominal_seconds = _solve_togt(base, config)
    nominal_forward = base.forward(nominal.x)
    started = time.perf_counter(); nominal_metrics = _audit_metrics(nominal_forward.trajectory, config, track)
    common = {"course_id": spec.course_id, "study": spec.study, "factor_label": spec.factor_label,
              "window_count": spec.window_count, "motion_scale": spec.motion_scale,
              "repeat": spec.repeat, "spec": asdict(spec)}
    togt = {**common, "method": "TOGT", "flight_time_s": float(nominal_forward.trajectory.total_time),
            "planning_seconds": nominal_seconds, "audit_seconds": time.perf_counter()-started,
            "optimizer_evaluations": nominal_evals, "output_status": "nominal", **nominal_metrics}
    safety = HandwrittenConvexSafetyIntegral(track.windows, base.head, base.tail,
                                             ConvexSafetyConfig(body_radius=BODY_RADIUS), base.core)
    _, _, nominal_safety = _soft_values(base, safety, nominal.x)
    mode = "dynamic_only" if nominal_safety == 0.0 else "joint"
    seed_x = np.asarray(nominal.x, dtype=float); repair_seconds = 0.; repair_evals = 0
    if mode == "joint":
        started = time.perf_counter()
        repaired = _minimize_togt_lbfgs(SafetyAugmentedTOGTObjective(base, safety).scipy_value_and_gradient,
                                        seed_x, replace(config, max_iterations=repair_iterations))
        repair_seconds = time.perf_counter()-started; repair_evals = int(repaired.nfev); seed_x = np.asarray(repaired.x, dtype=float)
    cem = ConvexDualCEMConfig(seed=seed + 7919 * spec.repeat + 101 * spec.window_count + int(100 * spec.motion_scale),
                              population=population, elite=max(8, population//4), memory=max(4, population//8),
                              maximum_rounds=rounds, maximum_seconds=cem_wall_seconds)
    started = time.perf_counter(); rows, _, cem_summaries = conditional_dual_cem(base, safety, seed_x, mode, cem); cem_seconds = time.perf_counter()-started
    candidates = sorted((x for x in rows if x["both_soft_zero"]), key=lambda x: (x["flight_time"], x["id"]))
    accepted = None; accepted_time = None; candidate_id = None; audit_started = time.perf_counter()
    for row in candidates:
        metrics = _audit_metrics(base.forward(row["x"]).trajectory, config, track)
        if metrics["joint_feasible"]:
            accepted, candidate_id = metrics, int(row["id"])
            accepted_time = float(row["flight_time"])
            break
    audit_seconds = time.perf_counter()-audit_started
    diagnostic_fallback_time = None
    if accepted is None:
        # Keep the fallback only for diagnostic values.  It is explicitly not an output solution.
        fallback = base.forward(seed_x)
        accepted = _audit_metrics(fallback.trajectory, config, track)
        diagnostic_fallback_time = float(fallback.trajectory.total_time)
        status = "no_audited_solution_under_budget"
    else:
        status = "audited_solution"
    conditional = {**common, "method": "Conditional Dual-Constraint CEM",
                   "flight_time_s": accepted_time,
                   "planning_seconds": nominal_seconds+repair_seconds+cem_seconds, "audit_seconds": audit_seconds,
                   "optimizer_evaluations": nominal_evals+repair_evals+len(rows), "branch": mode,
                   "output_status": status, "nominal_safety_integral": nominal_safety,
                   "diagnostic_fallback_flight_time_s": diagnostic_fallback_time,
                   "cem_candidate_count": len(rows), "cem_strict_double_zero_count": len(candidates),
                   "accepted_candidate_id": candidate_id, "cem_round_count": len(cem_summaries),
                   "cem_timed_out": bool(cem_summaries and cem_summaries[-1].get("timed_out", False)),
                   "cem_elapsed_seconds": cem_seconds, "cem_summaries": cem_summaries, **accepted}
    return [togt, conditional]


def write_report(rows, output, protocol):
    groups = [("motion", label) for label, _ in MOTION_LEVELS] + [("count", str(n)) for n in COUNT_LEVELS]
    lines = ["# Planar translation--rotation ablation", "", "## Protocol", "", "```json", json.dumps(_jsonable(protocol), indent=2), "```", "", "All openings are identical 3.30 m × 2.60 m physical rectangles. Windows remain at z=3.2 m; they only translate horizontally in y and yaw about the vertical axis. `no_audited_solution_under_budget` means no candidate passed the independent 1 ms audits within this generous finite budget; it is not an infeasibility proof.", "", "| Study | Factor | Method | Audited feasible | No audited output | Median plan (s) | Median feasible flight time (s) |", "|---|---|---|---:|---:|---:|---:|"]
    for study, label in groups:
        for method in ("TOGT", "Conditional Dual-Constraint CEM"):
            subset = [r for r in rows if r["study"] == study and r["factor_label"] == label and r["method"] == method]
            feasible = [r for r in subset if r["joint_feasible"]]
            noout = sum(r["output_status"] == "no_audited_solution_under_budget" for r in subset)
            flight = "—" if not feasible else f"{np.median([r['flight_time_s'] for r in feasible]):.3f}"
            lines.append(f"| {study} | {label} | {method} | {len(feasible)}/{len(subset)} | {noout}/{len(subset)} | {np.median([r['planning_seconds'] for r in subset]):.3f} | {flight} |")
    (output / "REPORT.md").write_text("\n".join(lines)+"\n", encoding="utf-8")


def write_metric_summary(rows, output):
    """Preserve every requested outcome, including failed returned diagnostics."""
    keys = ("flight_time_s", "planning_seconds", "audit_seconds", "min_safety_margin_m",
            "max_velocity_mps", "max_tilt_rad", "max_body_rate_xy_radps",
            "max_abs_body_rate_z_radps", "min_collective_thrust_n", "max_collective_thrust_n",
            "min_rotor_thrust_n", "max_rotor_thrust_n")
    summary = []
    for study, labels in (("motion", [x[0] for x in MOTION_LEVELS]),
                          ("count", [str(x) for x in COUNT_LEVELS])):
        for label in labels:
            for method in ("TOGT", "Conditional Dual-Constraint CEM"):
                subset = [r for r in rows if r["study"] == study and r["factor_label"] == label and r["method"] == method]
                entry = {"study": study, "factor_label": label, "method": method,
                         "instances": len(subset), "joint_feasible_count": sum(r["joint_feasible"] for r in subset),
                         "collision_count": sum(r["collision"] for r in subset),
                         "dynamic_pass_count": sum(r["dynamic_passed"] for r in subset),
                         "safety_pass_count": sum(r["safety_passed"] for r in subset),
                         "no_audited_output_count": sum(r["output_status"] == "no_audited_solution_under_budget" for r in subset)}
                for key in keys:
                    values = np.asarray([r[key] for r in subset if r[key] is not None], dtype=float)
                    entry[key] = None if not len(values) else {"median": float(np.median(values)), "mean": float(np.mean(values)), "std": float(np.std(values, ddof=0)), "q25": float(np.quantile(values, .25)), "q75": float(np.quantile(values, .75))}
                summary.append(entry)
    (output / "summary.json").write_text(json.dumps(_jsonable(summary), indent=2)+"\n", encoding="utf-8")


def make_figures(rows, output):
    """Paper-oriented summary plus a complete dynamics/safety diagnostic grid."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    output.mkdir(exist_ok=True)
    methods = ("TOGT", "Conditional Dual-Constraint CEM")
    colors = {"TOGT": "#5B7083", "Conditional Dual-Constraint CEM": "#D55E00"}
    short = {"TOGT": "TOGT", "Conditional Dual-Constraint CEM": "Conditional CEM"}
    studies = (("motion", [x[0] for x in MOTION_LEVELS], "motion magnitude"),
               ("count", [str(x) for x in COUNT_LEVELS], "window count"))
    for study, labels, xlabel in studies:
        pos = np.arange(len(labels)); width = .36
        fig, ax = plt.subplots(2, 2, figsize=(11.6, 7.4), constrained_layout=True)
        for mi, method in enumerate(methods):
            subsets = [[r for r in rows if r["study"] == study and r["factor_label"] == label and r["method"] == method] for label in labels]
            x = pos + (mi-.5)*width
            ax[0,0].bar(x, [np.mean([r["joint_feasible"] for r in s]) for s in subsets], width, color=colors[method], label=short[method])
            ax[0,1].bar(x, [np.mean([r["collision"] for r in s]) for s in subsets], width, color=colors[method])
            flight = [[r["flight_time_s"] for r in s if r["joint_feasible"]] for s in subsets]
            for j, values in enumerate(flight):
                if values: ax[1,0].scatter(np.full(len(values), x[j]), values, color=colors[method], s=28, alpha=.8)
            plan = [[r["planning_seconds"] for r in s] for s in subsets]
            boxes = ax[1,1].boxplot(plan, positions=x, widths=.27, patch_artist=True, showfliers=False)
            for box in boxes["boxes"]: box.set(facecolor=colors[method], alpha=.70)
        ax[0,0].set(title="Joint feasibility rate", ylabel="rate", ylim=(-.02,1.08))
        ax[0,1].set(title="Collision rate", ylabel="rate", ylim=(-.02,1.08))
        ax[1,0].set(title="Audited feasible flight time", ylabel="seconds")
        ax[1,1].set(title="Planning wall time", ylabel="seconds", yscale="log")
        for a in ax.flat: a.set_xticks(pos, labels)
        for a in ax[1,:]: a.set_xlabel(xlabel)
        ax[0,0].legend(frameon=False)
        fig.savefig(output / f"{study}_overview.png", dpi=220); plt.close(fig)
        metrics = (("min_safety_margin_m", "min safety margin (m)", 0.),
                   ("max_velocity_mps", "max velocity (m/s)", None),
                   ("max_tilt_rad", "max tilt (rad)", 6.28),
                   ("max_body_rate_xy_radps", "max XY body rate (rad/s)", 10.),
                   ("max_abs_body_rate_z_radps", "max |Z body rate| (rad/s)", 10.),
                   ("min_collective_thrust_n", "min collective thrust (N)", 1.),
                   ("max_collective_thrust_n", "max collective thrust (N)", 20.),
                   ("min_rotor_thrust_n", "min rotor thrust (N)", .25),
                   ("max_rotor_thrust_n", "max rotor thrust (N)", 5.))
        fig, axes = plt.subplots(3, 3, figsize=(13, 9), constrained_layout=True)
        for axis, (key, ylabel, limit) in zip(axes.flat, metrics):
            for mi, method in enumerate(methods):
                values = [[r[key] for r in rows if r["study"] == study and r["factor_label"] == label and r["method"] == method] for label in labels]
                boxes = axis.boxplot(values, positions=pos+(mi-.5)*width, widths=.27, patch_artist=True, showfliers=False)
                for box in boxes["boxes"]: box.set(facecolor=colors[method], alpha=.70)
            if limit is not None: axis.axhline(limit, color="black", linestyle="--", linewidth=1)
            axis.set(ylabel=ylabel, xticks=pos, xticklabels=labels)
        fig.savefig(output / f"{study}_safety_dynamics.png", dpi=220); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True); parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260911); parser.add_argument("--population", type=int, default=64)
    parser.add_argument("--rounds", type=int, default=50); parser.add_argument("--repair-iterations", type=int, default=600)
    parser.add_argument("--cem-wall-seconds", type=float, default=120.0)
    parser.add_argument("--workers", type=int, default=1); args = parser.parse_args(argv)
    if min(args.repeats, args.rounds, args.repair_iterations, args.workers, args.cem_wall_seconds) < 1 or args.population < 9:
        raise ValueError("repeats, rounds, repair-iterations and workers must be positive; population must be at least 9")
    output = args.outdir.resolve(); output.mkdir(parents=True, exist_ok=False)
    courses = generate_courses(args.repeats, args.seed)
    protocol = {"courses": len(courses), "seed": args.seed, "repeats": args.repeats, "population": args.population, "maximum_rounds": args.rounds, "repair_max_iterations": args.repair_iterations, "cem_wall_seconds": args.cem_wall_seconds, "workers": args.workers, "audit_step_s": 0.001, "fixed_opening_physical_size_m": [3.30, 2.60], "fixed_start_goal_distance_m": COURSE_LENGTH_M, "motion_sweep_fixed_window_count": FIXED_COUNT, "count_sweep_fixed_motion_scale": FIXED_MOTION, "motion_levels": MOTION_LEVELS, "window_count_levels": COUNT_LEVELS}
    (output / "protocol.json").write_text(json.dumps(_jsonable(protocol), indent=2)+"\n", encoding="utf-8")
    (output / "courses.json").write_text(json.dumps(_jsonable([asdict(x) for x in courses]), indent=2)+"\n", encoding="utf-8")
    rows=[]; started=time.perf_counter()
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        futures={executor.submit(evaluate_course, c, args.population, args.rounds, args.repair_iterations, args.cem_wall_seconds, args.seed): c for c in courses}
        for i, future in enumerate(as_completed(futures), 1):
            rows.extend(future.result()); print(f"[{i}/{len(courses)}] {futures[future].course_id}", flush=True)
    rows.sort(key=lambda r:(r["study"], r["factor_label"], r["repeat"], r["method"]))
    fields=sorted({k for r in rows for k in _flat_row(r)})
    with (output/"per_instance.csv").open("w", newline="", encoding="utf-8") as f:
        writer=csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(_flat_row(r) for r in rows)
    (output/"raw_results.json").write_text(json.dumps(_jsonable(rows), indent=2)+"\n", encoding="utf-8")
    protocol["wall_seconds"]=time.perf_counter()-started; (output/"protocol.json").write_text(json.dumps(_jsonable(protocol), indent=2)+"\n", encoding="utf-8")
    write_report(rows, output, protocol); write_metric_summary(rows, output); make_figures(rows, output / "figures"); print(f"done: {output}", flush=True)

if __name__ == "__main__": main()
