#!/usr/bin/env python3
"""Frozen multi-course benchmark: released TOGT vs Conditional CEM.

The benchmark deliberately compares planners, not controllers: every output
uses the same known periodic window motion and is independently audited on a
1 ms grid with the released TOGT C++ dynamics model and the real sphere-frame
geometry.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, replace
import csv
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from convex_timevarying_window.conditional_dual_constraint_cem.objective import (
    SafetyAugmentedTOGTObjective,
)
from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import (
    ConvexSafetyConfig,
    HandwrittenConvexSafetyIntegral,
)
from convex_timevarying_window.conditional_dual_constraint_cem.search import (
    ConvexDualCEMConfig,
    conditional_dual_cem,
)
from convex_timevarying_window.geometry import ConvexAperture, PeriodicConvexWindow
from convex_timevarying_window.togt.experiment import (
    ANGLES_RPY,
    BODY_RADIUS,
    CENTERS,
    SHAPE_NAMES,
    START,
    TOGT_MAX_TILT,
    WINDOW_MARGIN,
    dynamic_audit,
    safety_audit,
)
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective
from nonconvex_timevarying_window.sc_dynatogt.dynamics import (
    DynamicLimits,
    ObjectiveWeights,
    PenaltyWeights,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    OptimizationConfig,
    _minimize_togt_lbfgs,
)


@dataclass(frozen=True)
class Difficulty:
    name: str
    aperture_scale: float
    motion_amplitude_scale: float
    motion_rate_scale: float


DIFFICULTIES = (
    Difficulty("easy", 1.25, 0.55, 0.70),
    Difficulty("moderate", 1.16, 0.80, 0.90),
    Difficulty("hard", 1.08, 1.10, 1.15),
    Difficulty("extreme", 1.00, 1.40, 1.40),
)

BASE_PHASES = np.array((0.20, -0.60, 0.90, -0.30, 1.10, -1.00, 0.70))
BASE_TRANSLATION_AMPLITUDES = np.array([
    [0.18, 0.13, 0.10], [0.14, 0.20, 0.12], [0.20, 0.12, 0.15],
    [0.16, 0.18, 0.11], [0.13, 0.16, 0.18], [0.19, 0.15, 0.13],
    [0.15, 0.19, 0.16],
])
BASE_ROTATION_AMPLITUDES = np.deg2rad(np.array([
    [5.0, 7.0, 10.0], [7.0, 5.0, 9.0], [6.0, 8.0, 7.0],
    [8.0, 6.0, 10.0], [5.0, 9.0, 8.0], [7.0, 8.0, 6.0],
    [9.0, 5.0, 7.0],
]))
BASE_TRANSLATION_PERIODS = np.array([10.0 + 0.8 * index for index in range(7)])
BASE_ROTATION_PERIODS = np.array([8.5 + 0.7 * index for index in range(7)])


@dataclass(frozen=True)
class CourseSpec:
    course_id: str
    difficulty: str
    repeat: int
    phase_jitter: tuple[float, ...]
    period_jitter: tuple[float, ...]


def _apertures(scale: float):
    """Apply one frozen scale to physical and TOGT-safe apertures."""
    def regular(count, radius):
        angle = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
        direction = np.column_stack((np.cos(angle), np.sin(angle)))
        physical_radius = radius * scale
        if physical_radius <= WINDOW_MARGIN:
            raise ValueError("scaled regular aperture has no TOGT-safe radius")
        return ConvexAperture(
            "polygon", margin=WINDOW_MARGIN,
            physical_vertices=physical_radius * direction,
            safe_vertices=(physical_radius - WINDOW_MARGIN) * direction,
        )

    def rectangle(half_width, half_height):
        signs = np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])
        width, height = half_width * scale, half_height * scale
        safe = np.array([width - WINDOW_MARGIN / 2.0, height - WINDOW_MARGIN / 2.0])
        if np.any(safe <= 0.0):
            raise ValueError("scaled rectangle has no TOGT-safe aperture")
        return ConvexAperture(
            "polygon", margin=WINDOW_MARGIN,
            physical_vertices=signs * np.array([width, height]),
            safe_vertices=signs * safe,
        )

    return (
        rectangle(1.45, 1.15),
        ConvexAperture("circle", radius=1.35 * scale, margin=WINDOW_MARGIN),
        regular(5, 1.50),
        ConvexAperture("circle", radius=1.25 * scale, margin=WINDOW_MARGIN),
        regular(6, 1.45),
        ConvexAperture("circle", radius=1.40 * scale, margin=WINDOW_MARGIN),
        rectangle(1.35, 1.08),
    )


def optimization_config():
    return OptimizationConfig(
        initial_speed=1.0, minimum_initial_duration=0.20, max_iterations=0,
        max_line_search_steps=64, memory_size=256, past_iterations=32,
        function_tolerance=1e-5, gradient_tolerance=0.0,
        samples_per_segment=None, include_window_time_gradient=True,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        penalty_weights=PenaltyWeights(
            velocity=0.0, collective_thrust=0.0, body_rate=1.0, rotor_thrust=1.0,
        ),
        dynamic_limits=DynamicLimits(
            max_velocity=60.0, min_collective_thrust=1.0,
            max_collective_thrust=20.0, max_body_rate_xy=10.0,
            max_body_rate_z=10.0, min_rotor_thrust=0.25, max_rotor_thrust=5.0,
        ),
    )


def generate_courses(repeats_per_difficulty: int, seed: int) -> tuple[CourseSpec, ...]:
    rng = np.random.default_rng(seed)
    courses = []
    for difficulty in DIFFICULTIES:
        for repeat in range(repeats_per_difficulty):
            courses.append(CourseSpec(
                course_id=f"{difficulty.name}_{repeat:02d}",
                difficulty=difficulty.name,
                repeat=repeat,
                phase_jitter=tuple(float(value) for value in rng.uniform(-0.70, 0.70, 7)),
                period_jitter=tuple(float(value) for value in rng.uniform(-0.08, 0.08, 7)),
            ))
    return tuple(courses)


def build_course(spec: CourseSpec):
    difficulty = next(item for item in DIFFICULTIES if item.name == spec.difficulty)
    apertures = _apertures(difficulty.aperture_scale)
    phase_jitter = np.asarray(spec.phase_jitter)
    period_jitter = np.asarray(spec.period_jitter)
    windows = []
    for index in range(7):
        motion = MotionProfile(
            translation_amplitude=(
                BASE_TRANSLATION_AMPLITUDES[index] * difficulty.motion_amplitude_scale
            ),
            rotation_amplitude=(
                BASE_ROTATION_AMPLITUDES[index] * difficulty.motion_amplitude_scale
            ),
            scale_amplitude=0.0,
            translation_period=(
                BASE_TRANSLATION_PERIODS[index] * (1.0 + period_jitter[index])
                / difficulty.motion_rate_scale
            ),
            rotation_period=(
                BASE_ROTATION_PERIODS[index] * (1.0 - 0.5 * period_jitter[index])
                / difficulty.motion_rate_scale
            ),
            scale_period=9.0,
            phase=float(BASE_PHASES[index] + phase_jitter[index]),
            scale_enabled=False,
        )
        windows.append(PeriodicConvexWindow(
            name=f"W{index + 1}_{SHAPE_NAMES[index]}", aperture=apertures[index],
            center0=CENTERS[index], angles0_rpy=ANGLES_RPY[index], motion=motion,
        ))
    return SCWindowTrack(
        name=f"benchmark_{spec.course_id}", start=START, goal=START.copy(),
        windows=tuple(windows), order=tuple(range(7)),
    ), optimization_config()


def _jsonable(value: Any):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    return value


def _soft_values(base, safety, x):
    forward = base.forward(x)
    base_cost, _ = base.value_and_gradient(x)
    safe = safety.evaluate(forward.trajectory, with_gradient=False).value
    return forward, max(float(base_cost - forward.trajectory.total_time), 0.0), max(float(safe), 0.0)


def _audit_metrics(trajectory, config, track):
    dynamic = dynamic_audit(trajectory, config)
    safety = safety_audit(trajectory, track)
    extrema = dynamic["extrema"]
    return {
        "joint_feasible": bool(dynamic["passed"] and safety["passed"]),
        "collision": not bool(safety["passed"]),
        "dynamic_passed": bool(dynamic["passed"]),
        "safety_passed": bool(safety["passed"]),
        "min_safety_margin_m": min(row["minimum_margin"] for row in safety["per_window"]),
        "max_velocity_mps": extrema["max_velocity"],
        "max_tilt_rad": extrema["max_tilt"],
        "max_body_rate_xy_radps": extrema["max_body_rate_xy"],
        "max_abs_body_rate_z_radps": extrema["max_abs_body_rate_z"],
        "min_collective_thrust_n": extrema["min_collective_thrust"],
        "max_collective_thrust_n": extrema["max_collective_thrust"],
        "min_rotor_thrust_n": float(np.min(extrema["min_rotor_thrust"])),
        "max_rotor_thrust_n": float(np.max(extrema["max_rotor_thrust"])),
        "dynamic_audit": dynamic,
        "safety_audit": safety,
    }


def _solve_togt(base, config):
    count = 0
    def measured(x):
        nonlocal count
        count += 1
        return base.scipy_value_and_gradient(x)
    started = time.perf_counter()
    result = _minimize_togt_lbfgs(measured, base.initial_guess(), config)
    seconds = time.perf_counter() - started
    return result, count, seconds


def evaluate_course(spec: CourseSpec, population: int, maximum_rounds: int,
                    repair_max_iterations: int, cem_seed: int):
    track, config = build_course(spec)
    base = NativeJointTOGTObjective(track, config)
    nominal_result, nominal_evaluations, nominal_seconds = _solve_togt(base, config)
    nominal_forward = base.forward(nominal_result.x)
    nominal_audit_started = time.perf_counter()
    nominal_metrics = _audit_metrics(nominal_forward.trajectory, config, track)
    nominal_audit_seconds = time.perf_counter() - nominal_audit_started
    common = {
        "course_id": spec.course_id, "difficulty": spec.difficulty,
        "repeat": spec.repeat, "spec": asdict(spec),
    }
    togt = {
        **common, "method": "TOGT", "flight_time_s": float(nominal_forward.trajectory.total_time),
        "planning_seconds": nominal_seconds, "audit_seconds": nominal_audit_seconds,
        "optimizer_evaluations": nominal_evaluations, **nominal_metrics,
    }

    safety = HandwrittenConvexSafetyIntegral(
        track.windows, base.head, base.tail,
        ConvexSafetyConfig(body_radius=BODY_RADIUS), base.core,
    )
    _, _, nominal_safety = _soft_values(base, safety, nominal_result.x)
    mode = "dynamic_only" if nominal_safety == 0.0 else "joint"
    seed_x = np.asarray(nominal_result.x, dtype=float)
    repair_seconds = 0.0
    repair_evaluations = 0
    if mode == "joint":
        augmented = SafetyAugmentedTOGTObjective(base, safety)
        repair_config = replace(config, max_iterations=repair_max_iterations)
        started = time.perf_counter()
        repair_result = _minimize_togt_lbfgs(
            augmented.scipy_value_and_gradient, seed_x, repair_config
        )
        repair_seconds = time.perf_counter() - started
        repair_evaluations = int(repair_result.nfev)
        seed_x = np.asarray(repair_result.x, dtype=float)
    cem_config = ConvexDualCEMConfig(
        seed=cem_seed + 1009 * spec.repeat + 97 * DIFFICULTIES.index(
            next(item for item in DIFFICULTIES if item.name == spec.difficulty)
        ),
        population=population, elite=max(4, population // 4), memory=max(2, population // 8),
        maximum_rounds=maximum_rounds,
    )
    cem_started = time.perf_counter()
    rows, _, rounds = conditional_dual_cem(base, safety, seed_x, mode, cem_config)
    cem_seconds = time.perf_counter() - cem_started
    candidates = sorted(
        (row for row in rows if row["both_soft_zero"]),
        key=lambda row: (row["flight_time"], row["id"]),
    )
    candidate_metrics = None
    candidate_id = None
    selected_time = None
    audit_started = time.perf_counter()
    for row in candidates:
        forward = base.forward(row["x"])
        metrics = _audit_metrics(forward.trajectory, config, track)
        if metrics["joint_feasible"]:
            candidate_metrics = metrics
            candidate_id = int(row["id"])
            selected_time = float(row["flight_time"])
            break
    # Retain a physically interpretable failed output rather than silently
    # dropping it from safety/dynamics plots.
    if candidate_metrics is None:
        fallback = base.forward(seed_x)
        candidate_metrics = _audit_metrics(fallback.trajectory, config, track)
        selected_time = float(fallback.trajectory.total_time)
    audit_seconds = time.perf_counter() - audit_started
    conditional = {
        **common, "method": "Conditional Dual-Constraint CEM",
        "flight_time_s": selected_time,
        "planning_seconds": nominal_seconds + repair_seconds + cem_seconds,
        "audit_seconds": audit_seconds,
        "optimizer_evaluations": nominal_evaluations + repair_evaluations + len(rows),
        "branch": mode, "nominal_safety_integral": nominal_safety,
        "cem_candidate_count": len(rows), "cem_strict_double_zero_count": len(candidates),
        "accepted_candidate_id": candidate_id,
        "cem_rounds": rounds, **candidate_metrics,
    }
    return [togt, conditional]


def _flat_row(row):
    return {key: value for key, value in row.items() if not isinstance(value, (dict, list, tuple))}


def _wilson(successes, total, z=1.96):
    if total == 0:
        return (np.nan, np.nan)
    p = successes / total
    denominator = 1.0 + z*z/total
    center = (p + z*z/(2.0*total)) / denominator
    radius = z*np.sqrt((p*(1.0-p) + z*z/(4.0*total))/total) / denominator
    return max(0.0, center-radius), min(1.0, center+radius)


def make_figures(rows, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    outdir.mkdir(parents=True, exist_ok=True)
    methods = ("TOGT", "Conditional Dual-Constraint CEM")
    levels = [item.name for item in DIFFICULTIES]
    colors = {methods[0]: "#5B7083", methods[1]: "#D55E00"}
    labels = {methods[0]: "TOGT", methods[1]: "Conditional CEM"}

    figure, axes = plt.subplots(2, 2, figsize=(11.5, 7.6), constrained_layout=True)
    positions = np.arange(len(levels)); width = 0.36
    for method_index, method in enumerate(methods):
        rates, lower, upper, collisions, times = [], [], [], [], []
        for level in levels:
            subset = [row for row in rows if row["method"] == method and row["difficulty"] == level]
            success = sum(row["joint_feasible"] for row in subset)
            lo, hi = _wilson(success, len(subset))
            rates.append(success / len(subset)); lower.append(success / len(subset) - lo); upper.append(hi - success / len(subset))
            collisions.append(np.mean([row["collision"] for row in subset]))
            times.append([row["planning_seconds"] for row in subset])
        x = positions + (method_index - 0.5) * width
        axes[0, 0].bar(x, rates, width, color=colors[method], label=labels[method])
        axes[0, 0].errorbar(x, rates, yerr=np.array([lower, upper]), fmt="none", color="black", capsize=3)
        axes[0, 1].bar(x, collisions, width, color=colors[method], label=labels[method])
        box_positions = positions + (method_index - 0.5) * width
        box = axes[1, 1].boxplot(times, positions=box_positions, widths=0.28, patch_artist=True, showfliers=False)
        for patch in box["boxes"]:
            patch.set(facecolor=colors[method], alpha=0.72)

    rng = np.random.default_rng(0)
    for level_index, level in enumerate(levels):
        for method_index, method in enumerate(methods):
            subset = [row for row in rows if row["difficulty"] == level and row["method"] == method]
            x = level_index + (method_index - 0.5) * width + rng.uniform(-0.045, 0.045, len(subset))
            feasible = np.array([row["joint_feasible"] for row in subset], dtype=bool)
            flight = np.array([row["flight_time_s"] for row in subset])
            axes[1, 0].scatter(
                x[feasible], flight[feasible], color=colors[method], s=34,
                marker="o", edgecolors="black", linewidths=0.4,
            )
            axes[1, 0].scatter(
                x[~feasible], flight[~feasible], color=colors[method], s=38,
                marker="x", linewidths=1.3,
            )

    axes[0, 0].set(title="Joint feasibility rate", ylabel="rate", ylim=(-0.02, 1.08), xticks=positions, xticklabels=levels)
    axes[0, 0].legend(frameon=False, fontsize=9)
    axes[0, 1].set(title="Collision rate", ylabel="rate", ylim=(-0.02, 1.08), xticks=positions, xticklabels=levels)
    axes[1, 0].set(title="Returned flight time (circle: joint feasible)", ylabel="flight time (s)", xticks=positions, xticklabels=levels)
    axes[1, 1].set(title="Planner wall time", ylabel="seconds (log)", yscale="log", xticks=positions, xticklabels=levels)
    figure.savefig(outdir / "benchmark_overview.png", dpi=220)
    plt.close(figure)

    metrics = (
        ("min_safety_margin_m", "minimum safety margin (m)", 0.0),
        ("max_rotor_thrust_n", "maximum rotor thrust (N)", 5.0),
        ("max_body_rate_xy_radps", "maximum body rate XY (rad/s)", 10.0),
        ("max_tilt_rad", "maximum tilt (rad)", TOGT_MAX_TILT),
    )
    figure, axes = plt.subplots(2, 2, figsize=(11.5, 7.6), constrained_layout=True)
    for axis, (key, ylabel, limit) in zip(axes.flat, metrics):
        for method_index, method in enumerate(methods):
            values = [[row[key] for row in rows if row["difficulty"] == level and row["method"] == method] for level in levels]
            box = axis.boxplot(values, positions=positions + (method_index - 0.5)*width, widths=0.28, patch_artist=True, showfliers=False)
            for patch in box["boxes"]:
                patch.set(facecolor=colors[method], alpha=0.72)
        axis.axhline(limit, color="black", linestyle="--", linewidth=1.0, label="limit" if key != "min_safety_margin_m" else "collision boundary")
        axis.set(ylabel=ylabel, xticks=positions, xticklabels=levels)
        if key == "min_safety_margin_m":
            axis.axhline(0.0, color="black", linestyle="--", linewidth=1.0)
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    axes[0, 0].legend(
        handles=[
            Patch(facecolor=colors[methods[0]], alpha=0.72, label=labels[methods[0]]),
            Patch(facecolor=colors[methods[1]], alpha=0.72, label=labels[methods[1]]),
            Line2D([0], [0], color="black", linestyle="--", label="limit / boundary"),
        ],
        frameon=False, fontsize=9,
    )
    figure.savefig(outdir / "safety_dynamics.png", dpi=220)
    plt.close(figure)


def write_summary(rows, outdir, protocol):
    levels = [item.name for item in DIFFICULTIES]
    methods = ("TOGT", "Conditional Dual-Constraint CEM")
    summary = []
    for level in levels:
        for method in methods:
            subset = [row for row in rows if row["difficulty"] == level and row["method"] == method]
            feasible = [row for row in subset if row["joint_feasible"]]
            summary.append({
                "difficulty": level, "method": method, "instances": len(subset),
                "joint_feasible_count": len(feasible), "joint_feasible_rate": len(feasible)/len(subset),
                "dynamic_pass_count": sum(row["dynamic_passed"] for row in subset),
                "dynamic_pass_rate": float(np.mean([row["dynamic_passed"] for row in subset])),
                "safety_pass_count": sum(row["safety_passed"] for row in subset),
                "safety_pass_rate": float(np.mean([row["safety_passed"] for row in subset])),
                "collision_rate": float(np.mean([row["collision"] for row in subset])),
                "median_planning_seconds": float(np.median([row["planning_seconds"] for row in subset])),
                "median_min_safety_margin_m": float(np.median([row["min_safety_margin_m"] for row in subset])),
                "median_feasible_flight_time_s": None if not feasible else float(np.median([row["flight_time_s"] for row in feasible])),
            })
    (outdir / "summary.json").write_text(json.dumps(_jsonable(summary), indent=2) + "\n", encoding="utf-8")
    lines = ["# TOGT vs Conditional Dual-Constraint CEM benchmark", "", "## Frozen protocol", "", "```json", json.dumps(_jsonable(protocol), indent=2), "```", "", "## Aggregate results", "", "| Difficulty | Method | Joint feasible | Dynamics pass | Safety pass | Collision rate | Median plan time (s) | Median min margin (m) | Median feasible flight time (s) |", "|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for row in summary:
        flight = "—" if row["median_feasible_flight_time_s"] is None else f"{row['median_feasible_flight_time_s']:.3f}"
        lines.append(f"| {row['difficulty']} | {row['method']} | {row['joint_feasible_count']}/{row['instances']} | {row['dynamic_pass_count']}/{row['instances']} | {row['safety_pass_count']}/{row['instances']} | {row['collision_rate']:.3f} | {row['median_planning_seconds']:.3f} | {row['median_min_safety_margin_m']:.4f} | {flight} |")
    paired_count = sum(
        1 for course in {row["course_id"] for row in rows}
        if all(
            row["joint_feasible"] for row in rows if row["course_id"] == course
        )
    )
    lines += [
        "",
        f"There are {paired_count} courses jointly feasible for both methods; only those courses support a fair paired flight-time comparison. The flight-time plot therefore marks all returned trajectories, with circles denoting jointly feasible outputs and crosses denoting failures.",
        "All methods use the same known-window model, `[K,D]` parameterization, native TOGT dynamics, and maximum 1 ms independent audits. Safety/dynamics are dense numerical evidence, not continuous certificates. Failures are retained in the raw CSV and rate plots.",
        "",
    ]
    (outdir / "REPORT.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--repeats-per-difficulty", type=int, default=6)
    parser.add_argument("--seed", type=int, default=20260911)
    parser.add_argument("--population", type=int, default=32)
    parser.add_argument("--maximum-rounds", type=int, default=24)
    parser.add_argument("--repair-max-iterations", type=int, default=300)
    parser.add_argument("--workers", type=int, default=1)
    args = parser.parse_args(argv)
    if args.repeats_per_difficulty < 1 or args.workers < 1:
        raise ValueError("repeats-per-difficulty and workers must be positive")
    output = args.outdir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    (output / "figures").mkdir()
    courses = generate_courses(args.repeats_per_difficulty, args.seed)
    protocol = {
        "courses": len(courses), "difficulty_levels": [asdict(item) for item in DIFFICULTIES],
        "course_generator_seed": args.seed, "population": args.population,
        "maximum_rounds": args.maximum_rounds, "repair_max_iterations": args.repair_max_iterations,
        "workers": args.workers,
        "final_audit_maximum_step_seconds": 0.001,
        "methods": ["TOGT", "Conditional Dual-Constraint CEM"],
    }
    (output / "protocol.json").write_text(json.dumps(_jsonable(protocol), indent=2) + "\n", encoding="utf-8")
    (output / "courses.json").write_text(json.dumps(_jsonable([asdict(course) for course in courses]), indent=2) + "\n", encoding="utf-8")
    rows = []
    started = time.perf_counter()
    if args.workers == 1:
        for index, course in enumerate(courses, start=1):
            print(f"[{index}/{len(courses)}] {course.course_id}", flush=True)
            rows.extend(evaluate_course(
                course, args.population, args.maximum_rounds,
                args.repair_max_iterations, args.seed,
            ))
            (output / "partial_rows.json").write_text(
                json.dumps(_jsonable(rows), indent=2) + "\n", encoding="utf-8"
            )
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(
                    evaluate_course, course, args.population, args.maximum_rounds,
                    args.repair_max_iterations, args.seed,
                ): course for course in courses
            }
            for index, future in enumerate(as_completed(futures), start=1):
                course = futures[future]
                print(f"[{index}/{len(courses)}] completed {course.course_id}", flush=True)
                rows.extend(future.result())
                (output / "partial_rows.json").write_text(
                    json.dumps(_jsonable(rows), indent=2) + "\n", encoding="utf-8"
                )
    rows.sort(key=lambda row: (row["difficulty"], row["repeat"], row["method"]))
    fieldnames = sorted({key for row in rows for key in _flat_row(row)})
    with (output / "per_instance.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(_flat_row(row))
    (output / "raw_results.json").write_text(json.dumps(_jsonable(rows), indent=2) + "\n", encoding="utf-8")
    protocol["wall_seconds"] = time.perf_counter() - started
    (output / "protocol.json").write_text(json.dumps(_jsonable(protocol), indent=2) + "\n", encoding="utf-8")
    write_summary(rows, output, protocol)
    make_figures(rows, output / "figures")
    print(f"done: {output}", flush=True)


if __name__ == "__main__":
    main()
