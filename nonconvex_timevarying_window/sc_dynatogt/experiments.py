"""Reproducible experiment groups E0--E5 from the final study plan."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass, is_dataclass, replace
import json
from pathlib import Path
import time
from typing import Any, Iterable

import numpy as np

from .baselines import (
    BaselineTOGTObjective,
    BaselineTrack,
    StaticBaselineWindow,
    convex_hull_polygon,
    optimize_baseline_track,
)
from .boundary import BoundaryPreprocessError, adaptive_chang_resample
from .dynamics import DynamicLimits, ObjectiveWeights, PenaltyWeights
from .environment import MotionProfile, SCDynamicWindow, SCWindowTrack
from .optimizer import JointTOGTObjective, OptimizationConfig, optimize_track
from .preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    e1_boundaries,
    preprocess_boundary,
)
from .sc_mapping import SCDiskMap
from .scenarios import build_canonical_scenario
from .validation import check_joint_objective_gradient, check_window_gradients, validate_sc_mapping
from .visualization import (
    export_dynamic_window_gif,
    export_trajectory_csv,
    plot_preprocessing,
    plot_trajectory,
)


@dataclass(frozen=True)
class ExperimentSettings:
    suite: str = "smoke"
    seed: int = 0
    replicates: int | None = None
    mapping_samples: int | None = None
    make_gif: bool = False

    def __post_init__(self) -> None:
        if self.suite not in {"smoke", "default"}:
            raise ValueError("suite must be 'smoke' or 'default'")
        if self.replicates is not None and self.replicates < 1:
            raise ValueError("replicates must be positive when provided")
        if self.mapping_samples is not None and self.mapping_samples < 1:
            raise ValueError("mapping_samples must be positive when provided")

    @property
    def is_smoke(self) -> bool:
        return self.suite == "smoke"

    def repetitions(self, group: str) -> int:
        if self.replicates is not None:
            return self.replicates
        if self.is_smoke:
            return 1
        return 30 if group == "E2" else 155

    @property
    def validation_samples(self) -> int:
        if self.mapping_samples is not None:
            return self.mapping_samples
        return 1_000 if self.is_smoke else 1_000_000


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


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _jsonable(row.get(key, "")) for key in keys})
    return path


def _optimization_config(settings: ExperimentSettings, *, full_time_gradient: bool = True) -> OptimizationConfig:
    return OptimizationConfig(
        initial_speed=1.0,
        # standard_lbfgs.yaml uses zero for unlimited iterations; convergence
        # is governed by its 32-iterate relative-cost test.
        max_iterations=0,
        samples_per_segment=8 if settings.is_smoke else None,
        include_window_time_gradient=full_time_gradient,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        # Matches standard_planning.yaml: velocity disabled, body-rate and
        # single-rotor thrust enabled.  Its near-unbounded tilt limit is inert.
        penalty_weights=PenaltyWeights(
            velocity=0.0, collective_thrust=0.0, body_rate=1.0, rotor_thrust=1.0
        ),
        dynamic_limits=DynamicLimits(
            max_velocity=60.0,
            max_body_rate_xy=10.0,
            max_body_rate_z=10.0,
            min_rotor_thrust=0.25,
            max_rotor_thrust=5.0,
        ),
    )


def _preprocessing_config(settings: ExperimentSettings) -> PreprocessingConfig:
    # The production vertex ladder is used in both suites.  Only quadrature
    # order is lowered in smoke mode; every geometry threshold remains fixed.
    options = {"quadrature_order": 32} if settings.is_smoke else {}
    return PreprocessingConfig(sc_fit_options=options)


def _designated_crossings_valid(track: SCWindowTrack, result) -> bool:
    """Check the prescribed crossing instants, order, and true safe sets."""

    if len(result.traversal_times) != len(track.order):
        return False
    if np.any(np.diff(result.traversal_times) <= 0.0):
        return False
    for crossing, window_index in enumerate(track.order):
        trajectory_point = result.trajectory.evaluate(result.traversal_times[crossing])
        if not np.allclose(trajectory_point, result.waypoints[crossing], atol=2e-7):
            return False
        if not track.windows[window_index].contains(result.waypoints[crossing], result.traversal_times[crossing]):
            return False
    return True


def _sampled_dynamic_limits_satisfied(result, config: OptimizationConfig) -> bool:
    """Report whether the diagnostic trajectory samples satisfy every limit.

    TOGT retains the reproduction's soft integrated penalties, so this is an
    output diagnostic rather than an extra hard constraint or pass criterion.
    """

    extrema = result.constraint_extrema
    limits = config.dynamic_limits
    tolerance = 1.0e-6

    def below(value: float, upper: float) -> bool:
        return not np.isfinite(upper) or value <= upper + tolerance * max(1.0, abs(upper))

    def above(value: float, lower: float) -> bool:
        return not np.isfinite(lower) or value >= lower - tolerance * max(1.0, abs(lower))

    return bool(
        below(float(extrema["max_velocity"]), limits.max_velocity)
        and below(float(extrema["max_body_rate_xy"]), limits.max_body_rate_xy)
        and below(float(extrema["max_abs_body_rate_z"]), limits.max_body_rate_z)
        and above(float(extrema["min_collective_thrust"]), limits.min_collective_thrust)
        and below(float(extrema["max_collective_thrust"]), limits.max_collective_thrust)
        and np.all(
            np.asarray(extrema["min_rotor_thrust"], dtype=float)
            >= limits.min_rotor_thrust
            - tolerance * max(1.0, abs(limits.min_rotor_thrust))
        )
        and np.all(
            np.asarray(extrema["max_rotor_thrust"], dtype=float)
            <= limits.max_rotor_thrust
            + tolerance * max(1.0, abs(limits.max_rotor_thrust))
        )
    )


def _constraint_extrema_columns(result) -> dict[str, Any]:
    """Flatten sampled extrema so CSV rows retain the reason for a failure."""

    extrema = result.constraint_extrema
    minimum_rotor = np.asarray(extrema["min_rotor_thrust"], dtype=float)
    maximum_rotor = np.asarray(extrema["max_rotor_thrust"], dtype=float)
    return {
        "sampled_max_velocity": extrema["max_velocity"],
        "sampled_max_body_rate_xy": extrema["max_body_rate_xy"],
        "sampled_max_abs_body_rate_z": extrema["max_abs_body_rate_z"],
        "sampled_min_collective_thrust": extrema["min_collective_thrust"],
        "sampled_max_collective_thrust": extrema["max_collective_thrust"],
        "sampled_min_rotor_thrust": float(np.min(minimum_rotor)),
        "sampled_max_rotor_thrust": float(np.max(maximum_rotor)),
    }


def _save_main_artifacts(
    output: Path,
    gates: Iterable[PreprocessedGate],
    track: SCWindowTrack,
    result,
    make_gif: bool,
) -> None:
    for index, gate in enumerate(gates):
        gate_output = output / "preprocessed_gates" / f"{index:02d}_{gate.name}"
        gate.save(gate_output)
        plot_preprocessing(gate, gate_output / "preprocessing.png", samples_per_line=40)
    plot_trajectory(track, result, output / "trajectory.png", num_samples=241)
    export_trajectory_csv(result, output / "trajectory.csv", num_samples=301)
    if make_gif:
        export_dynamic_window_gif(track, result, output / "dynamic_windows.gif", num_frames=36)


def run_e0(output: Path, settings: ExperimentSettings) -> dict[str, Any]:
    """Original convex TOGT map versus SC on the identical static rectangle."""

    polygon = np.array([[-1.2, -0.9], [1.2, -0.9], [1.2, 0.9], [-1.2, 0.9]])
    mapping = SCDiskMap.fit(polygon, quadrature_order=32 if settings.is_smoke else 64)
    center = np.array([0.0, 0.0, 1.4])
    rpy = np.array([0.0, np.pi / 2.0, 0.0])
    start, goal = np.array([-3.0, 0.0, 1.4]), np.array([3.0, 0.0, 1.4])
    sc_window = SCDynamicWindow("rectangle_sc", mapping, polygon, center, rpy, MotionProfile.static())
    sc_track = SCWindowTrack("E0_SC", start, goal, (sc_window,), (0,))
    baseline_window = StaticBaselineWindow("rectangle_original", polygon, center, rpy, kind="original_convex")
    baseline_track = BaselineTrack("E0_original", start, goal, (baseline_window,))
    config = _optimization_config(settings)
    original = optimize_baseline_track(baseline_track, config=config)
    sc = optimize_track(sc_track, config=config)
    relative_time_error = abs(sc.total_time - original.total_time) / original.total_time
    payload = {
        "experiment": "E0",
        "original": original.to_dict(),
        "sc": sc.to_dict(),
        "sampled_dynamic_limits_satisfied": {
            "original": _sampled_dynamic_limits_satisfied(original, config),
            "sc": _sampled_dynamic_limits_satisfied(sc, config),
        },
        "relative_total_time_error": relative_time_error,
        "threshold": 0.01,
        "passed": bool(
            original.success and sc.success and relative_time_error <= 0.01
            and _designated_crossings_valid(sc_track, sc)
        ),
    }
    _write_json(output / "summary.json", payload)
    plot_trajectory(sc_track, sc, output / "sc_trajectory.png", num_samples=241)
    export_trajectory_csv(sc, output / "sc_trajectory.csv", num_samples=301)
    return payload


def run_e1(output: Path, settings: ExperimentSettings) -> dict[str, Any]:
    """Boundary-only Chang resampling study over the prescribed six families."""

    catalog = e1_boundaries()
    counts = (256,) if settings.is_smoke else (256, 512, 1024, 2048, 3200)
    rows: list[dict[str, Any]] = []
    for name, boundary in catalog.items():
        for count in counts:
            started = time.perf_counter()
            try:
                sample = adaptive_chang_resample(boundary, vertex_counts=(count,))
                report = sample.report
                assert report is not None
            except BoundaryPreprocessError as error:
                report = error.reports[-1]
            elapsed = time.perf_counter() - started
            rows.append(
                {
                    "boundary": name,
                    "target_m": count,
                    "output_m": report.target_count,
                    "max_boundary_error_m": report.max_boundary_error,
                    "max_concavity_error_m": report.max_concavity_error,
                    "is_simple": report.is_simple,
                    "is_ccw": report.is_ccw,
                    "corners_preserved": report.corners_preserved,
                    "accepted": report.accepted,
                    "elapsed_seconds": elapsed,
                    "failure_reasons": " | ".join(report.failure_reasons),
                }
            )
    payload = {
        "experiment": "E1",
        "rows": rows,
        "all_boundaries_have_an_accepted_count": all(
            any(row["boundary"] == name and row["accepted"] for row in rows) for name in catalog
        ),
    }
    payload["passed"] = payload["all_boundaries_have_an_accepted_count"]
    _write_rows(output / "boundary_sampling.csv", rows)
    _write_json(output / "summary.json", payload)
    return payload


def _perturbed_guess(objective, rng: np.random.Generator) -> np.ndarray:
    """Apply the same seeded temporal/spatial perturbation to every method."""

    x = objective.initial_guess()
    x[: objective.temporal_dimension] += rng.normal(0.0, 0.04, objective.temporal_dimension)
    spatial_dimension = len(x) - objective.temporal_dimension
    x[objective.temporal_dimension :] += rng.normal(0.0, 0.12, spatial_dimension)
    return x


def run_e2(output: Path, settings: ExperimentSettings) -> dict[str, Any]:
    """Static non-convex comparison: fixed center, convex hull, and SC."""

    scenario = build_canonical_scenario(
        mode="static", preprocessing_config=_preprocessing_config(settings), gate_count=1
    )
    window = scenario.track.windows[0]
    center, rpy = window.center0, window.angles0
    hull = convex_hull_polygon(window.safe_polygon)
    fixed_track = BaselineTrack(
        "E2_fixed", scenario.track.start, scenario.track.goal,
        (StaticBaselineWindow("fixed", window.safe_polygon, center, rpy, kind="fixed"),),
    )
    hull_track = BaselineTrack(
        "E2_hull", scenario.track.start, scenario.track.goal,
        (StaticBaselineWindow("hull", hull, center, rpy, kind="original_convex"),),
    )
    config = _optimization_config(settings)
    fixed_objective = BaselineTOGTObjective(fixed_track, config)
    hull_objective = BaselineTOGTObjective(hull_track, config)
    sc_objective = JointTOGTObjective(scenario.track, config)
    rows: list[dict[str, Any]] = []
    first_sc = None
    for seed_index in range(settings.repetitions("E2")):
        run_seed = settings.seed + seed_index
        fixed = optimize_baseline_track(
            fixed_track,
            config=config,
            initial_x=_perturbed_guess(fixed_objective, np.random.default_rng(run_seed)),
        )
        hull_result = optimize_baseline_track(
            hull_track,
            config=config,
            initial_x=_perturbed_guess(hull_objective, np.random.default_rng(run_seed)),
        )
        sc = optimize_track(
            scenario.track,
            config=config,
            initial_x=_perturbed_guess(sc_objective, np.random.default_rng(run_seed)),
        )
        if first_sc is None:
            first_sc = sc
        for method, result, legal in (
            ("fixed_center", fixed, window.contains(fixed.waypoints[0], fixed.traversal_times[0])),
            ("convex_hull", hull_result, window.contains(hull_result.waypoints[0], hull_result.traversal_times[0])),
            ("sc", sc, _designated_crossings_valid(scenario.track, sc)),
        ):
            rows.append(
                {"seed": run_seed, "method": method, "success": result.success,
                 "legal_in_true_safe_polygon": legal, "total_time": result.total_time,
                 "sampled_dynamic_limits_satisfied": _sampled_dynamic_limits_satisfied(result, config),
                 "objective": result.objective, "iterations": result.iterations,
                 **_constraint_extrema_columns(result)}
            )
    assert first_sc is not None
    sc_rows = [row for row in rows if row["method"] == "sc"]
    sc_legal_rate = float(np.mean([row["legal_in_true_safe_polygon"] for row in sc_rows]))
    sc_convergence_rate = float(np.mean([row["success"] for row in sc_rows]))
    mapping_validation = validate_sc_mapping(
        window.sc_map,
        sample_count=settings.validation_samples,
        seed=settings.seed,
        batch_size=1024,
    )
    payload = {
        "experiment": "E2", "rows": rows,
        "sc_legal_rate": sc_legal_rate,
        "sc_convergence_rate": sc_convergence_rate,
        "required_sc_convergence_rate": 0.95,
        "mapping_validation": mapping_validation,
        "representative_sc_result": first_sc.to_dict(),
        "passed": bool(
            sc_legal_rate == 1.0 and sc_convergence_rate >= 0.95
            and mapping_validation.passed
        ),
    }
    _write_rows(output / "comparison.csv", rows)
    _write_json(output / "summary.json", payload)
    _save_main_artifacts(output, scenario.preprocessed_gates, scenario.track, first_sc, False)
    return payload


def _run_dynamic_group(group: str, mode: str, output: Path, settings: ExperimentSettings) -> dict[str, Any]:
    scenario = build_canonical_scenario(
        mode=mode, preprocessing_config=_preprocessing_config(settings),
        gate_count=1 if settings.is_smoke else 3,
    )
    config = _optimization_config(settings)
    objective = JointTOGTObjective(scenario.track, config)
    rows: list[dict[str, Any]] = []
    first = None
    for run in range(settings.repetitions(group)):
        run_seed = settings.seed + run
        result = optimize_track(
            scenario.track,
            config=config,
            initial_x=_perturbed_guess(objective, np.random.default_rng(run_seed)),
        )
        if first is None:
            first = result
        rows.append(
            {"run": run, "seed": run_seed, "success": result.success,
             "designated_order_legal": _designated_crossings_valid(scenario.track, result),
             "sampled_dynamic_limits_satisfied": _sampled_dynamic_limits_satisfied(result, config),
             "total_time": result.total_time, "objective": result.objective,
             "iterations": result.iterations, "gradient_inf_norm": result.gradient_inf_norm,
             **_constraint_extrema_columns(result)}
        )
    assert first is not None
    # The time-only reproduction objective can have zero waypoint adjoints at
    # its slow center initialization.  Add a small snap term solely in this
    # numerical check so D and moving-window time chains are genuinely active.
    gradient_config = replace(
        config, objective_weights=ObjectiveWeights(time=1.0, snap_energy=1.0e-4)
    )
    gradient_objective = JointTOGTObjective(scenario.track, gradient_config)
    gradient_reports = [
        check_window_gradients(window, sample_count=12 if settings.is_smoke else 100, seed=settings.seed + index)
        for index, window in enumerate(scenario.track.windows)
    ]
    mapping_reports = [
        validate_sc_mapping(
            window.sc_map,
            sample_count=settings.validation_samples,
            seed=settings.seed + index,
            batch_size=1024,
        )
        for index, window in enumerate(scenario.track.windows)
    ]
    convergence_rate = float(np.mean([row["success"] for row in rows]))
    designated_order_legal_rate = float(np.mean([row["designated_order_legal"] for row in rows]))
    sampled_limit_rate = float(np.mean([row["sampled_dynamic_limits_satisfied"] for row in rows]))
    # Check the chain at the documented finite initialization d=0.  A solved
    # boundary optimum can have |d| in the hundreds, where B(d) is within
    # floating-point resolution of the unit circle and h=1e-6 cannot produce
    # a meaningful centered difference in mapped space.
    joint_report = check_joint_objective_gradient(
        gradient_objective, gradient_objective.initial_guess()
    )
    payload = {
        "experiment": group, "motion_mode": mode, "rows": rows,
        "convergence_rate": convergence_rate,
        "designated_order_legal_rate": designated_order_legal_rate,
        "sampled_dynamic_limits_satisfied_rate": sampled_limit_rate,
        "required_convergence_rate": 0.90,
        "gradient_reports": gradient_reports,
        "joint_objective_gradient_report": joint_report,
        "mapping_validation": mapping_reports,
        "representative_result": first.to_dict(),
        "passed": bool(
            convergence_rate >= 0.90 and designated_order_legal_rate == 1.0
            and all(report.passed for report in gradient_reports)
            and joint_report.passed
            and all(report.passed for report in mapping_reports)
        ),
    }
    _write_rows(output / "runs.csv", rows)
    _write_json(output / "summary.json", payload)
    _save_main_artifacts(output, scenario.preprocessed_gates, scenario.track, first, settings.make_gif)
    return payload


def run_e3(output: Path, settings: ExperimentSettings) -> dict[str, Any]:
    return _run_dynamic_group("E3", "translation", output, settings)


def run_e4(output: Path, settings: ExperimentSettings) -> dict[str, Any]:
    return _run_dynamic_group("E4", "full", output, settings)


def run_e5(output: Path, settings: ExperimentSettings) -> dict[str, Any]:
    scenario = build_canonical_scenario(
        mode="full", preprocessing_config=_preprocessing_config(settings),
        gate_count=1 if settings.is_smoke else 3,
    )
    full_config = _optimization_config(settings, full_time_gradient=True)
    ablated_config = _optimization_config(settings, full_time_gradient=False)
    full_objective = JointTOGTObjective(scenario.track, full_config)
    rows: list[dict[str, Any]] = []
    first_full = None
    for run in range(settings.repetitions("E5")):
        run_seed = settings.seed + run
        initial = _perturbed_guess(full_objective, np.random.default_rng(run_seed))
        for label, config in (("full_time_gradient", full_config), ("zero_window_time_gradient", ablated_config)):
            result = optimize_track(scenario.track, config=config, initial_x=initial)
            if first_full is None and label == "full_time_gradient":
                first_full = result
            rows.append(
                {"run": run, "seed": run_seed, "method": label, "success": result.success,
                 "designated_order_legal": _designated_crossings_valid(scenario.track, result),
                 "sampled_dynamic_limits_satisfied": _sampled_dynamic_limits_satisfied(result, config),
                 "total_time": result.total_time, "objective": result.objective,
                 "iterations": result.iterations, "evaluations": result.evaluations,
                 **_constraint_extrema_columns(result)}
            )
    method_rates = {
        method: {
            "convergence_rate": float(np.mean([row["success"] for row in rows if row["method"] == method])),
            "designated_order_legal_rate": float(np.mean([row["designated_order_legal"] for row in rows if row["method"] == method])),
            "sampled_dynamic_limits_satisfied_rate": float(np.mean([row["sampled_dynamic_limits_satisfied"] for row in rows if row["method"] == method])),
            "mean_total_time": float(np.mean([row["total_time"] for row in rows if row["method"] == method])),
            "mean_objective": float(np.mean([row["objective"] for row in rows if row["method"] == method])),
            "mean_iterations": float(np.mean([row["iterations"] for row in rows if row["method"] == method])),
            "mean_evaluations": float(np.mean([row["evaluations"] for row in rows if row["method"] == method])),
        }
        for method in ("full_time_gradient", "zero_window_time_gradient")
    }
    payload = {
        "experiment": "E5", "rows": rows, "method_rates": method_rates,
        "representative_full_result": first_full.to_dict() if first_full is not None else None,
        "passed": bool(
            method_rates["full_time_gradient"]["convergence_rate"] >= 0.90
            and all(value["designated_order_legal_rate"] == 1.0 for value in method_rates.values())
        ),
    }
    _write_rows(output / "ablation.csv", rows)
    _write_json(output / "summary.json", payload)
    assert first_full is not None
    _save_main_artifacts(
        output, scenario.preprocessed_gates, scenario.track, first_full, settings.make_gif
    )
    return payload


RUNNERS = {"E0": run_e0, "E1": run_e1, "E2": run_e2, "E3": run_e3, "E4": run_e4, "E5": run_e5}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run SC-DynaTOGT experiments E0--E5")
    parser.add_argument("--suite", choices=("smoke", "default"), default="smoke")
    parser.add_argument("--experiment", choices=("all", *RUNNERS), default="all")
    parser.add_argument("--outdir", type=Path, default=Path("nonconvex_timevarying_window/sc_dynatogt/results"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--replicates", type=int)
    parser.add_argument("--mapping-samples", type=int)
    parser.add_argument("--gif", action="store_true")
    args = parser.parse_args(argv)
    if args.replicates is not None and args.replicates < 1:
        parser.error("--replicates must be positive")
    if args.mapping_samples is not None and args.mapping_samples < 1:
        parser.error("--mapping-samples must be positive")
    settings = ExperimentSettings(
        args.suite, args.seed, args.replicates, args.mapping_samples, args.gif
    )
    selected: Iterable[str] = RUNNERS if args.experiment == "all" else (args.experiment,)
    summaries = {}
    for name in selected:
        summaries[name] = RUNNERS[name](args.outdir / name, settings)
    _write_json(args.outdir / "summary.json", {"settings": settings, "experiments": summaries})
    print(json.dumps(_jsonable({"outdir": args.outdir, "experiments": tuple(selected)}), ensure_ascii=False))
    return 0 if all(bool(summary.get("passed", False)) for summary in summaries.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "ExperimentSettings", "main", "run_e0", "run_e1", "run_e2", "run_e3", "run_e4", "run_e5",
]
