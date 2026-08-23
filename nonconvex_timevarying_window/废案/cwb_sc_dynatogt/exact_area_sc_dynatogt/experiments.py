"""Experiment B: real Old/Ours SC-DynaTOGT MINCO/L-BFGS solves."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.optimizer import JointTOGTObjective

from .solved_experiment import export_trajectory_csv, render_timeline, validate_forward
from .solver import SafetyObjectiveConfig, build_solver_problem, common_initial_guess, solve_method
from .stress_case import WORLD_CLEARANCE


EXPERIMENT_STATUS = {
    "A": "reserved: 155-seed E4 main statistics",
    "B": "implemented: real paired MINCO/L-BFGS solves and dense validation",
    "C": "reserved: fixed-margin safety/time sweep",
    "D": "reserved: dynamic-factor decomposition",
    "E": "reserved: continuous-time ablations",
    "F": "reserved: analytic-gradient verification",
}


def _method_payload(result, report) -> dict[str, object]:
    return {
        "method": result.method,
        "optimizer_success": result.optimizer_success,
        "optimizer_status": result.status,
        "optimizer_message": result.message,
        "iterations": result.iterations,
        "evaluations": result.evaluations,
        "solve_time_seconds": result.solve_time,
        "objective": result.objective,
        "base_objective": result.base_objective,
        "integrated_center_penalty_unweighted": result.center_penalty,
        "integrated_exact_area_penalty_unweighted": result.area_penalty,
        "total_time": result.total_time,
        "durations": result.forward.durations.tolist(),
        "traversal_times": result.forward.traversal_times.tolist(),
        "x0": result.x0.tolist(),
        "x": result.x.tolist(),
        "minimum_world_center_clearance": report.minimum_center_clearance,
        "required_world_center_clearance": WORLD_CLEARANCE,
        "maximum_outside_area": report.maximum_outside_area,
        "whole_body_collision": report.collision,
        "first_collision_time": report.first_collision_time,
        "contact_interval": [report.contact_start, report.contact_end],
        "dense_validation_samples": len(report.samples),
    }


def run_experiment_b(
    output_directory: str | Path,
    *,
    frames: int = 0,
    fps: int = 0,
) -> dict[str, object]:
    """Run both algorithms from one identical ``[K,D]`` initialization."""

    del frames, fps  # Backward-compatible flags; this static experiment makes no GIF.
    root = Path(output_directory)
    figures = root / "figures"
    root.mkdir(parents=True, exist_ok=True)
    figures.mkdir(parents=True, exist_ok=True)

    track, optimization, body = build_solver_problem()
    base = JointTOGTObjective(track, optimization)
    default_initial = common_initial_guess(base, seed=0)
    # This second point is a persisted output of the preliminary Old solve,
    # not a prescribed trajectory.  Both methods receive the identical pool.
    stress_initial = np.array([
        0.5853692027794527,
        0.3491365682753767,
        9.464752102973037,
        -30.6289386997479,
    ])
    initial_pool = (default_initial, stress_initial)
    safety = SafetyObjectiveConfig(
        samples=97,
        center_weight=1.0e10,
        exact_area_weight=1.0e18,
        finite_difference_step=1.0e-4,
        optimization_center_clearance=0.316,
        adaptive_center_samples=65,
    )
    candidates = {
        method: [
            solve_method(method, base, body, initial, safety_config=safety)
            for initial in initial_pool
        ]
        for method in ("Old-0.315", "Ours")
    }
    candidate_reports = {
        method: [
            validate_forward(
                result.forward, track, body, optimization, sample_count=15001
            )
            for result in method_candidates
        ]
        for method, method_candidates in candidates.items()
    }
    old_index = min(
        (
            index for index, result in enumerate(candidates["Old-0.315"])
            if result.optimizer_success
            and candidate_reports["Old-0.315"][index].minimum_center_clearance
            >= WORLD_CLEARANCE
        ),
        key=lambda index: candidates["Old-0.315"][index].base_objective,
    )
    ours_index = min(
        (
            index for index, result in enumerate(candidates["Ours"])
            if result.optimizer_success
            and candidate_reports["Ours"][index].minimum_center_clearance
            >= WORLD_CLEARANCE
            and not candidate_reports["Ours"][index].collision
        ),
        key=lambda index: candidates["Ours"][index].objective,
    )
    old = candidates["Old-0.315"][old_index]
    ours = candidates["Ours"][ours_index]
    results = {old.method: old, ours.method: ours}
    reports = {
        "Old-0.315": candidate_reports["Old-0.315"][old_index],
        "Ours": candidate_reports["Ours"][ours_index],
    }

    for method, result in results.items():
        if not result.optimizer_success:
            raise RuntimeError(f"{method} optimizer did not converge")
        if reports[method].minimum_center_clearance < WORLD_CLEARANCE:
            raise RuntimeError(f"{method} violates the 0.315 m world clearance")
    if not reports["Old-0.315"].collision:
        raise RuntimeError("the solved Old trajectory is not a counterexample")
    if reports["Ours"].collision:
        raise RuntimeError("the solved Ours trajectory is not whole-body safe")

    forwards = {method: result.forward for method, result in results.items()}
    trajectory_csv = root / "experiment_b.csv"
    timeline = figures / "full_planned_timeline.png"
    export_trajectory_csv(trajectory_csv, forwards, track, body, optimization, samples=1201)
    render_timeline(timeline, forwards, reports, track, body, optimization, dpi=145)
    np.savez_compressed(
        root / "optimized_solutions.npz",
        common_initial_pool=np.stack(initial_pool),
        old_x=old.x,
        ours_x=ours.x,
        old_durations=old.forward.durations,
        ours_durations=ours.forward.durations,
        old_coefficients=old.forward.trajectory.coefficients,
        ours_coefficients=ours.forward.trajectory.coefficients,
    )
    rows = [_method_payload(results[name], reports[name]) for name in ("Old-0.315", "Ours")]
    summary: dict[str, object] = {
        "method": "Exact-Area Whole-Body SC-DynaTOGT",
        "experiment": "B",
        "optimizer_claim": "both Old and Ours executed with MINCO + L-BFGS",
        "decision_variables": "[K,D]",
        "same_initial_pool": all(
            np.array_equal(candidates["Old-0.315"][index].x0,
                           candidates["Ours"][index].x0)
            for index in range(len(initial_pool))
        ),
        "initial_pool": [value.tolist() for value in initial_pool],
        "selected_initial_index": {"Old-0.315": old_index, "Ours": ours_index},
        "same_start_and_goal": bool(np.array_equal(track.start, track.goal)),
        "start": track.start.tolist(),
        "goal": track.goal.tolist(),
        "window": "five-point star, full E4 translation/rotation/scaling",
        "fixed_world_center_clearance": WORLD_CLEARANCE,
        "body_dimensions_m": (2.0 * body.half_extents).tolist(),
        "safety_gradient_backend": "centered finite difference for safety add-on; base TOGT gradient analytic",
        "collision_epsilon_area": 1.0e-6,
        "rows": rows,
        "all_candidate_runs": {
            method: [
                _method_payload(result, candidate_reports[method][index])
                for index, result in enumerate(method_candidates)
            ]
            for method, method_candidates in candidates.items()
        },
        "artifacts": {
            "trajectory_csv": str(trajectory_csv),
            "optimized_solutions": str(root / "optimized_solutions.npz"),
            "full_planned_timeline": str(timeline),
        },
        "experiment_status": EXPERIMENT_STATUS,
    }
    (root / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("B", "list"), default="B")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path("nonconvex_timevarying_window/exact_area_sc_dynatogt/results/experiment_b"),
    )
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--fps", type=int, default=0)
    args = parser.parse_args(argv)
    if args.suite == "list":
        print(json.dumps(EXPERIMENT_STATUS, ensure_ascii=False, indent=2))
        return 0
    summary = run_experiment_b(args.outdir, frames=args.frames, fps=args.fps)
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["EXPERIMENT_STATUS", "main", "run_experiment_b"]
