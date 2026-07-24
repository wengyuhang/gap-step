"""A0--A3 MSR-DynaTOGT experiment suites and result export."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict
import json
import multiprocessing as mp
import os
from pathlib import Path
import time
from typing import Any

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    PreprocessingConfig,
    five_point_star_boundary,
    l_shape_boundary,
    u_shape_boundary,
)
from nonconvex_timevarying_window.sc_dynatogt.scenarios import (
    Scenario,
    build_boundary_scenario,
    build_canonical_scenario,
    build_diverse_scenario,
)

from .comparison import (
    atlas_compatibility_note,
    plot_aggregate_comparisons,
    plot_trajectory_comparison,
)
from .config import MSRConfig
from .results_manager import (
    RESULTS_ROOT,
    jsonable,
    summarize_runs,
    timestamped_run_directory,
    write_csv,
    write_figure_explanations,
    write_json,
    write_report,
)
from .solver import MSRSolution, solve


SCENE_NAMES = (
    "static_single",
    "translation_three",
    "full_three",
    "paper_irregular_six",
    "hard_thrust_full",
)

_WORKER_SCENES: dict[str, Scenario] = {}
_WORKER_ALGORITHM: MSRConfig | None = None


def _limit_worker_threads() -> None:
    """Keep process-level parallelism from multiplying BLAS threads."""

    original_affinity = (
        os.sched_getaffinity(0) if hasattr(os, "sched_getaffinity") else None
    )
    for variable in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
        os.environ[variable] = "1"
    try:
        from threadpoolctl import threadpool_limits

        threadpool_limits(1)
    except ImportError:
        pass
    try:
        import torch

        torch.set_num_threads(1)
    except ImportError:
        pass
    if original_affinity is not None:
        # Some OpenMP/PyTorch builds narrow affinity while configuring their
        # internal pool.  The experiment uses process parallelism, so retain
        # the CPU set inherited from the parent process.
        os.sched_setaffinity(0, original_affinity)


def _worker_solve(task: tuple[str, int]) -> tuple[str, int, MSRSolution, float]:
    """Solve one independent scene/seed task in a forked worker."""

    if _WORKER_ALGORITHM is None:
        raise RuntimeError("experiment worker has not been initialized")
    scene_name, run_seed = task
    started = time.perf_counter()
    solution = solve(
        _WORKER_SCENES[scene_name].track,
        config=_WORKER_ALGORITHM,
        seed=run_seed,
    )
    return scene_name, run_seed, solution, time.perf_counter() - started


def _preprocessing(suite: str) -> PreprocessingConfig:
    if suite == "smoke":
        return PreprocessingConfig(sc_fit_options={"quadrature_order": 32})
    return PreprocessingConfig()


def build_scenes(suite: str) -> dict[str, Scenario]:
    """Build the five prescribed SC scenarios with shared physical settings."""

    preprocessing = _preprocessing(suite)
    scenes = {
        "static_single": build_canonical_scenario(
            mode="static", preprocessing_config=preprocessing, gate_count=1
        ),
        "translation_three": build_canonical_scenario(
            mode="translation", preprocessing_config=preprocessing, gate_count=3
        ),
        "full_three": build_canonical_scenario(
            mode="full", preprocessing_config=preprocessing, gate_count=3
        ),
        "paper_irregular_six": build_diverse_scenario(
            mode="full",
            preprocessing_config=preprocessing,
            layout="paper_irregular",
            motion_scale=3.5,
        ),
    }
    definitions = (
        ("L", l_shape_boundary()),
        ("U", u_shape_boundary()),
        ("star", five_point_star_boundary()),
    )
    scenes["hard_thrust_full"] = build_boundary_scenario(
        definitions,
        mode="full",
        preprocessing_config=preprocessing,
        centers=np.array(
            [
                [-5.0, -4.5, 1.2],
                [0.0, 4.8, 5.0],
                [5.0, -4.2, 1.1],
            ]
        ),
        angles=np.array(
            [
                [0.0, np.pi / 2.0 - 0.12, -0.35],
                [0.28, np.pi / 2.0 + 0.20, 0.52],
                [-0.25, np.pi / 2.0 - 0.18, -0.60],
            ]
        ),
        motion_scale=4.5,
        start=np.array([-10.0, 1.5, 1.4]),
        goal=np.array([10.0, 1.0, 1.4]),
        name="hard_thrust_zigzag",
    )
    return scenes


def _scenario_metadata(scene: Scenario) -> dict[str, object]:
    return {
        "name": scene.name,
        "mode": scene.mode,
        "track_name": scene.track.name,
        "start": scene.track.start,
        "goal": scene.track.goal,
        "window_count": len(scene.track.windows),
        "order": scene.track.order,
        "windows": [
            {
                "name": window.name,
                "center0": window.center0,
                "angles0_rpy": window.angles0,
                "translation_amplitude": window.motion.translation_amplitude,
                "rotation_amplitude_rpy": window.motion.rotation_amplitude,
                "scale_amplitude": window.motion.scale_amplitude,
            }
            for window in scene.track.windows
        ],
    }


def _candidate_payload(solution: MSRSolution, *, scene: str) -> dict[str, object]:
    raw = []
    for candidate in solution.raw_candidates:
        raw.append(
            {
                "initialization": asdict(candidate.initialization),
                "optimization_seconds": candidate.optimization_seconds,
                "result": candidate.result.to_dict(),
                "feasibility": candidate.feasibility.to_dict(),
            }
        )
    repaired = []
    for candidate in solution.repaired_candidates:
        repair_payload = None
        if candidate.repair is not None:
            repair_payload = {
                key: value
                for key, value in candidate.repair.__dict__.items()
                if key not in {"result", "feasibility"}
            }
            repair_payload["feasibility"] = candidate.repair.feasibility.to_dict()
        repaired.append(
            {
                "initialization": asdict(candidate.initialization),
                "optimization_seconds": candidate.optimization_seconds,
                "repair_seconds": candidate.repair_seconds,
                "raw_result": candidate.raw_result.to_dict(),
                "raw_feasibility": candidate.raw_feasibility.to_dict(),
                "final_result": candidate.result.to_dict(),
                "final_feasibility": candidate.feasibility.to_dict(),
                "repair": repair_payload,
            }
        )
    return {
        "track_name": solution.track_name,
        "seed": solution.seed,
        "wall_clock_seconds": solution.wall_clock_seconds,
        "raw_duplicates_removed": solution.raw_duplicates_removed,
        "repaired_duplicates_removed": solution.repaired_duplicates_removed,
        "raw_candidates": raw,
        "repaired_candidates": repaired,
        "run_rows": solution.run_rows(scene=scene),
    }


def _task_path(run_root: Path, scene_name: str, run_seed: int) -> Path:
    return run_root / "candidates" / f"{scene_name}_seed_{run_seed}.json"


def _load_completed_rows(
    run_root: Path,
    tasks: list[tuple[str, int]],
) -> tuple[list[dict[str, object]], set[tuple[str, int]]]:
    rows: list[dict[str, object]] = []
    completed: set[tuple[str, int]] = set()
    for scene_name, run_seed in tasks:
        path = _task_path(run_root, scene_name, run_seed)
        if not path.is_file():
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        task_rows = payload.get("run_rows")
        if not isinstance(task_rows, list) or len(task_rows) != 12:
            continue
        rows.extend(task_rows)
        completed.add((scene_name, run_seed))
    return rows, completed


def _sort_rows(
    rows: list[dict[str, object]],
    selected_scenes: tuple[str, ...],
) -> None:
    scene_order = {name: index for index, name in enumerate(selected_scenes)}
    protocol_order = {"native": 0, "matched_starts": 1, "matched_time": 2}
    method_order = {"A0": 0, "A1": 1, "A2": 2, "A3": 3}
    rows.sort(
        key=lambda row: (
            scene_order[str(row["scene"])],
            int(row["seed"]),
            protocol_order[str(row["comparison_protocol"])],
            method_order[str(row["method"])],
        )
    )


def _default_workers(suite: str) -> int:
    if suite == "smoke":
        return 1
    try:
        available = len(os.sched_getaffinity(0))
    except AttributeError:
        available = os.cpu_count() or 1
    return max(1, min(12, available))


def run_suite(
    suite: str,
    *,
    seed: int = 0,
    seed_count: int | None = None,
    repair_mode: str = "local",
    selected_scenes: tuple[str, ...] = SCENE_NAMES,
    results_root: Path = RESULTS_ROOT,
    workers: int | None = None,
    resume_dir: Path | None = None,
) -> Path:
    if suite not in {"smoke", "formal"}:
        raise ValueError("suite must be 'smoke' or 'formal'")
    unknown = sorted(set(selected_scenes) - set(SCENE_NAMES))
    if unknown:
        raise ValueError(f"unknown scenes: {unknown}")
    repetitions = (1 if suite == "smoke" else 155) if seed_count is None else seed_count
    if repetitions < 1:
        raise ValueError("seed_count must be positive")
    worker_count = _default_workers(suite) if workers is None else workers
    if worker_count < 1:
        raise ValueError("workers must be positive")

    run_root = (
        timestamped_run_directory(results_root, suite)
        if resume_dir is None
        else Path(resume_dir).expanduser().resolve()
    )
    algorithm = MSRConfig.for_suite(suite, repair_mode=repair_mode)  # type: ignore[arg-type]
    scenes = build_scenes(suite)
    seeds = tuple(range(seed, seed + repetitions))
    atlas_note = atlas_compatibility_note()
    config_payload = {
        "algorithm": "MSR-DynaTOGT",
        "full_name": "Multi-Start and Repair DynaTOGT",
        "suite": suite,
        "seeds": seeds,
        "selected_scenes": selected_scenes,
        "workers": worker_count,
        "msr_config": algorithm,
        "fair_comparisons": ["matched_starts", "matched_time"],
        "atlas_auxiliary_comparison": atlas_note,
        "feasibility_scope": "high-density sampled feasibility only",
        "scenes": {
            name: _scenario_metadata(scenes[name]) for name in selected_scenes
        },
    }
    if resume_dir is None:
        write_json(run_root / "config.json", config_payload)
        write_json(run_root / "atlas_auxiliary_comparison.json", atlas_note)
    else:
        config_path = run_root / "config.json"
        if not config_path.is_file():
            raise FileNotFoundError(f"resume directory has no config.json: {run_root}")
        status_path = run_root / "status.json"
        if status_path.is_file():
            prior_status = json.loads(status_path.read_text(encoding="utf-8"))
            if prior_status.get("experiment_completed") is True:
                raise ValueError(f"refusing to resume an already completed run: {run_root}")
        prior = json.loads(config_path.read_text(encoding="utf-8"))
        expected = {
            "suite": suite,
            "seeds": list(seeds),
            "selected_scenes": list(selected_scenes),
        }
        actual = {key: prior.get(key) for key in expected}
        if actual != expected:
            raise ValueError(
                f"resume configuration mismatch: expected {expected}, found {actual}"
            )
        prior_mode = prior.get("msr_config", {}).get("repair", {}).get("mode")
        if prior_mode != repair_mode:
            raise ValueError(
                f"resume repair mode mismatch: expected {repair_mode}, found {prior_mode}"
            )
        previous_workers = prior.get("workers")
        prior["workers"] = worker_count
        prior["resumed"] = True
        prior["previous_workers"] = previous_workers
        write_json(config_path, prior)

    tasks = [(scene_name, run_seed) for scene_name in selected_scenes for run_seed in seeds]
    rows, completed = _load_completed_rows(run_root, tasks)
    pending = [task for task in tasks if task not in completed]
    started = time.perf_counter()
    write_json(
        run_root / "status.json",
        {
            "experiment_completed": False,
            "completed_tasks": len(completed),
            "total_tasks": len(tasks),
            "pending_tasks": len(pending),
            "workers": worker_count,
            "resumed": resume_dir is not None,
        },
    )

    def record(solution_item: tuple[str, int, MSRSolution, float]) -> None:
        scene_name, run_seed, solution, elapsed = solution_item
        task_rows = solution.run_rows(scene=scene_name)
        rows.extend(task_rows)
        if run_seed == seeds[0]:
            plot_trajectory_comparison(
                scenes[scene_name].track,
                solution.native,
                run_root / "figures" / f"trajectory_comparison_{scene_name}.png",
            )
        write_json(
            _task_path(run_root, scene_name, run_seed),
            _candidate_payload(solution, scene=scene_name),
        )
        completed.add((scene_name, run_seed))
        _sort_rows(rows, selected_scenes)
        write_csv(run_root / "runs.partial.csv", rows)
        write_json(
            run_root / "status.json",
            {
                "experiment_completed": False,
                "completed_tasks": len(completed),
                "total_tasks": len(tasks),
                "pending_tasks": len(tasks) - len(completed),
                "workers": worker_count,
                "last_completed": {"scene": scene_name, "seed": run_seed},
            },
        )
        print(
            json.dumps(
                {
                    "scene": scene_name,
                    "seed": run_seed,
                    "elapsed_seconds": elapsed,
                    "progress": f"{len(completed)}/{len(tasks)}",
                    "A3_sampled_feasible": solution.best.feasibility.sampled_feasible,
                    "A3_total_time": solution.best.result.total_time,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )

    global _WORKER_SCENES, _WORKER_ALGORITHM
    _WORKER_SCENES = scenes
    _WORKER_ALGORITHM = algorithm
    if worker_count == 1:
        _limit_worker_threads()
        for task in pending:
            record(_worker_solve(task))
    elif pending:
        context = mp.get_context("fork")
        with ProcessPoolExecutor(
            max_workers=min(worker_count, len(pending)),
            mp_context=context,
            initializer=_limit_worker_threads,
        ) as executor:
            futures = [executor.submit(_worker_solve, task) for task in pending]
            for future in as_completed(futures):
                record(future.result())

    expected_rows = 12 * len(tasks)
    if len(rows) != expected_rows:
        raise RuntimeError(f"expected {expected_rows} run rows, found {len(rows)}")
    missing_figures = [
        name
        for name in selected_scenes
        if not (run_root / "figures" / f"trajectory_comparison_{name}.png").is_file()
    ]
    if missing_figures:
        raise RuntimeError(
            "representative trajectory figures are missing for completed tasks: "
            + ", ".join(missing_figures)
        )

    _sort_rows(rows, selected_scenes)
    plot_aggregate_comparisons(rows, run_root / "figures")
    write_csv(run_root / "runs.csv", rows)
    aggregate = summarize_runs(rows)
    _, figure_explanations = write_figure_explanations(
        run_root / "FIGURE_EXPLANATIONS.md",
        rows,
        selected_scenes=selected_scenes,
        representative_seed=seeds[0],
    )
    summary = {
        "algorithm": "MSR-DynaTOGT",
        "suite": suite,
        "experiment_completed": True,
        "scene_count": len(selected_scenes),
        "seed_count": repetitions,
        "row_count": len(rows),
        "workers": worker_count,
        "completed_tasks": len(completed),
        "wall_clock_seconds": time.perf_counter() - started,
        "feasibility_scope": "高密度采样可行；不是连续时间严格证明",
        **aggregate,
        "artifacts": {
            "config": "config.json",
            "runs": "runs.csv",
            "report": "REPORT.md",
            "figure_explanations": "FIGURE_EXPLANATIONS.md",
            "trajectory_comparisons": [
                f"figures/trajectory_comparison_{name}.png" for name in selected_scenes
            ],
            "total_time_comparison": "figures/total_time_comparison.png",
            "computation_time_comparison": "figures/computation_time_comparison.png",
            "sampled_dynamic_feasibility_rate": "figures/sampled_dynamic_feasibility_rate.png",
            "repair_thrust_before_after": "figures/repair_thrust_before_after.png",
        },
    }
    write_json(run_root / "summary.json", summary)
    write_report(
        run_root / "REPORT.md",
        suite=suite,
        summary=summary,
        rows=rows,
        atlas_note=atlas_note,
        figure_explanations=figure_explanations,
    )
    write_json(
        run_root / "status.json",
        {
            "experiment_completed": True,
            "completed_tasks": len(completed),
            "total_tasks": len(tasks),
            "pending_tasks": 0,
            "workers": worker_count,
        },
    )
    return run_root


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run MSR-DynaTOGT A0--A3 experiments")
    parser.add_argument("--suite", choices=("smoke", "formal"), default="smoke")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seed-count", type=int)
    parser.add_argument("--repair-mode", choices=("uniform", "local"), default="local")
    parser.add_argument(
        "--workers",
        type=int,
        help="independent scene/seed worker processes (formal default: up to 12)",
    )
    parser.add_argument(
        "--resume",
        type=Path,
        help="resume an unfinished timestamp directory without replacing completed tasks",
    )
    parser.add_argument(
        "--scenes",
        default=",".join(SCENE_NAMES),
        help="comma-separated subset of the five prescribed scenes",
    )
    args = parser.parse_args(argv)
    selected = tuple(item.strip() for item in args.scenes.split(",") if item.strip())
    output = run_suite(
        args.suite,
        seed=args.seed,
        seed_count=args.seed_count,
        repair_mode=args.repair_mode,
        selected_scenes=selected,
        workers=args.workers,
        resume_dir=args.resume,
    )
    print(json.dumps({"outdir": str(output), "completed": True}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["SCENE_NAMES", "build_scenes", "main", "run_suite"]
