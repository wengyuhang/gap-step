"""Single-run, E1--E6 benchmark, ablation, plotting, and video CLIs."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Iterable

from .baselines import METHODS, NON_ORACLE_METHODS
from .config import MDGConfig, load_config
from .metrics import write_summary
from .planner import MDGPlanner
from .results import result_complete, save_result
from .scenario_generator import SHAPE_ORDER, generate_scenario
from .disk_tracking import build_scenario_tracks


def _result_path(root: Path, task: dict[str, Any]) -> Path:
    label = task.get("label", task["method"])
    return (
        root
        / "raw"
        / task["experiment"]
        / label
        / task["instance"]
    )


def _variant_config(base: MDGConfig, variant: str | None) -> MDGConfig:
    if not variant or variant == "default":
        return base
    if variant.startswith("k"):
        return replace(
            base,
            disks=replace(base.disks, max_disks_per_gate=int(variant[1:])),
        )
    if variant.startswith("dt"):
        value = float(variant[2:].replace("p", "."))
        return replace(base, graph=replace(base.graph, dt_coarse=value))
    if variant == "no_fine":
        return replace(base, graph=replace(base.graph, enable_refine=False))
    if variant == "no_lazy":
        return replace(base, backend=replace(base.backend, max_lazy_repairs=0))
    if variant == "no_shrink":
        return replace(
            base,
            tracking=replace(base.tracking, enable_validation_shrink=False),
        )
    raise ValueError(f"unknown ablation variant {variant!r}")


def _execute_task(
    task: dict[str, Any],
    config_dict: dict[str, Any],
    output_root: str,
    save_video: bool,
) -> str:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    from .config import config_from_mapping

    config = config_from_mapping(config_dict)
    try:
        import torch

        torch.set_num_threads(1)
    except ImportError:
        pass
    variant = task.get("variant")
    config = _variant_config(config, variant)
    scenario = generate_scenario(
        config,
        seed=task["seed"],
        gate_count=task["gate_count"],
        difficulty=task["difficulty"],
        closed_ratio=task["closed_ratio"],
        shape=task.get("shape"),
    )
    target = _result_path(Path(output_root), task)
    if result_complete(target) and not config.runtime.overwrite:
        return f"skip {target}"
    result = MDGPlanner(config).plan(scenario, method=task["method"])
    if task.get("label"):
        result.method = task["label"]
    save_result(
        scenario,
        result,
        config,
        target,
        experiment=task["experiment"],
        save_video=save_video,
    )
    return f"done {target} success={result.success}"


def _track_cache_key(config: MDGConfig, method: str) -> str:
    mode = "mdg_free" if method in {"mdg_free", "mdg_center"} else method
    payload = json.dumps(
        {
            "mode": mode,
            "disks": config.to_dict()["disks"],
            "tracking": config.to_dict()["tracking"],
            "safety": config.to_dict()["safety"],
        },
        sort_keys=True,
    )
    return mode + "_" + hashlib.sha256(payload.encode()).hexdigest()[:12]


def _execute_group(
    tasks: list[dict[str, Any]],
    config_dict: dict[str, Any],
    output_root: str,
    save_video: bool,
) -> list[str]:
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    from .config import config_from_mapping

    base = config_from_mapping(config_dict)
    first = tasks[0]
    scenario = generate_scenario(
        base,
        seed=first["seed"],
        gate_count=first["gate_count"],
        difficulty=first["difficulty"],
        closed_ratio=first["closed_ratio"],
        shape=first.get("shape"),
    )
    track_cache: dict[str, Any] = {}
    messages: list[str] = []
    for task in tasks:
        config = _variant_config(base, task.get("variant"))
        target = _result_path(Path(output_root), task)
        if result_complete(target) and not config.runtime.overwrite:
            messages.append(f"skip {target}")
            continue
        cache_key = _track_cache_key(config, task["method"])
        if cache_key not in track_cache:
            track_mode = (
                "mdg_free"
                if task["method"] in {"mdg_free", "mdg_center"}
                else task["method"]
            )
            track_cache[cache_key] = build_scenario_tracks(
                scenario, config, method=track_mode
            )
        result = MDGPlanner(config).plan(
            scenario,
            method=task["method"],
            disc_tracks=track_cache[cache_key],
        )
        if task.get("label"):
            result.method = task["label"]
        save_result(
            scenario,
            result,
            config,
            target,
            experiment=task["experiment"],
            save_video=save_video,
        )
        messages.append(f"done {target} success={result.success}")
    return messages


def run_single(
    config: MDGConfig,
    *,
    seed: int,
    method: str,
    output: str | Path,
    gate_count: int = 8,
    difficulty: str = "medium",
    closed_ratio: float = 0.20,
    save_video: bool = True,
):
    scenario = generate_scenario(
        config,
        seed=seed,
        gate_count=gate_count,
        difficulty=difficulty,
        closed_ratio=closed_ratio,
    )
    result = MDGPlanner(config).plan(scenario, method=method)
    save_result(
        scenario,
        result,
        config,
        output,
        experiment="single",
        save_video=save_video,
    )
    return result


def formal_tasks(experiment: str = "all") -> list[dict[str, Any]]:
    tasks: list[dict[str, Any]] = []
    requested = {"E1", "E2", "E3", "E4", "E5"} if experiment == "all" else {experiment.upper()}
    if "E1" in requested:
        for shape in SHAPE_ORDER:
            for difficulty in ("low", "medium", "high"):
                for seed in range(10):
                    instance = f"{shape}_{difficulty}_seed_{seed:02d}"
                    for method in NON_ORACLE_METHODS:
                        tasks.append(
                            dict(
                                experiment="E1",
                                instance=instance,
                                seed=seed,
                                gate_count=8,
                                difficulty=difficulty,
                                closed_ratio=0.20,
                                shape=shape,
                                method=method,
                            )
                        )
    if "E2" in requested:
        for ratio in (0.0, 0.2, 0.4, 0.6):
            for seed in range(20):
                instance = f"close_{int(100*ratio):02d}_seed_{seed:02d}"
                for method in NON_ORACLE_METHODS:
                    tasks.append(
                        dict(
                            experiment="E2",
                            instance=instance,
                            seed=seed,
                            gate_count=8,
                            difficulty="medium",
                            closed_ratio=ratio,
                            method=method,
                        )
                    )
    if "E3" in requested:
        for gate_count in (5, 10, 20, 30):
            for seed in range(20):
                instance = f"gates_{gate_count:02d}_seed_{seed:02d}"
                for method in NON_ORACLE_METHODS:
                    tasks.append(
                        dict(
                            experiment="E3",
                            instance=instance,
                            seed=seed,
                            gate_count=gate_count,
                            difficulty="medium",
                            closed_ratio=0.20,
                            method=method,
                        )
                    )
    if "E4" in requested:
        for difficulty in ("low", "medium"):
            for seed in range(20):
                instance = f"{difficulty}_seed_{seed:02d}"
                for method in METHODS:
                    tasks.append(
                        dict(
                            experiment="E4",
                            instance=instance,
                            seed=seed,
                            gate_count=4,
                            difficulty=difficulty,
                            closed_ratio=0.20,
                            method=method,
                        )
                    )
    if "E5" in requested:
        variants = (
            ("default", "mdg_free", "mdg_free_default"),
            ("k1", "mdg_free", "mdg_free_k1"),
            ("k3", "mdg_free", "mdg_free_k3"),
            ("k8", "mdg_free", "mdg_free_k8"),
            ("dt0p20", "mdg_free", "mdg_free_dt0p20"),
            ("dt0p05", "mdg_free", "mdg_free_dt0p05"),
            ("no_fine", "mdg_free", "mdg_free_no_fine"),
            ("default", "mdg_center", "mdg_center"),
            ("no_lazy", "mdg_free", "mdg_free_no_lazy"),
            ("no_shrink", "mdg_free", "mdg_free_no_shrink"),
        )
        for seed in range(30):
            instance = f"seed_{seed:02d}"
            for variant, method, label in variants:
                tasks.append(
                    dict(
                        experiment="E5",
                        instance=instance,
                        seed=seed,
                        gate_count=10,
                        difficulty="medium",
                        closed_ratio=0.20,
                        method=method,
                        label=label,
                        variant=variant,
                    )
                )
    return tasks


def smoke_tasks(experiment: str = "all") -> list[dict[str, Any]]:
    tasks = formal_tasks(experiment)
    selected: dict[tuple[str, str], dict[str, Any]] = {}
    for task in tasks:
        key = (task["experiment"], task.get("label", task["method"]))
        if key not in selected:
            item = dict(task)
            item["gate_count"] = min(4, item["gate_count"])
            item["closed_ratio"] = min(0.20, item["closed_ratio"])
            selected[key] = item
    return list(selected.values())


def run_tasks(
    tasks: list[dict[str, Any]],
    config: MDGConfig,
    output_root: str | Path,
    *,
    workers: int | None = None,
    save_video: bool | None = None,
) -> list[str]:
    root = Path(output_root)
    root.mkdir(parents=True, exist_ok=True)
    worker_count = config.runtime.workers if workers is None else workers
    render = config.runtime.save_video if save_video is None else save_video
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for task in tasks:
        grouped.setdefault((task["experiment"], task["instance"]), []).append(task)
    groups = list(grouped.values())
    messages: list[str] = []
    if worker_count <= 1:
        for group in groups:
            for message in _execute_group(
                group, config.to_dict(), str(root), render
            ):
                print(message, flush=True)
                messages.append(message)
    else:
        with ProcessPoolExecutor(max_workers=worker_count) as executor:
            futures = {
                executor.submit(
                    _execute_group, group, config.to_dict(), str(root), render
                ): group
                for group in groups
            }
            for future in as_completed(futures):
                for message in future.result():
                    print(message, flush=True)
                    messages.append(message)
    write_summary(root)
    return messages


def _common_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--config", default="configs/default.yaml")
    return parser


def single_main(argv: Iterable[str] | None = None) -> int:
    parser = _common_parser("Run one MDG instance")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--method", choices=METHODS, default="mdg_free")
    parser.add_argument("--gate-count", type=int, default=8)
    parser.add_argument("--difficulty", choices=("low", "medium", "high"), default="medium")
    parser.add_argument("--closed-ratio", type=float, default=0.20)
    parser.add_argument("--outdir", default="results/raw/single/mdg_free/seed_0")
    parser.add_argument("--save-video", action="store_true")
    args = parser.parse_args(argv)
    result = run_single(
        load_config(args.config),
        seed=args.seed,
        method=args.method,
        output=args.outdir,
        gate_count=args.gate_count,
        difficulty=args.difficulty,
        closed_ratio=args.closed_ratio,
        save_video=args.save_video,
    )
    print(json.dumps(result.metrics_dict(), ensure_ascii=False, indent=2))
    return 0 if result.success else 1


def benchmark_main(argv: Iterable[str] | None = None) -> int:
    parser = _common_parser("Run MDG E1--E5 and derive E6")
    parser.add_argument("--suite", choices=("smoke", "formal"), default="smoke")
    parser.add_argument("--experiment", default="all")
    parser.add_argument("--outdir", default="results")
    parser.add_argument("--workers", type=int)
    parser.add_argument("--save-video", action=argparse.BooleanOptionalAction, default=None)
    args = parser.parse_args(argv)
    tasks = (
        smoke_tasks(args.experiment)
        if args.suite == "smoke"
        else formal_tasks(args.experiment)
    )
    run_tasks(
        tasks,
        load_config(args.config),
        args.outdir,
        workers=args.workers,
        save_video=args.save_video,
    )
    return 0


def ablation_main(argv: Iterable[str] | None = None) -> int:
    if argv is None:
        import sys

        arguments = sys.argv[1:]
    else:
        arguments = list(argv)
    return benchmark_main(["--experiment", "E5", *arguments])


def plot_main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Aggregate MDG metrics")
    parser.add_argument("--results", default="results")
    args = parser.parse_args(argv)
    paths = write_summary(args.results)
    print("\n".join(str(path) for path in paths))
    return 0


def video_main(argv: Iterable[str] | None = None) -> int:
    parser = _common_parser("Solve one MDG instance and render MP4")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--method", choices=METHODS, default="mdg_free")
    parser.add_argument("--outdir", default="results/videos/demo_seed_0")
    args = parser.parse_args(argv)
    result = run_single(
        load_config(args.config),
        seed=args.seed,
        method=args.method,
        output=args.outdir,
        save_video=True,
    )
    return 0 if result.success else 1


__all__ = [
    "ablation_main",
    "benchmark_main",
    "formal_tasks",
    "plot_main",
    "run_single",
    "run_tasks",
    "single_main",
    "smoke_tasks",
    "video_main",
]
