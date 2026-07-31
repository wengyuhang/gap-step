"""Atomic, resumable persistence for MDG runs."""

from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import shutil
import tempfile
import time
from typing import Any

import numpy as np

from .config import MDGConfig, dump_config
from .dynamic_gate import Scenario
from .models import PlanResult
from .visualization import (
    make_video,
    plot_gate_diagnostics,
    plot_overview,
    plot_profiles,
)


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def write_json(path: Path, value: Any) -> Path:
    path.write_text(
        json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path


def result_complete(path: str | Path) -> bool:
    directory = Path(path)
    metric = directory / "metrics.json"
    if not metric.exists():
        return False
    try:
        return bool(json.loads(metric.read_text(encoding="utf-8")).get("run_complete"))
    except (json.JSONDecodeError, OSError):
        return False


def save_result(
    scenario: Scenario,
    result: PlanResult,
    config: MDGConfig,
    output: str | Path,
    *,
    experiment: str = "single",
    save_video: bool | None = None,
) -> Path:
    target = Path(output)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{target.name}.", dir=str(target.parent))
    )
    started = time.perf_counter()
    try:
        dump_config(config, temporary / "config_resolved.yaml")
        scenario.save(temporary / "scenario.json")
        write_json(
            temporary / "disc_tracks.json",
            {
                str(key): [item.to_dict() for item in values]
                for key, values in result.disc_tracks.items()
            },
        )
        if result.graph_coarse is not None:
            write_json(temporary / "graph_coarse.json", result.graph_coarse.to_dict())
        if result.graph_fine is not None:
            write_json(temporary / "graph_fine.json", result.graph_fine.to_dict())
            write_json(
                temporary / "selected_path.json",
                {
                    "selected_nodes": [
                        item.to_dict() for item in result.graph_fine.selected_nodes
                    ],
                    "lazy_attempts": [
                        {
                            key: value
                            for key, value in attempt.items()
                            if not key.endswith("_seconds")
                        }
                        for attempt in result.lazy_attempts
                    ],
                },
            )
        if result.backend is not None:
            write_json(temporary / "backend.json", result.backend.to_dict())
            samples = result.backend.trajectory.sample(samples_per_segment=129)
            columns = np.column_stack(
                (
                    samples.time,
                    np.real(samples.position),
                    np.real(samples.velocity),
                    np.real(samples.acceleration),
                    np.real(samples.jerk),
                    np.real(samples.snap),
                    np.real(samples.crackle),
                )
            )
            header = [
                "time",
                *[f"position_{axis}" for axis in "xyz"],
                *[f"velocity_{axis}" for axis in "xyz"],
                *[f"acceleration_{axis}" for axis in "xyz"],
                *[f"jerk_{axis}" for axis in "xyz"],
                *[f"snap_{axis}" for axis in "xyz"],
                *[f"crackle_{axis}" for axis in "xyz"],
            ]
            with (temporary / "optimized_trajectory.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.writer(stream)
                writer.writerow(header)
                writer.writerows(columns)
            plot_overview(scenario, result, temporary / "overview.png")
            plot_gate_diagnostics(
                scenario, result, config, temporary / "gate_diagnostics.png"
            )
            plot_profiles(result, temporary / "profiles.png")
            render = config.runtime.save_video if save_video is None else save_video
            if render:
                make_video(
                    scenario, result, config, temporary / "visualization.mp4"
                )
        metrics = result.metrics_dict()
        metrics.update(
            {
                "experiment": experiment,
                "seed": scenario.seed,
                "optimizer_iterations": 0
                if result.backend is None
                else result.backend.iterations,
                "frontend_time": sum(
                    float(item.get("frontend_seconds", 0.0))
                    for item in result.lazy_attempts
                ),
                "backend_time": sum(
                    float(item.get("backend_seconds", 0.0))
                    for item in result.lazy_attempts
                ),
                "total_planning_time": time.perf_counter() - started,
                "run_complete": True,
            }
        )
        write_json(temporary / "metrics.json", metrics)
        (temporary / "debug.log").write_text(
            "\n".join(
                json.dumps(_jsonable(item), ensure_ascii=False)
                for item in result.lazy_attempts
            )
            + "\n",
            encoding="utf-8",
        )
        if target.exists():
            if not config.runtime.overwrite:
                raise FileExistsError(f"result already exists: {target}")
            shutil.rmtree(target)
        os.replace(temporary, target)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return target


__all__ = ["result_complete", "save_result", "write_json"]
