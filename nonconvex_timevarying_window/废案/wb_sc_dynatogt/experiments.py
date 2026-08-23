"""Reproducible WBSC-DynaTOGT candidate and paired comparison experiments."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import csv
from dataclasses import asdict, dataclass, is_dataclass, replace
from datetime import datetime
import json
import multiprocessing as mp
from pathlib import Path
import time
from types import SimpleNamespace
from typing import Any, Callable

import numpy as np
import shapely
from shapely.geometry import Polygon

from nonconvex_timevarying_window.sc_dynatogt.dynamics import flatness_from_trajectory
from nonconvex_timevarying_window.sc_dynatogt.environment import SCDynamicWindow, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.offset import inward_offset
from nonconvex_timevarying_window.sc_dynatogt.optimizer import optimize_track as optimize_sphere_track
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap

from .collider import CuboidCollider
from .config import WBSCOptimizationConfig
from .optimizer import WBSCOptimizationResult, optimize_track
from .preprocessing import WBPreprocessingConfig, WBPreprocessedGate, preprocess_boundary
from .scenarios import (
    WBScenario,
    build_dynamic_lus_scenario,
    build_static_narrow_scenario,
    candidate_boundary_catalog,
)
from .statistics import paired_bootstrap_interval, wilson_interval
from .validation import validate_legacy_sphere, validate_whole_body
from .visualization import (
    plot_attitude_and_clearance,
    plot_candidate_recovery,
    plot_method_comparison,
)
from .yaw import YawTrajectory


RESULTS_ROOT = Path("nonconvex_timevarying_window/wb_sc_dynatogt/results")


@dataclass(frozen=True)
class SuiteSpec:
    name: str
    candidate_samples: int
    static_seeds: int
    dynamic_seeds: int
    optimizer_iterations: int
    bootstrap_resamples: int = 10_000
    workers: int = 1


SUITES = {
    "smoke": SuiteSpec("smoke", 2_000, 1, 1, 24, workers=1),
    "formal": SuiteSpec("formal", 100_000, 30, 155, 0, workers=20),
}


METHODS = (
    "sc_sphere",
    "point_model",
    "wbsc_dynatogt",
)
FORMAL_METHODS = ("sc_sphere", "wbsc_dynatogt")


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
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, payload: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(payload), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows([{key: _jsonable(row.get(key, "")) for key in keys} for row in rows])
    return path


def _timestamped_directory(root: Path, suite: str) -> Path:
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    base = root / suite / f"{stamp}_{suite}"
    candidate = base
    index = 2
    while candidate.exists():
        candidate = base.with_name(f"{base.name}_{index:02d}")
        index += 1
    candidate.mkdir(parents=True)
    return candidate


def _rotation_matrices(roll: np.ndarray, pitch: np.ndarray, yaw: np.ndarray) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    matrices = np.empty((len(roll), 3, 3), dtype=float)
    matrices[:, 0, 0] = cy * cp
    matrices[:, 0, 1] = cy * sp * sr - sy * cr
    matrices[:, 0, 2] = cy * sp * cr + sy * sr
    matrices[:, 1, 0] = sy * cp
    matrices[:, 1, 1] = sy * sp * sr + cy * cr
    matrices[:, 1, 2] = sy * sp * cr - cy * sr
    matrices[:, 2, 0] = -sp
    matrices[:, 2, 1] = cp * sr
    matrices[:, 2, 2] = cp * cr
    return matrices


def _sample_centers_in_polygon(
    polygon: Polygon,
    count: int,
    rng: np.random.Generator,
) -> np.ndarray:
    bounds = np.asarray(polygon.bounds, dtype=float)
    accepted: list[np.ndarray] = []
    remaining = count
    while remaining:
        proposals = rng.uniform(bounds[[0, 1]], bounds[[2, 3]], size=(max(remaining * 2, 1024), 2))
        mask = shapely.contains_xy(polygon, proposals[:, 0], proposals[:, 1])
        selected = proposals[mask][:remaining]
        accepted.append(selected)
        remaining -= len(selected)
    return np.vstack(accepted)


def candidate_recovery_experiment(
    output: Path,
    *,
    samples: int,
    seed: int,
    preprocessing_config: WBPreprocessingConfig,
    collider: CuboidCollider,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    rng = np.random.default_rng(seed)
    body_points = collider.corners
    for shape_index, (name, boundary) in enumerate(candidate_boundary_catalog().items()):
        gate = preprocess_boundary(boundary, name=name, config=preprocessing_config)
        polygon = Polygon(gate.dense_boundary.vertices)
        sphere_region = polygon.buffer(-collider.config.legacy_sphere_radius)
        cuboid_region = polygon.buffer(-collider.config.clearance)
        centers = _sample_centers_in_polygon(polygon, samples, rng)
        roll = rng.uniform(-0.5 * np.pi, 0.5 * np.pi, samples)
        pitch = rng.uniform(-0.5 * np.pi, 0.5 * np.pi, samples)
        yaw = rng.uniform(-np.pi, np.pi, samples)
        rotations = _rotation_matrices(roll, pitch, yaw)
        sphere_feasible = (
            np.zeros(samples, dtype=bool)
            if sphere_region.is_empty
            else shapely.contains_xy(sphere_region, centers[:, 0], centers[:, 1])
        )
        cuboid_feasible = np.ones(samples, dtype=bool)
        chunk_size = 2_048
        for start in range(0, samples, chunk_size):
            finish = min(samples, start + chunk_size)
            projected = np.einsum(
                "nij,pj->npi", rotations[start:finish], body_points, optimize=True
            )[:, :, :2]
            projected += centers[start:finish, None, :]
            flat = projected.reshape(-1, 2)
            inside = shapely.contains_xy(cuboid_region, flat[:, 0], flat[:, 1]).reshape(
                finish - start, len(body_points)
            )
            cuboid_feasible[start:finish] = np.all(inside, axis=1)
        recovered = cuboid_feasible & ~sphere_feasible
        recovered_count = int(np.count_nonzero(recovered))
        interval = wilson_interval(recovered_count, samples)
        cuboid_count = int(np.count_nonzero(cuboid_feasible))
        conditional = recovered_count / cuboid_count if cuboid_count else float("nan")
        row = {
            "shape": name,
            "samples": samples,
            "sphere_feasible": int(np.count_nonzero(sphere_feasible)),
            "cuboid_feasible": cuboid_count,
            "recovered": recovered_count,
            "recovered_fraction": recovered_count / samples,
            "recovered_fraction_wilson_low": interval[0],
            "recovered_fraction_wilson_high": interval[1],
            "recovered_among_cuboid_feasible": conditional,
            "body_projection_samples": len(body_points),
        }
        rows.append(row)
        plot_candidate_recovery(
            gate.dense_boundary.vertices,
            centers,
            sphere_feasible,
            cuboid_feasible,
            output / "figures" / f"candidate_{shape_index:02d}_{name}.png",
        )
    _write_rows(output / "candidate_recovery.csv", rows)
    return rows


_SPHERE_GEOMETRY_CACHE: dict[tuple[Any, ...], tuple[tuple[np.ndarray, SCDiskMap], ...]] = {}


def _sphere_track(scenario: WBScenario) -> SCWindowTrack:
    cache_key = tuple(
        (gate.name, gate.candidate_polygon.shape, gate.candidate_polygon.tobytes())
        for gate in scenario.preprocessed_gates
    )
    geometries = _SPHERE_GEOMETRY_CACHE.get(cache_key)
    if geometries is None:
        built = []
        for gate in scenario.preprocessed_gates:
            inset = inward_offset(gate.sampled_boundary, distance=0.315)
            mapping = SCDiskMap.fit(inset.vertices, **dict(gate.config.sc_fit_options))
            built.append((inset.vertices.copy(), mapping))
        geometries = tuple(built)
        _SPHERE_GEOMETRY_CACHE[cache_key] = geometries
    windows = []
    for source, (inset_vertices, mapping) in zip(scenario.track.windows, geometries):
        windows.append(
            SCDynamicWindow(
                source.name,
                mapping,
                inset_vertices,
                source.center0.copy(),
                source.angles0.copy(),
                source.motion,
                physical_boundary=source.physical_boundary,
            )
        )
    return SCWindowTrack(
        f"{scenario.track.name}_legacy_sphere",
        scenario.track.start.copy(),
        scenario.track.goal.copy(),
        tuple(windows),
        scenario.track.order,
    )


def _attitude_peaks(result: Any) -> tuple[float, float, float]:
    times = np.linspace(0.0, float(np.sum(result.durations)), 65)
    maximum = np.zeros(3)
    for time_value in times:
        yaw = float(result.yaw_trajectory.evaluate(time_value))
        state = flatness_from_trajectory(
            result.trajectory,
            float(time_value),
            yaw=yaw,
            yaw_rate=float(result.yaw_trajectory.evaluate(time_value, 1)),
            yaw_acceleration=float(result.yaw_trajectory.evaluate(time_value, 2)),
            parameters=result.config.quadrotor,
        )
        rotation = np.asarray(np.real(state.rotation), dtype=float)
        pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        yaw_angle = np.arctan2(rotation[1, 0], rotation[0, 0])
        maximum = np.maximum(maximum, np.abs([roll, pitch, yaw_angle]))
    return tuple(float(value) for value in maximum)


def _dynamic_feasible(result: Any) -> bool:
    extrema = result.extrema if hasattr(result, "extrema") else result.constraint_extrema
    limits = result.config.dynamic_limits
    return bool(
        float(extrema["max_velocity"]) <= limits.max_velocity + 1.0e-6
        and float(extrema["max_body_rate_xy"]) <= limits.max_body_rate_xy + 1.0e-6
        and float(extrema["max_abs_body_rate_z"]) <= limits.max_body_rate_z + 1.0e-6
        and np.min(extrema["min_rotor_thrust"]) >= limits.min_rotor_thrust - 1.0e-6
        and np.max(extrema["max_rotor_thrust"]) <= limits.max_rotor_thrust + 1.0e-6
    )


def _failed_row(method: str, family: str, seed: int, elapsed: float, error: Exception) -> dict[str, Any]:
    return {
        "method": method,
        "family": family,
        "seed": seed,
        "safety_model": "legacy_sphere_inset" if method == "sc_sphere" else "oriented_cuboid",
        "optimizer_success": False,
        "optimizer_message": "not run",
        "model_safe": False,
        "whole_body_safe": False,
        "safe_success": False,
        "wall_time": elapsed,
        "failure_reason": f"{type(error).__name__}: {error}",
    }


def _run_method(
    method: str,
    scenario: WBScenario,
    config: WBSCOptimizationConfig,
    output: Path,
    family: str,
) -> dict[str, Any]:
    start = time.perf_counter()
    try:
        if method == "sc_sphere":
            sphere_track = _sphere_track(scenario)
            raw = optimize_sphere_track(sphere_track, config=config)
            adapter = SimpleNamespace(
                durations=raw.durations,
                traversal_times=raw.traversal_times,
                trajectory=raw.trajectory,
                yaw_trajectory=YawTrajectory(np.zeros(len(scenario.track.order)), raw.durations),
                config=config,
            )
            report = validate_legacy_sphere(scenario.track, adapter)
            adapter.safety_report = report
            adapter.total_time = raw.total_time
            optimizer_success = bool(raw.success)
            optimizer_message = str(raw.message)
            objective = float(raw.objective)
            iterations = int(raw.iterations)
            extrema = raw.constraint_extrema
            result_for_attitude = adapter
        else:
            method_config = config
            body_scale = 1.0
            if method == "point_model":
                body_scale = 0.0
            elif method != "wbsc_dynatogt":
                raise ValueError(f"unknown method {method}")
            raw = optimize_track(scenario.track, method_config, body_scale=body_scale)
            report = raw.safety_report
            assert report is not None
            optimizer_success = bool(raw.optimizer_success)
            optimizer_message = str(raw.message)
            objective = float(raw.objective)
            iterations = int(raw.iterations)
            extrema = raw.extrema
            result_for_attitude = raw
            if method == "wbsc_dynatogt" and scenario.seed == 0:
                _write_json(output / "trajectories" / f"{family}_{method}_seed0.json", raw.to_dict())
                plot_attitude_and_clearance(
                    raw,
                    output / "figures" / f"attitude_clearance_{family}.png",
                    track=scenario.track,
                )
        elapsed = time.perf_counter() - start
        roll_peak, pitch_peak, yaw_peak = _attitude_peaks(result_for_attitude)
        # Attach the common extrema/config interface needed by the diagnostic.
        if not hasattr(result_for_attitude, "extrema"):
            result_for_attitude.extrema = extrema
        dynamic_feasible = _dynamic_feasible(result_for_attitude)
        safe = bool(report.safe)
        return {
            "method": method,
            "family": family,
            "seed": scenario.seed,
            "optimizer_success": optimizer_success,
            "optimizer_message": optimizer_message,
            "safety_model": "legacy_sphere_inset" if method == "sc_sphere" else "oriented_cuboid",
            "model_safe": safe,
            "whole_body_safe": safe,
            "safe_success": bool(optimizer_success and safe),
            "wall_time": elapsed,
            "objective": objective,
            "iterations": iterations,
            "total_time": float(np.sum(result_for_attitude.durations)),
            "minimum_clearance": float(report.minimum_clearance),
            "hard_constraint_minimum": float(
                getattr(raw, "hard_constraint_minimum", float("nan"))
            ),
            "dynamic_sample_feasible": dynamic_feasible,
            "peak_abs_roll": roll_peak,
            "peak_abs_pitch": pitch_peak,
            "peak_abs_yaw": yaw_peak,
            "failure_reason": (
                ""
                if optimizer_success and safe
                else optimizer_message
                if not optimizer_success
                else report.violations[0].reason
                if report.violations
                else "cuboid projection invalid"
            ),
        }
    except Exception as error:
        return _failed_row(method, family, scenario.seed, time.perf_counter() - start, error)


def _paired_scenarios(spec: SuiteSpec, preprocessing: WBPreprocessingConfig):
    for family, count, builder in (
        ("static_narrow_L", spec.static_seeds, lambda seed, gates=None: build_static_narrow_scenario(seed=seed, variant="l", preprocessing_config=preprocessing, preprocessed_gates=gates)),
        ("static_U_curve", spec.static_seeds, lambda seed, gates=None: build_static_narrow_scenario(seed=seed, variant="u_curve", preprocessing_config=preprocessing, preprocessed_gates=gates)),
        ("dynamic_L_U_star", spec.dynamic_seeds, lambda seed, gates=None: build_dynamic_lus_scenario(seed=seed, preprocessing_config=preprocessing, preprocessed_gates=gates)),
    ):
        first = builder(0)
        yield family, first
        gates = first.preprocessed_gates
        for seed in range(1, count):
            yield family, builder(seed, gates)


_WORKER_GATE_CACHE: dict[str, tuple[WBPreprocessedGate, ...]] = {}
_WORKER_THREAD_LIMITER: Any | None = None


def _limit_worker_threads() -> None:
    global _WORKER_THREAD_LIMITER
    if _WORKER_THREAD_LIMITER is None:
        from threadpoolctl import threadpool_limits

        _WORKER_THREAD_LIMITER = threadpool_limits(limits=1)
    try:
        import torch

        torch.set_num_threads(1)
    except (ImportError, RuntimeError):
        pass


def _build_family_scenario(
    family: str,
    seed: int,
    preprocessing: WBPreprocessingConfig,
) -> WBScenario:
    gates = _WORKER_GATE_CACHE.get(family)
    if family == "static_narrow_L":
        scenario = build_static_narrow_scenario(
            seed=seed,
            variant="l",
            preprocessing_config=preprocessing,
            preprocessed_gates=gates,
        )
    elif family == "static_U_curve":
        scenario = build_static_narrow_scenario(
            seed=seed,
            variant="u_curve",
            preprocessing_config=preprocessing,
            preprocessed_gates=gates,
        )
    elif family == "dynamic_L_U_star":
        scenario = build_dynamic_lus_scenario(
            seed=seed,
            preprocessing_config=preprocessing,
            preprocessed_gates=gates,
        )
    else:
        raise ValueError(f"unknown scenario family {family}")
    _WORKER_GATE_CACHE[family] = scenario.preprocessed_gates
    return scenario


def _run_scenario_task(
    family: str,
    seed: int,
    preprocessing: WBPreprocessingConfig,
    config: WBSCOptimizationConfig,
    output: Path,
    preprocessed_gates: tuple[WBPreprocessedGate, ...] | None = None,
) -> list[dict[str, Any]]:
    _limit_worker_threads()
    if preprocessed_gates is not None:
        _WORKER_GATE_CACHE[family] = preprocessed_gates
    scenario = _build_family_scenario(family, seed, preprocessing)
    return [_run_method(method, scenario, config, output, family) for method in METHODS]


def _run_method_task(
    family: str,
    seed: int,
    method: str,
    preprocessing: WBPreprocessingConfig,
    config: WBSCOptimizationConfig,
    output: Path,
    preprocessed_gates: tuple[WBPreprocessedGate, ...],
) -> dict[str, Any]:
    _limit_worker_threads()
    _WORKER_GATE_CACHE[family] = preprocessed_gates
    scenario = _build_family_scenario(family, seed, preprocessing)
    return _run_method(method, scenario, config, output, family)


def comparison_experiment(
    output: Path,
    *,
    spec: SuiteSpec,
    preprocessing: WBPreprocessingConfig,
    config: WBSCOptimizationConfig,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    methods = FORMAL_METHODS if spec.name == "formal" else METHODS
    _WORKER_GATE_CACHE.clear()
    _SPHERE_GEOMETRY_CACHE.clear()
    rows: list[dict[str, Any]] = []
    if spec.workers == 1:
        for family, scenario in _paired_scenarios(spec, preprocessing):
            for method in methods:
                row = _run_method(method, scenario, config, output, family)
                rows.append(row)
                _write_rows(output / "method_runs.csv", rows)
    else:
        templates = {
            family: _build_family_scenario(family, 0, preprocessing).preprocessed_gates
            for family in ("static_narrow_L", "static_U_curve", "dynamic_L_U_star")
        }
        tasks = [
            (family, seed, method)
            for family, count in (
                ("static_narrow_L", spec.static_seeds),
                ("static_U_curve", spec.static_seeds),
                ("dynamic_L_U_star", spec.dynamic_seeds),
            )
            for seed in range(count)
            for method in methods
        ]
        # ``fork`` can inherit an already-initialized BLAS/PyTorch thread pool
        # and hang as soon as a worker enters reverse-mode differentiation.
        # Fresh interpreters make the formal suite slower to start but keep the
        # numerical workers isolated and reproducible.
        with ProcessPoolExecutor(
            max_workers=spec.workers,
            mp_context=mp.get_context("spawn"),
        ) as executor:
            futures = {
                executor.submit(
                    _run_method_task,
                    family,
                    seed,
                    method,
                    preprocessing,
                    config,
                    output,
                    templates[family],
                ): (family, seed, method)
                for family, seed, method in tasks
            }
            for future in as_completed(futures):
                family, seed, method = futures[future]
                try:
                    rows.append(future.result())
                except Exception as error:
                    rows.append(_failed_row(method, family, seed, 0.0, error))
                rows.sort(key=lambda row: (row["family"], int(row["seed"]), methods.index(row["method"])))
                _write_rows(output / "method_runs.csv", rows)
    summaries: dict[str, Any] = {}
    for method in methods:
        method_rows = [row for row in rows if row["method"] == method]
        converged_rows = [row for row in method_rows if bool(row["optimizer_success"])]
        safe_rows = [row for row in converged_rows if bool(row["whole_body_safe"])]
        summary: dict[str, Any] = {
            "runs": len(method_rows),
            "converged": len(converged_rows),
            "convergence_rate": len(converged_rows) / len(method_rows),
            "convergence_wilson_95": wilson_interval(len(converged_rows), len(method_rows)),
            "safe_converged": len(safe_rows),
            "safe_successes": len(safe_rows),
            "safe_success_rate": len(safe_rows) / len(method_rows),
            "safe_success_wilson_95": wilson_interval(len(safe_rows), len(method_rows)),
            "safe_given_converged": (
                len(safe_rows) / len(converged_rows) if converged_rows else None
            ),
            "safe_given_converged_wilson_95": (
                wilson_interval(len(safe_rows), len(converged_rows))
                if converged_rows
                else None
            ),
        }
        for metric in ("wall_time", "total_time", "minimum_clearance"):
            eligible = converged_rows if metric == "wall_time" else safe_rows
            values = np.asarray(
                [float(row[metric]) for row in eligible if metric in row and np.isfinite(row[metric])]
            )
            if len(values):
                suffix = "converged" if metric == "wall_time" else "converged_safe"
                summary[f"mean_{metric}_{suffix}"] = float(np.mean(values))
                summary[f"mean_{metric}_{suffix}_bootstrap_95"] = paired_bootstrap_interval(
                    values,
                    resamples=spec.bootstrap_resamples,
                    seed=17,
                )
        summaries[method] = summary
    # Explicit paired deltas against the complete method, using only seeds for
    # which both quantities exist; excluded pair counts remain visible.
    full_by_pair = {
        (row["family"], row["seed"]): row for row in rows if row["method"] == "wbsc_dynatogt"
    }
    paired: dict[str, Any] = {}
    for method in methods[:-1]:
        method_pairs = []
        for row in rows:
            if row["method"] != method or not bool(row["safe_success"]):
                continue
            reference = full_by_pair[(row["family"], row["seed"])]
            if bool(reference["safe_success"]):
                method_pairs.append(float(row["total_time"]) - float(reference["total_time"]))
        paired[method] = {
            "mutually_converged_safe_pairs": len(method_pairs),
            "mean_total_time_delta_vs_wbsc": float(np.mean(method_pairs)) if method_pairs else None,
            "bootstrap_95": paired_bootstrap_interval(
                np.asarray(method_pairs), resamples=spec.bootstrap_resamples, seed=23
            ) if method_pairs else None,
        }
    summary_payload = {"methods": summaries, "paired_differences": paired}
    _write_json(output / "comparison_summary.json", summary_payload)
    plot_method_comparison(rows, output / "figures" / "method_comparison.png")
    return rows, summary_payload


def _chinese_report(
    output: Path,
    spec: SuiteSpec,
    candidates: list[dict[str, Any]],
    comparison: dict[str, Any],
) -> Path:
    lines = [
        f"# WBSC-DynaTOGT {spec.name} 实验报告",
        "",
        "> 原算法按自身 0.315 m 球形内缩检查；WBSC 按指定穿越时刻的姿态长方体投影检查。二者都不是连续时间严格证书。",
        "",
        "## 候选集合恢复",
        "",
        "|窗口|样本数|球模型可行|长方体可行|恢复比例（Wilson 95%）|",
        "|---|---:|---:|---:|---:|",
    ]
    for row in candidates:
        lines.append(
            f"|{row['shape']}|{row['samples']}|{row['sphere_feasible']}|{row['cuboid_feasible']}|"
            f"{row['recovered_fraction']:.4f} [{row['recovered_fraction_wilson_low']:.4f}, {row['recovered_fraction_wilson_high']:.4f}]|"
        )
    lines.extend(
        [
            "",
            "## 方法对比",
            "",
            "|方法|运行数|收敛率（Wilson 95%）|收敛后模型安全率|",
            "|---|---:|---:|---:|",
        ]
    )
    for method, values in comparison["methods"].items():
        low, high = values["convergence_wilson_95"]
        conditional = values["safe_given_converged"]
        conditional_text = "--" if conditional is None else f"{conditional:.4f}"
        lines.append(
            f"|{method}|{values['runs']}|{values['convergence_rate']:.4f} "
            f"[{low:.4f}, {high:.4f}]|{conditional_text}|"
        )
    lines.extend(
        [
            "",
            "连续指标和配对飞行时间只使用收敛且各自模型安全的结果。",
            *(
                ["点模型仅是不安全的乐观上界。"]
                if "point_model" in comparison["methods"]
                else []
            ),
            "球形内缩为空的实例仍保留在收敛率分母中。",
            "本报告只陈述观察值，不设置 WBSC-DynaTOGT 必须胜出的阈值。",
            "",
        ]
    )
    path = output / "REPORT_ZH.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def run_suite(
    suite: str,
    *,
    outdir: str | Path = RESULTS_ROOT,
    candidate_samples: int | None = None,
    static_seeds: int | None = None,
    dynamic_seeds: int | None = None,
    workers: int | None = None,
) -> Path:
    if suite not in SUITES:
        raise ValueError("suite must be smoke or formal")
    spec = SUITES[suite]
    spec = replace(
        spec,
        candidate_samples=spec.candidate_samples if candidate_samples is None else candidate_samples,
        static_seeds=spec.static_seeds if static_seeds is None else static_seeds,
        dynamic_seeds=spec.dynamic_seeds if dynamic_seeds is None else dynamic_seeds,
        workers=spec.workers if workers is None else workers,
    )
    if min(spec.candidate_samples, spec.static_seeds, spec.dynamic_seeds, spec.workers) < 1:
        raise ValueError("sample and seed counts must be positive")
    output = _timestamped_directory(Path(outdir), suite)
    preprocessing = WBPreprocessingConfig(
        sc_fit_options={"quadrature_order": 32 if suite == "smoke" else 64}
    )
    config = WBSCOptimizationConfig(
        max_iterations=spec.optimizer_iterations,
        past_iterations=8 if suite == "smoke" else 32,
        function_tolerance=1.0e-4 if suite == "smoke" else 1.0e-5,
        samples_per_segment=4 if suite == "smoke" else 6,
    )
    collider = CuboidCollider(config.collider)
    manifest = {
        "algorithm": "WBSC-DynaTOGT",
        "suite": spec,
        "problem_definition": "nonconvex_timevarying_window/PROBLEM_DEFINITION.md",
        "methods": FORMAL_METHODS if suite == "formal" else METHODS,
        "config": config,
        "preprocessing": preprocessing,
        "collider": collider.manifest(),
        "safety_claim": "oriented cuboid projection checked at prescribed crossings only",
        "comparison_policy": (
            "same scenarios and seeds; legacy SC keeps its 0.315 m inset; "
            "formal continuous metrics use converged and model-safe results only"
        ),
        "references": [
            "https://doi.org/10.1109/ICRA55743.2025.11128088",
        ],
    }
    _write_json(output / "manifest.json", manifest)
    candidates = candidate_recovery_experiment(
        output,
        samples=spec.candidate_samples,
        seed=0,
        preprocessing_config=preprocessing,
        collider=collider,
    )
    rows, summary = comparison_experiment(
        output,
        spec=spec,
        preprocessing=preprocessing,
        config=config,
    )
    _chinese_report(output, spec, candidates, summary)
    _write_json(
        output / "summary.json",
        {"candidate_recovery": candidates, "comparison": summary, "run_count": len(rows)},
    )
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run WBSC-DynaTOGT comparisons")
    parser.add_argument("--suite", choices=tuple(SUITES), required=True)
    parser.add_argument("--outdir", type=Path, default=RESULTS_ROOT)
    parser.add_argument("--candidate-samples", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--static-seeds", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--dynamic-seeds", type=int, help=argparse.SUPPRESS)
    parser.add_argument("--workers", type=int, help="parallel workers (formal default: 20)")
    args = parser.parse_args(argv)
    output = run_suite(
        args.suite,
        outdir=args.outdir,
        candidate_samples=args.candidate_samples,
        static_seeds=args.static_seeds,
        dynamic_seeds=args.dynamic_seeds,
        workers=args.workers,
    )
    print(json.dumps({"suite": args.suite, "output": str(output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "METHODS",
    "FORMAL_METHODS",
    "RESULTS_ROOT",
    "SUITES",
    "SuiteSpec",
    "candidate_recovery_experiment",
    "comparison_experiment",
    "main",
    "run_suite",
]
