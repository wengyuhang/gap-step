"""Run the frozen SC->SIP motion-rate benchmark with compact console output."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
from dataclasses import asdict, dataclass
from pathlib import Path
from time import perf_counter
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import scipy

from nonconvex_timevarying_window.sc_dynatogt.dynamics import DynamicLimits, ObjectiveWeights, PenaltyWeights
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationConfig, optimize_track
from nonconvex_timevarying_window.sip_dynatogt.certificate import certify
from nonconvex_timevarying_window.sip_dynatogt.io import save_run
from nonconvex_timevarying_window.sip_dynatogt.model import (
    PolynomialTrajectory, SIPConfig, SIPProblem, problem_to_dict,
)
from nonconvex_timevarying_window.sip_dynatogt.solver import solve

from .intersection import IntersectionResult, IntersectionStatus
from .scenario import BASE_SEED, MOTION_LEVELS, build_benchmark_scenario


@dataclass(frozen=True)
class RunSettings:
    instances: int = 12
    seed_start: int = 0
    levels: tuple[str, ...] = ("slow", "nominal", "fast")
    max_cells: int = 2_000_000
    max_depth: int = 28

    def __post_init__(self) -> None:
        if self.instances < 1 or self.seed_start < 0:
            raise ValueError("instances must be positive")
        if not self.levels or any(level not in MOTION_LEVELS for level in self.levels):
            raise ValueError("invalid motion level")
        if self.max_cells < 1 or self.max_depth < 1:
            raise ValueError("certificate budgets must be positive")


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not math.isfinite(value):
        return "Infinity" if value > 0.0 else "-Infinity" if value < 0.0 else "NaN"
    if hasattr(value, "to_dict"):
        return _jsonable(value.to_dict())
    if hasattr(value, "value"):
        return _jsonable(value.value)
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(_jsonable(value), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _sha256(value: Any) -> str:
    data = json.dumps(_jsonable(value), ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(data).hexdigest()


def _limits() -> DynamicLimits:
    return DynamicLimits(
        max_velocity=60.0,
        max_body_rate_xy=10.0,
        max_body_rate_z=10.0,
        min_rotor_thrust=0.25,
        max_rotor_thrust=5.0,
    )


def _sc_config() -> OptimizationConfig:
    return OptimizationConfig(
        initial_speed=20.0,
        minimum_initial_duration=0.30,
        max_iterations=400,
        samples_per_segment=12,
        objective_weights=ObjectiveWeights(time=1.0, snap_energy=0.0),
        penalty_weights=PenaltyWeights(
            velocity=0.0, collective_thrust=0.0, body_rate=1.0, rotor_thrust=1.0,
        ),
        dynamic_limits=_limits(),
    )


def _sip_config(scenario, settings: RunSettings) -> SIPConfig:
    return SIPConfig(
        body=scenario.body,
        clearance=scenario.net_clearance,
        planning_clearance_buffer=0.001,
        dynamic_limits=_limits(),
        dynamic_guard_fraction=0.005,
        initial_speed=20.0,
        minimum_initial_duration=0.30,
        separator_grid_size=9,
        max_exchange_iterations=32,
        max_witnesses_per_iteration=8,
        slsqp_max_iterations=240,
        precision_bits=(128, 256),
        max_cells=settings.max_cells,
        max_depth=settings.max_depth,
    )


def _method_record(total_time: float, wall_seconds: float, certificate, intersection, *, optimizer_success: bool, optimizer_iterations: int, extra: dict[str, Any] | None = None) -> dict[str, Any]:
    output = {
        "total_time": float(total_time),
        "solve_wall_seconds": float(wall_seconds),
        "full_certificate": certificate.to_dict(),
        "physical_intersection": intersection.to_dict(),
        "optimizer_success": bool(optimizer_success),
        "optimizer_iterations": int(optimizer_iterations),
    }
    if extra:
        output.update(extra)
    return output


def _physical_status_from_clearance(certificate) -> IntersectionResult:
    """Use a completed 15 mm certificate as the strict no-contact proof.

    For failed/undecidable clearance certification, collision is deliberately
    left unresolved in the batch table.  A separate full intersection audit
    remains available for representative trajectories; this avoids treating
    a nonzero-clearance violation as physical penetration.
    """
    if certificate.status.value == "CERTIFIED_FEASIBLE":
        return IntersectionResult(
            IntersectionStatus.NO_INTERSECTION_CERTIFIED,
            "the stricter 15 mm full-domain clearance certificate implies no physical intersection",
            certificate.precision_bits, certificate.checked_cells, certificate.maximum_depth,
        )
    return IntersectionResult(
        IntersectionStatus.INTERSECTION_UNRESOLVED,
        "clearance was not certified; this batch does not infer physical intersection from a nonzero-clearance violation",
        certificate.precision_bits, certificate.checked_cells, certificate.maximum_depth,
    )


def _run_one(root: Path, seed: int, level: str, settings: RunSettings) -> dict[str, Any]:
    instance_root = root / "instances" / f"seed_{seed:02d}" / level
    benchmark = build_benchmark_scenario(seed, level)
    scenario = benchmark.value
    problem = SIPProblem.from_track(scenario.track, boundaries=scenario.sip_boundaries)
    sc_config = _sc_config()
    sip_config = _sip_config(scenario, settings)
    scenario_payload = {
        "seed_index": seed,
        "random_seed": BASE_SEED + seed,
        "motion_level": level,
        "rate_multiplier": benchmark.rate_multiplier,
        "problem": problem_to_dict(problem),
        "start": scenario.track.start,
        "goal": scenario.track.goal,
        "order": scenario.track.order,
        "min_scale": min(window.motion.minimum_scale for window in scenario.track.windows),
        "sc_config": asdict(sc_config),
        "sip_config": sip_config.to_dict(),
    }
    _write_json(instance_root / "scenario.json", scenario_payload)
    _write_json(instance_root / "manifest.json", {
        "scenario_sha256": _sha256(scenario_payload),
        "generator": "motion_rate_benchmark_v1",
        "initialization": "sc_warm_start",
    })

    started = perf_counter()
    sc = optimize_track(scenario.track, config=sc_config)
    sc_seconds = perf_counter() - started
    sc_trajectory = PolynomialTrajectory.from_minco(sc.trajectory)
    sc_certificate = certify(problem, sc_trajectory, sip_config)
    sc_intersection = _physical_status_from_clearance(sc_certificate)
    _write_json(instance_root / "sc_dynatogt" / "result.json", sc.to_dict())

    started = perf_counter()
    sip = solve(problem, sip_config, initial_x=sc.x)
    sip_seconds = perf_counter() - started
    sip_intersection = _physical_status_from_clearance(sip.certificate)
    save_run(instance_root / "sip_dynatogt" / "run", problem, sip_config, sip)

    return {
        "seed": seed,
        "level": level,
        "scenario_sha256": _sha256(scenario_payload),
        "sc_dynatogt": _method_record(
            sc_trajectory.total_time, sc_seconds, sc_certificate, sc_intersection,
            optimizer_success=sc.success, optimizer_iterations=sc.iterations,
        ),
        "sip_dynatogt": _method_record(
            sip.total_time, sip_seconds, sip.certificate, sip_intersection,
            optimizer_success=sip.optimizer_success, optimizer_iterations=sip.optimizer_iterations,
            extra={
                "end_to_end_wall_seconds": sc_seconds + sip_seconds,
                "exchange_rounds": len(sip.history),
                "active_witnesses": len(sip.active_witnesses),
            },
        ),
    }


def _rows(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows = []
    for record in records:
        for algorithm in ("sc_dynatogt", "sip_dynatogt"):
            value = record[algorithm]
            rows.append({
                "seed": record["seed"],
                "level": record["level"],
                "algorithm": algorithm,
                "scenario_sha256": record["scenario_sha256"],
                "total_time_s": value["total_time"],
                "solve_wall_seconds": value["solve_wall_seconds"],
                "end_to_end_wall_seconds": value.get("end_to_end_wall_seconds", value["solve_wall_seconds"]),
                "optimizer_success": value["optimizer_success"],
                "optimizer_iterations": value["optimizer_iterations"],
                "full_certificate": value["full_certificate"]["status"],
                "physical_intersection": value["physical_intersection"]["status"],
                "intersection_cells": value["physical_intersection"]["checked_cells"],
            })
    return rows


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    for level in MOTION_LEVELS:
        for algorithm in ("sc_dynatogt", "sip_dynatogt"):
            subset = [row for row in rows if row["level"] == level and row["algorithm"] == algorithm]
            if not subset:
                continue
            states: dict[str, int] = {}
            certificates: dict[str, int] = {}
            for row in subset:
                states[row["physical_intersection"]] = states.get(row["physical_intersection"], 0) + 1
                certificates[row["full_certificate"]] = certificates.get(row["full_certificate"], 0) + 1
            output[f"{level}/{algorithm}"] = {
                "instances": len(subset),
                "physical_intersection_counts": states,
                "physical_intersection_rate": states.get("PHYSICAL_INTERSECTION_CONFIRMED", 0) / len(subset),
                "certificate_counts": certificates,
                "median_total_time_s": float(np.median([row["total_time_s"] for row in subset])),
                "median_end_to_end_wall_seconds": float(np.median([row["end_to_end_wall_seconds"] for row in subset])),
            }
    return output


def _plot(rows: list[dict[str, Any]], path: Path) -> None:
    levels = [level for level in MOTION_LEVELS if any(row["level"] == level for row in rows)]
    labels, values = [], []
    for level in levels:
        for algorithm in ("sc_dynatogt", "sip_dynatogt"):
            subset = [row for row in rows if row["level"] == level and row["algorithm"] == algorithm]
            labels.append(f"{level}\n{algorithm.replace('_dynatogt', '').upper()}")
            values.append(sum(row["physical_intersection"] == "PHYSICAL_INTERSECTION_CONFIRMED" for row in subset) / len(subset))
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].bar(labels, values, color=["#be4b48" if "SC" in label else "#287f8f" for label in labels])
    axes[0].set_ylim(0.0, 1.0)
    axes[0].set_ylabel("confirmed physical-intersection rate")
    axes[0].set_title("Entity collision rate")
    time_data, time_labels = [], []
    for level in levels:
        for algorithm in ("sc_dynatogt", "sip_dynatogt"):
            subset = [row["total_time_s"] for row in rows if row["level"] == level and row["algorithm"] == algorithm]
            time_data.append(subset)
            time_labels.append(f"{level}\n{algorithm.replace('_dynatogt', '').upper()}")
    axes[1].boxplot(time_data, tick_labels=time_labels)
    axes[1].set_ylabel("candidate flight time (s)")
    axes[1].set_title("Flight time; safety state reported separately")
    figure.tight_layout()
    figure.savefig(path, dpi=180)
    plt.close(figure)


def run_benchmark(output: str | Path, settings: RunSettings = RunSettings()) -> dict[str, Any]:
    root = Path(output).expanduser()
    if root.exists() and any(root.iterdir()):
        raise FileExistsError(f"refusing to overwrite frozen benchmark directory: {root}")
    root.mkdir(parents=True, exist_ok=True)
    environment = {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "base_seed": BASE_SEED,
        "settings": asdict(settings),
        "initialization": "SC warm start; SC time is included in SIP end-to-end time",
    }
    _write_json(root / "manifest.json", environment)
    records = [_run_one(root, seed, level, settings) for seed in range(settings.seed_start, settings.seed_start + settings.instances) for level in settings.levels]
    rows = _rows(records)
    aggregate = _aggregate(rows)
    _write_json(root / "summary.json", {"environment": environment, "records": records, "aggregate": aggregate})
    _write_csv(root / "metrics.csv", rows)
    _plot(rows, root / "aggregate.png")
    return {"records": len(records), "aggregate": aggregate}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--instances", type=int, default=12)
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--levels", default="slow,nominal,fast")
    parser.add_argument("--max-cells", type=int, default=2_000_000)
    parser.add_argument("--max-depth", type=int, default=28)
    args = parser.parse_args(argv)
    settings = RunSettings(args.instances, args.seed_start, tuple(part.strip() for part in args.levels.split(",") if part.strip()), args.max_cells, args.max_depth)
    result = run_benchmark(args.outdir, settings)
    print(json.dumps({"instances_completed": result["records"], "outdir": str(args.outdir)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
