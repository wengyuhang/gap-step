"""Small executable experiment for CWB-SC-DynaTOGT."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    JointTOGTObjective,
    OptimizationConfig,
)
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.scenarios import build_canonical_scenario

from .body_model import CuboidBody
from .config import WholeBodySafetyConfig
from .constraint_generation import optimize_with_whole_body_safety
from .whole_body_visualization import plot_crossing_diagnostics


def run_smoke(output_directory: str | Path) -> dict[str, object]:
    """Run one static L-window ``[K,D]`` solve and V1 whole-body verification."""

    root = Path(output_directory)
    root.mkdir(parents=True, exist_ok=True)
    scenario = build_canonical_scenario(
        mode="static",
        gate_count=1,
        preprocessing_config=PreprocessingConfig(vertex_counts=(64, 128, 256)),
    )
    objective = JointTOGTObjective(
        scenario.track,
        OptimizationConfig(max_iterations=80, samples_per_segment=8),
    )
    safety = WholeBodySafetyConfig(
        time_tolerance=2.0e-3,
        lambda_tolerance=2.0e-3,
        max_interval_depth=18,
        max_outer_iterations=3,
    )
    body = CuboidBody(np.asarray(safety.half_extents))
    result = optimize_with_whole_body_safety(objective, objective.initial_guess(), body, safety)
    summary = {
        "method": "CWB-SC-DynaTOGT",
        "decision_variables": "[K,D]",
        "yaw": "constant",
        "optimizer_success": result.optimizer_success,
        "whole_body_status": result.status.value,
        "verification_status": result.safety_report.status.value,
        "certified": result.status.value == "safe_certified",
        "total_time": float(np.sum(result.forward.durations)),
        "outer_iterations": len(result.history),
        "active_witnesses": len(result.active_constraints),
    }
    (root / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    plot_crossing_diagnostics(
        forward=result.forward, track=scenario.track, body=body, config=safety,
        report=result.safety_report, output=root / "crossing_diagnostics.png",
        parameters=objective.config.quadrotor,
    )
    return summary


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point for the smoke protocol."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("smoke",), default="smoke")
    parser.add_argument(
        "--outdir",
        type=Path,
        default=Path("nonconvex_timevarying_window/cwb_sc_dynatogt/results/smoke"),
    )
    args = parser.parse_args(argv)
    summary = run_smoke(args.outdir)
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["main", "run_smoke"]
