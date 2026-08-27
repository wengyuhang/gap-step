"""Reproducible timing experiment for fixed-plane RS certification."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import platform
import time

import numpy as np

from nonconvex_timevarying_window.sip_dynatogt.certificate import (
    certify as baseline_certify,
)
from nonconvex_timevarying_window.sip_dynatogt.constraints import point_flatness
from nonconvex_timevarying_window.sip_dynatogt.model import SIPConfig

from .certificate import certify
from .model import PlanarRSConfig
from .scenario import build_benchmark, build_ordinary
from .solver import solve


def _sampled_dynamic_diagnostics(
    trajectory, config: SIPConfig, nodes_per_segment: int = 1001
):
    speed = []
    rate_xy = []
    rate_z = []
    collective = []
    rotors = []
    force = []
    heading = []
    for segment in range(trajectory.num_segments):
        for tau in np.linspace(0.0, 1.0, nodes_per_segment):
            flat = point_flatness(trajectory, segment, float(tau), config)
            speed.append(float(np.linalg.norm(flat.velocity)))
            rate_xy.append(float(np.linalg.norm(flat.body_rate[:2])))
            rate_z.append(abs(float(flat.body_rate[2])))
            collective.append(float(flat.collective_thrust))
            rotors.append(np.asarray(flat.rotor_thrusts, dtype=float))
            force.append(float(np.sqrt(flat.specific_force_norm2)))
            heading.append(float(np.sqrt(flat.heading_cross_norm2)))
    rotor_values = np.asarray(rotors)
    return {
        "role": "dense diagnostic only; the interval certificate is the safety proof",
        "nodes_per_segment": nodes_per_segment,
        "max_speed_m_s": max(speed),
        "max_body_rate_xy_rad_s": max(rate_xy),
        "max_body_rate_z_rad_s": max(rate_z),
        "collective_thrust_range_n": [min(collective), max(collective)],
        "rotor_thrust_minima_n": rotor_values.min(axis=0).tolist(),
        "rotor_thrust_maxima_n": rotor_values.max(axis=0).tolist(),
        "minimum_specific_force_m_s2": min(force),
        "minimum_heading_cross_norm": min(heading),
    }


def _environment() -> dict[str, object]:
    import scipy

    try:
        import flint

        flint_version = getattr(flint, "__version__", "unknown")
    except ImportError:
        flint_version = None
    return {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "cpu": platform.processor() or platform.machine(),
        "logical_cpus": os.cpu_count(),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "python_flint": flint_version,
    }


def run(
    case: str,
    outdir: str | Path,
    *,
    baseline: bool = False,
    max_exchange_iterations: int | None = None,
    progress=None,
) -> dict[str, object]:
    root = Path(outdir).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    cache = root / "preprocessing_cache"
    build = build_ordinary if case == "ordinary" else build_benchmark
    started = time.perf_counter()
    scenario, problem = build(cache_directory=cache)
    built = time.perf_counter()
    sip = SIPConfig(
        precision_bits=(128,),
        max_cells=500_000,
        max_depth=22,
        max_exchange_iterations=(
            int(max_exchange_iterations)
            if max_exchange_iterations is not None
            else (6 if case == "ordinary" else 8)
        ),
        slsqp_max_iterations=120 if case == "ordinary" else 150,
        feasibility_max_iterations=400 if case == "ordinary" else 500,
    )
    config = PlanarRSConfig(sip=sip, plane_prune_max_depth=12)
    result = solve(problem, config, progress=progress)
    solved = time.perf_counter()
    replay = certify(problem, result.trajectory, config)
    replayed = time.perf_counter()
    payload = {
        "case": case,
        "method": "planar_rs_dynatogt",
        "guarantee_scope": "nominal model; fixed center and fixed plane; in-plane rotation and uniform scale",
        "environment": _environment(),
        "window_count": len(problem.windows),
        "primitive_counts": [len(window.boundary) for window in problem.windows],
        "start": scenario.track.start.tolist(),
        "goal": scenario.track.goal.tolist(),
        "motions": [
            {
                "angle_amplitude_rad": window.motion.angle_amplitude,
                "angle_period_s": window.motion.angle_period,
                "scale_amplitude": window.motion.scale_amplitude,
                "scale_period_s": window.motion.scale_period,
                "phase_rad": window.motion.phase,
                "minimum_scale": window.motion.minimum_scale,
            }
            for window in problem.windows
        ],
        "config": {
            "sip": config.sip.to_dict(),
            "plane_prune_max_depth": config.plane_prune_max_depth,
            "plane_prune_min_time_width": config.plane_prune_min_time_width,
        },
        "timing_s": {
            "build_or_cache_load": built - started,
            "solve": solved - built,
            "pruned_replay": replayed - solved,
        },
        "result": result.to_dict(),
        "replay_certificate": replay.to_dict(),
        "sampled_dynamic_diagnostics": _sampled_dynamic_diagnostics(
            result.trajectory, sip
        ),
    }
    (root / "result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    np.savez_compressed(
        root / "trajectory.npz",
        durations=result.trajectory.durations,
        coefficients=result.trajectory.coefficients,
    )
    if baseline:
        base_started = time.perf_counter()
        base = baseline_certify(problem, result.trajectory, sip)
        payload["timing_s"]["baseline_replay"] = time.perf_counter() - base_started
        payload["baseline_certificate"] = base.to_dict()
        (root / "result.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    return payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=("ordinary", "hard"), default="ordinary")
    parser.add_argument(
        "--outdir", type=Path, default=Path(__file__).with_name("results") / "ordinary"
    )
    parser.add_argument("--baseline-certificate", action="store_true")
    parser.add_argument("--max-exchange-iterations", type=int)
    parser.add_argument("--progress", action="store_true")
    args = parser.parse_args(argv)
    run_started = time.perf_counter()

    def show_progress(record):
        print(
            json.dumps(
                {
                    "round": record.iteration,
                    "elapsed_s": time.perf_counter() - run_started,
                    "optimizer_success": record.optimizer_success,
                    "flight_time": record.total_time,
                    "active_witnesses": record.active_witnesses,
                    "status": record.certificate_status.value,
                    "cells": record.certificate_cells,
                }
            ),
            flush=True,
        )

    payload = run(
        args.case,
        args.outdir,
        baseline=args.baseline_certificate,
        max_exchange_iterations=args.max_exchange_iterations,
        progress=show_progress if args.progress else None,
    )
    print(
        json.dumps(
            {
                "status": payload["result"]["status"],
                "solve_s": payload["timing_s"]["solve"],
                "flight_time": payload["result"]["total_time"],
                "cells": payload["replay_certificate"]["checked_cells"],
                "outdir": str(args.outdir),
            }
        )
    )
    return 0 if payload["result"]["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
