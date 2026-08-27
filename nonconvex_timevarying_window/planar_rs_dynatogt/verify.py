"""Replay a saved trajectory through the fixed-plane interval certificate."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np

from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateStatus,
    PolynomialTrajectory,
    SIPConfig,
)

from .certificate import certify
from .model import PlanarRSConfig
from .scenario import build_benchmark, build_ordinary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_directory", type=Path)
    parser.add_argument(
        "--precision-bits", default=None, help="comma-separated Arb precisions"
    )
    parser.add_argument("--max-cells", type=int)
    parser.add_argument("--max-depth", type=int)
    parser.add_argument("--plane-prune-max-depth", type=int)
    parser.add_argument("--plane-prune-min-time-width", type=float)
    args = parser.parse_args(argv)
    root = args.result_directory.expanduser()
    saved = json.loads((root / "result.json").read_text(encoding="utf-8"))
    values = np.load(root / "trajectory.npz")
    trajectory = PolynomialTrajectory(values["durations"], values["coefficients"])
    case = str(saved["case"])
    build = build_ordinary if case == "ordinary" else build_benchmark
    _, problem = build(cache_directory=root / "preprocessing_cache")
    raw = dict(saved["config"])
    if "sip" in raw:
        sip = SIPConfig.from_dict(raw["sip"])
        prune_depth = int(raw["plane_prune_max_depth"])
        prune_width = float(raw["plane_prune_min_time_width"])
    else:  # compatibility with the first prototype result
        prune_depth = int(raw.pop("plane_prune_max_depth"))
        prune_width = float(raw.pop("plane_prune_min_time_width"))
        sip = SIPConfig.from_dict(raw)
    overrides = {}
    if args.precision_bits is not None:
        overrides["precision_bits"] = tuple(
            int(value.strip())
            for value in args.precision_bits.split(",")
            if value.strip()
        )
    if args.max_cells is not None:
        overrides["max_cells"] = args.max_cells
    if args.max_depth is not None:
        overrides["max_depth"] = args.max_depth
    if overrides:
        sip = replace(sip, **overrides)
    if args.plane_prune_max_depth is not None:
        prune_depth = args.plane_prune_max_depth
    if args.plane_prune_min_time_width is not None:
        prune_width = args.plane_prune_min_time_width
    started = time.perf_counter()
    report = certify(problem, trajectory, PlanarRSConfig(sip, prune_depth, prune_width))
    payload = {
        "elapsed_s": time.perf_counter() - started,
        "certificate": report.to_dict(),
    }
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if report.status is CertificateStatus.CERTIFIED_FEASIBLE else 2


if __name__ == "__main__":
    raise SystemExit(main())
