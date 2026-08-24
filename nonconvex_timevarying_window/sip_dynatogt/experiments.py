"""Small reproducible static/dynamic SIP-DynaTOGT experiment entry point."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.scenarios import build_canonical_scenario

from .io import save_run
from .model import SIPConfig, SIPProblem
from .solver import solve


def _cases(suite: str) -> tuple[tuple[str, int], ...]:
    if suite == "smoke":
        return (("static", 1),)
    if suite == "formal":
        return (("static", 1), ("translation", 1))
    raise ValueError(f"unknown suite {suite!r}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run compact SIP-DynaTOGT experiments")
    parser.add_argument("--suite", choices=("smoke", "formal"), default="smoke")
    parser.add_argument("--outdir", required=True, type=Path)
    parser.add_argument("--slsqp-iterations", type=int, default=80)
    parser.add_argument("--exchange-iterations", type=int, default=12)
    parser.add_argument("--max-cells", type=int, default=200_000)
    args = parser.parse_args(argv)
    args.outdir.mkdir(parents=True, exist_ok=True)
    config = SIPConfig(
        slsqp_max_iterations=args.slsqp_iterations,
        max_exchange_iterations=args.exchange_iterations,
        max_cells=args.max_cells,
    )
    rows: list[dict[str, object]] = []
    for mode, gate_count in _cases(args.suite):
        scenario = build_canonical_scenario(
            mode=mode,
            gate_count=gate_count,
            preprocessing_config=PreprocessingConfig(
                vertex_counts=(32,),
                sc_fit_options={"quadrature_order": 32, "max_nfev": 500},
            ),
        )
        problem = SIPProblem.from_track(scenario.track)
        result = solve(problem, config)
        run_name = f"{mode}_{gate_count}gate"
        save_run(args.outdir / run_name, problem, config, result)
        rows.append(
            {
                "case": run_name,
                "status": result.status.value,
                "total_time": result.total_time,
                "optimizer_success": result.optimizer_success,
                "exchange_rounds": len(result.history),
                "certificate_cells": result.certificate.checked_cells,
                "certificate_precision_bits": result.certificate.precision_bits,
                "minimum_safety_squared_margin": result.certificate.minimum_safety_squared_margin,
                "minimum_dynamic_margin": result.certificate.minimum_dynamic_margin,
            }
        )
    with (args.outdir / "summary.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (args.outdir / "summary.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(rows, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
