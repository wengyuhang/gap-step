"""Replay every saved SC and SIP trajectory after a sharded benchmark run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sip_dynatogt.certificate import certify
from nonconvex_timevarying_window.sip_dynatogt.io import load_run
from nonconvex_timevarying_window.sip_dynatogt.model import PolynomialTrajectory

from .experiment import _write_json
from .scenario import build_benchmark_scenario


def audit(shards: list[Path], output: Path) -> dict:
    rows = []
    for shard in shards:
        summary = json.loads((shard / "summary.json").read_text(encoding="utf-8"))
        for record in summary["records"]:
            instance = shard / "instances" / f"seed_{record['seed']:02d}" / record["level"]
            problem, config, sip_trajectory, _ = load_run(instance / "sip_dynatogt" / "run")
            track = build_benchmark_scenario(record["seed"], record["level"]).value.track
            sc_data = json.loads((instance / "sc_dynatogt" / "result.json").read_text(encoding="utf-8"))
            sc_minco = MincoSnap(
                BoundaryState(track.start), BoundaryState(track.goal),
                np.asarray(sc_data["waypoints"], dtype=float),
                np.asarray(sc_data["durations"], dtype=float),
            )
            sc_report = certify(problem, PolynomialTrajectory.from_minco(sc_minco), config)
            sip_report = certify(problem, sip_trajectory, config)
            expected_sc = record["sc_dynatogt"]["full_certificate"]["status"]
            expected_sip = record["sip_dynatogt"]["full_certificate"]["status"]
            rows.append({
                "shard": shard.name,
                "seed": record["seed"],
                "level": record["level"],
                "sc_expected": expected_sc,
                "sc_replayed": sc_report.status.value,
                "sip_expected": expected_sip,
                "sip_replayed": sip_report.status.value,
                "match": expected_sc == sc_report.status.value and expected_sip == sip_report.status.value,
            })
    result = {"rows": rows, "all_match": all(row["match"] for row in rows), "count": len(rows)}
    _write_json(output, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("shards", nargs="+", type=Path)
    args = parser.parse_args()
    result = audit(args.shards, args.out)
    print(json.dumps({"count": result["count"], "all_match": result["all_match"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
