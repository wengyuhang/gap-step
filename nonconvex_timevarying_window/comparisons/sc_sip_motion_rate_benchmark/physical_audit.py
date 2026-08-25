"""Strict physical-intersection audit for every saved benchmark trajectory.

Unlike the clearance certificate, this checker asks only whether an original
continuous window boundary enters the moving, oriented rectangular body.  A
row is persisted after each method so interrupted long audits remain usable.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sip_dynatogt.io import load_run
from nonconvex_timevarying_window.sip_dynatogt.model import PolynomialTrajectory

from .experiment import _write_json
from .intersection import certify_physical_intersection
from .scenario import build_benchmark_scenario


def audit(shards: list[Path], output: Path) -> dict:
    rows: list[dict] = []
    for shard in shards:
        summary = json.loads((shard / "summary.json").read_text(encoding="utf-8"))
        for record in summary["records"]:
            instance = shard / "instances" / f"seed_{record['seed']:02d}" / record["level"]
            problem, config, sip_trajectory, _ = load_run(instance / "sip_dynatogt" / "run")
            track = build_benchmark_scenario(record["seed"], record["level"]).value.track
            sc_data = json.loads((instance / "sc_dynatogt" / "result.json").read_text(encoding="utf-8"))
            sc_trajectory = PolynomialTrajectory.from_minco(MincoSnap(
                BoundaryState(track.start), BoundaryState(track.goal),
                np.asarray(sc_data["waypoints"], dtype=float),
                np.asarray(sc_data["durations"], dtype=float),
            ))
            for algorithm, trajectory in (("sc_dynatogt", sc_trajectory), ("sip_dynatogt", sip_trajectory)):
                result = certify_physical_intersection(problem, trajectory, config)
                rows.append({
                    "shard": shard.name,
                    "seed": record["seed"],
                    "level": record["level"],
                    "algorithm": algorithm,
                    "physical_intersection": result.to_dict(),
                })
                counts: dict[str, int] = {}
                for row in rows:
                    key = row["physical_intersection"]["status"]
                    counts[key] = counts.get(key, 0) + 1
                _write_json(output, {"rows": rows, "count": len(rows), "status_counts": counts, "complete": False})
    counts: dict[str, int] = {}
    for row in rows:
        key = row["physical_intersection"]["status"]
        counts[key] = counts.get(key, 0) + 1
    result = {"rows": rows, "count": len(rows), "status_counts": counts, "complete": True}
    _write_json(output, result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("shards", nargs="+", type=Path)
    args = parser.parse_args()
    result = audit(args.shards, args.out)
    print(json.dumps({"count": result["count"], "status_counts": result["status_counts"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
