"""Merge completed immutable benchmark shards without rerunning any solver."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .experiment import _aggregate, _plot, _rows, _write_csv, _write_json


def merge(output: Path, shards: list[Path]) -> None:
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"refusing to overwrite {output}")
    records = []
    manifests = []
    for shard in shards:
        summary = json.loads((shard / "summary.json").read_text(encoding="utf-8"))
        records.extend(summary["records"])
        manifests.append(summary["environment"])
    expected = {(record["seed"], record["level"]) for record in records}
    if len(expected) != len(records):
        raise ValueError("duplicate seed/level record across shards")
    output.mkdir(parents=True)
    rows = _rows(records)
    _write_json(output / "summary.json", {"shards": [str(x) for x in shards], "environments": manifests, "records": records, "aggregate": _aggregate(rows)})
    _write_csv(output / "metrics.csv", rows)
    _plot(rows, output / "aggregate.png")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("shards", nargs="+", type=Path)
    args = parser.parse_args()
    merge(args.outdir, args.shards)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
