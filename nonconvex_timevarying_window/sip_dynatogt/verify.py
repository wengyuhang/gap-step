"""Replay a saved certificate from serialized nominal inputs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .certificate import certify
from .io import _json_safe, load_run


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Recompute a SIP-DynaTOGT interval certificate")
    parser.add_argument("--run", required=True, type=Path)
    args = parser.parse_args(argv)
    problem, config, trajectory, stored = load_run(args.run)
    report = certify(problem, trajectory, config)
    status_matches = stored.get("status") == report.status.value
    output = _json_safe({
        "stored_status": stored.get("status"),
        "recomputed": report.to_dict(),
        "status_matches": status_matches,
    })
    (args.run / "verification.json").write_text(
        json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(output, ensure_ascii=False))
    return 0 if report.certified and status_matches else 2


if __name__ == "__main__":
    raise SystemExit(main())
