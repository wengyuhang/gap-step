"""Resume the certified SIP exchange loop from a saved comparison run."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
from time import perf_counter

from nonconvex_timevarying_window.sip_dynatogt.io import load_run, save_run
from nonconvex_timevarying_window.sip_dynatogt.model import SIPProblem, Witness
from nonconvex_timevarying_window.sip_dynatogt.solver import solve

from .scenario import build_fast_closed_loop_scenario


def resume(
    source_run: str | Path,
    output_run: str | Path,
    *,
    max_exchange_iterations: int = 32,
    max_cells: int = 2_000_000,
    planning_clearance_buffer: float = 0.005,
    additional_witnesses: str | Path | None = None,
) -> dict:
    source = Path(source_run).expanduser()
    _, old_config, _, stored = load_run(source)
    scenario = build_fast_closed_loop_scenario()
    problem = SIPProblem.from_track(
        scenario.track, boundaries=scenario.sip_boundaries
    )
    config = replace(
        old_config,
        body=scenario.body,
        clearance=scenario.net_clearance,
        planning_clearance_buffer=planning_clearance_buffer,
        separator_grid_size=9,
        max_exchange_iterations=max_exchange_iterations,
        max_cells=max_cells,
        precision_bits=(128,),
        max_depth=26,
    )
    active = tuple(Witness(**item) for item in stored["active_witnesses"])
    if additional_witnesses is not None:
        extra_data=json.loads(Path(additional_witnesses).read_text(encoding="utf-8"))
        active=active+tuple(Witness(**item) for item in extra_data)

    started = perf_counter()
    result = solve(
        problem,
        config,
        initial_x=stored["x"],
        active_witnesses=active,
        certify_initial=additional_witnesses is None,
        progress=lambda record: print(
            f"SIP resume round {record.iteration + 1:02d}: "
            f"T={record.total_time:.6f}s, active={record.active_witnesses}, "
            f"status={record.certificate_status.value}, "
            f"cells={record.certificate_cells}",
            flush=True,
        ),
    )
    wall_seconds = perf_counter() - started
    destination = Path(output_run).expanduser()
    save_run(destination, problem, config, result)
    summary = {
        "source_run": str(source),
        "resume_wall_seconds": wall_seconds,
        "planning_clearance_buffer": planning_clearance_buffer,
        "status": result.status.value,
        "total_time": result.total_time,
        "optimizer_success": result.optimizer_success,
        "optimizer_iterations": result.optimizer_iterations,
        "exchange_rounds": len(result.history),
        "certificate": result.certificate.to_dict(),
    }
    (destination / "resume_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_run", type=Path)
    parser.add_argument("output_run", type=Path)
    parser.add_argument("--max-exchange-iterations", type=int, default=32)
    parser.add_argument("--max-cells", type=int, default=2_000_000)
    parser.add_argument("--planning-clearance-buffer", type=float, default=0.005)
    parser.add_argument("--additional-witnesses", type=Path)
    args = parser.parse_args(argv)
    print(
        json.dumps(
            resume(
                args.source_run,
                args.output_run,
                max_exchange_iterations=args.max_exchange_iterations,
                max_cells=args.max_cells,
                planning_clearance_buffer=args.planning_clearance_buffer,
                additional_witnesses=args.additional_witnesses,
            ),
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["main", "resume"]
