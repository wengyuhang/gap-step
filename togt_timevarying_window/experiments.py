from __future__ import annotations

import argparse
import csv
from pathlib import Path

from .baselines import baseline_metrics, solve_baseline
from .environment import WindowTrack, canonical_track, make_scenario, random_track
from .optimizer import DynaTOGTConfig
from .visualize import draw_plan_gif, draw_plan_png, export_plan_csv

BASELINES = ["WaypointCenter", "StaticTOGT", "DiscreteDynamic", "DynaTOGT"]


def suite_tracks(suite: str) -> list[WindowTrack]:
    if suite == "smoke":
        return [make_scenario("canonical")]
    if suite == "default":
        tracks = [
            make_scenario("canonical"),
            make_scenario("translation_only"),
            make_scenario("rotation_only"),
            make_scenario("scale_only"),
            make_scenario("slow_dynamic"),
            make_scenario("fast_dynamic"),
        ]
        tracks.extend(random_track(seed) for seed in range(10))
        return tracks
    raise ValueError(f"unknown suite: {suite}")


def run_suite(suite: str, outdir: Path, frames: int = 18) -> list[dict[str, object]]:
    root = outdir / suite
    summary_path = root / "summary.csv"
    trajectory_dir = root / "trajectories"
    figure_dir = root / "figures"
    gif_dir = root / "gifs"
    root.mkdir(parents=True, exist_ok=True)
    config = DynaTOGTConfig(max_iter=25 if suite == "default" else 18)
    rows: list[dict[str, object]] = []
    for track in suite_tracks(suite):
        baselines = BASELINES if suite == "default" else ["WaypointCenter", "StaticTOGT", "DynaTOGT"]
        for baseline in baselines:
            plan = solve_baseline(baseline, track, config=config)
            metrics = baseline_metrics(baseline, plan, track, scenario=track.name)
            rows.append(metrics)
            stem = f"{track.name}_{baseline}"
            export_plan_csv(plan, track, trajectory_dir / f"{stem}.csv")
            if baseline == "DynaTOGT" or suite == "smoke":
                draw_plan_png(track, plan, figure_dir / f"{stem}.png", title=f"{track.name} {baseline}")
                draw_plan_gif(track, plan, gif_dir / f"{stem}.gif", frames=frames)
    write_summary(summary_path, rows)
    return rows


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scenario",
        "baseline",
        "mode",
        "success",
        "order",
        "duration",
        "path_length",
        "total_cost",
        "min_gate_margin",
        "max_speed",
        "max_acceleration",
        "mean_jerk",
        "optimization_time",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "") for field in fields})


def main() -> None:
    parser = argparse.ArgumentParser(description="Run DynaTOGT experiment suites.")
    parser.add_argument("--suite", choices=["smoke", "default"], default="smoke")
    parser.add_argument("--outdir", default="togt_timevarying_window/results")
    parser.add_argument("--frames", type=int, default=24)
    args = parser.parse_args()
    rows = run_suite(args.suite, Path(args.outdir), frames=args.frames)
    successes = sum(1 for row in rows if str(row["success"]) == "True" or row["success"] is True)
    print(f"suite={args.suite} rows={len(rows)} successes={successes} summary={Path(args.outdir) / args.suite / 'summary.csv'}")


if __name__ == "__main__":
    main()
