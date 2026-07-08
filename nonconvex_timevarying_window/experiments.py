from __future__ import annotations

import argparse
import csv
from pathlib import Path

from .environment import NonConvexWindowTrack, make_scenario, random_track
from .optimizer import AtlasDynaTOGTConfig, AtlasDynaTOGTOptimizer, plan_metrics
from .visualize import draw_plan_gif, draw_plan_png, export_plan_csv


def suite_tracks(suite: str) -> list[NonConvexWindowTrack]:
    """返回实验套件场景列表。"""
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
        tracks.extend(random_track(seed) for seed in range(8))
        return tracks
    raise ValueError(f"unknown suite: {suite}")


def run_suite(suite: str, outdir: Path, frames: int = 18, playback_speed: float = 1.0) -> list[dict[str, object]]:
    """运行非凸窗口实验套件并导出结果。"""
    root = outdir / suite
    summary_path = root / "summary.csv"
    trajectory_dir = root / "trajectories"
    figure_dir = root / "figures"
    gif_dir = root / "gifs"
    root.mkdir(parents=True, exist_ok=True)
    config = AtlasDynaTOGTConfig(max_iter=18 if suite == "default" else 4, chart_multistarts=2)
    optimizer = AtlasDynaTOGTOptimizer(config)
    rows: list[dict[str, object]] = []
    for track in suite_tracks(suite):
        plan = optimizer.solve(track, mode="ordered_dynamic")
        metrics = dict(plan_metrics(plan, track, dynamic_eval=True))
        metrics["scenario"] = track.name
        rows.append(metrics)
        export_plan_csv(plan, track, trajectory_dir / f"{track.name}.csv")
        draw_plan_png(track, plan, figure_dir / f"{track.name}.png", title=f"{track.name} 非凸动态窗口穿越")
        draw_plan_gif(track, plan, gif_dir / f"{track.name}.gif", frames=frames, playback_speed=playback_speed)
    write_summary(summary_path, rows)
    return rows


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    """写入 summary.csv。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "scenario",
        "mode",
        "success",
        "order",
        "chart_ids",
        "duration",
        "path_length",
        "total_cost",
        "min_boundary_margin",
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
    """命令行入口：运行 smoke/default 实验套件。"""
    parser = argparse.ArgumentParser(description="Run non-convex DynaTOGT experiment suites.")
    parser.add_argument("--suite", choices=["smoke", "default"], default="smoke")
    parser.add_argument("--outdir", default="nonconvex_timevarying_window/results")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--playback-speed", type=float, default=1.0)
    args = parser.parse_args()
    rows = run_suite(args.suite, Path(args.outdir), frames=args.frames, playback_speed=args.playback_speed)
    successes = sum(1 for row in rows if str(row["success"]) == "True" or row["success"] is True)
    print(f"suite={args.suite} rows={len(rows)} successes={successes} summary={Path(args.outdir) / args.suite / 'summary.csv'}")


if __name__ == "__main__":
    main()
