from __future__ import annotations

import argparse
from pathlib import Path

from .baselines import baseline_metrics, solve_baseline
from .environment import DEFAULT_ORDER, make_scenario
from .optimizer import DynaTOGTConfig
from .visualize import draw_plan_gif, draw_plan_png, export_plan_csv


def _parse_order(text: str) -> tuple[int, ...]:
    order = []
    for item in text.split(","):
        item = item.strip().upper()
        if not item:
            continue
        if item.startswith("G"):
            item = item[1:]
        order.append(int(item) - 1)
    return tuple(order)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export one DynaTOGT dynamic-window demo.")
    parser.add_argument("--scenario", default="canonical")
    parser.add_argument("--mode", choices=["static", "ordered_dynamic", "shuffled_dynamic"], default="ordered_dynamic")
    parser.add_argument("--order", default=",".join(f"G{i + 1}" for i in DEFAULT_ORDER))
    parser.add_argument("--outdir", default="togt_timevarying_window/results/demo")
    parser.add_argument("--frames", type=int, default=56)
    parser.add_argument("--max-iter", type=int, default=70)
    args = parser.parse_args()
    track = make_scenario(args.scenario)
    if args.order:
        track.order = _parse_order(args.order)
    baseline = "DynaTOGT"
    if args.mode == "static":
        baseline = "StaticTOGT"
    elif args.mode == "shuffled_dynamic":
        baseline = "ShuffledDynaTOGT"
    plan = solve_baseline(baseline, track, config=DynaTOGTConfig(max_iter=args.max_iter))
    outdir = Path(args.outdir)
    stem = f"{track.name}_{baseline}"
    csv_path = outdir / "trajectories" / f"{stem}.csv"
    png_path = outdir / "figures" / f"{stem}.png"
    gif_path = outdir / "gifs" / f"{stem}.gif"
    export_plan_csv(plan, track, csv_path)
    draw_plan_png(track, plan, png_path, title="动态时变窗口穿越演示")
    draw_plan_gif(track, plan, gif_path, frames=args.frames)
    metrics = baseline_metrics(baseline, plan, track, scenario=track.name)
    print(f"csv={csv_path}")
    print(f"png={png_path}")
    print(f"gif={gif_path}")
    print(
        f"success={metrics['success']} order={metrics['order']} duration={metrics['duration']:.3f} "
        f"path_length={metrics['path_length']:.3f} total_cost={metrics['total_cost']:.3f}"
    )


if __name__ == "__main__":
    main()
