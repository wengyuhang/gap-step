from __future__ import annotations

import argparse

from .baselines import baseline_metrics, solve_baseline
from .environment import DEFAULT_ORDER, make_scenario
from .optimizer import DynaTOGTConfig


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
    parser = argparse.ArgumentParser(description="Run one DynaTOGT dynamic-window TOGT demo.")
    parser.add_argument("--scenario", default="canonical")
    parser.add_argument("--mode", choices=["static", "ordered_dynamic", "shuffled_dynamic"], default="ordered_dynamic")
    parser.add_argument("--order", default=",".join(f"G{i + 1}" for i in DEFAULT_ORDER))
    parser.add_argument("--baseline", choices=["WaypointCenter", "StaticTOGT", "DiscreteDynamic", "DynaTOGT", "ShuffledDynaTOGT"], default=None)
    parser.add_argument("--max-iter", type=int, default=70)
    args = parser.parse_args()
    track = make_scenario(args.scenario)
    if args.order:
        track.order = _parse_order(args.order)
    baseline = args.baseline
    if baseline is None:
        baseline = "DynaTOGT" if args.mode != "static" else "StaticTOGT"
        if args.mode == "shuffled_dynamic":
            baseline = "ShuffledDynaTOGT"
    plan = solve_baseline(baseline, track, config=DynaTOGTConfig(max_iter=args.max_iter))
    metrics = baseline_metrics(baseline, plan, track, scenario=track.name)
    print(
        f"scenario={track.name} baseline={baseline} success={metrics['success']} "
        f"order={metrics['order']} duration={metrics['duration']:.3f} "
        f"path_length={metrics['path_length']:.3f} total_cost={metrics['total_cost']:.3f} "
        f"min_gate_margin={metrics['min_gate_margin']:.4f} "
        f"max_speed={metrics['max_speed']:.3f} max_acceleration={metrics['max_acceleration']:.3f} "
        f"mean_jerk={metrics['mean_jerk']:.3f} optimization_time={metrics['optimization_time']:.3f}"
    )


if __name__ == "__main__":
    main()
