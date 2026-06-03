from __future__ import annotations

import argparse

from .environment import demo_track
from .planner import DynamicTOGTPlanner, PlannerConfig


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone TOGT-style planner for dynamic gates/windows.")
    parser.add_argument("--static", action="store_true", help="Disable gate position/shape changes.")
    parser.add_argument("--max-time", type=float, default=70.0)
    args = parser.parse_args()

    track = demo_track(dynamic=not args.static)
    planner = DynamicTOGTPlanner(PlannerConfig(max_time=args.max_time, max_speed=2.35, wait_steps=8, gate_samples_per_axis=1))
    traj = planner.plan(track)
    if traj is None:
        print("planning_failed")
        return
    gate_summary = ",".join(f"{t:.2f}" for t in traj.gate_times)
    print(
        f"track={track.name} dynamic={not args.static} "
        f"lap_time={traj.lap_time:.2f} length={traj.length:.2f} gates={len(track.gates)} gate_times=[{gate_summary}]"
    )


if __name__ == "__main__":
    main()
