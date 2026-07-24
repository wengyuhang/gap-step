"""Render one deterministic FAPP-PPO rollout as a 3D PNG."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .environment import ClosedLoopWindowEnv
from .evaluate import load_policy, run_episode


def visualize_checkpoint(
    checkpoint: str | Path,
    output: str | Path,
    *,
    stage: str = "full",
    seed: int = 2026,
    device: str = "auto",
) -> dict:
    model, config, resolved_device, _ = load_policy(checkpoint, device)
    environment = ClosedLoopWindowEnv(
        config.environment, config.quadrotor, stage=stage, seed=seed
    )
    result, trajectory = run_episode(
        model,
        environment,
        resolved_device,
        seed=seed,
        record_trajectory=True,
    )
    positions = np.array([[row["x"], row["y"], row["z"]] for row in trajectory])
    figure = plt.figure(figsize=(10, 8))
    axis = figure.add_subplot(111, projection="3d")
    if len(positions):
        axis.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="#1565c0", linewidth=2.0)
        axis.scatter(*positions[0], color="#2e7d32", s=55, label="start")
        axis.scatter(*positions[-1], color="#c62828", s=55, label="end")
    for index, window in enumerate(environment.scenario.windows):
        crossing = next(
            (
                record
                for record in environment.crossing_records
                if int(record["window_index"]) == index
            ),
            None,
        )
        time = float(crossing["time"]) if crossing is not None else min(
            environment.time, environment.scenario.horizon
        )
        state = window.state(time)
        local = np.column_stack(
            (state.boundary, np.zeros(len(state.boundary)))
        )
        world = state.center[None, :] + local @ state.rotation.T
        world = np.vstack((world, world[0]))
        axis.plot(world[:, 0], world[:, 1], world[:, 2], color="#ef6c00", linewidth=1.8)
        axis.text(*state.center, window.name)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_zlabel("z [m]")
    axis.set_title(
        f"FAPP-PPO | stage={stage} | success={result['success']} | "
        f"time={result['time']:.2f}s"
    )
    axis.legend(loc="upper right")
    axis.set_box_aspect((1.0, 1.0, 0.45))
    figure.tight_layout()
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", default="closed_loop_deformable_window/fapp_ppo/results/rollout.png")
    parser.add_argument("--stage", choices=("static", "moving", "deforming", "full"), default="full")
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    result = visualize_checkpoint(
        args.checkpoint,
        args.output,
        stage=args.stage,
        seed=args.seed,
        device=args.device,
    )
    print(
        f"已保存 {args.output} | success={result['success']} | "
        f"crossings={result['crossings']}"
    )


if __name__ == "__main__":
    main()

