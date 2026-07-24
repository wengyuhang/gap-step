"""Deterministic checkpoint evaluation for FAPP-PPO."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np
import torch

from .config import config_from_dict, validate_config
from .environment import ClosedLoopWindowEnv
from .model import AsymmetricActorCritic
from .ppo import get_device


def load_policy(
    checkpoint_path: str | Path,
    requested_device: str = "auto",
) -> tuple[AsymmetricActorCritic, Any, torch.device, dict]:
    device = get_device(requested_device)
    checkpoint = torch.load(
        Path(checkpoint_path), map_location=device, weights_only=False
    )
    config = config_from_dict(checkpoint["config"])
    validate_config(config)
    model = AsymmetricActorCritic(
        int(checkpoint["actor_obs_dim"]),
        int(checkpoint["critic_obs_dim"]),
        config.model,
    ).to(device)
    model.load_state_dict(checkpoint["model_state"])
    model.eval()
    return model, config, device, checkpoint


def run_episode(
    model: AsymmetricActorCritic,
    environment: ClosedLoopWindowEnv,
    device: torch.device,
    *,
    seed: int,
    record_trajectory: bool = False,
) -> tuple[dict, list[dict]]:
    observation, reset_info = environment.reset(seed=seed)
    total_reward = 0.0
    trajectory: list[dict] = []
    final_info: dict = {}
    while True:
        actor = torch.as_tensor(
            observation.actor[None, :], dtype=torch.float32, device=device
        )
        critic = torch.as_tensor(
            observation.critic[None, :], dtype=torch.float32, device=device
        )
        with torch.no_grad():
            residual, _, _ = model.sample(actor, critic, deterministic=True)
        action = environment.compose_action(residual.squeeze(0).cpu().numpy())
        observation, reward, terminated, truncated, final_info = environment.step(action)
        total_reward += reward
        if record_trajectory:
            trajectory.append(
                {
                    "time": environment.time,
                    "x": float(environment.state.position[0]),
                    "y": float(environment.state.position[1]),
                    "z": float(environment.state.position[2]),
                    "vx": float(environment.state.velocity[0]),
                    "vy": float(environment.state.velocity[1]),
                    "vz": float(environment.state.velocity[2]),
                    "progress_index": environment.progress_index,
                    "action": action.tolist(),
                    "rotation": environment.state.rotation.tolist(),
                    "body_rate": environment.state.body_rate.tolist(),
                }
            )
        if terminated or truncated:
            break
    result = {
        **reset_info,
        **final_info,
        "episode_reward": float(total_reward),
        "crossings": len(environment.crossing_records),
        "crossing_records": list(environment.crossing_records),
    }
    return result, trajectory


def evaluate_checkpoint(
    checkpoint_path: str | Path,
    *,
    episodes: int,
    stage: str,
    seed: int,
    device: str = "auto",
) -> tuple[dict, list[dict]]:
    model, config, resolved_device, _ = load_policy(checkpoint_path, device)
    environment = ClosedLoopWindowEnv(
        config.environment,
        config.quadrotor,
        stage=stage,
        seed=seed,
    )
    records = [
        run_episode(
            model,
            environment,
            resolved_device,
            seed=seed + episode_index,
        )[0]
        for episode_index in range(episodes)
    ]
    successes = [bool(record["success"]) for record in records]
    successful_times = [
        float(record["time"]) for record in records if bool(record["success"])
    ]
    summary = {
        "checkpoint": str(checkpoint_path),
        "stage": stage,
        "episodes": episodes,
        "successes": int(sum(successes)),
        "success_rate": float(np.mean(successes)),
        "mean_time_success": float(np.mean(successful_times))
        if successful_times
        else None,
        "mean_crossings": float(np.mean([record["crossings"] for record in records])),
        "collision_rate": float(
            np.mean(
                [
                    record["failure"] in {"window_collision", "floor_collision"}
                    for record in records
                ]
            )
        ),
        "order_violation_rate": float(
            np.mean([record["failure"] == "order_violation" for record in records])
        ),
    }
    return summary, records


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--stage", choices=("static", "moving", "deforming", "full"), default="full")
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--outdir", default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary, records = evaluate_checkpoint(
        args.checkpoint,
        episodes=args.episodes,
        stage=args.stage,
        seed=args.seed,
        device=args.device,
    )
    if args.outdir is not None:
        output = Path(args.outdir)
        output.mkdir(parents=True, exist_ok=True)
        (output / "evaluation_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        flat_records = [
            {key: value for key, value in record.items() if key != "crossing_records"}
            for record in records
        ]
        if flat_records:
            with (output / "evaluation_episodes.csv").open(
                "w", encoding="utf-8", newline=""
            ) as stream:
                writer = csv.DictWriter(stream, fieldnames=list(flat_records[0].keys()))
                writer.writeheader()
                writer.writerows(flat_records)
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
