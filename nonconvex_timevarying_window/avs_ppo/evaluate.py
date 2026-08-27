"""Deterministic evaluation and independent safety accounting."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from .config import ExperimentConfig, load_config
from .environment import AVSEnvironment
from .model import MaskedActorCritic
from .ppo import device_from_config


def evaluate_policy(model: MaskedActorCritic, config: ExperimentConfig, episodes: int, *, seed: int = 10000, device: torch.device | None = None) -> dict[str, float]:
    device = device or next(model.parameters()).device
    records = []
    model.eval()
    for episode in range(episodes):
        environment = AVSEnvironment(config.environment, config.shield, seed=seed + episode)
        observation, _ = environment.reset(seed=seed + episode)
        done = False
        info: dict = {}
        while not done:
            mask = environment.action_mask()
            with torch.no_grad():
                distribution = model.distribution(
                    torch.as_tensor(observation[None], dtype=torch.float32, device=device),
                    torch.as_tensor(mask[None], dtype=torch.bool, device=device),
                )
                action = int(torch.argmax(distribution.logits, dim=-1).item())
            observation, _, terminated, truncated, info = environment.step(action, mask=mask)
            done = terminated or truncated
        records.append(info)
    successes = [float(record["success"]) for record in records]
    successful_times = [float(record["elapsed_time"]) for record in records if record["success"]]
    finite_margins = [float(record["minimum_crossing_margin"]) for record in records if np.isfinite(record["minimum_crossing_margin"])]
    return {
        "episodes": episodes,
        "success_rate": float(np.mean(successes)),
        "safety_violation_rate": float(np.mean([record["safety_violations"] > 0 for record in records])),
        "total_safety_violations": int(sum(record["safety_violations"] for record in records)),
        "mean_success_time": float(np.mean(successful_times)) if successful_times else float("nan"),
        "minimum_crossing_margin": min(finite_margins, default=float("nan")),
        "required_margin": config.environment.drone_radius + config.environment.geometry_guard,
    }


def load_checkpoint(path: str | Path, config: ExperimentConfig, device: torch.device) -> MaskedActorCritic:
    probe = AVSEnvironment(config.environment, config.shield)
    model = MaskedActorCritic(probe.observation_dim, probe.action_dim, config.ppo.hidden_size).to(device)
    payload = torch.load(path, map_location=device)
    model.load_state_dict(payload["model"])
    return model


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--seed", type=int, default=10000)
    parser.add_argument("--output")
    args = parser.parse_args()
    config = load_config(args.config)
    device = device_from_config(config.ppo.device)
    metrics = evaluate_policy(load_checkpoint(args.checkpoint, config, device), config, args.episodes, seed=args.seed, device=device)
    encoded = json.dumps(metrics, ensure_ascii=False, indent=2)
    print(encoded)
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        Path(args.output).write_text(encoded + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
