"""Minimal PPO training loop for complete-plan proposals."""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np
import torch

from .environment import PlanningEnvironment
from .model import PhaseActorCritic
from .ppo import PPOBatch, PPOSettings, ppo_update


@dataclass(frozen=True)
class TrainingRecord:
    update: int
    mean_reward: float
    certified_rate: float
    mean_certified_time: float
    loss: float
    approximate_kl: float


def train(
    environment: PlanningEnvironment,
    model: PhaseActorCritic,
    *,
    updates: int = 100,
    batch_size: int = 64,
    learning_rate: float = 3.0e-4,
    settings: PPOSettings | None = None,
    device: str | torch.device = "cpu",
) -> list[TrainingRecord]:
    """Train the network; safety status remains a hard plan-admission outcome."""

    if updates < 1 or batch_size < 2:
        raise ValueError("updates must be positive and batch_size must be at least two")
    target = torch.device(device)
    model.to(target)
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    encoded = environment.reset()
    records = []
    for update_index in range(1, updates + 1):
        old = copy.deepcopy(model).to(target).eval()
        observations = torch.as_tensor(
            np.repeat(encoded[None, :], batch_size, axis=0),
            dtype=torch.float32,
            device=target,
        )
        with torch.no_grad():
            actions, old_log_probabilities, old_values = old.act(observations)
        rewards = []
        accepted = []
        certified_times = []
        for action in actions.cpu().numpy():
            _, reward, _, info = environment.step(action)
            rewards.append(reward)
            accepted.append(bool(info["accepted"]))
            if info["accepted"]:
                certified_times.append(float(info["total_time"]))
        returns = torch.as_tensor(rewards, dtype=torch.float32, device=target)
        metrics = ppo_update(
            model,
            optimizer,
            PPOBatch(
                observations,
                actions.detach(),
                old_log_probabilities.detach(),
                returns - old_values,
                returns,
            ),
            settings,
        )
        records.append(
            TrainingRecord(
                update=update_index,
                mean_reward=float(np.mean(rewards)),
                certified_rate=float(np.mean(accepted)),
                mean_certified_time=(
                    float(np.mean(certified_times))
                    if certified_times
                    else float("inf")
                ),
                loss=metrics["loss"],
                approximate_kl=metrics["approximate_kl"],
            )
        )
    return records

