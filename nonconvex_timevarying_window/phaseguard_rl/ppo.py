"""One compact PPO update for the continuous crossing-point policy."""

from __future__ import annotations

from dataclasses import dataclass

import torch

from .model import PhaseActorCritic


@dataclass(frozen=True)
class PPOSettings:
    clip_ratio: float = 0.2
    value_coefficient: float = 0.5
    entropy_coefficient: float = 0.01
    maximum_gradient_norm: float = 1.0
    target_kl: float = 0.02


@dataclass(frozen=True)
class PPOBatch:
    observations: torch.Tensor
    actions: torch.Tensor
    old_log_probabilities: torch.Tensor
    advantages: torch.Tensor
    returns: torch.Tensor


def ppo_update(
    model: PhaseActorCritic,
    optimizer: torch.optim.Optimizer,
    batch: PPOBatch,
    settings: PPOSettings | None = None,
) -> dict[str, float]:
    config = settings or PPOSettings()
    advantages = (batch.advantages - batch.advantages.mean()) / (
        batch.advantages.std(unbiased=False) + 1.0e-8
    )
    log_probabilities, entropy, values = model.evaluate_actions(
        batch.observations, batch.actions
    )
    log_ratio = log_probabilities - batch.old_log_probabilities
    ratio = log_ratio.exp()
    objective = torch.minimum(
        ratio * advantages,
        ratio.clamp(1.0 - config.clip_ratio, 1.0 + config.clip_ratio) * advantages,
    )
    policy_loss = -objective.mean()
    value_loss = 0.5 * (values - batch.returns).square().mean()
    loss = (
        policy_loss
        + config.value_coefficient * value_loss
        - config.entropy_coefficient * entropy.mean()
    )
    optimizer.zero_grad(set_to_none=True)
    loss.backward()
    torch.nn.utils.clip_grad_norm_(model.parameters(), config.maximum_gradient_norm)
    optimizer.step()
    with torch.no_grad():
        approximate_kl = ((log_ratio.exp() - 1.0) - log_ratio).mean()
    return {
        "loss": float(loss.detach()),
        "policy_loss": float(policy_loss.detach()),
        "value_loss": float(value_loss.detach()),
        "entropy": float(entropy.mean().detach()),
        "approximate_kl": float(approximate_kl.detach()),
        "early_stop_recommended": float(approximate_kl > config.target_kl),
    }

