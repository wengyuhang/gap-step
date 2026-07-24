"""Asymmetric actor-critic: deployable preview actor and privileged critic."""

from __future__ import annotations

import torch
from torch import nn
from torch.distributions import Normal

from .config import ModelConfig


def _mlp(input_dim: int, hidden_dim: int, output_dim: int, layers: int) -> nn.Sequential:
    modules: list[nn.Module] = []
    current = input_dim
    for _ in range(max(1, layers)):
        modules.extend((nn.Linear(current, hidden_dim), nn.Tanh()))
        current = hidden_dim
    modules.append(nn.Linear(current, output_dim))
    return nn.Sequential(*modules)


class AsymmetricActorCritic(nn.Module):
    """PPO policy over a tanh-squashed residual CTBR action."""

    def __init__(
        self,
        actor_obs_dim: int,
        critic_obs_dim: int,
        config: ModelConfig,
        action_dim: int = 4,
    ):
        super().__init__()
        self.actor_obs_dim = int(actor_obs_dim)
        self.critic_obs_dim = int(critic_obs_dim)
        self.action_dim = int(action_dim)
        self.min_log_std = float(config.min_log_std)
        self.max_log_std = float(config.max_log_std)
        self.actor = _mlp(
            self.actor_obs_dim, config.hidden_dim, self.action_dim, config.layers
        )
        self.critic = _mlp(self.critic_obs_dim, config.hidden_dim, 1, config.layers)
        self.log_std = nn.Parameter(
            torch.full((self.action_dim,), float(config.log_std_init))
        )
        self._eps = 1.0e-6
        self._zero_actor_head()

    def _zero_actor_head(self) -> None:
        for module in reversed(self.actor):
            if isinstance(module, nn.Linear):
                nn.init.zeros_(module.weight)
                nn.init.zeros_(module.bias)
                break

    def effective_log_std(self) -> torch.Tensor:
        return torch.clamp(self.log_std, self.min_log_std, self.max_log_std)

    def distribution(self, actor_observation: torch.Tensor) -> Normal:
        mean = self.actor(actor_observation)
        std = torch.exp(self.effective_log_std()).expand_as(mean)
        return Normal(mean, std)

    def value(self, critic_observation: torch.Tensor) -> torch.Tensor:
        return self.critic(critic_observation).squeeze(-1)

    def _log_prob(self, distribution: Normal, raw_action: torch.Tensor) -> torch.Tensor:
        squashed = torch.tanh(raw_action)
        correction = torch.log(1.0 - squashed.square() + self._eps)
        return (distribution.log_prob(raw_action) - correction).sum(dim=-1)

    def sample(
        self,
        actor_observation: torch.Tensor,
        critic_observation: torch.Tensor,
        *,
        deterministic: bool = False,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        distribution = self.distribution(actor_observation)
        raw = distribution.mean if deterministic else distribution.rsample()
        residual = torch.tanh(raw)
        return residual, self._log_prob(distribution, raw), self.value(critic_observation)

    def evaluate_actions(
        self,
        actor_observation: torch.Tensor,
        critic_observation: torch.Tensor,
        residual_actions: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        distribution = self.distribution(actor_observation)
        clipped = torch.clamp(residual_actions, -1.0 + self._eps, 1.0 - self._eps)
        raw = 0.5 * (torch.log1p(clipped) - torch.log1p(-clipped))
        log_prob = self._log_prob(distribution, raw)
        entropy = distribution.entropy().sum(dim=-1).mean()
        return log_prob, entropy, self.value(critic_observation)

