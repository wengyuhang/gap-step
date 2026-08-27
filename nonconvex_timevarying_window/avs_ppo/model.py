"""Masked categorical actor-critic."""

from __future__ import annotations

import torch
from torch import nn
from torch.distributions import Categorical


class MaskedActorCritic(nn.Module):
    def __init__(self, observation_dim: int, action_dim: int, hidden_size: int = 96):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(observation_dim, hidden_size), nn.Tanh(),
            nn.Linear(hidden_size, hidden_size), nn.Tanh(),
        )
        self.actor = nn.Linear(hidden_size, action_dim)
        self.critic = nn.Linear(hidden_size, 1)
        # Exact uniform initial policy.  Deterministic evaluation therefore
        # selects the indexed backup action, giving a reproducible 0-update baseline.
        nn.init.zeros_(self.actor.weight)
        nn.init.zeros_(self.actor.bias)

    def distribution(self, observations: torch.Tensor, masks: torch.Tensor) -> Categorical:
        logits = self.actor(self.encoder(observations))
        logits = logits.masked_fill(~masks.bool(), torch.finfo(logits.dtype).min)
        return Categorical(logits=logits)

    def value(self, observations: torch.Tensor) -> torch.Tensor:
        return self.critic(self.encoder(observations)).squeeze(-1)

    def sample(self, observations: torch.Tensor, masks: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        distribution = self.distribution(observations, masks)
        actions = distribution.sample()
        return actions, distribution.log_prob(actions), self.value(observations)

    def evaluate(self, observations: torch.Tensor, masks: torch.Tensor, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        distribution = self.distribution(observations, masks)
        return distribution.log_prob(actions), distribution.entropy(), self.value(observations)
