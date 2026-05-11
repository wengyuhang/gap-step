from __future__ import annotations

import torch
from torch import nn
from torch.distributions import Normal


class TeacherActorCritic(nn.Module):
    def __init__(self, obs_dim: int = 39, action_dim: int = 2, max_acc: float = 3.0, hidden_dim: int = 256):
        super().__init__()
        self.max_acc = float(max_acc)
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, action_dim),
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, 1),
        )
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, obs: torch.Tensor) -> dict[str, torch.Tensor]:
        mean = torch.tanh(self.actor(obs)) * self.max_acc
        value = self.critic(obs).squeeze(-1)
        return {"mean": mean, "value": value}

    def distribution(self, obs: torch.Tensor) -> tuple[Normal, torch.Tensor]:
        out = self.forward(obs)
        std = torch.exp(self.log_std).expand_as(out["mean"])
        return Normal(out["mean"], std), out["value"]

    def act(self, obs: torch.Tensor, deterministic: bool = False) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist, value = self.distribution(obs)
        action = dist.mean if deterministic else dist.rsample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        action = torch.clamp(action, -self.max_acc, self.max_acc)
        return action, log_prob, value
