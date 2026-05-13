from __future__ import annotations

import torch
from torch import nn
from torch.distributions import Normal


class TeacherActorCritic(nn.Module):
    def __init__(
        self,
        obs_dim: int = 161,
        action_dim: int = 2,
        max_acc: float = 3.0,
        hidden_dim: int = 256,
        min_log_std: float = -1.0,
        max_log_std: float = 2.0,
    ):
        super().__init__()
        self.max_acc = float(max_acc)
        self.min_log_std = float(min_log_std)
        self.max_log_std = float(max_log_std)
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
        self._eps = 1e-6

    def forward(self, obs: torch.Tensor) -> dict[str, torch.Tensor]:
        mean = torch.tanh(self.actor(obs)) * self.max_acc
        value = self.critic(obs).squeeze(-1)
        return {"mean": mean, "value": value}

    def distribution(self, obs: torch.Tensor) -> tuple[Normal, torch.Tensor]:
        raw_mean = self.actor(obs)
        value = self.critic(obs).squeeze(-1)
        std = torch.exp(self.effective_log_std()).expand_as(raw_mean)
        return Normal(raw_mean, std), value

    def effective_log_std(self) -> torch.Tensor:
        return torch.clamp(self.log_std, self.min_log_std, self.max_log_std)

    def _squash(self, raw_action: torch.Tensor) -> torch.Tensor:
        return torch.tanh(raw_action) * self.max_acc

    def _atanh_scaled_action(self, action: torch.Tensor) -> torch.Tensor:
        scaled = torch.clamp(action / self.max_acc, -1.0 + self._eps, 1.0 - self._eps)
        return 0.5 * (torch.log1p(scaled) - torch.log1p(-scaled))

    def _squashed_log_prob(self, dist: Normal, raw_action: torch.Tensor) -> torch.Tensor:
        scaled = torch.tanh(raw_action)
        log_det = torch.log(self.max_acc * (1.0 - scaled.pow(2)) + self._eps)
        return (dist.log_prob(raw_action) - log_det).sum(dim=-1)

    def act(self, obs: torch.Tensor, deterministic: bool = False) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist, value = self.distribution(obs)
        raw_action = dist.mean if deterministic else dist.rsample()
        action = self._squash(raw_action)
        log_prob = self._squashed_log_prob(dist, raw_action)
        return action, log_prob, value

    def evaluate_actions(self, obs: torch.Tensor, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        dist, value = self.distribution(obs)
        raw_action = self._atanh_scaled_action(actions)
        log_prob = self._squashed_log_prob(dist, raw_action)
        entropy = dist.entropy().sum(dim=-1).mean()
        return log_prob, entropy, value
