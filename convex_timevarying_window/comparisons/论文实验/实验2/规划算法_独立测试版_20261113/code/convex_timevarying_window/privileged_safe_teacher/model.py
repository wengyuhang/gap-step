"""Small deterministic direct-control teacher network."""

from __future__ import annotations

import torch
from torch import nn


class DirectControlTeacher(nn.Module):
    def __init__(self, observation_dim: int = 34, hidden_dim: int = 256):
        super().__init__()
        self.register_buffer("observation_mean", torch.zeros(observation_dim))
        self.register_buffer("observation_std", torch.ones(observation_dim))
        self.network = nn.Sequential(
            nn.Linear(observation_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, 4),
            nn.Tanh(),
        )

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        normalized = (observation - self.observation_mean) / self.observation_std
        return self.network(normalized)

    def set_normalizer(self, mean: torch.Tensor, std: torch.Tensor) -> None:
        self.observation_mean.copy_(mean)
        self.observation_std.copy_(torch.clamp(std, min=1.0e-4))
