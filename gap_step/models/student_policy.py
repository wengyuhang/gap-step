from __future__ import annotations

import torch
from torch import nn
from torch.distributions import Normal

from gap_step.models.cnn_encoder import CNNEncoder


class StudentPolicy(nn.Module):
    def __init__(self, k_obs: int, proprio_dim: int, num_gates: int, max_acc: float, latent_dim: int = 128):
        super().__init__()
        self.num_gates = int(num_gates)
        self.max_acc = float(max_acc)
        self.encoder = CNNEncoder(k_obs, out_dim=latent_dim)
        self.proprio_net = nn.Sequential(
            nn.Linear(proprio_dim, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, 64),
            nn.ReLU(inplace=True),
        )
        self.trunk = nn.Sequential(
            nn.Linear(latent_dim + 64, latent_dim),
            nn.ReLU(inplace=True),
            nn.Linear(latent_dim, latent_dim),
            nn.ReLU(inplace=True),
        )
        self.acc_head = nn.Linear(latent_dim, 2)
        self.gate_head = nn.Linear(latent_dim, num_gates)
        self.width_head = nn.Linear(latent_dim, num_gates)
        self.safe_head = nn.Linear(latent_dim, num_gates)
        self.value_head = nn.Linear(latent_dim, 1)
        self.log_std = nn.Parameter(torch.zeros(2))

    def encode(self, image_stack: torch.Tensor, proprio: torch.Tensor) -> torch.Tensor:
        visual = self.encoder(image_stack)
        prop = self.proprio_net(proprio)
        return self.trunk(torch.cat([visual, prop], dim=-1))

    def forward(self, image_stack: torch.Tensor, proprio: torch.Tensor) -> dict[str, torch.Tensor]:
        z = self.encode(image_stack, proprio)
        acc = torch.tanh(self.acc_head(z)) * self.max_acc
        return {
            "acc": acc,
            "gate_logits": self.gate_head(z),
            "width": torch.nn.functional.softplus(self.width_head(z)),
            "safe_logits": self.safe_head(z),
            "value": self.value_head(z).squeeze(-1),
        }

    def distribution(self, image_stack: torch.Tensor, proprio: torch.Tensor) -> tuple[Normal, dict[str, torch.Tensor]]:
        out = self.forward(image_stack, proprio)
        std = torch.exp(self.log_std).expand_as(out["acc"])
        return Normal(out["acc"], std), out

    def act(self, image_stack: torch.Tensor, proprio: torch.Tensor, deterministic: bool = False):
        dist, out = self.distribution(image_stack, proprio)
        action = out["acc"] if deterministic else dist.rsample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        action = torch.clamp(action, -self.max_acc, self.max_acc)
        return action, log_prob, out
