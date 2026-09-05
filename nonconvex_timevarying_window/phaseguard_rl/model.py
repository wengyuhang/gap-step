"""Small continuous actor-critic and phase observation encoder."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin

import numpy as np
import torch
from torch import nn
from torch.distributions import Normal

from nonconvex_timevarying_window.sip_dynatogt.model import SIPProblem


@dataclass(frozen=True)
class VehicleState:
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    body_rate: np.ndarray
    time: float = 0.0
    next_gate: int = 0

    def __post_init__(self) -> None:
        p = np.asarray(self.position, dtype=float)
        v = np.asarray(self.velocity, dtype=float)
        r = np.asarray(self.rotation, dtype=float)
        w = np.asarray(self.body_rate, dtype=float)
        if p.shape != (3,) or v.shape != (3,) or r.shape != (3, 3) or w.shape != (3,):
            raise ValueError("vehicle state shapes must be (3,), (3,), (3,3), (3,)")
        if not np.all(np.isfinite(np.concatenate((p, v, r.ravel(), w)))) or not np.isfinite(self.time):
            raise ValueError("vehicle state must be finite")
        object.__setattr__(self, "position", p.copy())
        object.__setattr__(self, "velocity", v.copy())
        object.__setattr__(self, "rotation", r.copy())
        object.__setattr__(self, "body_rate", w.copy())


def _phase(period: float, base: float, time: float) -> tuple[float, float]:
    angle = 2.0 * pi * float(time) / float(period) + float(base)
    return sin(angle), cos(angle)


def observation(problem: SIPProblem, state: VehicleState, *, length_scale: float = 10.0) -> np.ndarray:
    """Encode current state and the three motion phases of every ordered gate."""

    if length_scale <= 0:
        raise ValueError("length_scale must be positive")
    features = [
        *(state.position / length_scale),
        *(state.velocity / length_scale),
        *state.rotation.ravel(),
        *state.body_rate,
        state.time / 10.0,
    ]
    one_hot = np.zeros(len(problem.order) + 1)
    one_hot[min(max(int(state.next_gate), 0), len(problem.order))] = 1.0
    features.extend(one_hot)
    for order_position, window_index in enumerate(problem.order):
        window = problem.windows[window_index]
        center, rotation, scale = window.state_at(state.time)
        motion = window.motion
        features.extend((center - state.position) / length_scale)
        features.extend(rotation[:, 2])
        features.append(scale)
        features.extend(_phase(motion.translation_period, motion.phase, state.time))
        features.extend(_phase(motion.rotation_period, motion.phase, state.time))
        features.extend(_phase(motion.scale_period, motion.phase, state.time))
        features.append(float(order_position == state.next_gate))
    return np.asarray(features, dtype=np.float32)


class PhaseActorCritic(nn.Module):
    """Two-layer Gaussian actor and value head.

    The squashed action lies in [-1, 1].  The planner maps it to local crossing
    coordinates and strictly positive segment durations.
    """

    def __init__(self, observation_dim: int, action_dim: int, hidden_size: int = 128):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(observation_dim, hidden_size),
            nn.Tanh(),
            nn.Linear(hidden_size, hidden_size),
            nn.Tanh(),
        )
        self.mean = nn.Linear(hidden_size, action_dim)
        self.log_std = nn.Parameter(torch.full((action_dim,), -0.5))
        self.value_head = nn.Linear(hidden_size, 1)

    def _normal(self, observations: torch.Tensor) -> tuple[Normal, torch.Tensor]:
        latent = self.encoder(observations)
        std = self.log_std.clamp(-5.0, 1.0).exp().expand_as(self.mean(latent))
        return Normal(self.mean(latent), std), latent

    @staticmethod
    def _squashed_log_prob(distribution: Normal, raw: torch.Tensor, action: torch.Tensor) -> torch.Tensor:
        correction = torch.log(torch.clamp(1.0 - action.square(), min=1.0e-6))
        return (distribution.log_prob(raw) - correction).sum(dim=-1)

    def act(
        self, observations: torch.Tensor, *, deterministic: bool = False
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        distribution, latent = self._normal(observations)
        raw = distribution.mean if deterministic else distribution.rsample()
        action = torch.tanh(raw)
        log_prob = self._squashed_log_prob(distribution, raw, action)
        return action, log_prob, self.value_head(latent).squeeze(-1)

    def evaluate_actions(
        self, observations: torch.Tensor, actions: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        distribution, latent = self._normal(observations)
        clipped = actions.clamp(-1.0 + 1.0e-6, 1.0 - 1.0e-6)
        raw = torch.atanh(clipped)
        log_prob = self._squashed_log_prob(distribution, raw, clipped)
        entropy = distribution.entropy().sum(dim=-1)
        return log_prob, entropy, self.value_head(latent).squeeze(-1)

