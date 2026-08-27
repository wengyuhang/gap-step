"""Configuration objects and YAML loading for AVS-PPO."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class EnvironmentConfig:
    dt: float = 0.10
    max_steps: int = 105
    gate_count: int = 3
    drone_radius: float = 0.16
    geometry_guard: float = 0.035
    max_accel_x: float = 3.2
    max_accel_yz: float = 4.0
    max_speed_x: float = 3.0
    max_speed_yz: float = 2.6
    goal_x: float = 10.8
    goal_tolerance: float = 0.35
    motion_scale: float = 1.0
    domain_randomization: float = 0.12


@dataclass
class ShieldConfig:
    backup_horizon_steps: int = 36
    backup_brake_accel: float = 3.0
    lateral_kp: float = 4.2
    lateral_kd: float = 2.8


@dataclass
class PPOConfig:
    seed: int = 7
    device: str = "auto"
    num_envs: int = 8
    rollout_steps: int = 128
    updates: int = 60
    learning_rate: float = 0.0007
    gamma: float = 0.995
    gae_lambda: float = 0.95
    clip_ratio: float = 0.20
    entropy_coef: float = 0.012
    value_coef: float = 0.5
    max_grad_norm: float = 0.7
    update_epochs: int = 4
    minibatch_size: int = 256
    target_kl: float = 0.025
    hidden_size: int = 96
    eval_interval: int = 10
    eval_episodes: int = 40


@dataclass
class ExperimentConfig:
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    shield: ShieldConfig = field(default_factory=ShieldConfig)
    ppo: PPOConfig = field(default_factory=PPOConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def load_config(path: str | Path) -> ExperimentConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return ExperimentConfig(
        environment=EnvironmentConfig(**raw.get("environment", {})),
        shield=ShieldConfig(**raw.get("shield", {})),
        ppo=PPOConfig(**raw.get("ppo", {})),
    )
