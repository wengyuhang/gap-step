"""Configuration objects and YAML loading for FAPP-PPO."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class QuadrotorConfig:
    mass: float = 1.0
    gravity: float = 9.81
    inertia: tuple[float, float, float] = (0.005, 0.005, 0.009)
    arm_length: float = 0.15
    yaw_moment_coefficient: float = 0.012
    rotor_thrust_min: float = 0.0
    rotor_thrust_max: float = 5.5
    max_body_rate: float = 5.0
    body_rate_time_constant: float = 0.07
    linear_drag: float = 0.12


@dataclass
class EnvironmentConfig:
    dt: float = 0.04
    episode_seconds: float = 18.0
    max_windows: int = 6
    full_windows: int = 4
    route_radius: float = 4.5
    preview_horizons: tuple[float, ...] = (0.0, 0.6, 1.2)
    preview_gate_count: int = 2
    safe_margin: float = 0.16
    frame_thickness: float = 0.08
    drone_radius: float = 0.10
    min_window_separation: float = 0.8
    workspace_radius: float = 12.0
    floor_height: float = 0.15
    cruise_speed: float = 2.6
    residual_scale: float = 0.20
    return_position_tolerance: float = 0.35
    return_velocity_tolerance: float = 0.40
    return_attitude_tolerance: float = 0.25
    return_rate_tolerance: float = 0.50
    reward_time: float = 1.0
    reward_progress: float = 1.5
    reward_gate: float = 8.0
    reward_success: float = 25.0
    reward_collision: float = -18.0
    reward_order_violation: float = -20.0
    reward_timeout: float = -10.0
    reward_smoothness: float = 0.025
    reward_energy: float = 0.003
    reward_missed_opportunity: float = -4.0
    motion_amplitude_multiplier: float = 1.0
    deformation_amplitude_multiplier: float = 1.0
    opportunity_mode: str = "always_open"
    opportunity_features: bool = False
    opportunity_aware_nominal: bool = False
    critic_time_feature: bool = True
    critic_privileged_route: bool = True
    opportunity_width: float = 1.2
    opportunity_transition: float = 0.30
    opportunity_closed_scale: float = 0.22
    opportunity_open_scale: float = 1.0
    opportunity_schedule_jitter: float = 0.0
    # Mean recurrence between independently generated opportunities.
    opportunity_rescue_delay: float = 4.0
    opportunity_holding_distance: float = 0.9
    opportunity_crossing_overshoot: float = 0.55
    minimum_safe_area: float = 1.0e-4

    @property
    def max_steps(self) -> int:
        return int(round(self.episode_seconds / self.dt))


@dataclass
class ModelConfig:
    hidden_dim: int = 192
    layers: int = 3
    log_std_init: float = -1.0
    min_log_std: float = -2.5
    max_log_std: float = 0.2


@dataclass
class PPOConfig:
    learning_rate: float = 3.0e-4
    rollout_steps: int = 512
    num_envs: int = 8
    update_epochs: int = 6
    minibatch_size: int = 512
    gamma: float = 0.995
    gae_lambda: float = 0.95
    clip_ratio: float = 0.2
    value_coef: float = 0.5
    entropy_coef: float = 0.002
    residual_prior_coef: float = 0.05
    max_grad_norm: float = 0.8
    target_kl: float = 0.02


@dataclass
class CurriculumStage:
    name: str
    updates: int
    opportunity_mode: str | None = None
    opportunity_width: float | None = None
    motion_amplitude_multiplier: float | None = None
    deformation_amplitude_multiplier: float | None = None
    opportunity_schedule_jitter: float | None = None


def _default_curriculum() -> list[CurriculumStage]:
    return [
        CurriculumStage("static", 40),
        CurriculumStage("moving", 60),
        CurriculumStage("deforming", 100),
        CurriculumStage("full", 200),
    ]


@dataclass
class TrainConfig:
    seed: int = 7
    device: str = "auto"
    output_dir: str = "closed_loop_deformable_window/fapp_ppo/runs/default"
    curriculum: list[CurriculumStage] = field(default_factory=_default_curriculum)
    save_every: int = 25


@dataclass
class ExperimentConfig:
    quadrotor: QuadrotorConfig = field(default_factory=QuadrotorConfig)
    environment: EnvironmentConfig = field(default_factory=EnvironmentConfig)
    model: ModelConfig = field(default_factory=ModelConfig)
    ppo: PPOConfig = field(default_factory=PPOConfig)
    train: TrainConfig = field(default_factory=TrainConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _construct(dataclass_type: type, values: dict[str, Any] | None):
    values = {} if values is None else dict(values)
    if dataclass_type is EnvironmentConfig and "preview_horizons" in values:
        values["preview_horizons"] = tuple(float(x) for x in values["preview_horizons"])
    if dataclass_type is EnvironmentConfig:
        # Legacy checkpoints used this to align openings with nominal arrival.
        # It is intentionally discarded by the independent exogenous generator.
        values.pop("opportunity_reference_speed", None)
    if dataclass_type is QuadrotorConfig and "inertia" in values:
        values["inertia"] = tuple(float(x) for x in values["inertia"])
    return dataclass_type(**values)


def config_from_dict(payload: dict[str, Any]) -> ExperimentConfig:
    train_values = dict(payload.get("train", {}))
    if "curriculum" in train_values:
        train_values["curriculum"] = [
            item if isinstance(item, CurriculumStage) else CurriculumStage(**item)
            for item in train_values["curriculum"]
        ]
    return ExperimentConfig(
        quadrotor=_construct(QuadrotorConfig, payload.get("quadrotor")),
        environment=_construct(EnvironmentConfig, payload.get("environment")),
        model=_construct(ModelConfig, payload.get("model")),
        ppo=_construct(PPOConfig, payload.get("ppo")),
        train=_construct(TrainConfig, train_values),
    )


def load_config(path: str | Path) -> ExperimentConfig:
    with Path(path).open("r", encoding="utf-8") as stream:
        payload = yaml.safe_load(stream) or {}
    if not isinstance(payload, dict):
        raise ValueError("configuration root must be a mapping")
    return config_from_dict(payload)


def validate_config(config: ExperimentConfig) -> None:
    env = config.environment
    ppo = config.ppo
    if env.dt <= 0.0 or env.episode_seconds <= env.dt:
        raise ValueError("dt and episode_seconds must define a non-empty episode")
    if env.full_windows < 1 or env.full_windows > env.max_windows:
        raise ValueError("full_windows must be in [1, max_windows]")
    if not env.preview_horizons or min(env.preview_horizons) < 0.0:
        raise ValueError("preview_horizons must be non-empty and non-negative")
    if env.safe_margin <= env.drone_radius:
        raise ValueError("safe_margin must exceed drone_radius")
    if not 0.0 < env.residual_scale <= 1.0:
        raise ValueError("residual_scale must lie in (0, 1]")
    if env.motion_amplitude_multiplier <= 0.0:
        raise ValueError("motion_amplitude_multiplier must be positive")
    if env.deformation_amplitude_multiplier <= 0.0:
        raise ValueError("deformation_amplitude_multiplier must be positive")
    if env.opportunity_mode not in {"always_open", "single_shot", "irregular_repeated"}:
        raise ValueError(
            "opportunity_mode must be always_open, single_shot, or irregular_repeated"
        )
    if env.opportunity_width <= 2.0 * env.dt:
        raise ValueError("opportunity_width must exceed two simulation steps")
    if env.opportunity_transition <= 0.0:
        raise ValueError("opportunity_transition must be positive")
    if env.opportunity_schedule_jitter < 0.0:
        raise ValueError("opportunity_schedule_jitter must be non-negative")
    if env.opportunity_rescue_delay <= 0.0:
        raise ValueError("opportunity_rescue_delay must be positive")
    if not 0.0 < env.opportunity_closed_scale < env.opportunity_open_scale:
        raise ValueError("opportunity scales must satisfy 0 < closed < open")
    if env.opportunity_holding_distance <= 0.0:
        raise ValueError("opportunity_holding_distance must be positive")
    if env.opportunity_crossing_overshoot <= 0.0:
        raise ValueError("opportunity_crossing_overshoot must be positive")
    if env.minimum_safe_area < 0.0:
        raise ValueError("minimum_safe_area must be non-negative")
    if ppo.rollout_steps < 2 or ppo.num_envs < 1:
        raise ValueError("PPO rollout dimensions are invalid")
    if ppo.minibatch_size < 1:
        raise ValueError("minibatch_size must be positive")
    if ppo.residual_prior_coef < 0.0:
        raise ValueError("residual_prior_coef must be non-negative")
    if not config.train.curriculum or any(stage.updates < 1 for stage in config.train.curriculum):
        raise ValueError("curriculum must contain positive update counts")
    for stage in config.train.curriculum:
        if (
            stage.opportunity_mode is not None
            and stage.opportunity_mode
            not in {"always_open", "single_shot", "irregular_repeated"}
        ):
            raise ValueError(f"invalid curriculum opportunity mode {stage.opportunity_mode!r}")
        if stage.opportunity_width is not None and stage.opportunity_width <= 2.0 * env.dt:
            raise ValueError("curriculum opportunity widths must exceed two steps")
        if (
            stage.motion_amplitude_multiplier is not None
            and stage.motion_amplitude_multiplier <= 0.0
        ):
            raise ValueError("curriculum motion multipliers must be positive")
        if (
            stage.deformation_amplitude_multiplier is not None
            and stage.deformation_amplitude_multiplier <= 0.0
        ):
            raise ValueError("curriculum deformation multipliers must be positive")
