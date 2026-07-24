from __future__ import annotations

import numpy as np
import torch

from closed_loop_deformable_window.fapp_ppo.config import (
    CurriculumStage,
    ExperimentConfig,
)
from closed_loop_deformable_window.fapp_ppo.environment import ClosedLoopWindowEnv
from closed_loop_deformable_window.fapp_ppo.model import AsymmetricActorCritic
from closed_loop_deformable_window.fapp_ppo.ppo import (
    collect_rollout,
    get_device,
    ppo_update,
    sync_old_policy,
)
from closed_loop_deformable_window.fapp_ppo.train import train_model


def test_observation_is_asymmetric_and_action_is_bounded():
    config = ExperimentConfig()
    environment = ClosedLoopWindowEnv(
        config.environment, config.quadrotor, stage="full", seed=3
    )
    observation, _ = environment.reset(seed=3)
    assert observation.critic.shape[0] > observation.actor.shape[0]
    assert np.all(np.isfinite(observation.actor))
    action = environment.compose_action(np.full(4, 10.0))
    assert np.all(action <= 1.0) and np.all(action >= -1.0)


def test_nominal_controller_completes_closed_loop_on_reference_seed():
    config = ExperimentConfig()
    environment = ClosedLoopWindowEnv(
        config.environment, config.quadrotor, stage="full", seed=0
    )
    environment.reset(seed=0)
    while True:
        _, _, terminated, truncated, info = environment.step(
            environment.compose_action(np.zeros(4))
        )
        if terminated or truncated:
            break
    assert info["success"]
    assert info["progress_index"] == config.environment.full_windows
    assert info["position_error"] <= config.environment.return_position_tolerance
    assert info["velocity_error"] <= config.environment.return_velocity_tolerance
    assert info["attitude_error"] <= config.environment.return_attitude_tolerance
    assert info["rate_error"] <= config.environment.return_rate_tolerance


def test_ppo_uses_nonnegative_kl_and_syncs_old_policy():
    config = ExperimentConfig()
    config.ppo.rollout_steps = 8
    config.ppo.num_envs = 2
    config.ppo.update_epochs = 1
    config.ppo.minibatch_size = 8
    environments = [
        ClosedLoopWindowEnv(
            config.environment, config.quadrotor, stage="static", seed=index
        )
        for index in range(2)
    ]
    actor_dim = environments[0].actor_obs_dim
    critic_dim = environments[0].critic_obs_dim
    model = AsymmetricActorCritic(actor_dim, critic_dim, config.model)
    model_old = AsymmetricActorCritic(actor_dim, critic_dim, config.model)
    sync_old_policy(model, model_old)
    batch, _ = collect_rollout(
        environments, model_old, config.ppo.rollout_steps, get_device("cpu")
    )
    optimizer = torch.optim.Adam(model.parameters(), lr=config.ppo.learning_rate)
    metrics = ppo_update(model, optimizer, batch, config.ppo, get_device("cpu"))
    assert metrics["approx_kl"] >= -1.0e-8
    sync_old_policy(model, model_old)
    for current, old in zip(model.parameters(), model_old.parameters()):
        assert torch.equal(current, old)


def test_tiny_training_writes_reloadable_checkpoint(tmp_path):
    config = ExperimentConfig()
    config.model.hidden_dim = 32
    config.model.layers = 1
    config.ppo.rollout_steps = 8
    config.ppo.num_envs = 1
    config.ppo.update_epochs = 1
    config.ppo.minibatch_size = 8
    config.train.device = "cpu"
    config.train.curriculum = [CurriculumStage("static", 1)]
    config.train.save_every = 10
    checkpoint, rows = train_model(config, output_dir=tmp_path / "run")
    assert checkpoint.is_file()
    assert len(rows) == 1
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    assert payload["update"] == 1
    assert payload["stage"] == "static"


def test_opportunity_observation_and_step_are_finite():
    config = ExperimentConfig()
    config.environment.episode_seconds = 22.0
    config.environment.route_radius = 5.2
    config.environment.workspace_radius = 14.0
    config.environment.opportunity_mode = "single_shot"
    config.environment.opportunity_features = True
    config.environment.opportunity_aware_nominal = True
    config.environment.opportunity_width = 1.1
    config.environment.opportunity_closed_scale = 0.16
    config.environment.opportunity_open_scale = 1.05
    config.environment.motion_amplitude_multiplier = 1.8
    config.environment.deformation_amplitude_multiplier = 2.0
    environment = ClosedLoopWindowEnv(
        config.environment, config.quadrotor, stage="full", seed=51
    )
    observation, _ = environment.reset(seed=51)
    assert np.all(np.isfinite(observation.actor))
    assert np.all(np.isfinite(observation.critic))
    assert environment.critic_obs_dim > environment.actor_obs_dim
    next_observation, reward, _, _, info = environment.step(
        environment.compose_action(np.zeros(4))
    )
    assert np.all(np.isfinite(next_observation.actor))
    assert np.all(np.isfinite(next_observation.critic))
    assert np.isfinite(reward)
    assert not info["opportunity_passable"]
