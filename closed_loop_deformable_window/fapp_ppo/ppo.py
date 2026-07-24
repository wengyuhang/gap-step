"""Vector rollout and explicit-old-policy PPO for FAPP-PPO."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch

from .config import PPOConfig
from .environment import ClosedLoopWindowEnv, Observation
from .model import AsymmetricActorCritic


@dataclass
class RolloutBatch:
    actor_observations: np.ndarray
    critic_observations: np.ndarray
    residual_actions: np.ndarray
    log_probs: np.ndarray
    values: np.ndarray
    rewards: np.ndarray
    dones: np.ndarray
    last_values: np.ndarray
    episode_infos: list[dict]
    episode_returns: list[float]


def get_device(requested: str) -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(requested)


def sync_old_policy(model: AsymmetricActorCritic, model_old: AsymmetricActorCritic) -> None:
    model_old.load_state_dict(model.state_dict())
    model_old.eval()


def collect_rollout(
    environments: list[ClosedLoopWindowEnv],
    model_old: AsymmetricActorCritic,
    steps: int,
    device: torch.device,
    observations: list[Observation] | None = None,
) -> tuple[RolloutBatch, list[Observation]]:
    if not environments:
        raise ValueError("at least one environment is required")
    model_old.eval()
    if observations is None:
        observations = [environment.reset()[0] for environment in environments]
    num_envs = len(environments)
    actor_buffer: list[np.ndarray] = []
    critic_buffer: list[np.ndarray] = []
    action_buffer: list[np.ndarray] = []
    logp_buffer: list[np.ndarray] = []
    value_buffer: list[np.ndarray] = []
    reward_buffer: list[np.ndarray] = []
    done_buffer: list[np.ndarray] = []
    episode_infos: list[dict] = []
    episode_returns: list[float] = []
    running_returns = np.zeros(num_envs, dtype=float)

    for _ in range(steps):
        actor_array = np.stack([observation.actor for observation in observations])
        critic_array = np.stack([observation.critic for observation in observations])
        actor_tensor = torch.as_tensor(actor_array, dtype=torch.float32, device=device)
        critic_tensor = torch.as_tensor(critic_array, dtype=torch.float32, device=device)
        with torch.no_grad():
            residual_tensor, logp_tensor, value_tensor = model_old.sample(
                actor_tensor, critic_tensor
            )
        residual_array = residual_tensor.cpu().numpy()
        next_observations: list[Observation] = []
        rewards = np.zeros(num_envs, dtype=np.float32)
        dones = np.zeros(num_envs, dtype=np.float32)
        for index, environment in enumerate(environments):
            applied_action = environment.compose_action(residual_array[index])
            next_observation, reward, terminated, truncated, info = environment.step(
                applied_action
            )
            done = terminated or truncated
            rewards[index] = reward
            dones[index] = float(done)
            running_returns[index] += reward
            if done:
                record = dict(info)
                record["episode_return"] = float(running_returns[index])
                episode_infos.append(record)
                episode_returns.append(float(running_returns[index]))
                running_returns[index] = 0.0
                next_observation, _ = environment.reset()
            next_observations.append(next_observation)

        actor_buffer.append(actor_array)
        critic_buffer.append(critic_array)
        action_buffer.append(residual_array)
        logp_buffer.append(logp_tensor.cpu().numpy())
        value_buffer.append(value_tensor.cpu().numpy())
        reward_buffer.append(rewards)
        done_buffer.append(dones)
        observations = next_observations

    last_critic = torch.as_tensor(
        np.stack([observation.critic for observation in observations]),
        dtype=torch.float32,
        device=device,
    )
    with torch.no_grad():
        last_values = model_old.value(last_critic).cpu().numpy()
    return (
        RolloutBatch(
            actor_observations=np.asarray(actor_buffer, dtype=np.float32),
            critic_observations=np.asarray(critic_buffer, dtype=np.float32),
            residual_actions=np.asarray(action_buffer, dtype=np.float32),
            log_probs=np.asarray(logp_buffer, dtype=np.float32),
            values=np.asarray(value_buffer, dtype=np.float32),
            rewards=np.asarray(reward_buffer, dtype=np.float32),
            dones=np.asarray(done_buffer, dtype=np.float32),
            last_values=np.asarray(last_values, dtype=np.float32),
            episode_infos=episode_infos,
            episode_returns=episode_returns,
        ),
        observations,
    )


def compute_gae(
    rewards: np.ndarray,
    values: np.ndarray,
    dones: np.ndarray,
    last_values: np.ndarray,
    gamma: float,
    gae_lambda: float,
) -> tuple[np.ndarray, np.ndarray]:
    if rewards.shape != values.shape or rewards.shape != dones.shape:
        raise ValueError("rewards, values, and dones must share [time, env] shape")
    advantages = np.zeros_like(rewards, dtype=np.float32)
    last_advantage = np.zeros(rewards.shape[1], dtype=np.float32)
    for time_index in reversed(range(rewards.shape[0])):
        next_values = (
            last_values if time_index == rewards.shape[0] - 1 else values[time_index + 1]
        )
        nonterminal = 1.0 - dones[time_index]
        delta = (
            rewards[time_index]
            + gamma * next_values * nonterminal
            - values[time_index]
        )
        last_advantage = (
            delta + gamma * gae_lambda * nonterminal * last_advantage
        )
        advantages[time_index] = last_advantage
    return advantages, advantages + values


def ppo_update(
    model: AsymmetricActorCritic,
    optimizer: torch.optim.Optimizer,
    batch: RolloutBatch,
    config: PPOConfig,
    device: torch.device,
) -> dict[str, float]:
    advantages, returns = compute_gae(
        batch.rewards,
        batch.values,
        batch.dones,
        batch.last_values,
        config.gamma,
        config.gae_lambda,
    )
    raw_advantages = advantages.reshape(-1)
    normalized_advantages = (
        raw_advantages - raw_advantages.mean()
    ) / (raw_advantages.std() + 1.0e-8)
    tensors = {
        "actor": torch.as_tensor(
            batch.actor_observations.reshape(-1, batch.actor_observations.shape[-1]),
            dtype=torch.float32,
            device=device,
        ),
        "critic": torch.as_tensor(
            batch.critic_observations.reshape(-1, batch.critic_observations.shape[-1]),
            dtype=torch.float32,
            device=device,
        ),
        "actions": torch.as_tensor(
            batch.residual_actions.reshape(-1, batch.residual_actions.shape[-1]),
            dtype=torch.float32,
            device=device,
        ),
        "old_logp": torch.as_tensor(
            batch.log_probs.reshape(-1), dtype=torch.float32, device=device
        ),
        "advantages": torch.as_tensor(
            normalized_advantages, dtype=torch.float32, device=device
        ),
        "returns": torch.as_tensor(
            returns.reshape(-1), dtype=torch.float32, device=device
        ),
    }
    sample_count = tensors["old_logp"].shape[0]
    indices = np.arange(sample_count)
    sums = {
        "loss": 0.0,
        "policy_loss": 0.0,
        "value_loss": 0.0,
        "residual_prior_loss": 0.0,
        "entropy": 0.0,
        "approx_kl": 0.0,
        "clip_fraction": 0.0,
    }
    evaluations = 0
    updates = 0
    stopped_early = False
    for epoch in range(config.update_epochs):
        np.random.shuffle(indices)
        for start in range(0, sample_count, config.minibatch_size):
            minibatch = indices[start : start + config.minibatch_size]
            logp, entropy, values = model.evaluate_actions(
                tensors["actor"][minibatch],
                tensors["critic"][minibatch],
                tensors["actions"][minibatch],
            )
            log_ratio = logp - tensors["old_logp"][minibatch]
            ratio = torch.exp(log_ratio)
            objective_1 = ratio * tensors["advantages"][minibatch]
            objective_2 = torch.clamp(
                ratio, 1.0 - config.clip_ratio, 1.0 + config.clip_ratio
            ) * tensors["advantages"][minibatch]
            policy_loss = -torch.min(objective_1, objective_2).mean()
            value_loss = torch.nn.functional.mse_loss(
                values, tensors["returns"][minibatch]
            )
            residual_prior_loss = torch.tanh(
                model.actor(tensors["actor"][minibatch])
            ).square().mean()
            loss = (
                policy_loss
                + config.value_coef * value_loss
                + config.residual_prior_coef * residual_prior_loss
                - config.entropy_coef * entropy
            )
            with torch.no_grad():
                # Schulman-style non-negative approximation to reverse KL.
                approximate_kl = ((torch.exp(log_ratio) - 1.0) - log_ratio).mean()
                clip_fraction = (
                    torch.abs(ratio - 1.0) > config.clip_ratio
                ).float().mean()
            for key, value in (
                ("loss", loss),
                ("policy_loss", policy_loss),
                ("value_loss", value_loss),
                ("residual_prior_loss", residual_prior_loss),
                ("entropy", entropy),
                ("approx_kl", approximate_kl),
                ("clip_fraction", clip_fraction),
            ):
                sums[key] += float(value.detach().cpu())
            evaluations += 1
            if float(approximate_kl) > 1.5 * config.target_kl:
                stopped_early = True
                break
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), config.max_grad_norm)
            optimizer.step()
            updates += 1
        if stopped_early:
            break
    metrics = {key: value / max(evaluations, 1) for key, value in sums.items()}
    metrics.update(
        {
            "updates": float(updates),
            "evaluations": float(evaluations),
            "early_stop_epoch": float(epoch + 1 if stopped_early else 0),
            "advantage_mean": float(raw_advantages.mean()),
            "advantage_std": float(raw_advantages.std()),
            "episodes": float(len(batch.episode_infos)),
            "success_rate": float(
                np.mean([bool(info["success"]) for info in batch.episode_infos])
                if batch.episode_infos
                else 0.0
            ),
            "mean_episode_return": float(
                np.mean(batch.episode_returns) if batch.episode_returns else 0.0
            ),
        }
    )
    return metrics
