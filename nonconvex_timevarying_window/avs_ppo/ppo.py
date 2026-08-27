"""Explicit-old-policy PPO over the viability-masked action distribution."""

from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np
import torch

from .config import PPOConfig
from .environment import AVSEnvironment
from .model import MaskedActorCritic


@dataclass
class Rollout:
    observations: np.ndarray
    masks: np.ndarray
    actions: np.ndarray
    log_probs: np.ndarray
    values: np.ndarray
    rewards: np.ndarray
    dones: np.ndarray
    last_values: np.ndarray
    episodes: list[dict]


def device_from_config(name: str) -> torch.device:
    return torch.device("cuda" if name == "auto" and torch.cuda.is_available() else ("cpu" if name == "auto" else name))


def collect_rollout(environments: list[AVSEnvironment], model_old: MaskedActorCritic, steps: int, device: torch.device, observations: list[np.ndarray] | None = None) -> tuple[Rollout, list[np.ndarray]]:
    if observations is None:
        observations = [environment.reset()[0] for environment in environments]
    buffers: dict[str, list[np.ndarray]] = {key: [] for key in ("observations", "masks", "actions", "log_probs", "values", "rewards", "dones")}
    episodes: list[dict] = []
    for _ in range(steps):
        masks = np.stack([environment.action_mask() for environment in environments])
        obs_array = np.stack(observations)
        with torch.no_grad():
            actions, log_probs, values = model_old.sample(
                torch.as_tensor(obs_array, dtype=torch.float32, device=device),
                torch.as_tensor(masks, dtype=torch.bool, device=device),
            )
        action_array = actions.cpu().numpy()
        rewards = np.zeros(len(environments), dtype=np.float32)
        dones = np.zeros(len(environments), dtype=np.float32)
        next_observations = []
        for index, environment in enumerate(environments):
            observation, reward, terminated, truncated, info = environment.step(int(action_array[index]), mask=masks[index])
            rewards[index], dones[index] = reward, float(terminated or truncated)
            if terminated or truncated:
                if info["safety_violations"]:
                    raise RuntimeError("training safety invariant violated")
                episodes.append(dict(info))
                observation, _ = environment.reset()
            next_observations.append(observation)
        buffers["observations"].append(obs_array)
        buffers["masks"].append(masks)
        buffers["actions"].append(action_array)
        buffers["log_probs"].append(log_probs.cpu().numpy())
        buffers["values"].append(values.cpu().numpy())
        buffers["rewards"].append(rewards)
        buffers["dones"].append(dones)
        observations = next_observations
    with torch.no_grad():
        last_values = model_old.value(torch.as_tensor(np.stack(observations), dtype=torch.float32, device=device)).cpu().numpy()
    return Rollout(**{key: np.asarray(value) for key, value in buffers.items()}, last_values=last_values, episodes=episodes), observations


def generalized_advantage(rollout: Rollout, gamma: float, gae_lambda: float) -> tuple[np.ndarray, np.ndarray]:
    advantages = np.zeros_like(rollout.rewards, dtype=np.float32)
    accumulator = np.zeros(rollout.rewards.shape[1], dtype=np.float32)
    for step in reversed(range(len(rollout.rewards))):
        next_values = rollout.last_values if step == len(rollout.rewards) - 1 else rollout.values[step + 1]
        alive = 1.0 - rollout.dones[step]
        delta = rollout.rewards[step] + gamma * next_values * alive - rollout.values[step]
        accumulator = delta + gamma * gae_lambda * alive * accumulator
        advantages[step] = accumulator
    return advantages, advantages + rollout.values


def update(model: MaskedActorCritic, optimizer: torch.optim.Optimizer, rollout: Rollout, config: PPOConfig, device: torch.device) -> dict[str, float]:
    advantages, returns = generalized_advantage(rollout, config.gamma, config.gae_lambda)
    flat_advantages = advantages.reshape(-1)
    flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std() + 1.0e-8)
    tensors = {
        "observations": torch.as_tensor(rollout.observations.reshape(-1, rollout.observations.shape[-1]), dtype=torch.float32, device=device),
        "masks": torch.as_tensor(rollout.masks.reshape(-1, rollout.masks.shape[-1]), dtype=torch.bool, device=device),
        "actions": torch.as_tensor(rollout.actions.reshape(-1), dtype=torch.long, device=device),
        "old_log_probs": torch.as_tensor(rollout.log_probs.reshape(-1), dtype=torch.float32, device=device),
        "advantages": torch.as_tensor(flat_advantages, dtype=torch.float32, device=device),
        "returns": torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=device),
    }
    indices = np.arange(len(flat_advantages))
    totals = {"policy_loss": 0.0, "value_loss": 0.0, "entropy": 0.0, "approx_kl": 0.0}
    batches = 0
    stopped_early = False
    for _ in range(config.update_epochs):
        np.random.shuffle(indices)
        for start in range(0, len(indices), config.minibatch_size):
            batch = indices[start:start + config.minibatch_size]
            log_probs, entropy, values = model.evaluate(tensors["observations"][batch], tensors["masks"][batch], tensors["actions"][batch])
            log_ratio = log_probs - tensors["old_log_probs"][batch]
            ratio = torch.exp(log_ratio)
            unclipped = ratio * tensors["advantages"][batch]
            clipped = torch.clamp(ratio, 1.0 - config.clip_ratio, 1.0 + config.clip_ratio) * tensors["advantages"][batch]
            policy_loss = -torch.minimum(unclipped, clipped).mean()
            value_loss = 0.5 * (values - tensors["returns"][batch]).square().mean()
            loss = policy_loss + config.value_coef * value_loss - config.entropy_coef * entropy.mean()
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), config.max_grad_norm)
            optimizer.step()
            with torch.no_grad():
                # Standard non-negative second-order approximation to KL(old || new).
                approx_kl = ((torch.exp(log_ratio) - 1.0) - log_ratio).mean()
            totals["policy_loss"] += float(policy_loss)
            totals["value_loss"] += float(value_loss)
            totals["entropy"] += float(entropy.mean())
            totals["approx_kl"] += float(approx_kl)
            batches += 1
            if approx_kl > config.target_kl:
                stopped_early = True
                break
        if stopped_early:
            break
    return {**{key: value / max(batches, 1) for key, value in totals.items()}, "early_stop": float(stopped_early)}


def make_old_policy(model: MaskedActorCritic) -> MaskedActorCritic:
    old = copy.deepcopy(model)
    old.eval()
    return old
