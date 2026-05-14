from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import torch

from gap_step.env import ContinuousMazeEnv
from gap_step.graph import GraphObs, collate_graph_obs
from gap_step.model import TeacherActorCritic


@dataclass
class PPOBatch:
    obs: list[GraphObs]
    actions: np.ndarray
    log_probs: np.ndarray
    values: np.ndarray
    rewards: np.ndarray
    dones: np.ndarray
    last_value: float
    episode_returns: list[float]
    episode_infos: list[dict]
    step_infos: list[dict]


def get_device(requested: str = "auto") -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(requested)


def collect_rollout(
    env: ContinuousMazeEnv,
    model_old: TeacherActorCritic,
    steps: int,
    device: torch.device,
    stage_name: str,
    split: str = "train",
    seed: int | None = None,
) -> PPOBatch:
    model_old.eval()
    obs, _ = env.reset(seed=seed, options={"stage_name": stage_name, "split": split})
    obs_buf: list[GraphObs] = []
    action_buf, logp_buf, value_buf, reward_buf, done_buf = [], [], [], [], []
    episode_returns: list[float] = []
    episode_infos: list[dict] = []
    step_infos: list[dict] = []
    ep_return = 0.0
    for _ in range(steps):
        obs_t = collate_graph_obs([obs], device)
        with torch.no_grad():
            action_t, logp_t, value_t = model_old.act(obs_t, deterministic=False)
        action = action_t.squeeze(0).cpu().numpy()
        next_obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated

        obs_buf.append(obs)
        action_buf.append(action)
        logp_buf.append(float(logp_t.item()))
        value_buf.append(float(value_t.item()))
        reward_buf.append(float(reward))
        done_buf.append(float(done))
        step_infos.append(dict(info))

        ep_return += float(reward)
        obs = next_obs
        if done:
            info = dict(info)
            info["return"] = ep_return
            episode_returns.append(ep_return)
            episode_infos.append(info)
            ep_return = 0.0
            obs, _ = env.reset(options={"stage_name": stage_name, "split": split})

    obs_t = collate_graph_obs([obs], device)
    with torch.no_grad():
        last_value = float(model_old.forward(obs_t)["value"].item())

    return PPOBatch(
        obs=obs_buf,
        actions=np.asarray(action_buf, dtype=np.float32),
        log_probs=np.asarray(logp_buf, dtype=np.float32),
        values=np.asarray(value_buf, dtype=np.float32),
        rewards=np.asarray(reward_buf, dtype=np.float32),
        dones=np.asarray(done_buf, dtype=np.float32),
        last_value=last_value,
        episode_returns=episode_returns,
        episode_infos=episode_infos,
        step_infos=step_infos,
    )


def compute_gae(rewards, values, dones, last_value, gamma: float, gae_lambda: float) -> tuple[np.ndarray, np.ndarray]:
    advantages = np.zeros_like(rewards, dtype=np.float32)
    last_adv = 0.0
    for t in reversed(range(len(rewards))):
        next_nonterminal = 1.0 - dones[t]
        next_value = last_value if t == len(rewards) - 1 else values[t + 1]
        delta = rewards[t] + gamma * next_value * next_nonterminal - values[t]
        last_adv = delta + gamma * gae_lambda * next_nonterminal * last_adv
        advantages[t] = last_adv
    returns = advantages + values
    return advantages, returns


def explained_variance(values: np.ndarray, returns: np.ndarray) -> float:
    value_var = float(np.var(returns))
    if value_var < 1e-8:
        return 0.0
    return float(1.0 - np.var(returns - values) / value_var)


def sync_policy_old(model: TeacherActorCritic, model_old: TeacherActorCritic) -> None:
    model_old.load_state_dict(model.state_dict())
    model_old.eval()


def ppo_update(
    model: TeacherActorCritic,
    optimizer: torch.optim.Optimizer,
    batch: PPOBatch,
    device: torch.device,
    *,
    gamma: float,
    gae_lambda: float,
    clip_ratio: float,
    value_coef: float,
    entropy_coef: float,
    update_epochs: int,
    minibatch_size: int,
    max_grad_norm: float,
    target_kl: float | None = None,
    normalize_advantage: bool = True,
) -> dict[str, float]:
    model.train()
    adv, ret = compute_gae(batch.rewards, batch.values, batch.dones, batch.last_value, gamma, gae_lambda)
    raw_adv = adv.copy()
    if normalize_advantage and len(adv) > 1:
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    tensors = {
        "actions": torch.as_tensor(batch.actions, dtype=torch.float32, device=device),
        "old_logp": torch.as_tensor(batch.log_probs, dtype=torch.float32, device=device),
        "advantages": torch.as_tensor(adv, dtype=torch.float32, device=device),
        "returns": torch.as_tensor(ret, dtype=torch.float32, device=device),
    }
    n = len(batch.rewards)
    inds = np.arange(n)
    metrics = {
        "loss": 0.0,
        "policy_loss": 0.0,
        "value_loss": 0.0,
        "entropy": 0.0,
        "approx_kl": 0.0,
        "old_approx_kl": 0.0,
        "clip_fraction": 0.0,
    }
    updates = 0
    evaluations = 0
    stopped_early = False
    for epoch in range(update_epochs):
        np.random.shuffle(inds)
        for start in range(0, n, minibatch_size):
            mb = inds[start : start + minibatch_size]
            obs_mb = collate_graph_obs([batch.obs[int(i)] for i in mb], device)
            logp, entropy, value = model.evaluate_actions(obs_mb, tensors["actions"][mb])
            log_ratio = logp - tensors["old_logp"][mb]
            ratio = torch.exp(log_ratio)
            pg1 = ratio * tensors["advantages"][mb]
            pg2 = torch.clamp(ratio, 1.0 - clip_ratio, 1.0 + clip_ratio) * tensors["advantages"][mb]
            policy_loss = -torch.min(pg1, pg2).mean()
            value_loss = torch.nn.functional.mse_loss(value, tensors["returns"][mb])
            with torch.no_grad():
                old_approx_kl = (tensors["old_logp"][mb] - logp).mean()
                approx_kl = ((torch.exp(log_ratio) - 1.0) - log_ratio).mean()
                clip_fraction = (torch.abs(ratio - 1.0) > clip_ratio).float().mean()
            loss = policy_loss + value_coef * value_loss - entropy_coef * entropy
            metrics["loss"] += float(loss.detach().cpu())
            metrics["policy_loss"] += float(policy_loss.detach().cpu())
            metrics["value_loss"] += float(value_loss.detach().cpu())
            metrics["entropy"] += float(entropy.detach().cpu())
            metrics["approx_kl"] += float(approx_kl.detach().cpu())
            metrics["old_approx_kl"] += float(old_approx_kl.detach().cpu())
            metrics["clip_fraction"] += float(clip_fraction.detach().cpu())
            evaluations += 1
            if target_kl is not None and float(approx_kl.detach().cpu()) > 1.5 * target_kl:
                stopped_early = True
                break
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm)
            optimizer.step()
            updates += 1
        if stopped_early:
            break
    out = {k: v / max(1, evaluations) for k, v in metrics.items()}
    out["ppo_updates"] = float(updates)
    out["ppo_evaluations"] = float(evaluations)
    out["early_stop_epoch"] = float(epoch + 1 if stopped_early else 0)
    out["advantage_mean"] = float(raw_adv.mean())
    out["advantage_std"] = float(raw_adv.std())
    out["explained_variance"] = explained_variance(batch.values, ret)
    with torch.no_grad():
        log_std = model.effective_log_std()
        out["log_std_mean"] = float(log_std.mean().detach().cpu())
        out["std_mean"] = float(torch.exp(log_std).mean().detach().cpu())
    return out
