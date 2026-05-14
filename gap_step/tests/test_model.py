from __future__ import annotations

import numpy as np
import torch

from gap_step.env import ContinuousMazeEnv
from gap_step.graph import GraphObs, collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.ppo import PPOBatch, compute_gae, ppo_update, sync_policy_old


def _graph_batch(count: int = 4):
    env = ContinuousMazeEnv({"stage_name": "C1"})
    obs = [env.reset(seed=i, options={"stage_name": "C1", "split": "train"})[0] for i in range(count)]
    return obs, collate_graph_obs(obs, torch.device("cpu"))


def test_teacher_actor_critic_shapes():
    model = TeacherActorCritic(max_acc=3.0)
    _, obs = _graph_batch(4)
    out = model(obs)
    assert out["mean"].shape == (4, 2)
    assert out["value"].shape == (4,)
    action, logp, value = model.act(obs)
    assert action.shape == (4, 2)
    assert logp.shape == (4,)
    assert value.shape == (4,)
    assert torch.all(action <= 3.0)
    assert torch.all(action >= -3.0)


def test_squashed_action_log_prob_is_finite():
    model = TeacherActorCritic(max_acc=3.0)
    _, obs = _graph_batch(8)
    action, logp, _ = model.act(obs)
    eval_logp, entropy, value = model.evaluate_actions(obs, action)
    assert torch.all(action <= 3.0)
    assert torch.all(action >= -3.0)
    assert torch.isfinite(logp).all()
    assert torch.isfinite(eval_logp).all()
    assert torch.isfinite(entropy)
    assert torch.isfinite(value).all()


def test_focused_readout_falls_back_without_agent_goal_flags():
    model = TeacherActorCritic(max_acc=3.0)
    obs_list, _ = _graph_batch(2)
    modified = []
    for obs in obs_list:
        node_features = obs.node_features.copy()
        node_features[:, 13] = 0.0
        node_features[:, 14] = 0.0
        modified.append(
            GraphObs(
                global_features=obs.global_features,
                node_features=node_features,
                node_type=obs.node_type,
                edge_index=obs.edge_index,
                edge_features=obs.edge_features,
            )
        )
    obs = collate_graph_obs(modified, torch.device("cpu"))
    out = model(obs)
    assert out["mean"].shape == (2, 2)
    assert out["value"].shape == (2,)
    assert torch.isfinite(out["mean"]).all()
    assert torch.isfinite(out["value"]).all()


def test_ppo_update_recomputes_squashed_action_log_prob():
    model = TeacherActorCritic(max_acc=3.0)
    model_old = TeacherActorCritic(max_acc=3.0)
    sync_policy_old(model, model_old)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    obs_list, obs_t = _graph_batch(16)
    with torch.no_grad():
        actions, logp, values = model_old.act(obs_t)
    before = [param.detach().clone() for param in model.parameters()]
    batch = PPOBatch(
        obs=obs_list,
        actions=actions.numpy().astype(np.float32),
        log_probs=logp.numpy().astype(np.float32),
        values=values.numpy().astype(np.float32),
        rewards=np.zeros(16, dtype=np.float32),
        dones=np.zeros(16, dtype=np.float32),
        last_value=0.0,
        episode_returns=[],
        episode_infos=[],
        step_infos=[],
    )
    metrics = ppo_update(
        model,
        optimizer,
        batch,
        torch.device("cpu"),
        gamma=0.99,
        gae_lambda=0.95,
        clip_ratio=0.2,
        value_coef=0.5,
        entropy_coef=0.01,
        update_epochs=1,
        minibatch_size=8,
        max_grad_norm=0.5,
        target_kl=0.03,
        normalize_advantage=True,
    )
    changed = any(not torch.allclose(old, new.detach()) for old, new in zip(before, model.parameters()))
    assert changed
    assert np.isfinite(metrics["policy_loss"])
    assert np.isfinite(metrics["value_loss"])
    assert np.isfinite(metrics["loss"])
    assert np.isfinite(metrics["entropy"])
    assert np.isfinite(metrics["log_std_mean"])
    assert np.isfinite(metrics["std_mean"])
    assert np.isfinite(metrics["approx_kl"])
    assert metrics["approx_kl"] >= 0.0
    assert np.isfinite(metrics["old_approx_kl"])
    assert np.isfinite(metrics["clip_fraction"])
    assert np.isfinite(metrics["explained_variance"])
    assert metrics["ppo_evaluations"] >= metrics["ppo_updates"]


def test_sync_policy_old_matches_current_policy():
    model = TeacherActorCritic(max_acc=3.0)
    model_old = TeacherActorCritic(max_acc=3.0)
    with torch.no_grad():
        next(model.parameters()).add_(1.0)
    sync_policy_old(model, model_old)
    for current, old in zip(model.parameters(), model_old.parameters()):
        assert torch.allclose(current, old)
    assert not model_old.training


def test_compute_gae_returns_finite_vectors():
    rewards = np.array([1.0, 0.5, -1.0], dtype=np.float32)
    values = np.array([0.2, 0.1, -0.3], dtype=np.float32)
    dones = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    advantages, returns = compute_gae(rewards, values, dones, last_value=0.0, gamma=0.99, gae_lambda=0.95)
    assert advantages.shape == rewards.shape
    assert returns.shape == rewards.shape
    assert np.isfinite(advantages).all()
    assert np.isfinite(returns).all()


def test_min_log_std_is_enforced():
    model = TeacherActorCritic(max_acc=3.0, min_log_std=-1.0)
    with torch.no_grad():
        model.log_std.fill_(-5.0)
    assert torch.allclose(model.effective_log_std(), torch.full_like(model.log_std, -1.0))
