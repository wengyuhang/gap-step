from __future__ import annotations

import numpy as np
import torch

from gap_step.model import TeacherActorCritic
from gap_step.ppo import PPOBatch, ppo_update


def test_teacher_actor_critic_shapes():
    model = TeacherActorCritic(obs_dim=161, max_acc=3.0)
    obs = torch.zeros(4, 161)
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
    model = TeacherActorCritic(obs_dim=161, max_acc=3.0)
    obs = torch.zeros(8, 161)
    action, logp, _ = model.act(obs)
    eval_logp, entropy, value = model.evaluate_actions(obs, action)
    assert torch.all(action <= 3.0)
    assert torch.all(action >= -3.0)
    assert torch.isfinite(logp).all()
    assert torch.isfinite(eval_logp).all()
    assert torch.isfinite(entropy)
    assert torch.isfinite(value).all()


def test_ppo_update_recomputes_squashed_action_log_prob():
    model = TeacherActorCritic(obs_dim=161, max_acc=3.0)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    obs_t = torch.zeros(16, 161)
    with torch.no_grad():
        actions, logp, values = model.act(obs_t)
    batch = PPOBatch(
        obs=obs_t.numpy().astype(np.float32),
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
    )
    assert np.isfinite(metrics["policy_loss"])
    assert np.isfinite(metrics["value_loss"])
    assert np.isfinite(metrics["entropy"])
    assert np.isfinite(metrics["log_std_mean"])
    assert np.isfinite(metrics["std_mean"])


def test_min_log_std_is_enforced():
    model = TeacherActorCritic(obs_dim=161, max_acc=3.0, min_log_std=-1.0)
    with torch.no_grad():
        model.log_std.fill_(-5.0)
    assert torch.allclose(model.effective_log_std(), torch.full_like(model.log_std, -1.0))
