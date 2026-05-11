from __future__ import annotations

import torch

from gap_step.model import TeacherActorCritic


def test_teacher_actor_critic_shapes():
    model = TeacherActorCritic(obs_dim=39, max_acc=3.0)
    obs = torch.zeros(4, 39)
    out = model(obs)
    assert out["mean"].shape == (4, 2)
    assert out["value"].shape == (4,)
    action, logp, value = model.act(obs)
    assert action.shape == (4, 2)
    assert logp.shape == (4,)
    assert value.shape == (4,)
    assert torch.all(action <= 3.0)
    assert torch.all(action >= -3.0)
