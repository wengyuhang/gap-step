from __future__ import annotations

import numpy as np

from gap_step.config import load_yaml
from gap_step.envs import GapStepEnv


def test_reset_step_render():
    env = GapStepEnv(load_yaml("configs/env_e1.yaml"))
    obs, info = env.reset(seed=0)
    assert obs["image_stack"].shape == (env.K_obs, env.image_size, env.image_size)
    assert obs["proprio"].shape == (6,)
    assert "gate_widths" in info
    action = np.zeros(2, dtype=np.float32)
    obs, reward, terminated, truncated, info = env.step(action)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    frame = env.render()
    assert frame.ndim == 3
    assert frame.shape[-1] == 3
