from __future__ import annotations

from gap_step.config import load_yaml
from gap_step.envs import GapStepEnv
from gap_step.teachers import HeuristicTeacher


def test_teacher_action_shape_and_gate():
    env = GapStepEnv(load_yaml("configs/env_e1.yaml"))
    env.reset(seed=0)
    teacher = HeuristicTeacher(env)
    acc, gate = teacher.act()
    assert acc.shape == (2,)
    assert 0 <= gate < env.num_gates


def test_teacher_succeeds_simple_env():
    env = GapStepEnv(load_yaml("configs/env_e1.yaml"))
    teacher = HeuristicTeacher(env)
    successes = 0
    episodes = 5
    for seed in range(episodes):
        env.reset(seed=seed)
        done = False
        final_info = {}
        while not done:
            acc, _ = teacher.act()
            _, _, terminated, truncated, final_info = env.step(acc)
            done = terminated or truncated
        successes += int(final_info.get("success", False))
    assert successes / episodes >= 0.8
