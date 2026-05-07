from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gap_step.config import load_yaml, resolve_path
from gap_step.envs import GapStepEnv
from gap_step.gif import save_gif


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-config", default="configs/env_e3.yaml")
    parser.add_argument("--output", default="runs/random_policy.gif")
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    env = GapStepEnv(load_yaml(args.env_config))
    obs, _ = env.reset(seed=args.seed)
    del obs
    rng = np.random.default_rng(args.seed)
    frames = []
    for step in range(args.steps):
        action = rng.uniform(-env.max_acc, env.max_acc, size=2).astype(np.float32)
        _, _, terminated, truncated, _ = env.step(action)
        if step % 2 == 0:
            frames.append(env.render())
        if terminated or truncated:
            break
    out = resolve_path(args.output)
    save_gif(frames, out)
    print(f"Saved random rollout to {out}")


if __name__ == "__main__":
    main()
