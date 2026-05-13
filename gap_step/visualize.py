from __future__ import annotations

import argparse

import torch

from gap_step.env import ContinuousMazeEnv
from gap_step.evaluate import load_teacher
from gap_step.gif import save_gif
from gap_step.graph import collate_graph_obs
from gap_step.ppo import get_device
from gap_step.utils import resolve_path


def rollout_gif(model, output: str, seed: int, split: str, device: torch.device, max_steps: int = 500) -> dict:
    env = ContinuousMazeEnv({"stage_name": "C5", "split": split})
    obs, _ = env.reset(seed=seed, options={"stage_name": "C5", "split": split})
    frames = []
    final_info = {}
    with torch.no_grad():
        for step in range(max_steps):
            obs_t = collate_graph_obs([obs], device)
            action, _, _ = model.act(obs_t, deterministic=True)
            obs, _, terminated, truncated, final_info = env.step(action.squeeze(0).cpu().numpy())
            if step % 2 == 0:
                frames.append(env.render())
            if terminated or truncated:
                break
    save_gif(frames, resolve_path(output))
    return final_info


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="checkpoints/teacher_best.pt")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--split", default="id_test")
    args = parser.parse_args()

    device = get_device(args.device)
    model = load_teacher(args.checkpoint, device)
    cases = [
        ("results/typical_success.gif", 10000),
        ("results/typical_wait.gif", 10005),
        ("results/typical_collision.gif", 10010),
    ]
    for output, seed in cases:
        info = rollout_gif(model, output, seed, args.split, device)
        print(f"Saved {output}: {info}")


if __name__ == "__main__":
    main()
