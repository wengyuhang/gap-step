from __future__ import annotations

import argparse
from pathlib import Path

import imageio.v2 as imageio
import torch

from gap_step.evaluate_window import load_checkpoint
from gap_step.graph import collate_graph_obs
from gap_step.ppo import get_device
from gap_step.utils import ensure_dir, resolve_path
from gap_step.window_maze_env import TimeVaryingWindowMazeEnv


def rollout_gif(checkpoint: str, output: str | Path, *, seed: int, split: str, stage: str, device_name: str) -> dict:
    device = get_device(device_name)
    model, config = load_checkpoint(checkpoint, device)
    env_config = dict(config.get("env", {}))
    env_config.update({"return_graph_obs": True, "stage_name": stage, "split": split, "render_width": 980, "render_height": 540})
    env = TimeVaryingWindowMazeEnv(env_config)
    obs, _ = env.reset(seed=seed, options={"stage_name": stage, "split": split})
    frames = [env.render()]
    done = False
    final_info = {}
    with torch.no_grad():
        while not done:
            obs_t = collate_graph_obs([obs], device)
            action, _, _ = model.act(obs_t, deterministic=True)
            obs, _, terminated, truncated, final_info = env.step(action.squeeze(0).cpu().numpy())
            frames.append(env.render())
            done = terminated or truncated
    output = resolve_path(output)
    ensure_dir(output.parent)
    imageio.mimsave(output, frames, duration=0.12)
    return dict(final_info)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="checkpoints/window_generated/C5/teacher_final.pt")
    parser.add_argument("--output-dir", default="results/window_generated/gifs")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--stage", default="C5")
    args = parser.parse_args()

    cases = [
        ("success_case", 60_002, "id_test"),
        ("collision_case", 60_000, "id_test"),
        ("timing_case", 60_001, "id_test"),
        ("ood_window_case", 70_001, "ood_window_test"),
        ("ood_maze_case", 80_001, "ood_maze_test"),
    ]
    output_dir = resolve_path(args.output_dir)
    ensure_dir(output_dir)
    for name, seed, split in cases:
        out = output_dir / f"{name}.gif"
        info = rollout_gif(args.checkpoint, out, seed=seed, split=split, stage=args.stage, device_name=args.device)
        print(f"{out}: success={info.get('success')} collision={info.get('collision')} timeout={info.get('timeout')}")


if __name__ == "__main__":
    main()
