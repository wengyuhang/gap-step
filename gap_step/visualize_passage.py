from __future__ import annotations

import argparse
from pathlib import Path

import torch

from gap_step.evaluate_passage import load_policy
from gap_step.gif import save_gif
from gap_step.graph import collate_graph_obs
from gap_step.passage_env import TimeVaryingPassageMazeEnv
from gap_step.passage_teacher import TimeExpandedPassageTeacher
from gap_step.utils import resolve_path


def visualize(
    checkpoint: str | Path,
    output_dir: str | Path,
    episodes: int,
    fps: int,
    split: str,
    stage: str,
    device_name: str,
) -> list[Path]:
    device = torch.device("cuda" if device_name == "auto" and torch.cuda.is_available() else ("cpu" if device_name == "auto" else device_name))
    model, config = load_policy(checkpoint, device)
    env_config = dict(config.get("env", {}))
    env_config.update({"stage_name": stage, "split": split, "return_graph_obs": False, "render_size": 900})
    teacher = TimeExpandedPassageTeacher(dict(config.get("teacher", {})))
    output_dir = resolve_path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []

    model.eval()
    with torch.no_grad():
        for episode in range(episodes):
            env = TimeVaryingPassageMazeEnv(env_config)
            env.reset(seed=1_100_000 + episode, options={"stage_name": stage, "split": split})
            frames = [env.render()]
            terminated = False
            truncated = False
            info = {}
            while not (terminated or truncated):
                prior = teacher.act(env)
                obs = env.graph_obs(prior_action=prior)
                action, _, _ = model.act(collate_graph_obs([obs], device), deterministic=True)
                _, _, terminated, truncated, info = env.step(action.squeeze(0).cpu().numpy())
                frames.append(env.render())
            suffix = "success" if info.get("success", False) else info.get("collision_type") or "timeout"
            path = output_dir / f"{stage}_{split}_ep{episode:02d}_{suffix}.gif"
            save_gif(frames, path, fps=fps)
            paths.append(path)
            print(f"已保存 GIF: {path} | 成功={info.get('success', False)} | 步数={info.get('step', 0)} | 碰撞={info.get('collision_type') or '无'}")
    return paths


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="checkpoints/passage_generated/C5/teacher_final.pt")
    parser.add_argument("--output-dir", default="results/passage_generated/C5/gifs")
    parser.add_argument("--episodes", type=int, default=6)
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--split", default="id_test")
    parser.add_argument("--stage", default="C5")
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    visualize(args.checkpoint, args.output_dir, args.episodes, args.fps, args.split, args.stage, args.device)


if __name__ == "__main__":
    main()
