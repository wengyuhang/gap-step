from __future__ import annotations

import argparse
import csv

import numpy as np
import torch

from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.ppo import get_device
from gap_step.utils import ensure_dir, resolve_path
from gap_step.window_maze_env import TimeVaryingWindowMazeEnv


SPLITS = ("id_test", "ood_window_test", "ood_maze_test")


def load_checkpoint(checkpoint: str, device: torch.device) -> tuple[TeacherActorCritic, dict]:
    ckpt = torch.load(resolve_path(checkpoint), map_location=device, weights_only=False)
    model = TeacherActorCritic(
        global_dim=int(ckpt.get("global_dim", GLOBAL_FEATURE_DIM)),
        node_dim=int(ckpt.get("node_dim", NODE_FEATURE_DIM)),
        edge_dim=int(ckpt.get("edge_dim", EDGE_FEATURE_DIM)),
        hidden_dim=int(ckpt.get("hidden_dim", 128)),
        gnn_layers=int(ckpt.get("gnn_layers", 4)),
        max_acc=float(ckpt.get("max_acc", 1.0)),
        min_log_std=float(ckpt.get("min_log_std", -2.0)),
        max_log_std=float(ckpt.get("max_log_std", 0.3)),
        log_std_init=float(ckpt.get("log_std_init", -0.35)),
    ).to(device)
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    return model, dict(ckpt.get("config", {}))


def evaluate_split(
    model: TeacherActorCritic,
    env_config: dict,
    split: str,
    episodes: int,
    device: torch.device,
    stage_name: str,
    *,
    seed_base: int,
) -> dict:
    cfg = dict(env_config)
    cfg.update({"return_graph_obs": True, "stage_name": stage_name, "split": split})
    num_eval_envs = max(1, min(8, episodes))
    envs = [TimeVaryingWindowMazeEnv(cfg) for _ in range(num_eval_envs)]
    rows: list[dict] = []
    with torch.no_grad():
        next_episode = 0
        active_obs = []
        active_envs = []
        active_returns = []
        for env in envs:
            if next_episode >= episodes:
                break
            obs, _ = env.reset(seed=seed_base + next_episode, options={"stage_name": stage_name, "split": split})
            active_obs.append(obs)
            active_envs.append(env)
            active_returns.append(0.0)
            next_episode += 1

        while active_envs:
            obs_t = collate_graph_obs(active_obs, device)
            actions, _, _ = model.act(obs_t, deterministic=True)
            next_obs = []
            next_envs = []
            next_returns = []
            for idx, env in enumerate(active_envs):
                obs, reward, terminated, truncated, final_info = env.step(actions[idx].cpu().numpy())
                ep_return = active_returns[idx] + float(reward)
                done = terminated or truncated
                if done:
                    rows.append({**final_info, "return": ep_return, "steps": env.step_count})
                    if next_episode < episodes:
                        obs, _ = env.reset(
                            seed=seed_base + next_episode,
                            options={"stage_name": stage_name, "split": split},
                        )
                        next_episode += 1
                        next_obs.append(obs)
                        next_envs.append(env)
                        next_returns.append(0.0)
                else:
                    next_obs.append(obs)
                    next_envs.append(env)
                    next_returns.append(ep_return)
            active_obs = next_obs
            active_envs = next_envs
            active_returns = next_returns
    return {
        "split": split,
        "stage": stage_name,
        "episodes": len(rows),
        "success_rate": float(np.mean([r["success"] for r in rows])) if rows else 0.0,
        "collision_rate": float(np.mean([r["collision"] for r in rows])) if rows else 0.0,
        "timeout_rate": float(np.mean([r["timeout"] for r in rows])) if rows else 0.0,
        "wall_collision_rate": float(np.mean([r["wall_collision"] for r in rows])) if rows else 0.0,
        "window_collision_rate": float(np.mean([r["window_collision"] for r in rows])) if rows else 0.0,
        "average_return": float(np.mean([r["return"] for r in rows])) if rows else 0.0,
        "average_steps": float(np.mean([r["steps"] for r in rows])) if rows else 0.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="checkpoints/window_generated/C5/teacher_final.pt")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--stage", default="C5")
    parser.add_argument("--splits", default="id_test,ood_window_test,ood_maze_test")
    parser.add_argument("--output", default="results/window_generated/eval_c5.csv")
    parser.add_argument("--seed-base", type=int, default=50_000)
    parser.add_argument("--max-step", type=float, default=None)
    args = parser.parse_args()

    device = get_device(args.device)
    model, config = load_checkpoint(args.checkpoint, device)
    env_config = dict(config.get("env", {}))
    if args.max_step is not None:
        env_config["max_step"] = float(args.max_step)
    splits = [s.strip() for s in args.splits.split(",") if s.strip()]
    unknown = [s for s in splits if s not in SPLITS]
    if unknown:
        raise ValueError(f"Unknown split: {unknown}")
    rows = [
        evaluate_split(model, env_config, split, args.episodes, device, args.stage, seed_base=args.seed_base)
        for split in splits
    ]
    output = resolve_path(args.output)
    ensure_dir(output.parent)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    for row in rows:
        print(row)
    print(f"Saved evaluation to {output}")


if __name__ == "__main__":
    main()
