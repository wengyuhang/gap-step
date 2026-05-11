from __future__ import annotations

import argparse
import csv

import numpy as np
import torch
from tqdm import trange

from gap_step.curriculum import STAGE_ORDER, stage_from_step
from gap_step.env import ContinuousMazeEnv
from gap_step.model import TeacherActorCritic
from gap_step.ppo import collect_rollout, get_device, ppo_update
from gap_step.utils import ensure_dir, load_yaml, resolve_path, set_seed


DEFAULT_CONFIG = {
    "seed": 0,
    "device": "auto",
    "steps_per_stage": 1_000_000,
    "total_steps": 5_000_000,
    "rollout_steps": 2048,
    "minibatch_size": 256,
    "update_epochs": 10,
    "learning_rate": 3e-4,
    "gamma": 0.99,
    "gae_lambda": 0.95,
    "clip_ratio": 0.2,
    "value_coef": 0.5,
    "entropy_coef": 0.01,
    "max_grad_norm": 0.5,
    "checkpoint_path": "checkpoints/teacher_final.pt",
    "train_metrics_path": "results/train_metrics.csv",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="gap_step/configs/train_teacher_smoke.yaml")
    args = parser.parse_args()

    config = dict(DEFAULT_CONFIG)
    path = resolve_path(args.config)
    if path.exists():
        config.update(load_yaml(path))

    seed = int(config["seed"])
    set_seed(seed)
    device = get_device(str(config["device"]))
    env = ContinuousMazeEnv(config.get("env", {}))
    model = TeacherActorCritic(obs_dim=env.observation_space.shape[0], max_acc=env.max_acc).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))

    total_steps = int(config["total_steps"])
    rollout_steps = int(config["rollout_steps"])
    steps_per_stage = int(config["steps_per_stage"])
    updates = max(1, total_steps // rollout_steps)
    rows: list[dict] = []
    global_steps = 0

    for update in trange(updates, desc="teacher PPO"):
        stage_name = stage_from_step(global_steps, steps_per_stage)
        batch = collect_rollout(env, model, rollout_steps, device, stage_name=stage_name, seed=seed + update)
        metrics = ppo_update(
            model,
            optimizer,
            batch,
            device,
            gamma=float(config["gamma"]),
            gae_lambda=float(config["gae_lambda"]),
            clip_ratio=float(config["clip_ratio"]),
            value_coef=float(config["value_coef"]),
            entropy_coef=float(config["entropy_coef"]),
            update_epochs=int(config["update_epochs"]),
            minibatch_size=int(config["minibatch_size"]),
            max_grad_norm=float(config["max_grad_norm"]),
        )
        global_steps += rollout_steps
        infos = batch.episode_infos
        row = {
            "update": update + 1,
            "global_steps": global_steps,
            "stage": stage_name,
            "episodes": len(infos),
            "average_return": float(np.mean(batch.episode_returns)) if batch.episode_returns else 0.0,
            "success_rate": float(np.mean([i["success"] for i in infos])) if infos else 0.0,
            "collision_rate": float(np.mean([i["collision"] for i in infos])) if infos else 0.0,
            "timeout_rate": float(np.mean([i["timeout"] for i in infos])) if infos else 0.0,
            **metrics,
        }
        rows.append(row)

    ckpt_path = resolve_path(config["checkpoint_path"])
    ensure_dir(ckpt_path.parent)
    torch.save(
        {
            "model_state": model.state_dict(),
            "obs_dim": env.observation_space.shape[0],
            "max_acc": env.max_acc,
            "config": config,
            "stages": STAGE_ORDER,
        },
        ckpt_path,
    )

    metrics_path = resolve_path(config["train_metrics_path"])
    ensure_dir(metrics_path.parent)
    if rows:
        with metrics_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    print(f"Saved teacher checkpoint to {ckpt_path}")
    print(f"Saved train metrics to {metrics_path}")


if __name__ == "__main__":
    main()
