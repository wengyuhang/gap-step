from __future__ import annotations

import argparse
import random

import numpy as np
import torch
import torch.nn.functional as F

from gap_step.graph import GraphObs, collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.ppo import get_device
from gap_step.train_window import _save_checkpoint
from gap_step.utils import load_yaml, resolve_path, set_seed
from gap_step.window_maze_env import TimeVaryingWindowMazeEnv
from gap_step.window_planner import plan_reference_actions


DEFAULT_CONFIG = {
    "seed": 0,
    "device": "auto",
    "stages": ["C2W0", "C2W", "C2B", "C2C", "C2D", "C3"],
    "episodes_per_stage": 16,
    "epochs": 20,
    "batch_size": 256,
    "learning_rate": 3e-4,
    "gnn_hidden_dim": 64,
    "gnn_layers": 3,
    "min_log_std": -3.0,
    "max_log_std": -1.0,
    "log_std_init": -1.6,
    "checkpoint": "checkpoints/window_generated/bc_warmstart.pt",
    "env": {"return_graph_obs": True, "period": 8, "max_steps": 700},
}


def collect_dataset(config: dict) -> tuple[list[GraphObs], np.ndarray]:
    observations: list[GraphObs] = []
    actions: list[np.ndarray] = []
    seed = int(config["seed"])
    env_config = dict(config["env"])
    env_config["return_graph_obs"] = True
    for stage_idx, stage_name in enumerate(config["stages"]):
        success = 0
        for ep in range(int(config["episodes_per_stage"])):
            env = TimeVaryingWindowMazeEnv({**env_config, "stage_name": stage_name, "split": "train"})
            obs, _ = env.reset(
                seed=seed + stage_idx * 100_000 + ep,
                options={"stage_name": stage_name, "split": "train"},
            )
            plan = plan_reference_actions(env)
            if plan is None:
                continue
            for action in plan:
                observations.append(obs)
                actions.append(np.asarray(action, dtype=np.float32))
                obs, _, terminated, truncated, info = env.step(action)
                if terminated or truncated:
                    success += int(info["success"])
                    break
        print(f"BC 数据 | {stage_name} | 成功规划 {success}/{int(config['episodes_per_stage'])}", flush=True)
    return observations, np.asarray(actions, dtype=np.float32)


def train_bc(config: dict) -> None:
    set_seed(int(config["seed"]))
    device = get_device(config["device"])
    obs, actions = collect_dataset(config)
    if not obs:
        raise RuntimeError("Planner produced no demonstrations")
    model = TeacherActorCritic(
        hidden_dim=int(config["gnn_hidden_dim"]),
        gnn_layers=int(config["gnn_layers"]),
        max_acc=1.0,
        min_log_std=float(config["min_log_std"]),
        max_log_std=float(config["max_log_std"]),
        log_std_init=float(config["log_std_init"]),
    ).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))
    indices = list(range(len(obs)))
    batch_size = int(config["batch_size"])
    target_actions = torch.as_tensor(actions, dtype=torch.float32, device=device)
    for epoch in range(1, int(config["epochs"]) + 1):
        random.shuffle(indices)
        losses = []
        for start in range(0, len(indices), batch_size):
            batch_idx = indices[start : start + batch_size]
            graph_batch = collate_graph_obs([obs[idx] for idx in batch_idx], device)
            pred = model(graph_batch)["mean"]
            loss = F.mse_loss(pred, target_actions[batch_idx])
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
        print(f"BC | epoch {epoch} | loss {float(np.mean(losses)):.6f}", flush=True)
    _save_checkpoint(resolve_path(config["checkpoint"]), config, model)
    print(f"BC checkpoint saved: {resolve_path(config['checkpoint'])}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = dict(DEFAULT_CONFIG)
    config.update(load_yaml(args.config))
    train_bc(config)


if __name__ == "__main__":
    main()
