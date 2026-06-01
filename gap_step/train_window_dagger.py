from __future__ import annotations

import argparse
import random

import numpy as np
import torch
import torch.nn.functional as F

from gap_step.evaluate_window import load_checkpoint
from gap_step.graph import GraphObs, collate_graph_obs
from gap_step.ppo import get_device
from gap_step.train_window import _save_checkpoint
from gap_step.utils import load_yaml, resolve_path, set_seed
from gap_step.window_maze_env import TimeVaryingWindowMazeEnv
from gap_step.window_planner import planner_action_from_state


DEFAULT_CONFIG = {
    "seed": 0,
    "device": "auto",
    "checkpoint": "checkpoints/window_generated/bc_warmstart.pt",
    "output": "checkpoints/window_generated/dagger.pt",
    "stage": "C2W",
    "iterations": 4,
    "episodes_per_iter": 32,
    "epochs_per_iter": 4,
    "batch_size": 256,
    "learning_rate": 2e-4,
    "max_steps_per_episode": 160,
    "env": {"return_graph_obs": True, "period": 8, "max_steps": 700},
}


def collect_policy_labels(model, config: dict, device: torch.device, iteration: int) -> tuple[list[GraphObs], np.ndarray]:
    observations: list[GraphObs] = []
    actions: list[np.ndarray] = []
    stage = str(config["stage"])
    env_cfg = dict(config["env"])
    env_cfg.update({"return_graph_obs": True, "stage_name": stage, "split": "train"})
    with torch.no_grad():
        for ep in range(int(config["episodes_per_iter"])):
            env = TimeVaryingWindowMazeEnv(env_cfg)
            obs, _ = env.reset(
                seed=int(config["seed"]) + iteration * 100_000 + ep,
                options={"stage_name": stage, "split": "train"},
            )
            for _ in range(int(config["max_steps_per_episode"])):
                label = planner_action_from_state(env)
                if label is not None:
                    observations.append(obs)
                    actions.append(label)
                obs_t = collate_graph_obs([obs], device)
                action, _, _ = model.act(obs_t, deterministic=True)
                obs, _, terminated, truncated, _ = env.step(action.squeeze(0).cpu().numpy())
                if terminated or truncated:
                    break
    return observations, np.asarray(actions, dtype=np.float32)


def train(config: dict) -> None:
    set_seed(int(config["seed"]))
    device = get_device(config["device"])
    model, ckpt_config = load_checkpoint(config["checkpoint"], device)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))
    all_obs: list[GraphObs] = []
    all_actions: list[np.ndarray] = []
    for iteration in range(1, int(config["iterations"]) + 1):
        obs, actions = collect_policy_labels(model, config, device, iteration)
        all_obs.extend(obs)
        all_actions.extend(list(actions))
        print(f"DAgger | iter {iteration} | 新样本 {len(obs)} | 总样本 {len(all_obs)}", flush=True)
        target = torch.as_tensor(np.asarray(all_actions, dtype=np.float32), dtype=torch.float32, device=device)
        indices = list(range(len(all_obs)))
        for epoch in range(1, int(config["epochs_per_iter"]) + 1):
            random.shuffle(indices)
            losses = []
            for start in range(0, len(indices), int(config["batch_size"])):
                batch_idx = indices[start : start + int(config["batch_size"])]
                graph_batch = collate_graph_obs([all_obs[idx] for idx in batch_idx], device)
                pred = model(graph_batch)["mean"]
                loss = F.mse_loss(pred, target[batch_idx])
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
                optimizer.step()
                losses.append(float(loss.detach().cpu()))
            print(f"DAgger | iter {iteration} | epoch {epoch} | loss {float(np.mean(losses)):.6f}", flush=True)
    merged_config = dict(ckpt_config)
    merged_config.update(config)
    _save_checkpoint(resolve_path(config["output"]), merged_config, model)
    print(f"DAgger checkpoint saved: {resolve_path(config['output'])}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = dict(DEFAULT_CONFIG)
    config.update(load_yaml(args.config))
    train(config)


if __name__ == "__main__":
    main()
