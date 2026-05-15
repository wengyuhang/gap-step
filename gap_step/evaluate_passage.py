from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

import numpy as np
import torch

from gap_step.graph import collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.passage_env import TimeVaryingPassageMazeEnv
from gap_step.passage_teacher import TimeExpandedPassageTeacher, rollout_teacher
from gap_step.train_passage import evaluate_policy
from gap_step.utils import resolve_path


def load_policy(checkpoint_path: str | Path, device: torch.device) -> tuple[TeacherActorCritic, dict[str, Any]]:
    payload = torch.load(resolve_path(checkpoint_path), map_location=device)
    if payload.get("kind") != "passage_gnn_ppo_teacher":
        raise ValueError(f"Unsupported checkpoint: {checkpoint_path}")
    model = TeacherActorCritic(
        max_acc=float(payload["max_acc"]),
        hidden_dim=int(payload["hidden_dim"]),
        gnn_layers=int(payload["gnn_layers"]),
        min_log_std=float(payload["min_log_std"]),
        max_log_std=float(payload["max_log_std"]),
        log_std_init=float(payload["log_std_init"]),
    ).to(device)
    model.load_state_dict(payload["model_state"])
    model.eval()
    return model, dict(payload["config"])


def evaluate(
    checkpoint: str | Path,
    episodes: int,
    splits: list[str],
    output: str | Path,
    stage: str = "C5",
    device_name: str = "auto",
) -> list[dict[str, Any]]:
    device = torch.device("cuda" if device_name == "auto" and torch.cuda.is_available() else ("cpu" if device_name == "auto" else device_name))
    model, config = load_policy(checkpoint, device)
    env_config = dict(config.get("env", {}))
    env_config["return_graph_obs"] = False
    teacher = TimeExpandedPassageTeacher(dict(config.get("teacher", {})))
    rows: list[dict[str, Any]] = []

    for split_idx, split in enumerate(splits):
        stats = evaluate_policy(
            model,
            env_config,
            teacher,
            device,
            stage=stage,
            split=split,
            episodes=episodes,
            seed_base=900_000 + split_idx * 100_000,
        )
        planner_env = TimeVaryingPassageMazeEnv({**env_config, "stage_name": stage, "split": split, "return_graph_obs": False})
        planner_successes = []
        for ep in range(episodes):
            planner_successes.append(float(rollout_teacher(planner_env, teacher, seed=950_000 + split_idx * 100_000 + ep)["success"]))
        row = {
            "split": split,
            "stage": stage,
            "episodes": episodes,
            **stats,
            "planner_success_rate": float(np.mean(planner_successes)),
            "planner_gap": float(np.mean(planner_successes) - stats["success_rate"]),
        }
        rows.append(row)
        print(
            f"{split}: PPO成功率={row['success_rate']:.2%} | "
            f"碰撞率={row['collision_rate']:.2%} | 超时率={row['timeout_rate']:.2%} | "
            f"Planner上界={row['planner_success_rate']:.2%}"
        )

    output_path = resolve_path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"已保存评估结果: {output_path}")
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="checkpoints/passage_generated/C5/teacher_final.pt")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--splits", default="id_test,ood_phase_test,ood_topology_test")
    parser.add_argument("--output", default="results/passage_generated/C5/eval_passage.csv")
    parser.add_argument("--stage", default="C5")
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()
    splits = [part.strip() for part in args.splits.split(",") if part.strip()]
    evaluate(args.checkpoint, args.episodes, splits, args.output, args.stage, args.device)


if __name__ == "__main__":
    main()
