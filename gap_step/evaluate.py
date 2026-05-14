from __future__ import annotations

import argparse
import csv

import numpy as np
import torch
from tqdm import tqdm

from gap_step.curriculum import STAGE_ORDER
from gap_step.env import ContinuousMazeEnv
from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.ppo import get_device
from gap_step.utils import ensure_dir, resolve_path


SPLITS = {
    "id_test": range(10000, 10200),
    "ood_size_test": range(20000, 20200),
    "ood_dynamics_test": range(30000, 30200),
}


def load_teacher(checkpoint: str, device: torch.device) -> TeacherActorCritic:
    ckpt = torch.load(resolve_path(checkpoint), map_location=device, weights_only=False)
    model = TeacherActorCritic(
        global_dim=int(ckpt.get("global_dim", GLOBAL_FEATURE_DIM)),
        node_dim=int(ckpt.get("node_dim", NODE_FEATURE_DIM)),
        edge_dim=int(ckpt.get("edge_dim", EDGE_FEATURE_DIM)),
        hidden_dim=int(ckpt.get("hidden_dim", 128)),
        gnn_layers=int(ckpt.get("gnn_layers", 4)),
        max_acc=float(ckpt.get("max_acc", 3.0)),
        min_log_std=float(ckpt.get("min_log_std", -0.5)),
        max_log_std=float(ckpt.get("max_log_std", 2.0)),
    ).to(device)
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    return model


def evaluate_split(model: TeacherActorCritic, split: str, episodes: int, device: torch.device, stage_name: str = "C5") -> dict:
    env = ContinuousMazeEnv({"stage_name": stage_name, "split": split})
    seeds = list(SPLITS[split])[:episodes]
    stats = []
    with torch.no_grad():
        for seed in tqdm(seeds, desc=f"eval {stage_name} {split}"):
            obs, _ = env.reset(seed=seed, options={"stage_name": stage_name, "split": split})
            done = False
            ep_return = 0.0
            action_norms = []
            final_info = {}
            while not done:
                obs_t = collate_graph_obs([obs], device)
                action, _, _ = model.act(obs_t, deterministic=True)
                action_np = action.squeeze(0).cpu().numpy()
                obs, reward, terminated, truncated, final_info = env.step(action_np)
                ep_return += reward
                action_norms.append(float(np.linalg.norm(action_np)))
                done = terminated or truncated
            row = dict(final_info)
            row["return"] = ep_return
            row["steps"] = env.step_count
            row["average_action_norm"] = float(np.mean(action_norms)) if action_norms else 0.0
            stats.append(row)

    return {
        "split": split,
        "stage": stage_name,
        "episodes": len(stats),
        "success_rate": float(np.mean([s["success"] for s in stats])),
        "collision_rate": float(np.mean([s["collision"] for s in stats])),
        "timeout_rate": float(np.mean([s["timeout"] for s in stats])),
        "average_return": float(np.mean([s["return"] for s in stats])),
        "average_steps": float(np.mean([s["steps"] for s in stats])),
        "closed_gate_collision_rate": float(np.mean([s["closed_gate_collision"] for s in stats])),
        "wall_collision_rate": float(np.mean([s["wall_collision"] for s in stats])),
        "boundary_collision_rate": float(np.mean([s["boundary_collision"] for s in stats])),
        "average_action_norm": float(np.mean([s["average_action_norm"] for s in stats])),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", default="checkpoints/C5/teacher_final.pt")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--output", default="results/eval_metrics.csv")
    parser.add_argument("--stages", default="")
    args = parser.parse_args()

    device = get_device(args.device)
    model = load_teacher(args.checkpoint, device)
    stages = [stage.strip() for stage in args.stages.split(",") if stage.strip()]
    if stages:
        unknown = [stage for stage in stages if stage not in STAGE_ORDER]
        if unknown:
            raise ValueError(f"Unknown stages: {unknown}")
        rows = [evaluate_split(model, "id_test", args.episodes, device, stage_name=stage) for stage in stages]
    else:
        rows = [evaluate_split(model, split, args.episodes, device, stage_name="C5") for split in SPLITS]
    output = resolve_path(args.output)
    ensure_dir(output.parent)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved evaluation to {output}")
    for row in rows:
        print(row)


if __name__ == "__main__":
    main()
