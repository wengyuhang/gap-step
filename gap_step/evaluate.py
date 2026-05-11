from __future__ import annotations

import argparse
import csv

import numpy as np
import torch
from tqdm import tqdm

from gap_step.env import ContinuousMazeEnv
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
    model = TeacherActorCritic(obs_dim=int(ckpt.get("obs_dim", 39)), max_acc=float(ckpt.get("max_acc", 3.0))).to(device)
    model.load_state_dict(ckpt["model_state"])
    model.eval()
    return model


def evaluate_split(model: TeacherActorCritic, split: str, episodes: int, device: torch.device) -> dict:
    env = ContinuousMazeEnv({"stage_name": "C5", "split": split})
    seeds = list(SPLITS[split])[:episodes]
    stats = []
    with torch.no_grad():
        for seed in tqdm(seeds, desc=f"eval {split}"):
            obs, _ = env.reset(seed=seed, options={"stage_name": "C5", "split": split})
            done = False
            ep_return = 0.0
            action_norms = []
            final_info = {}
            while not done:
                obs_t = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
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
    parser.add_argument("--checkpoint", default="checkpoints/teacher_final.pt")
    parser.add_argument("--episodes", type=int, default=200)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--output", default="results/eval_metrics.csv")
    args = parser.parse_args()

    device = get_device(args.device)
    model = load_teacher(args.checkpoint, device)
    rows = [evaluate_split(model, split, args.episodes, device) for split in SPLITS]
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
