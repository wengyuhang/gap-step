from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import torch
from tqdm import tqdm

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gap_step.config import ensure_dir, load_yaml, resolve_path
from gap_step.envs import GapStepEnv
from gap_step.models import StudentPolicy
from gap_step.teachers import HeuristicTeacher
from gap_step.torch_utils import get_device, obs_to_torch, set_seed


def safe_f1(pred: list[int], true: list[int]) -> float:
    if not pred:
        return float("nan")
    p = np.asarray(pred) > 0
    t = np.asarray(true) > 0
    tp = float(np.logical_and(p, t).sum())
    fp = float(np.logical_and(p, ~t).sum())
    fn = float(np.logical_and(~p, t).sum())
    denom = 2 * tp + fp + fn
    return 0.0 if denom == 0 else 2 * tp / denom


def load_model(path: Path, env: GapStepEnv, device: torch.device) -> StudentPolicy | None:
    if not path.exists():
        return None
    ckpt = torch.load(path, map_location=device, weights_only=False)
    model = StudentPolicy(env.K_obs, 6, env.num_gates, env.max_acc).to(device)
    model.load_state_dict(ckpt["model_state"], strict=False)
    model.eval()
    return model


def eval_policy(name: str, env_config: dict, device: torch.device, episodes: int, checkpoint: Path | None = None, seed: int = 0):
    env = GapStepEnv(env_config)
    teacher = HeuristicTeacher(env)
    model = None if checkpoint is None else load_model(checkpoint, env, device)
    if checkpoint is not None and model is None:
        print(f"Skipping {name}; missing checkpoint {checkpoint}")
        return None

    stats = []
    gate_correct = gate_total = 0
    width_abs = []
    safe_pred, safe_true = [], []
    for ep in tqdm(range(episodes), desc=f"Evaluating {name}"):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        ep_return = 0.0
        ep_info = {"success": False, "collision": False, "crossing_success": False, "closed_gate_attempt": False}
        while not done:
            teacher_acc, teacher_gate = teacher.act()
            if model is None:
                action = teacher_acc
                pred_gate = teacher_gate
            else:
                with torch.no_grad():
                    image, proprio = obs_to_torch(obs, device)
                    out = model(image, proprio)
                action = out["acc"].squeeze(0).cpu().numpy()
                pred_gate = int(torch.argmax(out["gate_logits"], dim=-1).item())
                widths, safe = env.get_gate_labels()
                width_abs.extend(np.abs(out["width"].squeeze(0).cpu().numpy() - widths).tolist())
                safe_pred.extend((torch.sigmoid(out["safe_logits"]).squeeze(0).cpu().numpy() > 0.5).astype(int).tolist())
                safe_true.extend(safe.astype(int).tolist())
            gate_correct += int(pred_gate == teacher_gate)
            gate_total += 1
            obs, reward, terminated, truncated, info = env.step(action)
            ep_return += reward
            done = terminated or truncated
            for key in ep_info:
                ep_info[key] = ep_info[key] or bool(info.get(key, False))
        ep_info["return"] = ep_return
        ep_info["time_to_goal"] = env.step_count if ep_info["success"] else np.nan
        stats.append(ep_info)

    time_values = np.asarray([s["time_to_goal"] for s in stats], dtype=np.float32)
    mean_time = float(np.nanmean(time_values)) if not np.all(np.isnan(time_values)) else np.nan
    row = {
        "model": name,
        "success_rate": np.mean([s["success"] for s in stats]),
        "collision_rate": np.mean([s["collision"] for s in stats]),
        "crossing_success_rate": np.mean([s["crossing_success"] for s in stats]),
        "closed_gate_attempt_rate": np.mean([s["closed_gate_attempt"] for s in stats]),
        "time_to_goal": mean_time,
        "gate_choice_accuracy": gate_correct / max(1, gate_total),
        "width_mae": float(np.mean(width_abs)) if width_abs else np.nan,
        "safe_f1": safe_f1(safe_pred, safe_true),
        "return": np.mean([s["return"] for s in stats]),
    }
    return row


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-config", default="configs/env_e3.yaml")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--output", default="logs/eval_results.csv")
    parser.add_argument("--checkpoint-dir", default="checkpoints")
    args = parser.parse_args()

    set_seed(0)
    env_config = load_yaml(args.env_config)
    device = get_device(args.device)
    ckpt_dir = resolve_path(args.checkpoint_dir)
    candidates = [
        ("Teacher-Heuristic", None),
        ("BC-only", ckpt_dir / "bc_only.pt"),
        ("BC+Aux", ckpt_dir / "bc_aux.pt"),
        ("Visual-PPO", ckpt_dir / "visual_ppo.pt"),
        ("BC+Aux+PPO", ckpt_dir / "bc_aux_ppo.pt"),
    ]
    rows = []
    for name, ckpt in candidates:
        row = eval_policy(name, env_config, device, args.episodes, ckpt)
        if row is not None:
            rows.append(row)
    df = pd.DataFrame(rows)
    output = resolve_path(args.output)
    ensure_dir(output.parent)
    df.to_csv(output, index=False)
    print(df.to_string(index=False))
    print(f"Saved evaluation to {output}")


if __name__ == "__main__":
    main()
