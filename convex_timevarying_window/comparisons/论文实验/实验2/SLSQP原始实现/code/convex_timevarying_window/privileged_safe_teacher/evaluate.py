#!/usr/bin/env python3
"""Evaluate a trained teacher with direct and safety-filtered online control."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from .model import DirectControlTeacher
from .train import evaluate, summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--seed", type=int, default=5000)
    parser.add_argument("--recovery-fraction", type=float, default=0.70)
    args = parser.parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    payload = torch.load(args.checkpoint, map_location=device, weights_only=False)
    model = DirectControlTeacher(observation_dim=int(payload["observation_dim"])).to(device)
    model.load_state_dict(payload["model_state"])
    records = evaluate(
        model,
        args.episodes,
        args.seed,
        device,
        use_safety_filter=True,
        recovery_fraction=args.recovery_fraction,
    )
    result = {
        "status": "SAMPLED_SAFE" if all(row["finished"] and not row["collision"] for row in records) else "FAILED",
        "checkpoint": str(args.checkpoint.resolve()),
        "learned_action_fraction": 1.0 - args.recovery_fraction,
        "planner_recovery_fraction": args.recovery_fraction,
        "summary": summary(records),
        "episodes": records,
        "evidence": "20 ms x500 rigid-body sampled rollout with predictive frame filter; not continuous certification or Gazebo/PX4 evidence",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
