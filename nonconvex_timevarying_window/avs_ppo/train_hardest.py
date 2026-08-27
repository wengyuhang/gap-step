"""Train AVS-PPO on the frozen hardest six-window comparison track."""

from __future__ import annotations

import argparse
import csv
import json
import random
from pathlib import Path

import numpy as np
import torch

from .hardest_comparison import HardestComparisonAVSEnvironment, load_hardest_config
from .hardest_evaluate import evaluate_hardest_policy
from .model import MaskedActorCritic
from .ppo import collect_rollout, device_from_config, make_old_policy, update


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--outdir", required=True)
    args = parser.parse_args()
    config = load_hardest_config(args.config)
    output = Path(args.outdir)
    output.mkdir(parents=True, exist_ok=True)
    seed = config.ppo.seed
    random.seed(seed); np.random.seed(seed); torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    device = device_from_config(config.ppo.device)
    environments = [
        HardestComparisonAVSEnvironment(config.environment, seed=seed + index)
        for index in range(config.ppo.num_envs)
    ]
    probe = environments[0]
    model = MaskedActorCritic(probe.observation_dim, probe.action_dim, config.ppo.hidden_size).to(device)
    model_old = make_old_policy(model).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=config.ppo.learning_rate)
    observations = None
    rows = []
    initial = evaluate_hardest_policy(model, config, config.ppo.eval_episodes, seed=30000)
    rows.append({"update": 0, **initial})
    print(json.dumps(rows[-1]))
    best_success = initial["success_rate"]
    best_time = initial["mean_success_time"] if np.isfinite(initial["mean_success_time"]) else float("inf")
    torch.save({"model": model.state_dict(), "config": config.to_dict(), "update": 0, "metrics": initial}, output / "best.pt")
    for update_index in range(1, config.ppo.updates + 1):
        model_old.load_state_dict(model.state_dict()); model_old.eval()
        rollout, observations = collect_rollout(
            environments, model_old, config.ppo.rollout_steps, device, observations
        )
        losses = update(model, optimizer, rollout, config.ppo, device)
        if update_index % config.ppo.eval_interval == 0 or update_index == config.ppo.updates:
            metrics = evaluate_hardest_policy(model, config, config.ppo.eval_episodes, seed=30000)
            row = {"update": update_index, **metrics, **losses}
            rows.append(row)
            print(json.dumps(row))
            candidate_time = metrics["mean_success_time"] if np.isfinite(metrics["mean_success_time"]) else float("inf")
            if metrics["success_rate"] > best_success or (
                metrics["success_rate"] == best_success and candidate_time < best_time
            ):
                best_success, best_time = metrics["success_rate"], candidate_time
                torch.save({
                    "model": model.state_dict(), "config": config.to_dict(),
                    "update": update_index, "metrics": metrics,
                }, output / "best.pt")
    torch.save({
        "model": model.state_dict(), "config": config.to_dict(),
        "update": config.ppo.updates, "metrics": rows[-1],
    }, output / "final.pt")
    with (output / "learning_curve.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=sorted({key for row in rows for key in row}))
        writer.writeheader(); writer.writerows(rows)
    (output / "config.json").write_text(
        json.dumps(config.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()

