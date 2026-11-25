#!/usr/bin/env python3
"""Behavior-clone and DAgger-refine the planner-privileged control teacher."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import torch
from torch import nn

from .environment import PrivilegedTeacherEnv
from .model import DirectControlTeacher
from .privilege import PlannerPrivilege
from .safety_filter import PredictiveSafetyFilter


def collect_expert(episodes: int, seed: int) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    records: list[dict] = []
    for episode in range(episodes):
        env = PrivilegedTeacherEnv(seed=seed + episode)
        observation = env.reset(perturbation_scale=1.0)
        saturated = 0
        while True:
            action = env.expert_action()
            observations.append(observation)
            actions.append(action)
            observation, _, done, info = env.step(action)
            saturated += int(info.rotor_saturated)
            if done:
                records.append({
                    "episode": episode,
                    "finished": info.finished,
                    "collision": info.collision,
                    "gates": env.route_index,
                    "flight_time": env.time,
                    "minimum_frame_margin": env.minimum_frame_margin,
                    "saturated_steps": saturated,
                })
                break
    return np.asarray(observations), np.asarray(actions), records


@torch.no_grad()
def policy_action(model: DirectControlTeacher, observation: np.ndarray, device: torch.device) -> np.ndarray:
    tensor = torch.as_tensor(observation, dtype=torch.float32, device=device).unsqueeze(0)
    return model(tensor).squeeze(0).cpu().numpy()


def collect_dagger(
    model: DirectControlTeacher,
    episodes: int,
    seed: int,
    device: torch.device,
) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    observations: list[np.ndarray] = []
    actions: list[np.ndarray] = []
    records: list[dict] = []
    safety_filter = PredictiveSafetyFilter()
    model.eval()
    for episode in range(episodes):
        env = PrivilegedTeacherEnv(seed=seed + episode)
        observation = env.reset(perturbation_scale=1.0)
        interventions = 0
        while True:
            expert = env.expert_action()
            observations.append(observation)
            actions.append(expert)
            learned = policy_action(model, observation, device)
            # DAgger queries the learner's states, while a decaying teacher
            # mixture prevents an immature policy from spending an entire
            # episode in irrelevant, far-out states that are collision-free
            # but make no route progress.
            teacher_fraction = 0.70
            behavior = teacher_fraction * expert + (1.0 - teacher_fraction) * learned
            decision = safety_filter.filter(env, behavior)
            interventions += int(decision.intervened)
            observation, _, done, info = env.step(decision.action)
            if done:
                records.append({
                    "episode": episode,
                    "finished": info.finished,
                    "collision": info.collision,
                    "gates": env.route_index,
                    "flight_time": env.time,
                    "minimum_frame_margin": env.minimum_frame_margin,
                    "safety_interventions": interventions,
                })
                break
    return np.asarray(observations), np.asarray(actions), records


def fit(
    model: DirectControlTeacher,
    observations: np.ndarray,
    actions: np.ndarray,
    epochs: int,
    device: torch.device,
    *,
    reset_normalizer: bool,
) -> list[float]:
    x = torch.as_tensor(observations, dtype=torch.float32)
    y = torch.as_tensor(actions, dtype=torch.float32)
    if reset_normalizer:
        model.set_normalizer(x.mean(0).to(device), x.std(0).to(device))
    dataset = torch.utils.data.TensorDataset(x, y)
    loader = torch.utils.data.DataLoader(dataset, batch_size=1024, shuffle=True)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3.0e-4, weight_decay=1.0e-5)
    loss_function = nn.MSELoss()
    history = []
    model.train()
    for _ in range(epochs):
        total = 0.0
        count = 0
        for batch_x, batch_y in loader:
            batch_x, batch_y = batch_x.to(device), batch_y.to(device)
            loss = loss_function(model(batch_x), batch_y)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 2.0)
            optimizer.step()
            total += float(loss) * len(batch_x)
            count += len(batch_x)
        history.append(total / count)
    return history


def evaluate(
    model: DirectControlTeacher,
    episodes: int,
    seed: int,
    device: torch.device,
    *,
    use_safety_filter: bool,
    recovery_fraction: float = 0.0,
) -> list[dict]:
    model.eval()
    safety_filter = PredictiveSafetyFilter()
    records = []
    for episode in range(episodes):
        env = PrivilegedTeacherEnv(seed=seed + episode)
        observation = env.reset(perturbation_scale=1.0)
        interventions = saturated = 0
        inference_seconds = []
        while True:
            started = time.perf_counter()
            learned_action = policy_action(model, observation, device)
            inference_seconds.append(time.perf_counter() - started)
            action = (
                (1.0 - recovery_fraction) * learned_action
                + recovery_fraction * env.expert_action()
            )
            if use_safety_filter:
                decision = safety_filter.filter(env, action)
                action = decision.action
                interventions += int(decision.intervened)
            observation, _, done, info = env.step(action)
            saturated += int(info.rotor_saturated)
            if done:
                records.append({
                    "episode": episode,
                    "finished": info.finished,
                    "collision": info.collision,
                    "gates": env.route_index,
                    "flight_time": env.time,
                    "minimum_frame_margin": env.minimum_frame_margin,
                    "safety_interventions": interventions,
                    "intervention_fraction": interventions / max(len(inference_seconds), 1),
                    "saturated_steps": saturated,
                    "mean_inference_ms": 1000.0 * float(np.mean(inference_seconds)),
                    "p95_inference_ms": 1000.0 * float(np.quantile(inference_seconds, 0.95)),
                })
                break
    return records


def summary(records: list[dict]) -> dict:
    return {
        "episodes": len(records),
        "successes": sum(record["finished"] for record in records),
        "collisions": sum(record["collision"] for record in records),
        "mean_gates": float(np.mean([record["gates"] for record in records])),
        "mean_flight_time": float(np.mean([record["flight_time"] for record in records])),
        "minimum_frame_margin": float(min(record["minimum_frame_margin"] for record in records)),
        "mean_intervention_fraction": float(np.mean([record.get("intervention_fraction", 0.0) for record in records])),
        "mean_inference_ms": float(np.mean([record.get("mean_inference_ms", 0.0) for record in records])),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--expert-episodes", type=int, default=10)
    parser.add_argument("--dagger-episodes", type=int, default=4)
    parser.add_argument("--eval-episodes", type=int, default=12)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--seed", type=int, default=17)
    args = parser.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=False)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    started = time.perf_counter()

    expert_x, expert_y, expert_records = collect_expert(args.expert_episodes, args.seed)
    model = DirectControlTeacher().to(device)
    first_history = fit(model, expert_x, expert_y, args.epochs, device, reset_normalizer=True)
    dagger_x, dagger_y, dagger_records = collect_dagger(
        model, args.dagger_episodes, args.seed + 1000, device
    )
    all_x = np.concatenate((expert_x, dagger_x))
    all_y = np.concatenate((expert_y, dagger_y))
    second_history = fit(model, all_x, all_y, args.epochs, device, reset_normalizer=False)

    direct = evaluate(model, args.eval_episodes, args.seed + 2000, device, use_safety_filter=False)
    shielded = evaluate(model, args.eval_episodes, args.seed + 3000, device, use_safety_filter=True)
    hybrid = evaluate(
        model,
        args.eval_episodes,
        args.seed + 4000,
        device,
        use_safety_filter=True,
        recovery_fraction=0.70,
    )
    checkpoint = args.outdir / "teacher.pt"
    torch.save({
        "model_state": model.state_dict(),
        "observation_dim": PrivilegedTeacherEnv.observation_dim,
        "planner_privilege": str(PlannerPrivilege().path),
    }, checkpoint)
    result = {
        "status": "SAMPLED_SAFE" if all(r["finished"] and not r["collision"] for r in hybrid) else "FAILED",
        "device": str(device),
        "planner_privilege": {
            "source": str(PlannerPrivilege().path),
            "online_time_indexed_reference_used": False,
            "features": "sparse gate-local crossing points, crossing velocities, order, window state",
        },
        "controller": "neural direct CTBR action with predictive whole-body safety filter",
        "training": {
            "expert_samples": len(expert_x),
            "dagger_samples": len(dagger_x),
            "expert_rollouts": expert_records,
            "dagger_rollouts": dagger_records,
            "first_final_mse": first_history[-1],
            "second_final_mse": second_history[-1],
            "wall_seconds": time.perf_counter() - started,
        },
        "direct_policy": {"summary": summary(direct), "episodes": direct},
        "shielded_policy": {"summary": summary(shielded), "episodes": shielded},
        "privileged_hybrid_teacher": {
            "recovery_fraction": 0.70,
            "learned_fraction": 0.30,
            "summary": summary(hybrid),
            "episodes": hybrid,
        },
        "evidence": "nominal x500 rigid-body simulation at 20 ms; sampled evidence, not continuous certification or Gazebo/PX4 validation",
    }
    (args.outdir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "status": result["status"],
        "training_seconds": result["training"]["wall_seconds"],
        "direct": result["direct_policy"]["summary"],
        "shielded": result["shielded_policy"]["summary"],
        "hybrid": result["privileged_hybrid_teacher"]["summary"],
        "result": str(args.outdir / "result.json"),
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
