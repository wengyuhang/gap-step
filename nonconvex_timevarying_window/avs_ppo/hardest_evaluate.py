"""Evaluation and dense whole-body audit for the hardest AVS-PPO transfer."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import differential_evolution
import torch

from nonconvex_timevarying_window.sc_dynatogt.collision import (
    point_to_oriented_cuboid_distance_squared,
)

from .hardest_comparison import (
    HardestComparisonAVSEnvironment,
    HardestExperimentConfig,
    load_hardest_config,
)
from .model import MaskedActorCritic
from .ppo import device_from_config


def load_hardest_checkpoint(
    path: str | Path,
    config: HardestExperimentConfig,
    device: torch.device,
) -> MaskedActorCritic:
    probe = HardestComparisonAVSEnvironment(config.environment)
    model = MaskedActorCritic(probe.observation_dim, probe.action_dim, config.ppo.hidden_size).to(device)
    payload = torch.load(path, map_location=device, weights_only=False)
    model.load_state_dict(payload["model"])
    return model


def rollout_policy(
    model: MaskedActorCritic,
    config: HardestExperimentConfig,
    *,
    seed: int = 0,
) -> tuple[HardestComparisonAVSEnvironment, dict[str, Any]]:
    device = next(model.parameters()).device
    environment = HardestComparisonAVSEnvironment(config.environment, seed=seed)
    observation, _ = environment.reset(seed=seed)
    done = False
    info: dict[str, Any] = {}
    model.eval()
    while not done:
        mask = environment.action_mask()
        with torch.no_grad():
            distribution = model.distribution(
                torch.as_tensor(observation[None], dtype=torch.float32, device=device),
                torch.as_tensor(mask[None], dtype=torch.bool, device=device),
            )
            action = int(torch.argmax(distribution.logits, dim=-1).item())
        observation, _, terminated, truncated, info = environment.step(action, mask=mask)
        done = terminated or truncated
    return environment, info


def evaluate_hardest_policy(
    model: MaskedActorCritic,
    config: HardestExperimentConfig,
    episodes: int,
    *,
    seed: int = 0,
) -> dict[str, float]:
    records = [rollout_policy(model, config, seed=seed + episode)[1] for episode in range(episodes)]
    successful_times = [float(record["elapsed_time"]) for record in records if record["success"]]
    return {
        "episodes": int(episodes),
        "success_rate": float(np.mean([record["success"] for record in records])),
        "safety_violation_rate": float(np.mean([record["safety_violations"] > 0 for record in records])),
        "total_safety_violations": int(sum(record["safety_violations"] for record in records)),
        "mean_success_time": float(np.mean(successful_times)) if successful_times else float("nan"),
        "minimum_whole_body_clearance": min(
            (float(record["minimum_whole_body_clearance"]) for record in records),
            default=float("nan"),
        ),
        "minimum_crossing_clearance": min(
            (float(record["minimum_crossing_clearance"]) for record in records),
            default=float("nan"),
        ),
        "required_clearance": 0.015,
        "shield_clearance_threshold": float(config.environment.numerical_guard + 0.015),
        "mean_gates_crossed": float(np.mean([record["gates_crossed"] for record in records])),
    }


def dense_whole_body_audit(
    environment: HardestComparisonAVSEnvironment,
    *,
    time_substeps: int = 10,
    boundary_samples: int = 101,
) -> dict[str, Any]:
    parameters = np.linspace(0.0, 1.0, boundary_samples)
    local = tuple(
        tuple(
            np.asarray([segment.evaluate(float(u)) for u in parameters], dtype=float)
            for segment in window.boundary
        )
        for window in environment.problem.windows
    )
    best = {
        "distance": float("inf"), "time": None, "window_index": None,
        "window_name": None, "boundary_index": None,
    }
    maximum_speed = 0.0
    maximum_acceleration = 0.0
    for record in environment.history:
        acceleration = np.asarray(record["acceleration"])
        rotation = np.asarray(record["rotation"])
        maximum_acceleration = max(maximum_acceleration, float(np.linalg.norm(acceleration)))
        for tau in np.linspace(0.0, environment.config.dt, time_substeps + 1):
            time = float(record["time"] + tau)
            position = record["position"] + record["velocity"] * tau + 0.5 * acceleration * tau * tau
            velocity = record["velocity"] + acceleration * tau
            maximum_speed = max(maximum_speed, float(np.linalg.norm(velocity)))
            for window_index, window in enumerate(environment.problem.windows):
                center, basis, scale = window.state_at(time)
                for boundary_index, points in enumerate(local[window_index]):
                    world = center + (
                        basis @ np.column_stack((scale * points, np.zeros(len(points)))).T
                    ).T
                    distance = np.sqrt(
                        point_to_oriented_cuboid_distance_squared(
                            world, position, rotation, environment.body
                        )
                    )
                    value = float(np.min(distance))
                    if value < best["distance"]:
                        best.update({
                            "distance": value,
                            "time": time,
                            "window_index": window_index,
                            "window_name": window.name,
                            "boundary_index": boundary_index,
                        })
    target_window = int(best["window_index"])
    target_boundary = int(best["boundary_index"])
    window = environment.problem.windows[target_window]
    boundary = window.boundary[target_boundary]
    refined = {"distance": float("inf"), "time": None, "boundary_parameter": None}
    for record in environment.history:
        if abs(float(record["time"]) - float(best["time"])) > 1.5 * environment.config.dt:
            continue

        def objective(values: np.ndarray) -> float:
            tau, parameter = map(float, values)
            time = float(record["time"] + tau)
            position = (
                record["position"] + record["velocity"] * tau
                + 0.5 * record["acceleration"] * tau * tau
            )
            center, basis, scale = window.state_at(time)
            point = center + basis @ np.r_[scale * boundary.evaluate(parameter), 0.0]
            return float(point_to_oriented_cuboid_distance_squared(
                point, position, record["rotation"], environment.body
            ))

        result = differential_evolution(
            objective,
            [(0.0, environment.config.dt), (0.0, 1.0)],
            seed=11,
            tol=1.0e-11,
            polish=True,
            workers=1,
        )
        distance = float(np.sqrt(max(0.0, result.fun)))
        if distance < refined["distance"]:
            refined = {
                "distance": distance,
                "time": float(record["time"] + result.x[0]),
                "window_index": target_window,
                "window_name": window.name,
                "boundary_index": target_boundary,
                "boundary_parameter": float(result.x[1]),
                "extra_clearance": distance - environment.required_clearance,
            }

    return {
        "status": "SAMPLED_SAFE" if best["distance"] >= environment.required_clearance else "SAMPLED_UNSAFE",
        "minimum_clearance": best,
        "required_clearance": environment.required_clearance,
        "extra_clearance": float(best["distance"] - environment.required_clearance),
        "locally_refined_minimum": refined,
        "time_substeps_per_control_step": int(time_substeps),
        "boundary_samples_per_segment": int(boundary_samples),
        "maximum_speed": maximum_speed,
        "maximum_acceleration": maximum_acceleration,
        "all_crossings_inside": bool(
            len(environment.crossing_records) == len(environment.problem.order)
            and all(record["inside"] for record in environment.crossing_records)
        ),
        "crossings": [
            {
                key: value for key, value in record.items()
                if key in {"route_index", "window_index", "window_name", "time", "inside", "clearance"}
            }
            for record in environment.crossing_records
        ],
        "note": "Dense numerical audit; unlike the SIP reference, the learned rollout has no Arb certificate.",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output")
    parser.add_argument("--dense-audit", action="store_true")
    args = parser.parse_args()
    config = load_hardest_config(args.config)
    device = device_from_config(config.ppo.device)
    model = load_hardest_checkpoint(args.checkpoint, config, device)
    metrics: dict[str, Any] = evaluate_hardest_policy(model, config, args.episodes, seed=args.seed)
    if args.dense_audit:
        environment, rollout = rollout_policy(model, config, seed=args.seed)
        metrics["representative_rollout"] = rollout
        metrics["dense_whole_body_audit"] = dense_whole_body_audit(environment)
    encoded = json.dumps(metrics, ensure_ascii=False, indent=2)
    print(encoded)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(encoded + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
