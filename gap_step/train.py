from __future__ import annotations

import argparse
import csv
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import torch
from tqdm import trange

from gap_step.curriculum import STAGE_ORDER, stage_from_step
from gap_step.env import ContinuousMazeEnv
from gap_step.model import TeacherActorCritic
from gap_step.ppo import collect_rollout, get_device, ppo_update
from gap_step.utils import ensure_dir, load_yaml, resolve_path, set_seed


DEFAULT_CONFIG = {
    "seed": 0,
    "device": "auto",
    "steps_per_stage": 1_000_000,
    "total_steps": 5_000_000,
    "rollout_steps": 2048,
    "minibatch_size": 256,
    "update_epochs": 10,
    "learning_rate": 3e-4,
    "gamma": 0.99,
    "gae_lambda": 0.95,
    "clip_ratio": 0.2,
    "value_coef": 0.5,
    "entropy_coef": 0.01,
    "max_grad_norm": 0.5,
    "checkpoint_path": "checkpoints/teacher_final.pt",
    "train_metrics_path": "results/train_metrics.csv",
    "curriculum_mode": "fixed",
    "promotion_success_rate": 0.70,
    "promotion_window_episodes": 100,
    "min_steps_per_stage": 500_000,
    "soft_max_steps_per_stage": 2_000_000,
    "hard_max_steps_per_stage": 5_000_000,
    "hard_max_policy": "stop",
}


@dataclass
class AdaptiveCurriculumState:
    stage_index: int = 0
    stage_steps: int = 0
    stage_episodes: int = 0
    recent_successes: deque[bool] = field(default_factory=deque)
    soft_warning_emitted: bool = False

    @property
    def stage_name(self) -> str:
        return STAGE_ORDER[self.stage_index]

    def rolling_success_rate(self) -> float:
        if not self.recent_successes:
            return 0.0
        return float(np.mean(self.recent_successes))

    def advance(self) -> None:
        self.stage_index += 1
        self.stage_steps = 0
        self.stage_episodes = 0
        self.recent_successes.clear()
        self.soft_warning_emitted = False


def _mean_info(infos: list[dict], key: str) -> float:
    if not infos:
        return 0.0
    return float(np.mean([float(info.get(key, 0.0)) for info in infos]))


def adaptive_stage_status(
    state: AdaptiveCurriculumState,
    *,
    min_steps: int,
    soft_max_steps: int,
    hard_max_steps: int,
    promotion_success_rate: float,
    hard_max_policy: str,
) -> str:
    rolling_success_rate = state.rolling_success_rate()
    can_promote = (
        state.stage_steps >= min_steps
        and len(state.recent_successes) == state.recent_successes.maxlen
        and rolling_success_rate >= promotion_success_rate
    )
    if can_promote:
        return "completed_success" if state.stage_index == len(STAGE_ORDER) - 1 else "promoted_success"
    if state.stage_steps >= hard_max_steps:
        if hard_max_policy != "stop":
            raise ValueError(f"Unsupported hard_max_policy: {hard_max_policy}")
        return "hard_max_stop"
    if state.stage_steps >= soft_max_steps and not state.soft_warning_emitted:
        state.soft_warning_emitted = True
        return "soft_max_warning"
    return "training"


def _build_row(
    update: int,
    global_steps: int,
    stage_name: str,
    batch,
    metrics: dict[str, float],
    *,
    stage_steps: int,
    stage_episodes: int,
    rolling_success_rate: float,
    stage_status: str,
) -> dict:
    infos = batch.episode_infos
    step_infos = batch.step_infos
    return {
        "update": update,
        "global_steps": global_steps,
        "stage": stage_name,
        "stage_steps": stage_steps,
        "stage_episodes": stage_episodes,
        "rolling_success_rate": rolling_success_rate,
        "stage_status": stage_status,
        "episodes": len(infos),
        "average_return": float(np.mean(batch.episode_returns)) if batch.episode_returns else 0.0,
        "success_rate": _mean_info(infos, "success"),
        "collision_rate": _mean_info(infos, "collision"),
        "timeout_rate": _mean_info(infos, "timeout"),
        "progress_reward_mean": _mean_info(step_infos, "progress_reward"),
        "progress_delta_mean": _mean_info(step_infos, "progress_delta"),
        "progress_delta_abs_mean": float(np.mean([abs(float(info.get("progress_delta", 0.0))) for info in step_infos]))
        if step_infos
        else 0.0,
        "dynamic_path_wait_time_mean": _mean_info(step_infos, "dynamic_path_wait_time"),
        "dynamic_path_uses_gate_rate": _mean_info(step_infos, "dynamic_path_uses_gate"),
        "closed_gate_collision_rate": _mean_info(infos, "closed_gate_collision"),
        "wall_collision_rate": _mean_info(infos, "wall_collision"),
        "boundary_collision_rate": _mean_info(infos, "boundary_collision"),
        "average_action_norm": _mean_info(step_infos, "action_norm"),
        **metrics,
    }


def _write_outputs(config: dict, env: ContinuousMazeEnv, model: TeacherActorCritic, rows: list[dict]) -> None:
    ckpt_path = resolve_path(config["checkpoint_path"])
    ensure_dir(ckpt_path.parent)
    torch.save(
        {
            "model_state": model.state_dict(),
            "obs_dim": env.observation_space.shape[0],
            "max_acc": env.max_acc,
            "config": config,
            "stages": STAGE_ORDER,
        },
        ckpt_path,
    )

    metrics_path = resolve_path(config["train_metrics_path"])
    ensure_dir(metrics_path.parent)
    if rows:
        with metrics_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    print(f"Saved teacher checkpoint to {ckpt_path}")
    print(f"Saved train metrics to {metrics_path}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="gap_step/configs/train_teacher_smoke.yaml")
    args = parser.parse_args()

    config = dict(DEFAULT_CONFIG)
    path = resolve_path(args.config)
    if path.exists():
        config.update(load_yaml(path))

    seed = int(config["seed"])
    set_seed(seed)
    device = get_device(str(config["device"]))
    env = ContinuousMazeEnv(config.get("env", {}))
    model = TeacherActorCritic(obs_dim=env.observation_space.shape[0], max_acc=env.max_acc).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))

    rollout_steps = int(config["rollout_steps"])
    curriculum_mode = str(config.get("curriculum_mode", "fixed"))
    total_steps = int(config["total_steps"])
    steps_per_stage = int(config["steps_per_stage"])
    rows: list[dict] = []
    global_steps = 0

    if curriculum_mode == "adaptive":
        hard_max_steps = int(config["hard_max_steps_per_stage"])
        updates = max(1, int(np.ceil(hard_max_steps * len(STAGE_ORDER) / rollout_steps)))
        state = AdaptiveCurriculumState(recent_successes=deque(maxlen=int(config["promotion_window_episodes"])))
        min_steps = int(config["min_steps_per_stage"])
        soft_max_steps = int(config["soft_max_steps_per_stage"])
        promotion_success_rate = float(config["promotion_success_rate"])
        hard_max_policy = str(config.get("hard_max_policy", "stop"))
        stop_training = False

        for update in trange(updates, desc="teacher PPO"):
            stage_name = state.stage_name
            batch = collect_rollout(env, model, rollout_steps, device, stage_name=stage_name, seed=seed + update)
            metrics = ppo_update(
                model,
                optimizer,
                batch,
                device,
                gamma=float(config["gamma"]),
                gae_lambda=float(config["gae_lambda"]),
                clip_ratio=float(config["clip_ratio"]),
                value_coef=float(config["value_coef"]),
                entropy_coef=float(config["entropy_coef"]),
                update_epochs=int(config["update_epochs"]),
                minibatch_size=int(config["minibatch_size"]),
                max_grad_norm=float(config["max_grad_norm"]),
            )
            global_steps += rollout_steps
            state.stage_steps += rollout_steps
            state.stage_episodes += len(batch.episode_infos)
            state.recent_successes.extend(bool(info["success"]) for info in batch.episode_infos)
            rolling_success_rate = state.rolling_success_rate()

            stage_status = adaptive_stage_status(
                state,
                min_steps=min_steps,
                soft_max_steps=soft_max_steps,
                hard_max_steps=hard_max_steps,
                promotion_success_rate=promotion_success_rate,
                hard_max_policy=hard_max_policy,
            )

            rows.append(
                _build_row(
                    update + 1,
                    global_steps,
                    stage_name,
                    batch,
                    metrics,
                    stage_steps=state.stage_steps,
                    stage_episodes=state.stage_episodes,
                    rolling_success_rate=rolling_success_rate,
                    stage_status=stage_status,
                )
            )

            if stage_status == "completed_success" or stage_status == "hard_max_stop":
                stop_training = True
            elif stage_status == "promoted_success":
                state.advance()
            if stop_training:
                break
    elif curriculum_mode == "fixed":
        updates = max(1, total_steps // rollout_steps)
        fixed_stage_episodes = {stage: 0 for stage in STAGE_ORDER}
        for update in trange(updates, desc="teacher PPO"):
            stage_name = stage_from_step(global_steps, steps_per_stage)
            stage_start = STAGE_ORDER.index(stage_name) * steps_per_stage
            stage_steps = global_steps - stage_start + rollout_steps
            batch = collect_rollout(env, model, rollout_steps, device, stage_name=stage_name, seed=seed + update)
            metrics = ppo_update(
                model,
                optimizer,
                batch,
                device,
                gamma=float(config["gamma"]),
                gae_lambda=float(config["gae_lambda"]),
                clip_ratio=float(config["clip_ratio"]),
                value_coef=float(config["value_coef"]),
                entropy_coef=float(config["entropy_coef"]),
                update_epochs=int(config["update_epochs"]),
                minibatch_size=int(config["minibatch_size"]),
                max_grad_norm=float(config["max_grad_norm"]),
            )
            global_steps += rollout_steps
            fixed_stage_episodes[stage_name] += len(batch.episode_infos)
            rows.append(
                _build_row(
                    update + 1,
                    global_steps,
                    stage_name,
                    batch,
                    metrics,
                    stage_steps=stage_steps,
                    stage_episodes=fixed_stage_episodes[stage_name],
                    rolling_success_rate=0.0,
                    stage_status="training",
                )
            )
    else:
        raise ValueError(f"Unsupported curriculum_mode: {curriculum_mode}")

    _write_outputs(config, env, model, rows)


if __name__ == "__main__":
    main()
