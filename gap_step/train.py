from __future__ import annotations

import argparse
import csv
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import torch

from gap_step.curriculum import STAGE_ORDER, stage_from_step
from gap_step.env import ContinuousMazeEnv
from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.ppo import collect_rollout, get_device, ppo_update, sync_policy_old
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
    "entropy_coef": 0.001,
    "target_kl": 0.03,
    "max_grad_norm": 0.5,
    "normalize_advantage": True,
    "min_log_std": -0.5,
    "max_log_std": 0.5,
    "model_type": "gnn",
    "gnn_hidden_dim": 128,
    "gnn_layers": 4,
    "checkpoint_path": "checkpoints/teacher_final.pt",
    "train_metrics_path": "results/train_metrics.csv",
    "checkpoint_dir": "checkpoints",
    "results_dir": "results",
    "stage_order": STAGE_ORDER,
    "log_interval_updates": 1,
    "curriculum_mode": "stagewise",
    "promotion_success_rate": 0.70,
    "promotion_eval_success_rate": 0.60,
    "promotion_eval_episodes": 50,
    "promotion_eval_interval_updates": 10,
    "promotion_window_episodes": 100,
    "min_steps_per_stage": 500_000,
    "soft_max_steps_per_stage": 5_000_000,
    "hard_max_steps_per_stage": 10_000_000,
    "hard_max_policy": "stop",
}


@dataclass
class AdaptiveCurriculumState:
    stage_index: int = 0
    stage_steps: int = 0
    stage_episodes: int = 0
    recent_successes: deque[bool] = field(default_factory=deque)
    soft_warning_emitted: bool = False
    last_promotion_eval_success_rate: float = 0.0
    last_promotion_eval_episodes: int = 0
    last_promotion_eval_update: int = -1

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
        self.last_promotion_eval_success_rate = 0.0
        self.last_promotion_eval_episodes = 0
        self.last_promotion_eval_update = -1


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
    promotion_eval_success_rate: float = 0.0,
) -> str:
    rolling_success_rate = state.rolling_success_rate()
    can_promote = (
        state.stage_steps >= min_steps
        and len(state.recent_successes) == state.recent_successes.maxlen
        and rolling_success_rate >= promotion_success_rate
        and state.last_promotion_eval_success_rate >= promotion_eval_success_rate
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
    promotion_eval_success_rate: float = 0.0,
    promotion_eval_episodes: int = 0,
    obs_dim: int = 0,
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
        "promotion_eval_success_rate": promotion_eval_success_rate,
        "promotion_eval_episodes": promotion_eval_episodes,
        "stage_status": stage_status,
        "obs_dim": obs_dim,
        "obs_type": "graph",
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


def _deterministic_promotion_eval(
    model: TeacherActorCritic,
    env_config: dict,
    stage_name: str,
    device: torch.device,
    *,
    episodes: int,
    seed_base: int,
) -> dict[str, float]:
    if episodes <= 0:
        return {"success_rate": 0.0, "collision_rate": 0.0, "timeout_rate": 0.0}
    eval_config = dict(env_config)
    eval_config.update({"stage_name": stage_name, "split": "train"})
    env = ContinuousMazeEnv(eval_config)
    successes: list[float] = []
    collisions: list[float] = []
    timeouts: list[float] = []
    was_training = model.training
    model.eval()
    with torch.no_grad():
        for idx in range(episodes):
            obs, _ = env.reset(seed=seed_base + idx, options={"stage_name": stage_name, "split": "train"})
            done = False
            final_info = {}
            while not done:
                obs_t = collate_graph_obs([obs], device)
                action, _, _ = model.act(obs_t, deterministic=True)
                obs, _, terminated, truncated, final_info = env.step(action.squeeze(0).cpu().numpy())
                done = terminated or truncated
            successes.append(float(final_info.get("success", False)))
            collisions.append(float(final_info.get("collision", False)))
            timeouts.append(float(final_info.get("timeout", False)))
    if was_training:
        model.train()
    return {
        "success_rate": float(np.mean(successes)),
        "collision_rate": float(np.mean(collisions)),
        "timeout_rate": float(np.mean(timeouts)),
    }


def _checkpoint_payload(config: dict, env: ContinuousMazeEnv, model: TeacherActorCritic) -> dict:
    return {
        "model_state": model.state_dict(),
        "model_type": "gnn",
        "global_dim": GLOBAL_FEATURE_DIM,
        "node_dim": NODE_FEATURE_DIM,
        "edge_dim": EDGE_FEATURE_DIM,
        "hidden_dim": model.hidden_dim,
        "gnn_layers": model.gnn_layers,
        "max_acc": env.max_acc,
        "min_log_std": model.min_log_std,
        "max_log_std": model.max_log_std,
        "config": config,
        "stages": STAGE_ORDER,
    }


def _save_checkpoint(path: str, config: dict, env: ContinuousMazeEnv, model: TeacherActorCritic) -> None:
    ckpt_path = resolve_path(path)
    ensure_dir(ckpt_path.parent)
    torch.save(_checkpoint_payload(config, env, model), ckpt_path)


def _write_rows(path: str, rows: list[dict]) -> None:
    metrics_path = resolve_path(path)
    ensure_dir(metrics_path.parent)
    if rows:
        with metrics_path.open("w", newline="", encoding="utf-8") as f:
            fieldnames: list[str] = []
            for row in rows:
                for key in row:
                    if key not in fieldnames:
                        fieldnames.append(key)
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


def _write_outputs(config: dict, env: ContinuousMazeEnv, model: TeacherActorCritic, rows: list[dict]) -> None:
    ckpt_path = resolve_path(config["checkpoint_path"])
    _save_checkpoint(config["checkpoint_path"], config, env, model)
    _write_rows(config["train_metrics_path"], rows)
    print(f"已保存最终教师模型: {ckpt_path}")
    print(f"已保存训练指标: {resolve_path(config['train_metrics_path'])}")


def _stage_checkpoint_path(config: dict, stage_name: str) -> str:
    return str(resolve_path(config.get("checkpoint_dir", "checkpoints")) / stage_name / "teacher_final.pt")


def _stage_metrics_path(config: dict, stage_name: str) -> str:
    return str(resolve_path(config.get("results_dir", "results")) / stage_name / "train_metrics.csv")


def _write_stage_outputs(config: dict, env: ContinuousMazeEnv, model: TeacherActorCritic, stage_name: str, rows: list[dict]) -> None:
    checkpoint_path = _stage_checkpoint_path(config, stage_name)
    metrics_path = _stage_metrics_path(config, stage_name)
    _save_checkpoint(checkpoint_path, config, env, model)
    _write_rows(metrics_path, rows)
    print(f"课程 {stage_name} 最终模型已保存: {resolve_path(checkpoint_path)}")
    print(f"课程 {stage_name} 训练指标已保存: {resolve_path(metrics_path)}")


def _format_training_log(row: dict) -> str:
    return (
        f"课程 {row['stage']} | 更新 {row['update']} | 阶段步数 {row['stage_steps']} | "
        f"回合 {row['episodes']} | 成功率 {row['success_rate']:.3f} | "
        f"滚动成功率 {row['rolling_success_rate']:.3f} | 碰撞率 {row['collision_rate']:.3f} | "
        f"超时率 {row['timeout_rate']:.3f} | 平均回报 {row['average_return']:.2f} | "
        f"动作范数 {row['average_action_norm']:.2f} | 熵 {row['entropy']:.3f} | "
        f"KL {row['approx_kl']:.4f} | 裁剪率 {row.get('clip_fraction', 0.0):.3f} | "
        f"解释方差 {row.get('explained_variance', 0.0):.3f} | PPO更新 {int(row['ppo_updates'])}"
    )


def _maybe_print_log(row: dict, log_interval_updates: int) -> None:
    if int(row["update"]) % max(1, log_interval_updates) == 0:
        print(_format_training_log(row), flush=True)


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
    print(f"CUDA 是否可用: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"CUDA 设备: {torch.cuda.get_device_name(0)}")
    device = get_device(str(config["device"]))
    print(f"训练设备: {device}")
    env = ContinuousMazeEnv(config.get("env", {}))
    model = TeacherActorCritic(
        max_acc=env.max_acc,
        hidden_dim=int(config["gnn_hidden_dim"]),
        gnn_layers=int(config["gnn_layers"]),
        min_log_std=float(config["min_log_std"]),
        max_log_std=float(config["max_log_std"]),
    ).to(device)
    model_old = TeacherActorCritic(
        max_acc=env.max_acc,
        hidden_dim=int(config["gnn_hidden_dim"]),
        gnn_layers=int(config["gnn_layers"]),
        min_log_std=float(config["min_log_std"]),
        max_log_std=float(config["max_log_std"]),
    ).to(device)
    sync_policy_old(model, model_old)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))

    rollout_steps = int(config["rollout_steps"])
    curriculum_mode = str(config.get("curriculum_mode", "fixed"))
    total_steps = int(config["total_steps"])
    steps_per_stage = int(config["steps_per_stage"])
    rows: list[dict] = []
    global_steps = 0

    if curriculum_mode == "stagewise":
        stage_order = [str(stage) for stage in config.get("stage_order", STAGE_ORDER)]
        unknown = [stage for stage in stage_order if stage not in STAGE_ORDER]
        if unknown:
            raise ValueError(f"Unknown stages: {unknown}")
        updates_per_stage = max(1, int(np.ceil(steps_per_stage / rollout_steps)))
        log_interval_updates = int(config.get("log_interval_updates", 1))
        print(f"逐课程训练: {','.join(stage_order)}")
        print(f"每个课程更新次数: {updates_per_stage}")
        for stage_idx, stage_name in enumerate(stage_order):
            print(f"开始课程 {stage_name}")
            optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))
            stage_rows: list[dict] = []
            stage_episodes = 0
            recent_successes: deque[bool] = deque(maxlen=int(config["promotion_window_episodes"]))
            for update in range(updates_per_stage):
                batch = collect_rollout(env, model_old, rollout_steps, device, stage_name=stage_name, seed=seed + stage_idx * 100_000 + update)
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
                    target_kl=float(config["target_kl"]) if config.get("target_kl") is not None else None,
                    normalize_advantage=bool(config.get("normalize_advantage", True)),
                )
                sync_policy_old(model, model_old)
                global_steps += rollout_steps
                stage_steps = min((update + 1) * rollout_steps, steps_per_stage)
                stage_episodes += len(batch.episode_infos)
                recent_successes.extend(bool(info["success"]) for info in batch.episode_infos)
                row = _build_row(
                    update + 1,
                    global_steps,
                    stage_name,
                    batch,
                    metrics,
                    stage_steps=stage_steps,
                    stage_episodes=stage_episodes,
                    rolling_success_rate=float(np.mean(recent_successes)) if recent_successes else 0.0,
                    stage_status="training" if update + 1 < updates_per_stage else "completed",
                    obs_dim="graph",
                )
                stage_rows.append(row)
                _maybe_print_log(row, log_interval_updates)
            _write_stage_outputs(config, env, model, stage_name, stage_rows)
            rows.extend(stage_rows)
    elif curriculum_mode == "adaptive":
        hard_max_steps = int(config["hard_max_steps_per_stage"])
        updates = max(1, int(np.ceil(hard_max_steps * len(STAGE_ORDER) / rollout_steps)))
        print(
            "自适应课程最大更新次数: "
            f"{updates} = ceil({hard_max_steps} 每课程硬上限步数 "
            f"* {len(STAGE_ORDER)} 个课程 / {rollout_steps} rollout步数)"
        )
        state = AdaptiveCurriculumState(recent_successes=deque(maxlen=int(config["promotion_window_episodes"])))
        min_steps = int(config["min_steps_per_stage"])
        soft_max_steps = int(config["soft_max_steps_per_stage"])
        promotion_success_rate = float(config["promotion_success_rate"])
        promotion_eval_success_rate = float(config["promotion_eval_success_rate"])
        promotion_eval_episodes = int(config["promotion_eval_episodes"])
        promotion_eval_interval = max(1, int(config["promotion_eval_interval_updates"]))
        hard_max_policy = str(config.get("hard_max_policy", "stop"))
        stop_training = False

        for update in range(updates):
            stage_name = state.stage_name
            batch = collect_rollout(env, model_old, rollout_steps, device, stage_name=stage_name, seed=seed + update)
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
                target_kl=float(config["target_kl"]) if config.get("target_kl") is not None else None,
                normalize_advantage=bool(config.get("normalize_advantage", True)),
            )
            sync_policy_old(model, model_old)
            global_steps += rollout_steps
            state.stage_steps += rollout_steps
            state.stage_episodes += len(batch.episode_infos)
            state.recent_successes.extend(bool(info["success"]) for info in batch.episode_infos)
            rolling_success_rate = state.rolling_success_rate()
            should_run_promotion_eval = (
                state.stage_steps >= min_steps
                and len(state.recent_successes) == state.recent_successes.maxlen
                and rolling_success_rate >= promotion_success_rate
                and (state.last_promotion_eval_update < 0 or (update + 1 - state.last_promotion_eval_update) >= promotion_eval_interval)
            )
            if should_run_promotion_eval:
                eval_stats = _deterministic_promotion_eval(
                    model,
                    config.get("env", {}),
                    stage_name,
                    device,
                    episodes=promotion_eval_episodes,
                    seed_base=seed + 1_000_000 + state.stage_index * 10_000 + update * promotion_eval_episodes,
                )
                state.last_promotion_eval_success_rate = eval_stats["success_rate"]
                state.last_promotion_eval_episodes = promotion_eval_episodes
                state.last_promotion_eval_update = update + 1

            stage_status = adaptive_stage_status(
                state,
                min_steps=min_steps,
                soft_max_steps=soft_max_steps,
                hard_max_steps=hard_max_steps,
                promotion_success_rate=promotion_success_rate,
                promotion_eval_success_rate=promotion_eval_success_rate,
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
                    promotion_eval_success_rate=state.last_promotion_eval_success_rate,
                    promotion_eval_episodes=state.last_promotion_eval_episodes,
                    obs_dim="graph",
                )
            )
            _maybe_print_log(rows[-1], int(config.get("log_interval_updates", 1)))

            if stage_status == "completed_success" or stage_status == "hard_max_stop":
                stop_training = True
            elif stage_status == "promoted_success":
                state.advance()
            if stop_training:
                break
    elif curriculum_mode == "fixed":
        updates = max(1, total_steps // rollout_steps)
        fixed_stage_episodes = {stage: 0 for stage in STAGE_ORDER}
        for update in range(updates):
            stage_name = stage_from_step(global_steps, steps_per_stage)
            stage_start = STAGE_ORDER.index(stage_name) * steps_per_stage
            stage_steps = global_steps - stage_start + rollout_steps
            batch = collect_rollout(env, model_old, rollout_steps, device, stage_name=stage_name, seed=seed + update)
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
                target_kl=float(config["target_kl"]) if config.get("target_kl") is not None else None,
                normalize_advantage=bool(config.get("normalize_advantage", True)),
            )
            sync_policy_old(model, model_old)
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
                    obs_dim="graph",
                )
            )
            _maybe_print_log(rows[-1], int(config.get("log_interval_updates", 1)))
    else:
        raise ValueError(f"Unsupported curriculum_mode: {curriculum_mode}")

    if curriculum_mode != "stagewise":
        _write_outputs(config, env, model, rows)


if __name__ == "__main__":
    main()
