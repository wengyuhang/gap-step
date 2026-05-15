from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path

import numpy as np
import torch

from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, GraphObs, collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.ppo import PPOBatch, get_device, ppo_update, sync_policy_old
from gap_step.utils import ensure_dir, load_yaml, resolve_path, set_seed
from gap_step.window_maze_env import TimeVaryingWindowMazeEnv


STAGE_ORDER = ["C1", "C2", "C3", "C4", "C5"]

DEFAULT_CONFIG = {
    "seed": 0,
    "device": "auto",
    "stage_order": STAGE_ORDER,
    "rollout_steps": 4096,
    "steps_per_stage": 200_000,
    "minibatch_size": 512,
    "update_epochs": 4,
    "learning_rate": 2.5e-4,
    "gamma": 0.99,
    "gae_lambda": 0.95,
    "clip_ratio": 0.2,
    "value_coef": 0.5,
    "entropy_coef": 0.004,
    "target_kl": 0.08,
    "max_grad_norm": 0.5,
    "normalize_advantage": True,
    "min_log_std": -2.0,
    "max_log_std": 0.3,
    "log_std_init": -0.35,
    "gnn_hidden_dim": 128,
    "gnn_layers": 4,
    "checkpoint_dir": "checkpoints/window_generated",
    "results_dir": "results/window_generated",
    "clean_outputs": True,
    "resume_checkpoint": "",
    "log_interval_updates": 1,
    "num_envs": 8,
    "validation_episodes": 24,
    "validation_interval_updates": 5,
    "promotion_success_rate": 0.68,
    "max_updates_per_stage": None,
    "env": {"return_graph_obs": True, "max_steps": 320, "period": 8},
}


def _mean_info(infos: list[dict], key: str) -> float:
    return float(np.mean([float(info.get(key, 0.0)) for info in infos])) if infos else 0.0


def _write_rows(path: str | Path, rows: list[dict]) -> None:
    path = resolve_path(path)
    ensure_dir(path.parent)
    if not rows:
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _checkpoint_payload(config: dict, model: TeacherActorCritic) -> dict:
    return {
        "model_state": model.state_dict(),
        "model_type": "window_gnn_ppo_teacher",
        "global_dim": GLOBAL_FEATURE_DIM,
        "node_dim": NODE_FEATURE_DIM,
        "edge_dim": EDGE_FEATURE_DIM,
        "hidden_dim": model.hidden_dim,
        "gnn_layers": model.gnn_layers,
        "max_acc": model.max_acc,
        "min_log_std": model.min_log_std,
        "max_log_std": model.max_log_std,
        "log_std_init": model.log_std_init,
        "config": config,
        "stages": STAGE_ORDER,
    }


def _save_checkpoint(path: str | Path, config: dict, model: TeacherActorCritic) -> None:
    path = resolve_path(path)
    ensure_dir(path.parent)
    torch.save(_checkpoint_payload(config, model), path)


def collect_window_rollout(
    envs: list[TimeVaryingWindowMazeEnv],
    model_old: TeacherActorCritic,
    rollout_steps: int,
    device: torch.device,
    *,
    stage_name: str,
    split: str,
    seed: int,
) -> PPOBatch:
    model_old.eval()
    obs_list: list[GraphObs] = []
    for idx, env in enumerate(envs):
        obs, _ = env.reset(seed=seed + idx * 10_003, options={"stage_name": stage_name, "split": split})
        obs_list.append(obs)

    n_envs = len(envs)
    horizon = max(1, int(np.ceil(rollout_steps / n_envs)))
    obs_parts: list[list[GraphObs]] = [[] for _ in envs]
    action_parts: list[list[np.ndarray]] = [[] for _ in envs]
    logp_parts: list[list[float]] = [[] for _ in envs]
    value_parts: list[list[float]] = [[] for _ in envs]
    reward_parts: list[list[float]] = [[] for _ in envs]
    done_parts: list[list[float]] = [[] for _ in envs]
    ep_returns = [0.0 for _ in envs]
    episode_returns: list[float] = []
    episode_infos: list[dict] = []
    step_infos: list[dict] = []

    for _ in range(horizon):
        obs_t = collate_graph_obs(obs_list, device)
        with torch.no_grad():
            actions_t, logp_t, value_t = model_old.act(obs_t, deterministic=False)
        actions = actions_t.detach().cpu().numpy()
        logps = logp_t.detach().cpu().numpy()
        values = value_t.detach().cpu().numpy()
        next_obs_list: list[GraphObs] = []
        for env_idx, env in enumerate(envs):
            obs = obs_list[env_idx]
            action = actions[env_idx]
            next_obs, reward, terminated, truncated, info = env.step(action)
            done = bool(terminated or truncated)

            obs_parts[env_idx].append(obs)
            action_parts[env_idx].append(action.astype(np.float32))
            logp_parts[env_idx].append(float(logps[env_idx]))
            value_parts[env_idx].append(float(values[env_idx]))
            reward_parts[env_idx].append(float(reward))
            done_parts[env_idx].append(float(done))
            step_infos.append(dict(info))
            ep_returns[env_idx] += float(reward)

            if done:
                final_info = dict(info)
                final_info["return"] = ep_returns[env_idx]
                episode_returns.append(ep_returns[env_idx])
                episode_infos.append(final_info)
                ep_returns[env_idx] = 0.0
                next_obs, _ = env.reset(options={"stage_name": stage_name, "split": split})
            next_obs_list.append(next_obs)
        obs_list = next_obs_list

    obs_buf: list[GraphObs] = []
    actions_buf: list[np.ndarray] = []
    logp_buf: list[float] = []
    value_buf: list[float] = []
    reward_buf: list[float] = []
    done_buf: list[float] = []
    for env_idx in range(n_envs):
        if done_parts[env_idx]:
            # Prevent GAE from leaking across independent env trajectories in
            # the flattened rollout. This trades a small bootstrap bias for a
            # much faster batched PPO collector.
            done_parts[env_idx][-1] = 1.0
        obs_buf.extend(obs_parts[env_idx])
        actions_buf.extend(action_parts[env_idx])
        logp_buf.extend(logp_parts[env_idx])
        value_buf.extend(value_parts[env_idx])
        reward_buf.extend(reward_parts[env_idx])
        done_buf.extend(done_parts[env_idx])

    return PPOBatch(
        obs=obs_buf,
        actions=np.asarray(actions_buf, dtype=np.float32),
        log_probs=np.asarray(logp_buf, dtype=np.float32),
        values=np.asarray(value_buf, dtype=np.float32),
        rewards=np.asarray(reward_buf, dtype=np.float32),
        dones=np.asarray(done_buf, dtype=np.float32),
        last_value=0.0,
        episode_returns=episode_returns,
        episode_infos=episode_infos,
        step_infos=step_infos,
    )


def evaluate_stage(
    model: TeacherActorCritic,
    env_config: dict,
    stage_name: str,
    device: torch.device,
    *,
    episodes: int,
    seed_base: int,
    split: str = "id_test",
) -> dict[str, float]:
    env_cfg = dict(env_config)
    env_cfg.update({"return_graph_obs": True, "stage_name": stage_name, "split": split})
    num_eval_envs = max(1, min(8, episodes))
    envs = [TimeVaryingWindowMazeEnv(env_cfg) for _ in range(num_eval_envs)]
    successes: list[float] = []
    collisions: list[float] = []
    timeouts: list[float] = []
    steps: list[float] = []
    was_training = model.training
    model.eval()
    with torch.no_grad():
        next_episode = 0
        active_obs: list[GraphObs] = []
        active_envs: list[TimeVaryingWindowMazeEnv] = []
        for env in envs:
            if next_episode >= episodes:
                break
            obs, _ = env.reset(seed=seed_base + next_episode, options={"stage_name": stage_name, "split": split})
            active_obs.append(obs)
            active_envs.append(env)
            next_episode += 1

        while active_envs:
            obs_t = collate_graph_obs(active_obs, device)
            actions, _, _ = model.act(obs_t, deterministic=True)
            next_obs: list[GraphObs] = []
            next_envs: list[TimeVaryingWindowMazeEnv] = []
            for idx, env in enumerate(active_envs):
                obs, _, terminated, truncated, final_info = env.step(actions[idx].cpu().numpy())
                done = terminated or truncated
                if done:
                    successes.append(float(final_info.get("success", False)))
                    collisions.append(float(final_info.get("collision", False)))
                    timeouts.append(float(final_info.get("timeout", False)))
                    steps.append(float(env.step_count))
                    if next_episode < episodes:
                        obs, _ = env.reset(
                            seed=seed_base + next_episode,
                            options={"stage_name": stage_name, "split": split},
                        )
                        next_episode += 1
                        next_obs.append(obs)
                        next_envs.append(env)
                else:
                    next_obs.append(obs)
                    next_envs.append(env)
            active_obs = next_obs
            active_envs = next_envs
    if was_training:
        model.train()
    return {
        "eval_success_rate": float(np.mean(successes)) if successes else 0.0,
        "eval_collision_rate": float(np.mean(collisions)) if collisions else 0.0,
        "eval_timeout_rate": float(np.mean(timeouts)) if timeouts else 0.0,
        "eval_average_steps": float(np.mean(steps)) if steps else 0.0,
    }


def _row(update: int, global_steps: int, stage_name: str, batch, metrics: dict, eval_stats: dict) -> dict:
    return {
        "update": update,
        "global_steps": global_steps,
        "stage": stage_name,
        "episodes": len(batch.episode_infos),
        "average_return": float(np.mean(batch.episode_returns)) if batch.episode_returns else 0.0,
        "success_rate": _mean_info(batch.episode_infos, "success"),
        "collision_rate": _mean_info(batch.episode_infos, "collision"),
        "timeout_rate": _mean_info(batch.episode_infos, "timeout"),
        "wall_collision_rate": _mean_info(batch.episode_infos, "wall_collision"),
        "window_collision_rate": _mean_info(batch.episode_infos, "window_collision"),
        "progress_delta_mean": _mean_info(batch.step_infos, "progress_delta"),
        "progress_reward_mean": _mean_info(batch.step_infos, "progress_reward"),
        "risk_penalty_mean": _mean_info(batch.step_infos, "risk_penalty"),
        "wall_penalty_mean": _mean_info(batch.step_infos, "wall_penalty"),
        "center_penalty_mean": _mean_info(batch.step_infos, "center_penalty"),
        "closed_window_penalty_mean": _mean_info(batch.step_infos, "closed_window_penalty"),
        "closing_window_penalty_mean": _mean_info(batch.step_infos, "closing_window_penalty"),
        "alignment_reward_mean": _mean_info(batch.step_infos, "alignment_reward"),
        "gap_alignment_reward_mean": _mean_info(batch.step_infos, "gap_alignment_reward"),
        "average_action_norm": _mean_info(batch.step_infos, "action_norm"),
        **eval_stats,
        **metrics,
    }


def _print_row(row: dict) -> None:
    print(
        f"窗口PPO | {row['stage']} | 更新 {row['update']} | 步数 {row['global_steps']} | "
        f"采样成功 {row['success_rate']:.3f} | 验证成功 {row.get('eval_success_rate', 0.0):.3f} | "
        f"碰撞 {row['collision_rate']:.3f} | 超时 {row['timeout_rate']:.3f} | "
        f"回报 {row['average_return']:.2f} | KL {row['approx_kl']:.4f} | 熵 {row['entropy']:.3f}",
        flush=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="gap_step/configs/train_window_teacher.yaml")
    args = parser.parse_args()

    config = dict(DEFAULT_CONFIG)
    path = resolve_path(args.config)
    if path.exists():
        config.update(load_yaml(path))
    config["env"] = {**DEFAULT_CONFIG["env"], **dict(config.get("env", {}))}

    seed = int(config["seed"])
    set_seed(seed)
    print(f"CUDA 是否可用: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"CUDA 设备: {torch.cuda.get_device_name(0)}")
    device = get_device(str(config["device"]))
    print(f"训练设备: {device}")

    results_dir = resolve_path(config["results_dir"])
    checkpoint_dir = resolve_path(config["checkpoint_dir"])
    if bool(config.get("clean_outputs", True)):
        for out_dir in (results_dir, checkpoint_dir):
            if out_dir.exists():
                shutil.rmtree(out_dir)
    ensure_dir(results_dir)
    ensure_dir(checkpoint_dir)

    num_envs = max(1, int(config.get("num_envs", 1)))
    env = TimeVaryingWindowMazeEnv({**config["env"], "return_graph_obs": True})
    rollout_envs = [TimeVaryingWindowMazeEnv({**config["env"], "return_graph_obs": True}) for _ in range(num_envs)]
    model = TeacherActorCritic(
        max_acc=env.max_acc,
        hidden_dim=int(config["gnn_hidden_dim"]),
        gnn_layers=int(config["gnn_layers"]),
        min_log_std=float(config["min_log_std"]),
        max_log_std=float(config["max_log_std"]),
        log_std_init=float(config["log_std_init"]),
    ).to(device)
    model_old = TeacherActorCritic(
        max_acc=env.max_acc,
        hidden_dim=int(config["gnn_hidden_dim"]),
        gnn_layers=int(config["gnn_layers"]),
        min_log_std=float(config["min_log_std"]),
        max_log_std=float(config["max_log_std"]),
        log_std_init=float(config["log_std_init"]),
    ).to(device)
    resume_checkpoint = str(config.get("resume_checkpoint", "")).strip()
    if resume_checkpoint:
        ckpt = torch.load(resolve_path(resume_checkpoint), map_location=device, weights_only=False)
        missing, unexpected = model.load_state_dict(ckpt["model_state"], strict=False)
        if missing or unexpected:
            print(f"继续训练兼容加载: missing={list(missing)}, unexpected={list(unexpected)}")
        print(f"已加载继续训练模型: {resolve_path(resume_checkpoint)}")
    sync_policy_old(model, model_old)

    rollout_steps = int(config["rollout_steps"])
    updates_per_stage = max(1, int(np.ceil(int(config["steps_per_stage"]) / rollout_steps)))
    if config.get("max_updates_per_stage") is not None:
        updates_per_stage = min(updates_per_stage, int(config["max_updates_per_stage"]))
    validation_interval = max(1, int(config["validation_interval_updates"]))
    validation_episodes = int(config["validation_episodes"])
    rows_all: list[dict] = []
    global_steps = 0
    print(f"训练课程: {','.join(config['stage_order'])} | 每课程更新 {updates_per_stage} | 并行环境 {num_envs}")

    for stage_idx, stage_name in enumerate([str(s) for s in config["stage_order"]]):
        optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))
        stage_rows: list[dict] = []
        print(f"开始课程 {stage_name}")
        for update in range(1, updates_per_stage + 1):
            batch = collect_window_rollout(
                rollout_envs,
                model_old,
                rollout_steps,
                device,
                stage_name=stage_name,
                split="train",
                seed=seed + stage_idx * 200_000 + update * 101,
            )
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
                normalize_advantage=bool(config["normalize_advantage"]),
            )
            sync_policy_old(model, model_old)
            global_steps += int(len(batch.rewards))
            eval_stats = {}
            if update == 1 or update % validation_interval == 0 or update == updates_per_stage:
                eval_stats = evaluate_stage(
                    model,
                    config["env"],
                    stage_name,
                    device,
                    episodes=validation_episodes,
                    seed_base=seed + 9_000_000 + stage_idx * 10_000 + update * 100,
                    split="id_test",
                )
            row = _row(update, global_steps, stage_name, batch, metrics, eval_stats)
            stage_rows.append(row)
            rows_all.append(row)
            if update % max(1, int(config["log_interval_updates"])) == 0:
                _print_row(row)
            if (
                update >= 3
                and eval_stats
                and stage_name != "C5"
                and eval_stats["eval_success_rate"] >= float(config["promotion_success_rate"])
            ):
                print(f"{stage_name} 验证成功率达到 {eval_stats['eval_success_rate']:.3f}，升阶。", flush=True)
                break
        stage_ckpt = checkpoint_dir / stage_name / "teacher_final.pt"
        stage_metrics = results_dir / stage_name / "train_metrics.csv"
        _save_checkpoint(stage_ckpt, config, model)
        _write_rows(stage_metrics, stage_rows)
        print(f"课程 {stage_name} 已保存: {stage_ckpt}")

    _save_checkpoint(checkpoint_dir / "teacher_final.pt", config, model)
    _write_rows(results_dir / "train_metrics.csv", rows_all)
    print(f"最终模型: {checkpoint_dir / 'teacher_final.pt'}")
    print(f"训练指标: {results_dir / 'train_metrics.csv'}")


if __name__ == "__main__":
    main()
