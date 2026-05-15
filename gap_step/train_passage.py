from __future__ import annotations

import argparse
import csv
import shutil
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import torch

from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, GraphObs, collate_graph_obs
from gap_step.model import TeacherActorCritic
from gap_step.passage_env import PASSAGE_STAGE_ORDER, TimeVaryingPassageMazeEnv
from gap_step.passage_teacher import TimeExpandedPassageTeacher
from gap_step.ppo import PPOBatch, compute_gae, get_device, ppo_update, sync_policy_old
from gap_step.utils import load_yaml, resolve_path, set_seed


DEFAULT_CONFIG: dict[str, Any] = {
    "seed": 7,
    "device": "auto",
    "output_root": "results/passage_generated",
    "checkpoint_root": "checkpoints/passage_generated",
    "clean_outputs": True,
    "stage_order": list(PASSAGE_STAGE_ORDER),
    "rollout_steps": 768,
    "updates_per_stage": 2,
    "minibatch_size": 256,
    "update_epochs": 2,
    "learning_rate": 3e-5,
    "gamma": 0.985,
    "gae_lambda": 0.92,
    "clip_ratio": 0.15,
    "value_coef": 0.5,
    "entropy_coef": 0.00005,
    "target_kl": 0.08,
    "max_grad_norm": 0.5,
    "bc_episodes_per_stage": 24,
    "bc_epochs": 2,
    "bc_batch_size": 256,
    "bc_learning_rate": 3e-5,
    "reset_actor_residual": True,
    "eval_episodes": 50,
    "gnn_hidden_dim": 128,
    "gnn_layers": 4,
    "min_log_std": -2.0,
    "max_log_std": -0.4,
    "log_std_init": -1.4,
    "env": {"max_steps": 240, "return_graph_obs": False},
    "teacher": {"replan_horizon": 300, "arrival_radius": 0.18},
}


def _obs_with_prior(env: TimeVaryingPassageMazeEnv, teacher: TimeExpandedPassageTeacher) -> tuple[GraphObs, np.ndarray]:
    prior = teacher.act(env)
    return env.graph_obs(prior_action=prior), prior


def _collect_bc_dataset(
    env_config: dict[str, Any],
    teacher: TimeExpandedPassageTeacher,
    stage: str,
    episodes: int,
    seed_base: int,
) -> tuple[list[GraphObs], np.ndarray]:
    obs_list: list[GraphObs] = []
    actions: list[np.ndarray] = []
    env = TimeVaryingPassageMazeEnv({**env_config, "stage_name": stage, "return_graph_obs": False})
    for ep in range(episodes):
        env.reset(seed=seed_base + ep, options={"stage_name": stage, "split": "train"})
        done = False
        while not done:
            obs, action = _obs_with_prior(env, teacher)
            obs_list.append(obs)
            actions.append(action.astype(np.float32))
            _, _, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
    return obs_list, np.asarray(actions, dtype=np.float32)


def _bc_pretrain(
    model: TeacherActorCritic,
    obs_list: list[GraphObs],
    actions: np.ndarray,
    device: torch.device,
    *,
    epochs: int,
    batch_size: int,
    learning_rate: float,
) -> float:
    if not obs_list:
        return 0.0
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    inds = np.arange(len(obs_list))
    losses: list[float] = []
    for _ in range(epochs):
        np.random.shuffle(inds)
        for start in range(0, len(inds), batch_size):
            mb = inds[start : start + batch_size]
            batch = collate_graph_obs([obs_list[int(i)] for i in mb], device)
            target = torch.as_tensor(actions[mb], dtype=torch.float32, device=device)
            pred = model.forward(batch)["mean"]
            loss = torch.nn.functional.mse_loss(pred, target)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
            optimizer.step()
            losses.append(float(loss.detach().cpu()))
    return float(np.mean(losses)) if losses else 0.0


def _collect_rollout(
    env: TimeVaryingPassageMazeEnv,
    teacher: TimeExpandedPassageTeacher,
    model_old: TeacherActorCritic,
    steps: int,
    device: torch.device,
    stage: str,
    seed: int,
    split: str = "train",
) -> PPOBatch:
    model_old.eval()
    env.return_graph_obs = False
    env.reset(seed=seed, options={"stage_name": stage, "split": split})
    obs_buf: list[GraphObs] = []
    action_buf, logp_buf, value_buf, reward_buf, done_buf = [], [], [], [], []
    episode_returns: list[float] = []
    episode_infos: list[dict] = []
    step_infos: list[dict] = []
    ep_return = 0.0

    for step_idx in range(steps):
        obs, _ = _obs_with_prior(env, teacher)
        obs_t = collate_graph_obs([obs], device)
        with torch.no_grad():
            action_t, logp_t, value_t = model_old.act(obs_t, deterministic=False)
        action = action_t.squeeze(0).cpu().numpy()
        _, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated

        obs_buf.append(obs)
        action_buf.append(action)
        logp_buf.append(float(logp_t.item()))
        value_buf.append(float(value_t.item()))
        reward_buf.append(float(reward))
        done_buf.append(float(done))
        step_infos.append(dict(info))
        ep_return += float(reward)

        if done:
            final_info = dict(info)
            final_info["return"] = ep_return
            episode_returns.append(ep_return)
            episode_infos.append(final_info)
            ep_return = 0.0
            env.reset(seed=seed + 10_000 + step_idx, options={"stage_name": stage, "split": split})

    obs, _ = _obs_with_prior(env, teacher)
    with torch.no_grad():
        last_value = float(model_old.forward(collate_graph_obs([obs], device))["value"].item())

    return PPOBatch(
        obs=obs_buf,
        actions=np.asarray(action_buf, dtype=np.float32),
        log_probs=np.asarray(logp_buf, dtype=np.float32),
        values=np.asarray(value_buf, dtype=np.float32),
        rewards=np.asarray(reward_buf, dtype=np.float32),
        dones=np.asarray(done_buf, dtype=np.float32),
        last_value=last_value,
        episode_returns=episode_returns,
        episode_infos=episode_infos,
        step_infos=step_infos,
    )


def evaluate_policy(
    model: TeacherActorCritic,
    env_config: dict[str, Any],
    teacher: TimeExpandedPassageTeacher,
    device: torch.device,
    *,
    stage: str,
    split: str,
    episodes: int,
    seed_base: int,
) -> dict[str, float]:
    env = TimeVaryingPassageMazeEnv({**env_config, "stage_name": stage, "split": split, "return_graph_obs": False})
    successes: list[float] = []
    collisions: list[float] = []
    timeouts: list[float] = []
    steps: list[float] = []
    was_training = model.training
    model.eval()
    with torch.no_grad():
        for ep in range(episodes):
            env.reset(seed=seed_base + ep, options={"stage_name": stage, "split": split})
            done = False
            info: dict[str, Any] = {}
            while not done:
                obs, _ = _obs_with_prior(env, teacher)
                action, _, _ = model.act(collate_graph_obs([obs], device), deterministic=True)
                _, _, terminated, truncated, info = env.step(action.squeeze(0).cpu().numpy())
                done = terminated or truncated
            successes.append(float(info.get("success", False)))
            collisions.append(float(info.get("collision", False)))
            timeouts.append(float(info.get("timeout", False)))
            steps.append(float(info.get("step", 0)))
    if was_training:
        model.train()
    return {
        "success_rate": float(np.mean(successes)),
        "collision_rate": float(np.mean(collisions)),
        "timeout_rate": float(np.mean(timeouts)),
        "average_steps": float(np.mean(steps)),
    }


def _save_checkpoint(path: Path, config: dict[str, Any], model: TeacherActorCritic) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "kind": "passage_gnn_ppo_teacher",
            "model_state": model.state_dict(),
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
        },
        path,
    )


def _write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def train(config: dict[str, Any]) -> Path:
    cfg = {**DEFAULT_CONFIG, **config}
    env_config = {**DEFAULT_CONFIG["env"], **dict(config.get("env", {}))}
    teacher_config = {**DEFAULT_CONFIG["teacher"], **dict(config.get("teacher", {}))}
    cfg["env"] = env_config
    cfg["teacher"] = teacher_config
    seed = int(cfg["seed"])
    set_seed(seed)
    output_root = resolve_path(cfg["output_root"])
    checkpoint_root = resolve_path(cfg["checkpoint_root"])
    if bool(cfg.get("clean_outputs", True)):
        shutil.rmtree(output_root, ignore_errors=True)
        shutil.rmtree(checkpoint_root, ignore_errors=True)
    output_root.mkdir(parents=True, exist_ok=True)
    checkpoint_root.mkdir(parents=True, exist_ok=True)

    device = get_device(str(cfg["device"]))
    print(f"CUDA 是否可用: {torch.cuda.is_available()}", flush=True)
    if torch.cuda.is_available():
        print(f"CUDA 设备: {torch.cuda.get_device_name(0)}", flush=True)
    print(f"训练设备: {device}", flush=True)

    teacher = TimeExpandedPassageTeacher(teacher_config)
    model = TeacherActorCritic(
        max_acc=1.0,
        hidden_dim=int(cfg["gnn_hidden_dim"]),
        gnn_layers=int(cfg["gnn_layers"]),
        min_log_std=float(cfg["min_log_std"]),
        max_log_std=float(cfg["max_log_std"]),
        log_std_init=float(cfg["log_std_init"]),
    ).to(device)
    model_old = TeacherActorCritic(
        max_acc=1.0,
        hidden_dim=int(cfg["gnn_hidden_dim"]),
        gnn_layers=int(cfg["gnn_layers"]),
        min_log_std=float(cfg["min_log_std"]),
        max_log_std=float(cfg["max_log_std"]),
        log_std_init=float(cfg["log_std_init"]),
    ).to(device)
    sync_policy_old(model, model_old)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(cfg["learning_rate"]))
    rows: list[dict[str, Any]] = []
    global_update = 0

    stage_order = [str(s) for s in cfg["stage_order"]]
    for stage_idx, stage in enumerate(stage_order):
        print(f"开始课程 {stage}", flush=True)
        bc_obs, bc_actions = _collect_bc_dataset(env_config, teacher, stage, int(cfg["bc_episodes_per_stage"]), seed + stage_idx * 1000)
        bc_loss = _bc_pretrain(
            model,
            bc_obs,
            bc_actions,
            device,
            epochs=int(cfg["bc_epochs"]),
            batch_size=int(cfg["bc_batch_size"]),
            learning_rate=float(cfg["bc_learning_rate"]),
        )
        if bool(cfg.get("reset_actor_residual", True)):
            model._zero_actor_head()
        sync_policy_old(model, model_old)
        optimizer = torch.optim.Adam(model.parameters(), lr=float(cfg["learning_rate"]))
        env = TimeVaryingPassageMazeEnv({**env_config, "stage_name": stage, "return_graph_obs": False})
        stage_rows: list[dict[str, Any]] = []
        recent_successes: deque[bool] = deque(maxlen=100)
        for update in range(int(cfg["updates_per_stage"])):
            global_update += 1
            batch = _collect_rollout(env, teacher, model_old, int(cfg["rollout_steps"]), device, stage, seed + 50_000 * stage_idx + update)
            metrics = ppo_update(
                model,
                optimizer,
                batch,
                device,
                gamma=float(cfg["gamma"]),
                gae_lambda=float(cfg["gae_lambda"]),
                clip_ratio=float(cfg["clip_ratio"]),
                value_coef=float(cfg["value_coef"]),
                entropy_coef=float(cfg["entropy_coef"]),
                update_epochs=int(cfg["update_epochs"]),
                minibatch_size=int(cfg["minibatch_size"]),
                max_grad_norm=float(cfg["max_grad_norm"]),
                target_kl=float(cfg["target_kl"]),
                normalize_advantage=True,
            )
            if bool(cfg.get("reset_actor_residual", True)):
                model._zero_actor_head()
            sync_policy_old(model, model_old)
            recent_successes.extend(bool(info.get("success", False)) for info in batch.episode_infos)
            eval_stats = evaluate_policy(
                model,
                env_config,
                teacher,
                device,
                stage=stage,
                split="id_test",
                episodes=int(cfg["eval_episodes"]),
                seed_base=seed + 800_000 + stage_idx * 10_000 + update * int(cfg["eval_episodes"]),
            )
            row = {
                "stage": stage,
                "update": update + 1,
                "global_update": global_update,
                "bc_loss": bc_loss,
                "episodes": len(batch.episode_infos),
                "train_success_rate": float(np.mean([info.get("success", False) for info in batch.episode_infos])) if batch.episode_infos else 0.0,
                "rolling_success_rate": float(np.mean(recent_successes)) if recent_successes else 0.0,
                **metrics,
                **{f"eval_{k}": v for k, v in eval_stats.items()},
            }
            rows.append(row)
            stage_rows.append(row)
            print(
                f"课程 {stage} | 更新 {update + 1}/{cfg['updates_per_stage']} | "
                f"训练成功率 {row['train_success_rate']:.2%} | "
                f"评估成功率 {row['eval_success_rate']:.2%} | "
                f"碰撞率 {row['eval_collision_rate']:.2%} | BC {bc_loss:.5f}"
            , flush=True)
        _write_rows(output_root / stage / "train_metrics.csv", stage_rows)
        _save_checkpoint(checkpoint_root / stage / "teacher_final.pt", cfg, model)

    final_stage = stage_order[-1]
    final_path = checkpoint_root / final_stage / "teacher_final.pt"
    _write_rows(output_root / "train_metrics.csv", rows)
    print(f"已保存最终 C5 PPO 教师: {final_path}", flush=True)
    print(f"已保存训练指标: {output_root / 'train_metrics.csv'}", flush=True)
    return final_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="gap_step/configs/train_passage_teacher.yaml")
    args = parser.parse_args()
    train(load_yaml(args.config))


if __name__ == "__main__":
    main()
