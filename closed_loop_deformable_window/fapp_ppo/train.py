"""Training entry point for Future-Aware Privileged-Preview PPO."""

from __future__ import annotations

import argparse
import csv
import json
import random
from dataclasses import replace
from pathlib import Path

import numpy as np
import torch
import yaml

from .config import ExperimentConfig, load_config, validate_config
from .environment import ClosedLoopWindowEnv
from .model import AsymmetricActorCritic
from .ppo import collect_rollout, get_device, ppo_update, sync_old_policy


def _seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _make_environments(
    config: ExperimentConfig,
    curriculum_stage,
    seed_offset: int,
) -> list[ClosedLoopWindowEnv]:
    environment_config = config.environment
    overrides = {
        field: getattr(curriculum_stage, field)
        for field in (
            "opportunity_mode",
            "opportunity_width",
            "motion_amplitude_multiplier",
            "deformation_amplitude_multiplier",
            "opportunity_schedule_jitter",
        )
        if getattr(curriculum_stage, field) is not None
    }
    if overrides:
        environment_config = replace(environment_config, **overrides)
    return [
        ClosedLoopWindowEnv(
            environment_config,
            config.quadrotor,
            stage=curriculum_stage.name,
            seed=config.train.seed + seed_offset + 1009 * index,
        )
        for index in range(config.ppo.num_envs)
    ]


def _save_checkpoint(
    path: Path,
    *,
    model: AsymmetricActorCritic,
    optimizer: torch.optim.Optimizer,
    config: ExperimentConfig,
    actor_obs_dim: int,
    critic_obs_dim: int,
    update: int,
    stage: str,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    torch.save(
        {
            "model_state": model.state_dict(),
            "optimizer_state": optimizer.state_dict(),
            "config": config.to_dict(),
            "actor_obs_dim": actor_obs_dim,
            "critic_obs_dim": critic_obs_dim,
            "update": update,
            "stage": stage,
        },
        temporary,
    )
    temporary.replace(path)


def train_model(
    config: ExperimentConfig,
    *,
    output_dir: str | Path | None = None,
) -> tuple[Path, list[dict[str, float | int | str]]]:
    validate_config(config)
    _seed_everything(config.train.seed)
    device = get_device(config.train.device)
    output = Path(config.train.output_dir if output_dir is None else output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / "config.yaml").write_text(
        yaml.safe_dump(config.to_dict(), allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    first_stage = config.train.curriculum[0].name
    first_environment = config.environment
    first_curriculum = config.train.curriculum[0]
    first_overrides = {
        field: getattr(first_curriculum, field)
        for field in (
            "opportunity_mode",
            "opportunity_width",
            "motion_amplitude_multiplier",
            "deformation_amplitude_multiplier",
            "opportunity_schedule_jitter",
        )
        if getattr(first_curriculum, field) is not None
    }
    if first_overrides:
        first_environment = replace(first_environment, **first_overrides)
    probe = ClosedLoopWindowEnv(
        first_environment,
        config.quadrotor,
        stage=first_stage,
        seed=config.train.seed,
    )
    actor_obs_dim = probe.actor_obs_dim
    critic_obs_dim = probe.critic_obs_dim
    model = AsymmetricActorCritic(
        actor_obs_dim, critic_obs_dim, config.model
    ).to(device)
    model_old = AsymmetricActorCritic(
        actor_obs_dim, critic_obs_dim, config.model
    ).to(device)
    sync_old_policy(model, model_old)
    optimizer = torch.optim.Adam(model.parameters(), lr=config.ppo.learning_rate)

    rows: list[dict[str, float | int | str]] = []
    global_update = 0
    for stage_index, curriculum_stage in enumerate(config.train.curriculum):
        environments = _make_environments(
            config, curriculum_stage, 100_000 * stage_index
        )
        observations = None
        for local_update in range(1, curriculum_stage.updates + 1):
            global_update += 1
            batch, observations = collect_rollout(
                environments,
                model_old,
                config.ppo.rollout_steps,
                device,
                observations,
            )
            metrics = ppo_update(model, optimizer, batch, config.ppo, device)
            sync_old_policy(model, model_old)
            row: dict[str, float | int | str] = {
                "global_update": global_update,
                "stage": curriculum_stage.name,
                "stage_update": local_update,
                **metrics,
            }
            rows.append(row)
            print(
                f"更新 {global_update:04d} | 阶段 {curriculum_stage.name:<9} | "
                f"成功率 {metrics['success_rate']:.3f} | "
                f"回报 {metrics['mean_episode_return']:.3f} | "
                f"KL {metrics['approx_kl']:.5f}"
            )
            if global_update % config.train.save_every == 0:
                _save_checkpoint(
                    output / "checkpoints" / f"update_{global_update:05d}.pt",
                    model=model,
                    optimizer=optimizer,
                    config=config,
                    actor_obs_dim=actor_obs_dim,
                    critic_obs_dim=critic_obs_dim,
                    update=global_update,
                    stage=curriculum_stage.name,
                )

    final_checkpoint = output / "checkpoints" / "fapp_ppo_final.pt"
    _save_checkpoint(
        final_checkpoint,
        model=model,
        optimizer=optimizer,
        config=config,
        actor_obs_dim=actor_obs_dim,
        critic_obs_dim=critic_obs_dim,
        update=global_update,
        stage=config.train.curriculum[-1].name,
    )
    if rows:
        metrics_path = output / "train_metrics.csv"
        with metrics_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        from .figures import plot_training_curves

        plot_training_curves(
            metrics_path,
            output / "training_curves.png",
            title="FAPP-PPO 训练奖励与优化诊断",
        )
    (output / "train_summary.json").write_text(
        json.dumps(
            {
                "updates": global_update,
                "final_stage": config.train.curriculum[-1].name,
                "checkpoint": str(final_checkpoint),
                "device": str(device),
                "last_metrics": rows[-1] if rows else {},
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    return final_checkpoint, rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="YAML configuration")
    parser.add_argument("--output-dir", default=None, help="override configured output directory")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    checkpoint, _ = train_model(load_config(args.config), output_dir=args.output_dir)
    print(f"训练完成：{checkpoint}")


if __name__ == "__main__":
    main()
