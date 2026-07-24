"""Train reproducible opportunity-timing ablations from one base YAML."""

from __future__ import annotations

import argparse
import copy
from pathlib import Path

from .config import CurriculumStage, load_config, validate_config
from .train import train_model


VARIANTS = (
    "full",
    "reactive_actor",
    "no_privileged_critic",
    "no_schedule_nominal",
    "no_residual_prior",
    "no_curriculum",
    "small_motion_training",
)


def make_variant(base, variant: str, *, seed: int, output: Path):
    if variant not in VARIANTS:
        raise ValueError(f"unknown ablation {variant!r}")
    config = copy.deepcopy(base)
    config.train.seed = int(seed)
    config.train.output_dir = str(output / f"{variant}_seed{seed}")
    if variant == "reactive_actor":
        config.environment.preview_horizons = (0.0,)
        config.environment.opportunity_features = False
    elif variant == "no_privileged_critic":
        config.environment.critic_time_feature = False
        config.environment.critic_privileged_route = False
    elif variant == "no_schedule_nominal":
        config.environment.opportunity_aware_nominal = False
    elif variant == "no_residual_prior":
        config.ppo.residual_prior_coef = 0.0
    elif variant == "no_curriculum":
        updates = sum(stage.updates for stage in config.train.curriculum)
        config.train.curriculum = [
            CurriculumStage(
                "full",
                updates,
                opportunity_mode=config.environment.opportunity_mode,
                opportunity_width=config.environment.opportunity_width,
                motion_amplitude_multiplier=config.environment.motion_amplitude_multiplier,
                deformation_amplitude_multiplier=config.environment.deformation_amplitude_multiplier,
                opportunity_schedule_jitter=config.environment.opportunity_schedule_jitter,
            )
        ]
    elif variant == "small_motion_training":
        config.environment.motion_amplitude_multiplier = 1.0
        config.environment.deformation_amplitude_multiplier = 1.0
        for stage in config.train.curriculum:
            stage.motion_amplitude_multiplier = min(
                stage.motion_amplitude_multiplier or 1.0, 1.0
            )
            stage.deformation_amplitude_multiplier = min(
                stage.deformation_amplitude_multiplier or 1.0, 1.0
            )
    validate_config(config)
    return config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=(
            "closed_loop_deformable_window/fapp_ppo/configs/"
            "train_opportunity_default.yaml"
        ),
    )
    parser.add_argument("--variant", choices=(*VARIANTS, "all"), default="all")
    parser.add_argument("--seeds", default="17,29,41,53,67")
    parser.add_argument(
        "--outdir",
        default="closed_loop_deformable_window/fapp_ppo/runs/ablations",
    )
    args = parser.parse_args()
    base = load_config(args.config)
    variants = VARIANTS if args.variant == "all" else (args.variant,)
    seeds = [int(item) for item in args.seeds.split(",") if item.strip()]
    for variant in variants:
        for seed in seeds:
            output = Path(args.outdir) / f"{variant}_seed{seed}"
            config = make_variant(base, variant, seed=seed, output=Path(args.outdir))
            checkpoint, _ = train_model(config, output_dir=output)
            print(f"{variant} seed={seed}: {checkpoint}")


if __name__ == "__main__":
    main()

