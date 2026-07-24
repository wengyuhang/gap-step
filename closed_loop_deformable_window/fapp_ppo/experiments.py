"""One-command smoke/default training, evaluation, and visualization."""

from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

from .config import load_config
from .evaluate import evaluate_checkpoint
from .train import train_model
from .visualize import visualize_checkpoint


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", choices=("smoke", "default"), default="smoke")
    parser.add_argument("--outdir", default="closed_loop_deformable_window/fapp_ppo/results")
    args = parser.parse_args()
    module_dir = Path(__file__).resolve().parent
    config_path = module_dir / "configs" / f"train_{args.suite}.yaml"
    run_name = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + f"_{args.suite}"
    output = Path(args.outdir) / run_name
    config = load_config(config_path)
    checkpoint, _ = train_model(config, output_dir=output)
    episodes = 2 if args.suite == "smoke" else 100
    summary, records = evaluate_checkpoint(
        checkpoint,
        episodes=episodes,
        stage="full",
        seed=20_260,
        device=config.train.device,
    )
    (output / "evaluation_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    (output / "evaluation_records.json").write_text(
        json.dumps(records, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    visualize_checkpoint(
        checkpoint,
        output / "rollout.png",
        stage="full",
        seed=20_260,
        device=config.train.device,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"结果目录：{output}")


if __name__ == "__main__":
    main()

