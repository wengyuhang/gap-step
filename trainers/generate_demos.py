from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from tqdm import tqdm

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gap_step.config import ensure_dir, load_yaml, resolve_path
from gap_step.envs import GapStepEnv
from gap_step.teachers import HeuristicTeacher


def collect_dataset(config: dict, env_config: dict) -> dict[str, np.ndarray]:
    env = GapStepEnv(env_config)
    teacher = HeuristicTeacher(env)
    rng = np.random.default_rng(int(config.get("seed", 0)))
    n_episodes = int(config.get("num_demo_episodes", 200))

    images, proprios, accs, gates, widths, safe_flags = [], [], [], [], [], []
    episode_returns = []
    for ep in tqdm(range(n_episodes), desc="Generating demos"):
        obs, _ = env.reset(seed=int(rng.integers(0, 2**31 - 1)))
        done = False
        ep_return = 0.0
        while not done:
            acc, gate = teacher.act()
            true_widths, true_safe = env.get_gate_labels()
            images.append(obs["image_stack"].copy())
            proprios.append(obs["proprio"].copy())
            accs.append(acc.copy())
            gates.append(gate)
            widths.append(true_widths.copy())
            safe_flags.append(true_safe.copy())
            obs, reward, terminated, truncated, _ = env.step(acc)
            ep_return += reward
            done = terminated or truncated
        episode_returns.append(ep_return)

    arrays = {
        "image_stack": np.asarray(images, dtype=np.float32),
        "proprio": np.asarray(proprios, dtype=np.float32),
        "teacher_acc": np.asarray(accs, dtype=np.float32),
        "teacher_gate": np.asarray(gates, dtype=np.int64),
        "true_widths": np.asarray(widths, dtype=np.float32),
        "true_safe_flags": np.asarray(safe_flags, dtype=np.float32),
        "episode_returns": np.asarray(episode_returns, dtype=np.float32),
    }
    n = len(arrays["teacher_gate"])
    perm = rng.permutation(n)
    split = int(float(config.get("train_split", 0.9)) * n)
    arrays["train_idx"] = perm[:split]
    arrays["val_idx"] = perm[split:]
    return arrays


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/train_bc.yaml")
    parser.add_argument("--env-config", default=None)
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    config = load_yaml(args.config)
    env_config = load_yaml(args.env_config or config.get("env_config", "configs/env_e3.yaml"))
    arrays = collect_dataset(config, env_config)
    output = resolve_path(args.output or config.get("dataset_path", "data/demos_e3.npz"))
    ensure_dir(output.parent)
    np.savez(output, **arrays)
    print(f"Saved {len(arrays['teacher_gate'])} samples to {output}")


if __name__ == "__main__":
    main()
