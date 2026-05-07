from __future__ import annotations

import argparse
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gap_step.config import load_yaml, resolve_path
from gap_step.envs import GapStepEnv
from gap_step.gif import save_gif
from gap_step.models import StudentPolicy
from gap_step.torch_utils import get_device, obs_to_torch


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env-config", default="configs/env_e3.yaml")
    parser.add_argument("--checkpoint", default="checkpoints/bc_aux_ppo.pt")
    parser.add_argument("--output", default="runs/trained_policy.gif")
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()

    device = get_device(args.device)
    env = GapStepEnv(load_yaml(args.env_config))
    ckpt_path = resolve_path(args.checkpoint)
    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    model = StudentPolicy(env.K_obs, 6, env.num_gates, env.max_acc).to(device)
    model.load_state_dict(ckpt["model_state"], strict=False)
    model.eval()

    obs, _ = env.reset(seed=0)
    frames = []
    for step in range(args.steps):
        with torch.no_grad():
            image, proprio = obs_to_torch(obs, device)
            out = model(image, proprio)
        obs, _, terminated, truncated, _ = env.step(out["acc"].squeeze(0).cpu().numpy())
        if step % 2 == 0:
            frames.append(env.render())
        if terminated or truncated:
            break
    out_path = resolve_path(args.output)
    save_gif(frames, out_path)
    print(f"Saved trained rollout to {out_path}")


if __name__ == "__main__":
    main()
