from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from tqdm import trange

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gap_step.config import ensure_dir, load_yaml, resolve_path
from gap_step.envs import GapStepEnv
from gap_step.models import StudentPolicy
from gap_step.torch_utils import get_device, obs_to_torch, set_seed

try:
    from torch.utils.tensorboard import SummaryWriter
except Exception:  # pragma: no cover
    class SummaryWriter:
        def __init__(self, *args, **kwargs): pass
        def add_scalar(self, *args, **kwargs): pass
        def close(self): pass


def load_policy(env: GapStepEnv, checkpoint: str | None, device: torch.device) -> StudentPolicy:
    model = StudentPolicy(env.K_obs, 6, env.num_gates, env.max_acc).to(device)
    if checkpoint:
        path = resolve_path(checkpoint)
        if path.exists():
            ckpt = torch.load(path, map_location=device, weights_only=False)
            model.load_state_dict(ckpt["model_state"], strict=False)
            print(f"Initialized PPO from {path}")
        else:
            print(f"Checkpoint {path} not found; training PPO from scratch.")
    return model


def evaluate_deterministic(model: StudentPolicy, env_config: dict, device: torch.device, episodes: int, seed: int) -> dict[str, float]:
    env = GapStepEnv(env_config)
    was_training = model.training
    model.eval()
    returns: list[float] = []
    successes = 0
    collisions = 0
    crossings = 0
    with torch.no_grad():
        for ep in range(episodes):
            obs, _ = env.reset(seed=seed + ep)
            done = False
            ep_return = 0.0
            final_info = {}
            while not done:
                image, proprio = obs_to_torch(obs, device)
                out = model(image, proprio)
                obs, reward, terminated, truncated, final_info = env.step(out["acc"].squeeze(0).cpu().numpy())
                ep_return += reward
                done = terminated or truncated
            returns.append(ep_return)
            successes += int(final_info.get("success", False))
            collisions += int(final_info.get("collision", False))
            crossings += int(final_info.get("crossing_success", False) or final_info.get("crossed_wall", False))
    if was_training:
        model.train()
    return {
        "return": float(np.mean(returns)),
        "success_rate": successes / max(1, episodes),
        "collision_rate": collisions / max(1, episodes),
        "crossing_rate": crossings / max(1, episodes),
    }


def collect_rollout(env, model, steps: int, device: torch.device, seed: int | None = None):
    obs, _ = env.reset(seed=seed)
    buf = {k: [] for k in ["image", "proprio", "action", "logp", "value", "reward", "done", "width", "safe"]}
    ep_returns, ep_return = [], 0.0
    for _ in range(steps):
        image_t, proprio_t = obs_to_torch(obs, device)
        with torch.no_grad():
            action_t, logp_t, out = model.act(image_t, proprio_t, deterministic=False)
        action = action_t.squeeze(0).cpu().numpy()
        widths, safe = env.get_gate_labels()
        next_obs, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated

        buf["image"].append(obs["image_stack"])
        buf["proprio"].append(obs["proprio"])
        buf["action"].append(action)
        buf["logp"].append(float(logp_t.item()))
        buf["value"].append(float(out["value"].item()))
        buf["reward"].append(float(reward))
        buf["done"].append(float(done))
        buf["width"].append(widths)
        buf["safe"].append(safe)
        ep_return += reward
        obs = next_obs
        if done:
            ep_returns.append(ep_return)
            ep_return = 0.0
            obs, _ = env.reset()

    with torch.no_grad():
        image_t, proprio_t = obs_to_torch(obs, device)
        last_value = float(model(image_t, proprio_t)["value"].item())
    return {k: np.asarray(v, dtype=np.float32) for k, v in buf.items()}, last_value, ep_returns


def compute_gae(rewards, values, dones, last_value, gamma, gae_lambda):
    adv = np.zeros_like(rewards, dtype=np.float32)
    last_adv = 0.0
    for t in reversed(range(len(rewards))):
        next_nonterminal = 1.0 - dones[t]
        next_value = last_value if t == len(rewards) - 1 else values[t + 1]
        delta = rewards[t] + gamma * next_value * next_nonterminal - values[t]
        last_adv = delta + gamma * gae_lambda * next_nonterminal * last_adv
        adv[t] = last_adv
    returns = adv + values
    return adv, returns


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/train_ppo.yaml")
    parser.add_argument("--init", default=None, help="Override init checkpoint; use 'none' for scratch.")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    config = load_yaml(args.config)
    env_config = load_yaml(config.get("env_config", "configs/env_e3.yaml"))
    set_seed(int(config.get("seed", 1)))
    device = get_device(config.get("device", "auto"))
    env = GapStepEnv(env_config)
    init = config.get("init_checkpoint", "checkpoints/bc_aux.pt")
    if args.init is not None:
        init = None if args.init.lower() == "none" else args.init
    model = load_policy(env, init, device)
    if "initial_log_std" in config:
        model.log_std.data.fill_(float(config["initial_log_std"]))
    opt = torch.optim.Adam(model.parameters(), lr=float(config.get("lr", 1e-4)))
    writer = SummaryWriter(str(ensure_dir(config.get("runs_dir", "runs")) / "ppo"))

    rollout_steps = int(config.get("rollout_steps", 512))
    minibatch_size = int(config.get("minibatch_size", 128))
    gamma = float(config.get("gamma", 0.99))
    gae_lambda = float(config.get("gae_lambda", 0.95))
    clip_ratio = float(config.get("clip_ratio", 0.2))
    value_coef = float(config.get("value_coef", 0.5))
    entropy_coef = float(config.get("entropy_coef", 0.01))
    aux_coef = float(config.get("aux_coef", 0.05))
    max_grad_norm = float(config.get("max_grad_norm", 0.5))
    eval_interval = int(config.get("eval_interval", 10))
    eval_episodes = int(config.get("eval_episodes", 20))
    best_metric = str(config.get("best_metric", "success_then_return"))
    best_score = -float("inf")
    best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

    def score_eval(metrics: dict[str, float]) -> float:
        if best_metric == "return":
            return metrics["return"]
        return 1000.0 * metrics["success_rate"] + metrics["return"]

    initial_metrics = evaluate_deterministic(model, env_config, device, eval_episodes, int(config.get("seed", 1)) + 10000)
    best_score = score_eval(initial_metrics)
    writer.add_scalar("eval/return", initial_metrics["return"], 0)
    writer.add_scalar("eval/success_rate", initial_metrics["success_rate"], 0)
    writer.add_scalar("eval/collision_rate", initial_metrics["collision_rate"], 0)

    global_step = 0
    for update in trange(int(config.get("total_updates", 20)), desc="PPO updates"):
        batch, last_value, ep_returns = collect_rollout(env, model, rollout_steps, device)
        adv, ret = compute_gae(batch["reward"], batch["value"], batch["done"], last_value, gamma, gae_lambda)
        adv = (adv - adv.mean()) / (adv.std() + 1e-8)
        n = len(adv)
        inds = np.arange(n)

        tensors = {
            "image": torch.as_tensor(batch["image"], dtype=torch.float32, device=device),
            "proprio": torch.as_tensor(batch["proprio"], dtype=torch.float32, device=device),
            "action": torch.as_tensor(batch["action"], dtype=torch.float32, device=device),
            "old_logp": torch.as_tensor(batch["logp"], dtype=torch.float32, device=device),
            "adv": torch.as_tensor(adv, dtype=torch.float32, device=device),
            "ret": torch.as_tensor(ret, dtype=torch.float32, device=device),
            "width": torch.as_tensor(batch["width"], dtype=torch.float32, device=device),
            "safe": torch.as_tensor(batch["safe"], dtype=torch.float32, device=device),
        }

        for _ in range(int(config.get("epochs", 4))):
            np.random.shuffle(inds)
            for start in range(0, n, minibatch_size):
                mb = inds[start : start + minibatch_size]
                dist, out = model.distribution(tensors["image"][mb], tensors["proprio"][mb])
                logp = dist.log_prob(tensors["action"][mb]).sum(dim=-1)
                ratio = torch.exp(logp - tensors["old_logp"][mb])
                pg1 = ratio * tensors["adv"][mb]
                pg2 = torch.clamp(ratio, 1.0 - clip_ratio, 1.0 + clip_ratio) * tensors["adv"][mb]
                policy_loss = -torch.min(pg1, pg2).mean()
                value_loss = torch.nn.functional.mse_loss(out["value"], tensors["ret"][mb])
                entropy = dist.entropy().sum(dim=-1).mean()
                width_loss = torch.nn.functional.l1_loss(out["width"], tensors["width"][mb])
                safe_loss = torch.nn.functional.binary_cross_entropy_with_logits(out["safe_logits"], tensors["safe"][mb])
                loss = policy_loss + value_coef * value_loss - entropy_coef * entropy + aux_coef * (width_loss + safe_loss)
                opt.zero_grad(set_to_none=True)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_grad_norm)
                opt.step()
                global_step += len(mb)

        writer.add_scalar("ppo/return_mean", float(np.mean(ep_returns)) if ep_returns else 0.0, update)
        writer.add_scalar("ppo/reward_mean", float(batch["reward"].mean()), update)
        if (update + 1) % eval_interval == 0 or update == int(config.get("total_updates", 20)) - 1:
            metrics = evaluate_deterministic(model, env_config, device, eval_episodes, int(config.get("seed", 1)) + 10000 + update)
            writer.add_scalar("eval/return", metrics["return"], update + 1)
            writer.add_scalar("eval/success_rate", metrics["success_rate"], update + 1)
            writer.add_scalar("eval/collision_rate", metrics["collision_rate"], update + 1)
            score = score_eval(metrics)
            if score > best_score:
                best_score = score
                best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}

    output = resolve_path(args.output or config.get("output_checkpoint", "checkpoints/bc_aux_ppo.pt"))
    ensure_dir(output.parent)
    model.load_state_dict(best_state)
    torch.save(
        {
            "model_state": model.state_dict(),
            "mode": "ppo",
            "env_config": env_config,
            "k_obs": env.K_obs,
            "num_gates": env.num_gates,
            "max_acc": env.max_acc,
            "best_score": best_score,
        },
        output,
    )
    writer.close()
    print(f"Saved PPO checkpoint to {output}")


if __name__ == "__main__":
    main()
