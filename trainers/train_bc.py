from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, Dataset
from tqdm import tqdm

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gap_step.config import ensure_dir, load_yaml, resolve_path
from gap_step.envs import GapStepEnv
from gap_step.models import StudentPolicy
from gap_step.torch_utils import get_device, set_seed

try:
    from torch.utils.tensorboard import SummaryWriter
except Exception:  # pragma: no cover
    class SummaryWriter:
        def __init__(self, *args, **kwargs): pass
        def add_scalar(self, *args, **kwargs): pass
        def close(self): pass


class DemoDataset(Dataset):
    def __init__(self, data: np.lib.npyio.NpzFile, indices: np.ndarray):
        self.image_stack = data["image_stack"]
        self.proprio = data["proprio"]
        self.teacher_acc = data["teacher_acc"]
        self.teacher_gate = data["teacher_gate"]
        self.true_widths = data["true_widths"]
        self.true_safe_flags = data["true_safe_flags"]
        self.indices = indices.astype(np.int64)

    def __len__(self) -> int:
        return len(self.indices)

    def __getitem__(self, item: int) -> dict[str, torch.Tensor]:
        idx = self.indices[item]
        return {
            "image_stack": torch.from_numpy(self.image_stack[idx]),
            "proprio": torch.from_numpy(self.proprio[idx]),
            "teacher_acc": torch.from_numpy(self.teacher_acc[idx]),
            "teacher_gate": torch.tensor(self.teacher_gate[idx], dtype=torch.long),
            "true_widths": torch.from_numpy(self.true_widths[idx]),
            "true_safe_flags": torch.from_numpy(self.true_safe_flags[idx]),
        }


def compute_loss(model, batch, lambda_g: float, lambda_w: float, lambda_s: float):
    out = model(batch["image_stack"], batch["proprio"])
    acc_loss = nn.functional.mse_loss(out["acc"], batch["teacher_acc"])
    gate_loss = nn.functional.cross_entropy(out["gate_logits"], batch["teacher_gate"])
    width_loss = nn.functional.l1_loss(out["width"], batch["true_widths"])
    safe_loss = nn.functional.binary_cross_entropy_with_logits(out["safe_logits"], batch["true_safe_flags"])
    total = acc_loss + lambda_g * gate_loss + lambda_w * width_loss + lambda_s * safe_loss
    return total, {
        "acc": acc_loss.detach(),
        "gate": gate_loss.detach(),
        "width": width_loss.detach(),
        "safe": safe_loss.detach(),
        "total": total.detach(),
    }


def run_one_mode(mode: str, config: dict, env_config: dict, device: torch.device) -> Path:
    data_path = resolve_path(config.get("dataset_path", "data/demos_e3.npz"))
    if not data_path.exists():
        raise FileNotFoundError(f"Dataset not found: {data_path}. Run python trainers/generate_demos.py first.")
    data = np.load(data_path)
    train_ds = DemoDataset(data, data["train_idx"])
    val_ds = DemoDataset(data, data["val_idx"])
    train_loader = DataLoader(train_ds, batch_size=int(config.get("batch_size", 128)), shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=int(config.get("batch_size", 128)), shuffle=False)

    env = GapStepEnv(env_config)
    model = StudentPolicy(env.K_obs, 6, env.num_gates, env.max_acc).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=float(config.get("lr", 3e-4)))
    writer = SummaryWriter(str(ensure_dir(config.get("runs_dir", "runs")) / mode))

    lambda_g = float(config.get("lambda_g", 0.5))
    lambda_w = 0.0 if mode == "bc_only" else float(config.get("lambda_w", 1.0))
    lambda_s = 0.0 if mode == "bc_only" else float(config.get("lambda_s", 1.0))
    global_step = 0

    for epoch in range(int(config.get("epochs", 5))):
        model.train()
        for batch in tqdm(train_loader, desc=f"{mode} epoch {epoch + 1}"):
            batch = {k: v.to(device) for k, v in batch.items()}
            loss, parts = compute_loss(model, batch, lambda_g, lambda_w, lambda_s)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            for name, value in parts.items():
                writer.add_scalar(f"train/{name}", float(value), global_step)
            global_step += 1

        model.eval()
        val_parts = []
        with torch.no_grad():
            for batch in val_loader:
                batch = {k: v.to(device) for k, v in batch.items()}
                _, parts = compute_loss(model, batch, lambda_g, lambda_w, lambda_s)
                val_parts.append({k: float(v) for k, v in parts.items()})
        if val_parts:
            for key in val_parts[0]:
                writer.add_scalar(f"val/{key}", float(np.mean([p[key] for p in val_parts])), epoch)

    ckpt_dir = ensure_dir(config.get("checkpoints_dir", "checkpoints"))
    ckpt_path = ckpt_dir / f"{mode}.pt"
    torch.save(
        {
            "model_state": model.state_dict(),
            "mode": mode,
            "env_config": env_config,
            "k_obs": env.K_obs,
            "num_gates": env.num_gates,
            "max_acc": env.max_acc,
        },
        ckpt_path,
    )
    writer.close()
    print(f"Saved {mode} checkpoint to {ckpt_path}")
    return ckpt_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/train_bc.yaml")
    parser.add_argument("--mode", choices=["bc_only", "bc_aux", "both"], default="both")
    parser.add_argument("--device", default="auto")
    args = parser.parse_args()

    config = load_yaml(args.config)
    env_config = load_yaml(config.get("env_config", "configs/env_e3.yaml"))
    set_seed(int(config.get("seed", 0)))
    device = get_device(args.device)
    modes = ["bc_only", "bc_aux"] if args.mode == "both" else [args.mode]
    for mode in modes:
        run_one_mode(mode, config, env_config, device)


if __name__ == "__main__":
    main()
