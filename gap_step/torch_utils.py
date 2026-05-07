from __future__ import annotations

import random

import numpy as np
import torch


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def get_device(device: str = "auto") -> torch.device:
    if device == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device)


def obs_to_torch(obs: dict, device: torch.device) -> tuple[torch.Tensor, torch.Tensor]:
    image = torch.as_tensor(obs["image_stack"], dtype=torch.float32, device=device).unsqueeze(0)
    proprio = torch.as_tensor(obs["proprio"], dtype=torch.float32, device=device).unsqueeze(0)
    return image, proprio
