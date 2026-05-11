from __future__ import annotations

import random
from pathlib import Path
from typing import Any

import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def resolve_path(path: str | Path) -> Path:
    path = Path(path)
    if path.is_absolute():
        return path
    return PROJECT_ROOT / path


def ensure_dir(path: str | Path) -> Path:
    path = resolve_path(path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def load_yaml(path: str | Path) -> dict[str, Any]:
    path = resolve_path(path)
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)


def wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def circle_intersects_rect(center: np.ndarray, radius: float, rect: dict[str, float]) -> bool:
    closest_x = np.clip(center[0], rect["xmin"], rect["xmax"])
    closest_y = np.clip(center[1], rect["ymin"], rect["ymax"])
    dx = center[0] - closest_x
    dy = center[1] - closest_y
    return bool(dx * dx + dy * dy <= radius * radius)


def ray_rect_distance(origin: np.ndarray, direction: np.ndarray, rect: dict[str, float]) -> float | None:
    tmin = -float("inf")
    tmax = float("inf")
    for axis, lo_key, hi_key in ((0, "xmin", "xmax"), (1, "ymin", "ymax")):
        d = float(direction[axis])
        o = float(origin[axis])
        lo = float(rect[lo_key])
        hi = float(rect[hi_key])
        if abs(d) < 1e-8:
            if o < lo or o > hi:
                return None
            continue
        t1 = (lo - o) / d
        t2 = (hi - o) / d
        t_near = min(t1, t2)
        t_far = max(t1, t2)
        tmin = max(tmin, t_near)
        tmax = min(tmax, t_far)
        if tmin > tmax:
            return None
    if tmax < 1e-8:
        return None
    return float(max(tmin, 0.0))
