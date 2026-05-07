from __future__ import annotations

import numpy as np


def world_to_pixel(pos: np.ndarray, width: float, height: float, image_size: int) -> tuple[int, int]:
    x = int(np.clip(pos[0] / width * (image_size - 1), 0, image_size - 1))
    y = int(np.clip((1.0 - pos[1] / height) * (image_size - 1), 0, image_size - 1))
    return x, y


def _draw_disk(img: np.ndarray, cx: int, cy: int, radius_px: int, value) -> None:
    h, w = img.shape[:2]
    y, x = np.ogrid[:h, :w]
    mask = (x - cx) ** 2 + (y - cy) ** 2 <= radius_px**2
    img[mask] = value


def render_gray(env, image_size: int = 64) -> np.ndarray:
    img = np.full((image_size, image_size), 0.08, dtype=np.float32)
    wall_col, _ = world_to_pixel(np.array([env.wall_x, 0.0]), env.W, env.H, image_size)
    img[:, max(0, wall_col - 1) : min(image_size, wall_col + 2)] = 0.75

    widths = env.gates.widths(env.t)
    for gate, width in zip(env.gates.gates, widths):
        lo = gate.y - 0.5 * width
        hi = gate.y + 0.5 * width
        _, py_hi = world_to_pixel(np.array([env.wall_x, lo]), env.W, env.H, image_size)
        _, py_lo = world_to_pixel(np.array([env.wall_x, hi]), env.W, env.H, image_size)
        img[min(py_lo, py_hi) : max(py_lo, py_hi) + 1, max(0, wall_col - 1) : min(image_size, wall_col + 2)] = 0.08

    gx, gy = world_to_pixel(env.goal, env.W, env.H, image_size)
    _draw_disk(img, gx, gy, 2, 0.45)
    ax, ay = world_to_pixel(env.pos, env.W, env.H, image_size)
    radius_px = max(2, int(env.robot_radius / env.W * image_size))
    _draw_disk(img, ax, ay, radius_px, 1.0)
    return img


def render_rgb(env, image_size: int = 512, trajectory: list[np.ndarray] | None = None) -> np.ndarray:
    img = np.full((image_size, image_size, 3), 245, dtype=np.uint8)
    wall_col, _ = world_to_pixel(np.array([env.wall_x, 0.0]), env.W, env.H, image_size)
    img[:, max(0, wall_col - 3) : min(image_size, wall_col + 4)] = np.array([45, 45, 45], dtype=np.uint8)

    widths = env.gates.widths(env.t)
    safe = env.gates.safe_flags(env.t)
    for idx, (gate, width) in enumerate(zip(env.gates.gates, widths)):
        lo = gate.y - 0.5 * width
        hi = gate.y + 0.5 * width
        _, py_hi = world_to_pixel(np.array([env.wall_x, lo]), env.W, env.H, image_size)
        _, py_lo = world_to_pixel(np.array([env.wall_x, hi]), env.W, env.H, image_size)
        color = np.array([232, 248, 238] if safe[idx] > 0.5 else [252, 226, 226], dtype=np.uint8)
        img[min(py_lo, py_hi) : max(py_lo, py_hi) + 1, max(0, wall_col - 4) : min(image_size, wall_col + 5)] = color

    if trajectory:
        pts = [world_to_pixel(p, env.W, env.H, image_size) for p in trajectory]
        for x, y in pts:
            if 0 <= x < image_size and 0 <= y < image_size:
                img[max(0, y - 1) : min(image_size, y + 2), max(0, x - 1) : min(image_size, x + 2)] = np.array([70, 130, 180], dtype=np.uint8)

    gx, gy = world_to_pixel(env.goal, env.W, env.H, image_size)
    _draw_disk(img, gx, gy, 8, np.array([30, 140, 70], dtype=np.uint8))
    ax, ay = world_to_pixel(env.pos, env.W, env.H, image_size)
    radius_px = max(5, int(env.robot_radius / env.W * image_size))
    _draw_disk(img, ax, ay, radius_px, np.array([225, 80, 40], dtype=np.uint8))
    return img
