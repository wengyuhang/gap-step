from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from gap_step.gif import save_gif
from gap_step.utils import resolve_path
from gap_step.window_maze_env import TimeVaryingWindowMazeEnv


def generate_preview(
    gif_path: str | Path = "preview/high_difficulty_window_maze.gif",
    phases_path: str | Path = "preview/high_difficulty_window_maze_phases.png",
    *,
    fps: int = 4,
    cycles: int = 3,
) -> tuple[Path, Path]:
    env = TimeVaryingWindowMazeEnv({"render_width": 980, "render_height": 540, "show_reference_path": True})
    frames: list[np.ndarray] = []
    for t in range(env.period * cycles):
        env.reset(options={"phase": t % env.period})
        frames.append(env.render())

    gif_path = resolve_path(gif_path)
    phases_path = resolve_path(phases_path)
    save_gif(frames, gif_path, fps=fps)

    phase_frames = []
    for t in range(env.period):
        env.reset(options={"phase": t})
        frame = Image.fromarray(env.render())
        draw = ImageDraw.Draw(frame)
        draw.rectangle((0, 0, 90, 34), fill=(255, 255, 255))
        draw.text((12, 8), f"t = {t}", fill=(10, 10, 10))
        phase_frames.append(frame.resize((490, 270)))

    canvas = Image.new("RGB", (490 * 4, 270 * 2 + 46), (255, 255, 255))
    title_draw = ImageDraw.Draw(canvas)
    title_draw.text((16, 12), "High-difficulty 2D maze filled with aperture windows: each wall-to-wall line/curve has one time-varying gap", fill=(15, 15, 15))
    for idx, frame in enumerate(phase_frames):
        x = (idx % 4) * 490
        y = 46 + (idx // 4) * 270
        canvas.paste(frame, (x, y))
    phases_path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(phases_path)
    return gif_path, phases_path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gif", default="preview/high_difficulty_window_maze.gif")
    parser.add_argument("--phases", default="preview/high_difficulty_window_maze_phases.png")
    parser.add_argument("--fps", type=int, default=4)
    parser.add_argument("--cycles", type=int, default=3)
    args = parser.parse_args()
    gif_path, phases_path = generate_preview(args.gif, args.phases, fps=args.fps, cycles=args.cycles)
    print(f"已保存 GIF 预览: {gif_path}")
    print(f"已保存相位拼图: {phases_path}")


if __name__ == "__main__":
    main()
