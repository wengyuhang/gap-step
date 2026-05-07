from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image


def save_gif(frames: list[np.ndarray], path: str | Path, fps: int = 20) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pil_frames = [Image.fromarray(frame.astype(np.uint8)) for frame in frames]
    if not pil_frames:
        raise ValueError("No frames to save.")
    pil_frames[0].save(
        path,
        save_all=True,
        append_images=pil_frames[1:],
        duration=int(1000 / fps),
        loop=0,
    )
