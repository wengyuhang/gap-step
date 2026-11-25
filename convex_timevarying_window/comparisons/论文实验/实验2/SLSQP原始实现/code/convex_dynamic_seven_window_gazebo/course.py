"""Standalone loader and motion equations for the convex seven-window course."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent


def load_course() -> dict:
    return json.loads((HERE / "course_spec.json").read_text(encoding="utf-8"))


def local_boundary(window: dict, circle_samples: int = 128) -> np.ndarray:
    if window["aperture_kind"] == "circle":
        angles = np.linspace(0.0, 2.0 * math.pi, circle_samples, endpoint=False)
        radius = float(window["radius"])
        return radius * np.column_stack((np.cos(angles), np.sin(angles)))
    return np.asarray(window["polygon_vertices_local"], dtype=float)


def pose_at(window: dict, instant: float) -> tuple[np.ndarray, np.ndarray]:
    motion = window["motion"]
    phase = float(motion["phase"])
    translation_phase = phase + np.asarray((0.0, 0.7, 1.4))
    rotation_phase = phase + np.asarray((0.0, 0.9, 1.8))
    position = np.asarray(window["center0"], dtype=float) + np.asarray(
        motion["translation_amplitude"], dtype=float
    ) * np.sin(2.0 * math.pi * float(instant) / float(motion["translation_period"]) + translation_phase)
    rpy = np.asarray(window["angles0_rpy_rad"], dtype=float) + np.asarray(
        motion["rotation_amplitude"], dtype=float
    ) * np.sin(2.0 * math.pi * float(instant) / float(motion["rotation_period"]) + rotation_phase)
    return position, rpy

