"""Deterministic static and dynamic scenarios used by experiments E2--E5."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from .environment import MotionProfile, SCDynamicWindow, SCWindowTrack
from .preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    five_point_star_boundary,
    l_shape_boundary,
    preprocess_boundary,
    u_shape_boundary,
)


MotionMode = Literal["static", "translation", "full"]


@dataclass(frozen=True)
class Scenario:
    name: str
    mode: MotionMode
    track: SCWindowTrack
    preprocessed_gates: tuple[PreprocessedGate, ...]


def _motion(mode: MotionMode, index: int) -> MotionProfile:
    phase = 0.55 * index
    if mode == "static":
        return MotionProfile.static()
    translation = np.array([0.12, 0.28, 0.20]) * (1.0 + 0.08 * index)
    if mode == "translation":
        return MotionProfile(
            translation,
            np.zeros(3),
            0.0,
            translation_period=6.5 + 0.4 * index,
            phase=phase,
            rotation_enabled=False,
            scale_enabled=False,
        )
    if mode == "full":
        return MotionProfile(
            translation,
            np.array([0.16, 0.11, 0.09]),
            0.12,
            translation_period=6.5 + 0.4 * index,
            rotation_period=7.5 + 0.5 * index,
            scale_period=8.5 + 0.3 * index,
            phase=phase,
        )
    raise ValueError(f"unknown motion mode {mode!r}")


def build_canonical_scenario(
    *,
    mode: MotionMode = "static",
    preprocessing_config: PreprocessingConfig | None = None,
    gate_count: int = 3,
) -> Scenario:
    """Build a one-pass L/U/star track with production preprocessing defaults."""

    if gate_count < 1 or gate_count > 3:
        raise ValueError("canonical gate_count must be in [1, 3]")
    settings = PreprocessingConfig() if preprocessing_config is None else preprocessing_config
    definitions = (
        ("L", l_shape_boundary()),
        ("U", u_shape_boundary()),
        ("star", five_point_star_boundary()),
    )[:gate_count]
    gates = tuple(
        preprocess_boundary(boundary, name=name, config=settings)
        for name, boundary in definitions
    )
    x_positions = np.linspace(-2.2, 2.2, gate_count) if gate_count > 1 else np.array([0.0])
    windows: list[SCDynamicWindow] = []
    for index, (x_position, gate) in enumerate(zip(x_positions, gates)):
        windows.append(
            SCDynamicWindow(
                name=gate.name,
                sc_map=gate.sc_map,
                safe_polygon=gate.safe_polygon,
                center0=np.array([x_position, 0.0, 1.4]),
                # RPY: pitch=pi/2 turns TOGT's local x-y gate plane vertical,
                # with its normal approximately aligned to the track x-axis.
                angles0=np.array([0.02 * index, np.pi / 2.0 - 0.025 * index, 0.04 * index]),
                motion=_motion(mode, index),
            )
        )
    extent = max(4.5, float(np.max(np.abs(x_positions))) + 2.3)
    track = SCWindowTrack(
        name=f"canonical_{mode}_{gate_count}",
        start=np.array([-extent, 0.0, 1.4]),
        goal=np.array([extent, 0.0, 1.4]),
        windows=tuple(windows),
        order=tuple(range(gate_count)),
    )
    return Scenario(track.name, mode, track, gates)


__all__ = ["MotionMode", "Scenario", "build_canonical_scenario"]
