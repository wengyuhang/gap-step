"""Deterministic static and dynamic scenarios used by experiments E2--E5."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Sequence

import numpy as np
from numpy.typing import ArrayLike

from .boundary import DenseBoundary
from .environment import MotionProfile, SCDynamicWindow, SCWindowTrack
from .preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    e1_boundaries,
    five_point_star_boundary,
    l_shape_boundary,
    preprocess_boundary,
    u_shape_boundary,
)


MotionMode = Literal["static", "translation", "full"]
DiverseLayout = Literal["compact", "spacious"]


@dataclass(frozen=True)
class Scenario:
    name: str
    mode: MotionMode
    track: SCWindowTrack
    preprocessed_gates: tuple[PreprocessedGate, ...]


def _motion(
    mode: MotionMode,
    index: int,
    *,
    amplitude_scale: float = 1.0,
) -> MotionProfile:
    multiplier = float(amplitude_scale)
    if not np.isfinite(multiplier) or multiplier <= 0.0:
        raise ValueError("motion amplitude scale must be positive and finite")
    if mode == "full" and 0.12 * multiplier >= 1.0:
        raise ValueError("motion amplitude scale makes the uniform scale non-positive")
    phase = 0.55 * index
    if mode == "static":
        return MotionProfile.static()
    translation = (
        multiplier * np.array([0.12, 0.28, 0.20]) * (1.0 + 0.08 * index)
    )
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
            multiplier * np.array([0.16, 0.11, 0.09]),
            multiplier * 0.12,
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
                physical_boundary=gate.dense_boundary.vertices,
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


def build_boundary_scenario(
    definitions: Sequence[tuple[str, DenseBoundary]],
    *,
    mode: MotionMode = "full",
    preprocessing_config: PreprocessingConfig | None = None,
    spacing: float = 2.2,
    centers: Sequence[ArrayLike] | None = None,
    angles: Sequence[ArrayLike] | None = None,
    motion_scale: float = 1.0,
    name: str = "custom",
) -> Scenario:
    """Build a one-pass dynamic track from any ordered simple-boundary list.

    This is the general scenario entry point.  Polygonal, smooth, mixed, and
    CSV-derived boundaries all reach the same preprocessing/SC/window path as
    long as they are represented by :class:`DenseBoundary` and satisfy the
    documented simple, hole-free topology.
    """

    items = tuple(definitions)
    if not items:
        raise ValueError("a boundary scenario requires at least one window")
    labels = tuple(str(label) for label, _ in items)
    if any(not label for label in labels) or len(set(labels)) != len(labels):
        raise ValueError("boundary scenario names must be non-empty and unique")
    distance = float(spacing)
    if not np.isfinite(distance) or distance <= 0.0:
        raise ValueError("spacing must be positive and finite")
    if mode not in {"static", "translation", "full"}:
        raise ValueError(f"unknown motion mode {mode!r}")

    settings = PreprocessingConfig() if preprocessing_config is None else preprocessing_config
    gates = tuple(
        preprocess_boundary(boundary, name=label, config=settings)
        for label, boundary in items
    )
    count = len(gates)
    if centers is None:
        x_positions = distance * (np.arange(count, dtype=float) - 0.5 * (count - 1))
        center_values = np.column_stack(
            (x_positions, np.zeros(count), np.full(count, 1.4))
        )
    else:
        center_values = np.asarray(centers, dtype=float)
        if center_values.shape != (count, 3) or not np.all(np.isfinite(center_values)):
            raise ValueError(f"centers must be finite with shape ({count}, 3)")

    if angles is None:
        angle_values = np.array(
            [
                [0.02 * index, np.pi / 2.0 - 0.025 * index, 0.04 * index]
                for index in range(count)
            ]
        )
    else:
        angle_values = np.asarray(angles, dtype=float)
        if angle_values.shape != (count, 3) or not np.all(np.isfinite(angle_values)):
            raise ValueError(f"angles must be finite with shape ({count}, 3)")

    windows = tuple(
        SCDynamicWindow(
            name=gate.name,
            sc_map=gate.sc_map,
            safe_polygon=gate.safe_polygon,
            center0=center.copy(),
            angles0=angle.copy(),
            motion=_motion(mode, index, amplitude_scale=motion_scale),
            physical_boundary=gate.dense_boundary.vertices,
        )
        for index, (gate, center, angle) in enumerate(
            zip(gates, center_values, angle_values)
        )
    )
    if centers is None:
        extent = max(4.5, float(np.max(np.abs(center_values[:, 0]))) + 2.5)
        start = np.array([-extent, 0.0, 1.4])
        goal = np.array([extent, 0.0, 1.4])
    else:
        # Keep the endpoints near the first/last gate while leaving enough
        # approach and departure distance to make the crossing visible.
        start = center_values[0] - np.array([3.5, 0.0, 0.0])
        goal = center_values[-1] + np.array([3.5, 0.0, 0.0])
    track_name = f"{name}_{mode}_{count}"
    track = SCWindowTrack(
        name=track_name,
        start=start,
        goal=goal,
        windows=windows,
        order=tuple(range(count)),
    )
    return Scenario(track_name, mode, track, gates)


def build_diverse_scenario(
    *,
    mode: MotionMode = "full",
    preprocessing_config: PreprocessingConfig | None = None,
    spacing: float = 2.2,
    layout: DiverseLayout = "spacious",
    motion_scale: float = 2.5,
) -> Scenario:
    """Build the six-window polygon/smooth/mixed visualization scenario.

    ``spacious`` is the visualization default: centres span all three axes,
    initial attitudes differ visibly, and motion amplitudes are larger than in
    the controlled E3--E5 experiments.  ``compact`` retains the original
    straight x-axis arrangement for reproducibility.
    """

    if layout not in {"compact", "spacious"}:
        raise ValueError("layout must be compact or spacious")

    catalog = e1_boundaries()
    definitions = (
        ("L", catalog["l_shape"]),
        ("U", catalog["u_shape"]),
        ("star", catalog["five_point_star"]),
        ("limacon", catalog["limacon"]),
        ("wavy", catalog["wavy"]),
        ("line_bezier", catalog["line_bezier_mixed"]),
    )
    centers = None
    angles = None
    if layout == "spacious":
        centers = np.array(
            [
                [-10.0, -3.5, 1.5],
                [-6.2, 0.5, 4.8],
                [-2.2, 3.6, 2.5],
                [2.0, 1.2, 5.8],
                [6.0, -3.3, 3.7],
                [10.2, 0.8, 1.8],
            ]
        )
        angles = np.array(
            [
                [0.00, np.pi / 2.0 - 0.10, 0.00],
                [0.18, np.pi / 2.0 + 0.15, 0.20],
                [-0.12, np.pi / 2.0 - 0.20, -0.25],
                [0.25, np.pi / 2.0 + 0.22, 0.30],
                [-0.20, np.pi / 2.0 - 0.16, -0.32],
                [0.14, np.pi / 2.0 + 0.12, 0.18],
            ]
        )
    return build_boundary_scenario(
        definitions,
        mode=mode,
        preprocessing_config=preprocessing_config,
        spacing=spacing,
        centers=centers,
        angles=angles,
        motion_scale=motion_scale,
        name="diverse",
    )


__all__ = [
    "DiverseLayout",
    "MotionMode",
    "Scenario",
    "build_boundary_scenario",
    "build_canonical_scenario",
    "build_diverse_scenario",
]
