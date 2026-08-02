"""Scenarios that use the physical (non-inset) WBSC aperture maps."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Sequence

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import DenseBoundary
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile,
    SCDynamicWindow,
    SCWindowTrack,
)
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    e1_boundaries,
    five_point_star_boundary,
    l_shape_boundary,
    line_bezier_mixed_boundary,
    u_shape_boundary,
)

from .preprocessing import WBPreprocessedGate, WBPreprocessingConfig, preprocess_boundary


MotionMode = Literal["static", "translation", "full"]


@dataclass(frozen=True)
class WBScenario:
    name: str
    mode: MotionMode
    track: SCWindowTrack
    preprocessed_gates: tuple[WBPreprocessedGate, ...]
    seed: int


def scaled_boundary(boundary: DenseBoundary, scale: float) -> DenseBoundary:
    value = float(scale)
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError("boundary scale must be finite and positive")
    return DenseBoundary(
        boundary.vertices * value,
        boundary.corners * value,
        boundary.corner_indices,
    )


def _motion(mode: MotionMode, index: int, seed: int) -> MotionProfile:
    rng = np.random.default_rng(10_000 + 97 * seed + index)
    phase = float(0.55 * index + rng.uniform(-0.18, 0.18))
    if mode == "static":
        return MotionProfile.static()
    translation = np.array([0.10, 0.18, 0.13]) * (1.0 + 0.05 * index)
    if mode == "translation":
        return MotionProfile(
            translation,
            np.zeros(3),
            0.0,
            translation_period=6.2 + 0.35 * index,
            phase=phase,
            rotation_enabled=False,
            scale_enabled=False,
        )
    if mode == "full":
        return MotionProfile(
            translation,
            np.array([0.13, 0.10, 0.08]),
            0.10,
            translation_period=6.2 + 0.35 * index,
            rotation_period=7.1 + 0.40 * index,
            scale_period=8.0 + 0.25 * index,
            phase=phase,
        )
    raise ValueError(f"unknown motion mode {mode!r}")


def build_boundary_scenario(
    definitions: Sequence[tuple[str, DenseBoundary]],
    *,
    mode: MotionMode,
    seed: int = 0,
    spacing: float = 2.0,
    preprocessing_config: WBPreprocessingConfig | None = None,
    preprocessed_gates: Sequence[WBPreprocessedGate] | None = None,
    name: str = "wb_custom",
) -> WBScenario:
    items = tuple(definitions)
    if not items:
        raise ValueError("a WBSC scenario needs at least one window")
    settings = WBPreprocessingConfig() if preprocessing_config is None else preprocessing_config
    if preprocessed_gates is None:
        gates = tuple(
            preprocess_boundary(boundary, name=label, config=settings)
            for label, boundary in items
        )
    else:
        gates = tuple(preprocessed_gates)
        if len(gates) != len(items):
            raise ValueError("preprocessed_gates must match the boundary definitions")
    rng = np.random.default_rng(seed)
    count = len(gates)
    x_positions = spacing * (np.arange(count, dtype=float) - 0.5 * (count - 1))
    windows = []
    for index, (x_position, gate) in enumerate(zip(x_positions, gates)):
        center = np.array(
            [x_position, rng.uniform(-0.05, 0.05), 1.4 + rng.uniform(-0.04, 0.04)]
        )
        angles = np.array(
            [
                rng.uniform(-0.04, 0.04),
                np.pi / 2.0 + rng.uniform(-0.045, 0.045),
                rng.uniform(-0.08, 0.08),
            ]
        )
        windows.append(
            SCDynamicWindow(
                name=gate.name,
                sc_map=gate.sc_map,
                safe_polygon=gate.candidate_polygon,
                center0=center,
                angles0=angles,
                motion=_motion(mode, index, seed),
                physical_boundary=gate.dense_boundary.vertices,
            )
        )
    extent = max(3.0, float(np.max(np.abs(x_positions))) + 2.2)
    track = SCWindowTrack(
        name=f"{name}_{mode}_seed{seed}",
        start=np.array([-extent, 0.0, 1.4]),
        goal=np.array([extent, 0.0, 1.4]),
        windows=tuple(windows),
        order=tuple(range(count)),
    )
    return WBScenario(track.name, mode, track, gates, int(seed))


def build_static_narrow_scenario(
    *,
    seed: int = 0,
    variant: Literal["l", "u_curve"] = "l",
    preprocessing_config: WBPreprocessingConfig | None = None,
    preprocessed_gates: Sequence[WBPreprocessedGate] | None = None,
) -> WBScenario:
    """The two paired static benchmark families from the experiment protocol."""

    if variant == "l":
        definitions = (("narrow_L", scaled_boundary(l_shape_boundary(), 0.21)),)
    elif variant == "u_curve":
        definitions = (
            ("narrow_U", scaled_boundary(u_shape_boundary(), 0.23)),
            ("narrow_curve", scaled_boundary(line_bezier_mixed_boundary(), 0.22)),
        )
    else:
        raise ValueError("variant must be 'l' or 'u_curve'")
    return build_boundary_scenario(
        definitions,
        mode="static",
        seed=seed,
        preprocessing_config=preprocessing_config,
        preprocessed_gates=preprocessed_gates,
        name=f"static_{variant}",
    )


def build_dynamic_lus_scenario(
    *,
    seed: int = 0,
    preprocessing_config: WBPreprocessingConfig | None = None,
    preprocessed_gates: Sequence[WBPreprocessedGate] | None = None,
) -> WBScenario:
    definitions = (
        ("L", scaled_boundary(l_shape_boundary(), 0.60)),
        ("U", scaled_boundary(u_shape_boundary(), 0.60)),
        ("star", scaled_boundary(five_point_star_boundary(), 0.60)),
    )
    return build_boundary_scenario(
        definitions,
        mode="full",
        seed=seed,
        preprocessing_config=preprocessing_config,
        preprocessed_gates=preprocessed_gates,
        name="dynamic_L_U_star",
    )


def candidate_boundary_catalog() -> dict[str, DenseBoundary]:
    return e1_boundaries()


__all__ = [
    "MotionMode",
    "WBScenario",
    "build_boundary_scenario",
    "build_dynamic_lus_scenario",
    "build_static_narrow_scenario",
    "candidate_boundary_catalog",
    "scaled_boundary",
]
