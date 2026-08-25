"""Deterministic scenarios for the SC/SIP motion-rate benchmark.

The only rejection conditions are geometry/preprocessing invariants.  No
solver output participates in selecting a seed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from functools import lru_cache

import numpy as np

from nonconvex_timevarying_window.comparisons.sc_sip_fast_closed_loop.scenario import (
    FastClosedLoopScenario,
    build_fast_closed_loop_scenario,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile


BASE_SEED = 20_260_824
MOTION_LEVELS: dict[str, float] = {"slow": 0.5, "nominal": 1.0, "fast": 1.5}
# One circular and one four-piece Bézier aperture retain genuine continuous
# curves while keeping the complete interval proof practical for 36 runs.
WINDOW_SELECTION = (1, 5)


@dataclass(frozen=True)
class BenchmarkScenario:
    seed: int
    level: str
    rate_multiplier: float
    value: FastClosedLoopScenario


def _normal_angles(previous: np.ndarray, center: np.ndarray) -> np.ndarray:
    direction = np.asarray(center, dtype=float) - np.asarray(previous, dtype=float)
    normal = direction / np.linalg.norm(direction)
    return np.asarray((
        0.0,
        math.acos(float(np.clip(normal[2], -1.0, 1.0))),
        math.atan2(float(normal[1]), float(normal[0])),
    ))


def _motion(base: MotionProfile, rng: np.random.Generator, rate: float) -> MotionProfile:
    amplitude_factor = float(rng.uniform(0.85, 1.15))
    # SC's precomputed fixed-world-clearance inset is valid down to the base
    # minimum scale.  Never increase scale amplitude without recomputing it.
    scale = float(np.clip(base.scale_amplitude * rng.uniform(0.80, 1.0), 0.35, base.scale_amplitude))
    return MotionProfile(
        np.asarray(base.translation_amplitude, dtype=float) * amplitude_factor,
        np.asarray(base.rotation_amplitude, dtype=float) * amplitude_factor,
        scale,
        translation_period=float(base.translation_period / rate),
        rotation_period=float(base.rotation_period / rate),
        scale_period=float(base.scale_period / rate),
        phase=float(rng.uniform(0.0, 2.0 * math.pi)),
        translation_enabled=base.translation_enabled,
        rotation_enabled=base.rotation_enabled,
        scale_enabled=base.scale_enabled,
    )


@lru_cache(maxsize=1)
def _base() -> FastClosedLoopScenario:
    """Preprocess the common exact primitives once per benchmark process."""
    return build_fast_closed_loop_scenario()


def build_benchmark_scenario(seed_index: int, level: str) -> BenchmarkScenario:
    """Create one frozen seed/level pair without consulting either solver."""
    if level not in MOTION_LEVELS:
        raise ValueError(f"unknown motion level {level!r}")
    if seed_index < 0:
        raise ValueError("seed_index must be nonnegative")
    base = _base()
    rng = np.random.default_rng(BASE_SEED + int(seed_index))
    rate = MOTION_LEVELS[level]
    selected_windows = tuple(base.track.windows[index] for index in WINDOW_SELECTION)
    base_centers = np.asarray([window.center0 for window in selected_windows], dtype=float)
    centers = base_centers + rng.uniform((-1.5, -1.5, -0.75), (1.5, 1.5, 0.75), size=base_centers.shape)
    order = tuple(int(item) for item in rng.permutation(len(selected_windows)))
    motions = tuple(_motion(window.motion, rng, rate) for window in selected_windows)
    angles: list[np.ndarray | None] = [None] * len(order)
    previous = np.asarray(base.track.start, dtype=float)
    for index in order:
        angles[index] = _normal_angles(previous, centers[index])
        previous = centers[index]
    if min(motion.minimum_scale for motion in motions) < 0.40:
        raise RuntimeError("benchmark generator violated minimum-scale invariant")
    windows = tuple(
        replace(window, center0=centers[index], angles0=np.asarray(angles[index]), motion=motions[index])
        for index, window in enumerate(selected_windows)
    )
    track = replace(base.track, name=f"motion_rate_seed{seed_index:02d}_{level}", windows=windows, order=order)
    value = replace(
        base, name=track.name, track=track,
        preprocessed_gates=tuple(base.preprocessed_gates[index] for index in WINDOW_SELECTION),
        sip_boundaries=tuple(base.sip_boundaries[index] for index in WINDOW_SELECTION),
    )
    return BenchmarkScenario(seed_index, level, rate, value)


__all__ = ["BASE_SEED", "BenchmarkScenario", "MOTION_LEVELS", "build_benchmark_scenario"]
