"""Reproducible spatial/temporal starts for MSR-DynaTOGT."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import (
    JointTOGTObjective,
    OptimizationConfig,
)

from .config import InitializationConfig


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class InitialGuess:
    kind: str
    index: int
    seed: int
    x: FloatArray
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def label(self) -> str:
        return f"{self.kind}:{self.index}"


def _point_in_polygon(point: FloatArray, polygon: FloatArray) -> bool:
    p = np.asarray(point, dtype=float)
    a = np.asarray(polygon, dtype=float)
    b = np.roll(a, -1, axis=0)
    edges = b - a
    lengths2 = np.einsum("ij,ij->i", edges, edges)
    projection = np.divide(
        np.einsum("ij,ij->i", p - a, edges),
        lengths2,
        out=np.zeros(len(a)),
        where=lengths2 > 0.0,
    )
    closest = a + np.clip(projection, 0.0, 1.0)[:, None] * edges
    if float(np.min(np.linalg.norm(closest - p, axis=1))) <= 1.0e-10:
        return True
    crosses = (a[:, 1] > p[1]) != (b[:, 1] > p[1])
    x_cross = np.divide(
        edges[:, 0] * (p[1] - a[:, 1]),
        edges[:, 1],
        out=np.zeros(len(a)),
        where=np.abs(edges[:, 1]) > 1.0e-15,
    ) + a[:, 0]
    return bool(np.count_nonzero(crosses & (p[0] < x_cross)) % 2)


def _disk_to_unconstrained(disk: FloatArray) -> FloatArray:
    z = np.asarray(disk, dtype=float)
    radius2 = float(z @ z)
    if radius2 >= 1.0:
        z = z * ((1.0 - 1.0e-10) / math.sqrt(radius2))
        radius2 = float(z @ z)
    return z / math.sqrt(max(1.0 - radius2, np.finfo(float).eps))


def _interior_point_toward(
    center: FloatArray,
    direction: FloatArray,
    polygon: FloatArray,
    fraction: float,
) -> FloatArray:
    norm = float(np.linalg.norm(direction))
    if norm <= 1.0e-12:
        return center.copy()
    unit = direction / norm
    span = max(float(np.linalg.norm(np.ptp(polygon, axis=0))), 1.0)
    distances = np.linspace(0.0, 2.0 * span, 257)
    last_inside = 0.0
    first_outside = distances[-1]
    for distance in distances[1:]:
        if not _point_in_polygon(center + distance * unit, polygon):
            first_outside = float(distance)
            break
        last_inside = float(distance)
    for _ in range(32):
        middle = 0.5 * (last_inside + first_outside)
        if _point_in_polygon(center + middle * unit, polygon):
            last_inside = middle
        else:
            first_outside = middle
    return center + fraction * last_inside * unit


def _turn_aware_d(
    track: SCWindowTrack,
    objective: JointTOGTObjective,
    fraction: float,
) -> FloatArray:
    base = objective.forward(objective.initial_guess())
    centers = []
    for crossing, window_index in enumerate(track.order):
        centers.append(track.windows[window_index].state_at(base.traversal_times[crossing])[0])
    route = np.vstack((track.start, np.asarray(centers), track.goal))
    spatial = np.zeros((objective.window_count, 2), dtype=float)
    for crossing, window_index in enumerate(track.order):
        window = track.windows[window_index]
        instant = float(base.traversal_times[crossing])
        center, basis, *_ = window.state_at(instant)
        to_previous = route[crossing] - center
        to_next = route[crossing + 2] - center
        for vector in (to_previous, to_next):
            length = float(np.linalg.norm(vector))
            if length > 1.0e-12:
                vector /= length
        local_direction = basis.T @ (to_previous + to_next)
        local_center = np.asarray(window.sc_map.evaluate(0.0j), dtype=float)
        target = _interior_point_toward(
            local_center,
            local_direction,
            window.safe_polygon,
            fraction,
        )
        disk = window.sc_map.inverse(target)
        spatial[crossing] = _disk_to_unconstrained(disk)
    return spatial


def generate_initial_guesses(
    track: SCWindowTrack,
    optimization: OptimizationConfig,
    settings: InitializationConfig,
    *,
    seed: int,
) -> list[InitialGuess]:
    """Generate center, random, turn-aware, and region-dispersed starts."""

    objective = JointTOGTObjective(track, optimization)
    guesses = [
        InitialGuess(
            kind="sc_center",
            index=0,
            seed=seed,
            x=objective.initial_guess(),
            metadata={"description": "SC 映射中心点"},
        )
    ]

    for index in range(settings.random_starts):
        child_seed = int(np.random.SeedSequence([seed, 101, index]).generate_state(1)[0])
        rng = np.random.default_rng(child_seed)
        spatial = rng.normal(
            0.0,
            settings.spatial_noise_std,
            size=(objective.window_count, 2),
        )
        x = objective.initial_guess(spatial)
        x[: objective.temporal_dimension] += rng.normal(
            0.0,
            settings.temporal_noise_std,
            size=objective.temporal_dimension,
        )
        guesses.append(
            InitialGuess(
                kind="random_perturbation",
                index=index,
                seed=child_seed,
                x=x,
                metadata={
                    "temporal_noise_std": settings.temporal_noise_std,
                    "spatial_noise_std": settings.spatial_noise_std,
                },
            )
        )

    for index in range(settings.turn_aware_starts):
        fraction = min(0.9, settings.turn_fraction * (0.9 + 0.1 * index))
        spatial = _turn_aware_d(track, objective, fraction)
        guesses.append(
            InitialGuess(
                kind="turn_aware",
                index=index,
                seed=seed,
                x=objective.initial_guess(spatial),
                metadata={"interior_fraction": fraction},
            )
        )

    radius = settings.dispersed_disk_radius
    unconstrained_radius = radius / math.sqrt(1.0 - radius * radius)
    for index in range(settings.dispersed_starts):
        spatial = np.empty((objective.window_count, 2), dtype=float)
        for crossing in range(objective.window_count):
            angle = 2.0 * math.pi * (
                index / max(settings.dispersed_starts, 1)
                + crossing / max(objective.window_count, 1) / 3.0
            )
            spatial[crossing] = unconstrained_radius * np.array(
                [math.cos(angle), math.sin(angle)]
            )
        guesses.append(
            InitialGuess(
                kind="dispersed_region",
                index=index,
                seed=seed,
                x=objective.initial_guess(spatial),
                metadata={"disk_radius": radius},
            )
        )

    return guesses


__all__ = ["InitialGuess", "generate_initial_guesses"]
