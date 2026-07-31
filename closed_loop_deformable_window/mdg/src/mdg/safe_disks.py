"""Deterministic safe-disc and baseline point generation."""

from __future__ import annotations

import numpy as np
from shapely.geometry import Point

from .config import MDGConfig
from .dynamic_gate import DynamicGate
from .geometry import circle_covered, interior_grid
from .models import Disc


def generate_safe_disks(
    gate: DynamicGate,
    time: float,
    config: MDGConfig,
    *,
    max_disks: int | None = None,
) -> list[Disc]:
    safe = gate.safe_polygon(time, config.safety.safety_radius)
    if safe.is_empty:
        return []
    coordinates, radii = interior_grid(safe, config.disks.grid_resolution)
    if not len(coordinates):
        return []
    # Radius first, then coordinates, makes ties deterministic across platforms.
    order = np.lexsort((coordinates[:, 1], coordinates[:, 0], -radii))
    selected: list[Disc] = []
    limit = config.disks.max_disks_per_gate if max_disks is None else int(max_disks)
    for index in order:
        center = coordinates[index]
        radius = float(radii[index])
        if radius < config.disks.min_radius:
            break
        if any(
            np.linalg.norm(center - item.center)
            <= config.disks.nms_ratio * min(radius, item.radius)
            for item in selected
        ):
            continue
        while radius >= config.disks.min_radius and not circle_covered(
            safe, center, radius
        ):
            radius *= 0.98
        if radius >= config.disks.min_radius:
            selected.append(Disc(center.copy(), radius))
        if len(selected) >= limit:
            break
    return selected


def centroid_candidate(
    gate: DynamicGate, time: float, config: MDGConfig
) -> list[Disc]:
    safe = gate.safe_polygon(time, config.safety.safety_radius)
    if safe.is_empty:
        return []
    centroid = safe.centroid
    point = centroid if safe.covers(centroid) else safe.representative_point()
    return [Disc(np.array((point.x, point.y)), 1.0e-9)]


def uniform_point_candidates(
    gate: DynamicGate, time: float, config: MDGConfig, count: int = 5
) -> list[Disc]:
    safe = gate.safe_polygon(time, config.safety.safety_radius)
    if safe.is_empty:
        return []
    coordinates, _ = interior_grid(safe, config.disks.grid_resolution)
    if not len(coordinates):
        point = safe.representative_point()
        return [Disc(np.array((point.x, point.y)), 1.0e-9)]
    # Greedy farthest-point sampling from the representative point.
    representative = np.array((safe.representative_point().x, safe.representative_point().y))
    first = int(np.argmin(np.linalg.norm(coordinates - representative, axis=1)))
    chosen = [first]
    minimum_distance = np.linalg.norm(coordinates - coordinates[first], axis=1)
    while len(chosen) < min(count, len(coordinates)):
        index = int(np.argmax(minimum_distance))
        chosen.append(index)
        minimum_distance = np.minimum(
            minimum_distance, np.linalg.norm(coordinates - coordinates[index], axis=1)
        )
    return [Disc(coordinates[index].copy(), 1.0e-9) for index in chosen]


def candidates_for_method(
    gate: DynamicGate,
    time: float,
    config: MDGConfig,
    method: str,
) -> list[Disc]:
    normalized = method.lower().replace("-", "_")
    if normalized == "center":
        return centroid_candidate(gate, time, config)
    if normalized == "uniform_point":
        return uniform_point_candidates(gate, time, config)
    maximum = 8 if normalized == "dense_oracle" else None
    discs = generate_safe_disks(gate, time, config, max_disks=maximum)
    if normalized == "largest_disc":
        return discs[:1]
    return discs


__all__ = [
    "candidates_for_method",
    "centroid_candidate",
    "generate_safe_disks",
    "uniform_point_candidates",
]

