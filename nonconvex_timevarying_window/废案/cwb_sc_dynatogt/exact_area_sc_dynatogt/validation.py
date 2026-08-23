"""Independent exact-area whole-body validation helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from .geometry import Cuboid, GateFrame, IntersectionMetrics, exact_intersection_metrics, plane_section


@dataclass(frozen=True)
class ValidationSample:
    time: float
    metrics: IntersectionMetrics


@dataclass(frozen=True)
class ValidationReport:
    samples: tuple[ValidationSample, ...]
    whole_body_collision: bool
    degenerate_contact: bool
    max_outside_area: float
    max_outside_ratio: float
    worst_time: float


def validate_trajectory(
    *,
    times: Sequence[float],
    cuboid: Cuboid,
    body_pose: Callable[[float], tuple[np.ndarray, np.ndarray]],
    gate_frame: Callable[[float], GateFrame],
    gate_boundary_local: np.ndarray,
    area_epsilon: float = 1.0e-10,
) -> ValidationReport:
    """Use one common geometric judge for any candidate trajectory."""

    grid = np.asarray(times, dtype=float)
    if grid.ndim != 1 or len(grid) == 0 or np.any(np.diff(grid) < 0.0):
        raise ValueError("times must be a nonempty nondecreasing sequence")
    samples: list[ValidationSample] = []
    for instant in grid:
        center, rotation = body_pose(float(instant))
        frame = gate_frame(float(instant))
        section = plane_section(cuboid, center, rotation, frame)
        metrics = exact_intersection_metrics(
            section, frame.boundary_world_2d(gate_boundary_local), area_epsilon=area_epsilon
        )
        samples.append(ValidationSample(float(instant), metrics))
    outside = np.asarray([sample.metrics.outside_area for sample in samples])
    section_areas = np.asarray([sample.metrics.section_area for sample in samples])
    ratios = np.divide(outside, section_areas, out=np.zeros_like(outside), where=section_areas > area_epsilon)
    worst = int(np.argmax(outside))
    return ValidationReport(
        tuple(samples),
        any(sample.metrics.whole_body_collision for sample in samples),
        any(sample.metrics.degenerate_contact for sample in samples),
        float(outside[worst]),
        float(np.max(ratios)),
        float(grid[worst]),
    )


__all__ = ["ValidationReport", "ValidationSample", "validate_trajectory"]
