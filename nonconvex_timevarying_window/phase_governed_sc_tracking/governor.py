"""A minimal execution layer that changes timing without changing the path.

The governor is deliberately limited to the simplified RotSync model: one
fixed-centre, fixed-plane window rotating at a known constant angular rate.
Before leaving a safe start state it previews whole-body traversal for a grid
of start delays.  It selects the earliest delay that preserves the SC path,
crosses through the eroded aperture, and is collision-free on the sampled
preview.  If no delay within the configured bound is admissible, it reports
failure instead of silently executing the unsafe reference.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.rot_sync_sc_togt.collision import (
    _slab_cross_section,
    body_rotations,
    cuboid_window_collision,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import TrajectorySamples


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class DelaySearchConfig:
    max_delay: float
    delay_step: float = 5.0e-4
    audit_step: float = 1.0e-3
    # The preview uses 20 mm so the independent critical-time refinement can
    # still retain the experiment's 15 mm reporting margin.
    minimum_sampled_clearance: float = 20.0e-3

    def __post_init__(self) -> None:
        values = (
            self.max_delay,
            self.delay_step,
            self.audit_step,
            self.minimum_sampled_clearance,
        )
        if not all(np.isfinite(values)):
            raise ValueError("delay-search settings must be finite")
        if self.max_delay < 0.0 or self.minimum_sampled_clearance < 0.0:
            raise ValueError("delay and clearance bounds must be nonnegative")
        if self.delay_step <= 0.0 or self.audit_step <= 0.0:
            raise ValueError("sampling steps must be positive")


@dataclass(frozen=True)
class DelayCandidate:
    delay: float
    collision_free: bool
    crossing_valid: bool
    local_point_in_safe_region: bool
    colliding_samples: int
    minimum_sampled_clearance: float | None
    crossing_local_point: FloatArray

    @property
    def admissible(self) -> bool:
        return bool(
            self.collision_free
            and self.crossing_valid
            and self.local_point_in_safe_region
        )


@dataclass(frozen=True)
class DelaySearchResult:
    selected: DelayCandidate | None
    evaluated_candidates: int
    candidates: tuple[DelayCandidate, ...]


class WaitThenTrackTrajectory:
    """Hold the nominal zero-PVAJ start, then track the path without retiming."""

    def __init__(self, nominal, delay: float) -> None:
        self.nominal = nominal
        self.delay = float(delay)
        if not np.isfinite(self.delay) or self.delay < 0.0:
            raise ValueError("delay must be finite and nonnegative")
        self._start = np.asarray(nominal.evaluate(0.0, 0), dtype=float)
        for order in range(1, 4):
            if np.linalg.norm(nominal.evaluate(0.0, order)) > 1.0e-9:
                raise ValueError("wait-then-track requires a zero-PVAJ nominal start")

    @property
    def total_time(self) -> float:
        return self.delay + float(self.nominal.total_time)

    @property
    def durations(self) -> FloatArray:
        nominal = np.asarray(self.nominal.durations, dtype=float)
        if self.delay == 0.0:
            return nominal.copy()
        return np.r_[self.delay, nominal]

    def evaluate(self, time: ArrayLike, derivative: int = 0):
        if derivative < 0:
            raise ValueError("derivative order must be nonnegative")
        query = np.asarray(time, dtype=float)
        if np.any(query < -1.0e-12) or np.any(query > self.total_time + 1.0e-12):
            raise ValueError("query lies outside the delayed trajectory")
        scalar = query.ndim == 0
        flat = query.reshape(-1)
        output = np.zeros((len(flat), 3), dtype=float)
        waiting = flat < self.delay
        if derivative == 0:
            output[waiting] = self._start
        active = ~waiting
        if np.any(active):
            shifted = np.clip(flat[active] - self.delay, 0.0, self.nominal.total_time)
            output[active] = self.nominal.evaluate(shifted, derivative)
        reshaped = output.reshape(query.shape + (3,))
        return reshaped[()] if scalar else reshaped

    __call__ = evaluate

    def sample(
        self,
        *,
        times: ArrayLike | None = None,
        num_samples: int | None = None,
        samples_per_segment: int | None = None,
    ) -> TrajectorySamples:
        if sum(value is not None for value in (times, num_samples, samples_per_segment)) > 1:
            raise ValueError("specify only one sampling mode")
        if times is not None:
            grid = np.asarray(times, dtype=float)
        elif samples_per_segment is not None:
            if samples_per_segment < 2:
                raise ValueError("samples_per_segment must be at least two")
            chunks = []
            elapsed = 0.0
            for index, duration in enumerate(self.durations):
                local = np.linspace(0.0, duration, samples_per_segment)
                if index:
                    local = local[1:]
                chunks.append(elapsed + local)
                elapsed += duration
            grid = np.concatenate(chunks)
        else:
            count = 101 if num_samples is None else int(num_samples)
            if count < 2:
                raise ValueError("num_samples must be at least two")
            grid = np.linspace(0.0, self.total_time, count)
        if grid.ndim != 1:
            raise ValueError("times must be one-dimensional")
        return TrajectorySamples(
            time=grid,
            position=self.evaluate(grid, 0),
            velocity=self.evaluate(grid, 1),
            acceleration=self.evaluate(grid, 2),
            jerk=self.evaluate(grid, 3),
            snap=self.evaluate(grid, 4),
            crackle=self.evaluate(grid, 5),
        )

    def interface_residuals(self) -> FloatArray:
        residuals = [
            np.linalg.norm(self.nominal.evaluate(0.0, 0) - self._start),
            *(
                np.linalg.norm(self.nominal.evaluate(0.0, order))
                for order in range(1, 4)
            ),
        ]
        if hasattr(self.nominal, "interface_residuals"):
            residuals.extend(np.ravel(self.nominal.interface_residuals()))
        return np.asarray(residuals, dtype=float)

    def snap_energy(self):
        return self.nominal.snap_energy()


def _preview_grid(trajectory, crossing_time: float, step: float) -> FloatArray:
    count = int(np.ceil(float(trajectory.total_time) / step))
    grid = np.linspace(0.0, float(trajectory.total_time), count + 1)
    return np.unique(np.r_[grid, float(crossing_time)])


def _delay_grid(max_delay: float, step: float) -> FloatArray:
    if max_delay == 0.0:
        return np.zeros(1)
    count = int(np.floor(max_delay / step))
    grid = np.arange(count + 1, dtype=float) * step
    if grid[-1] < max_delay - 1.0e-12:
        grid = np.r_[grid, max_delay]
    return grid


def find_safe_delay(
    scenario,
    nominal_trajectory,
    nominal_crossing_time: float,
    *,
    config: DelaySearchConfig,
) -> DelaySearchResult:
    """Select the earliest sampled safe delay for the unchanged nominal path."""

    if len(scenario.windows) != 1:
        raise ValueError("the minimal phase governor supports exactly one window")
    window = scenario.windows[0]
    grid = _preview_grid(nominal_trajectory, nominal_crossing_time, config.audit_step)
    positions = np.asarray(nominal_trajectory.evaluate(grid), dtype=float)
    rotations = body_rotations(nominal_trajectory, grid)
    signed = (positions - window.center) @ window.normal
    half = np.asarray(scenario.body.half_extents, dtype=float)
    projection = np.sum(
        np.abs(np.einsum("nij,j->ni", np.transpose(rotations, (0, 2, 1)), window.normal))
        * half,
        axis=1,
    )
    overlap = np.abs(signed) <= 0.5 * window.thickness + projection
    overlap_indices = np.flatnonzero(overlap)
    crossing_position = np.asarray(
        nominal_trajectory.evaluate(float(nominal_crossing_time)), dtype=float
    )
    crossing_rotation = body_rotations(
        nominal_trajectory, np.asarray((nominal_crossing_time,))
    )[0]
    safe_polygon = Polygon(window.safe_polygon).buffer(1.0e-9)
    physical_polygon = Polygon(window.physical_polygon)

    rows: list[DelayCandidate] = []
    selected = None
    for delay in _delay_grid(config.max_delay, config.delay_step):
        clearances = []
        collisions = 0
        for index in overlap_indices:
            hit, clearance = cuboid_window_collision(
                window,
                float(grid[index] + delay),
                positions[index],
                rotations[index],
                scenario.body,
            )
            collisions += int(hit)
            if np.isfinite(clearance):
                clearances.append(float(clearance))
        crossing_absolute_time = float(nominal_crossing_time + delay)
        basis = window.rotated_basis(crossing_absolute_time)
        crossing_local = basis.T @ (crossing_position - window.center)
        local_safe = bool(safe_polygon.covers(Point(crossing_local)))
        section = _slab_cross_section(
            window,
            crossing_absolute_time,
            crossing_position,
            crossing_rotation,
            scenario.body,
            tolerance=1.0e-9,
        )
        crossing_valid = bool(
            section is not None
            and not section.is_empty
            and physical_polygon.covers(section)
            and float(section.distance(physical_polygon.boundary))
            >= config.minimum_sampled_clearance
        )
        minimum = min(clearances) if clearances else None
        collision_free = bool(
            collisions == 0
            and minimum is not None
            and minimum >= config.minimum_sampled_clearance
        )
        candidate = DelayCandidate(
            delay=float(delay),
            collision_free=collision_free,
            crossing_valid=crossing_valid,
            local_point_in_safe_region=local_safe,
            colliding_samples=collisions,
            minimum_sampled_clearance=minimum,
            crossing_local_point=np.asarray(crossing_local, dtype=float),
        )
        rows.append(candidate)
        if candidate.admissible:
            selected = candidate
            break
    return DelaySearchResult(selected, len(rows), tuple(rows))


__all__ = [
    "DelayCandidate",
    "DelaySearchConfig",
    "DelaySearchResult",
    "WaitThenTrackTrajectory",
    "find_safe_delay",
]
