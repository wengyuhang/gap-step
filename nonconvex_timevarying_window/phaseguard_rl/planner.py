"""Map one policy action to crossing points, durations, and a fixed MINCO plan."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sip_dynatogt.model import PolynomialTrajectory, SIPProblem


class InvalidTraversalPoint(ValueError):
    pass


@dataclass(frozen=True)
class PlanBounds:
    local_half_width: float = 1.5
    minimum_duration: float = 0.5
    maximum_duration: float = 6.0
    boundary_samples_per_segment: int = 256

    def __post_init__(self) -> None:
        if self.local_half_width <= 0:
            raise ValueError("local_half_width must be positive")
        if not 0 < self.minimum_duration < self.maximum_duration:
            raise ValueError("duration bounds must satisfy 0 < min < max")
        if self.boundary_samples_per_segment < 16:
            raise ValueError("boundary_samples_per_segment must be at least 16")


@dataclass(frozen=True)
class PlanProposal:
    trajectory: PolynomialTrajectory
    local_points: np.ndarray
    world_points: np.ndarray
    durations: np.ndarray
    traversal_times: np.ndarray
    start_time: float
    next_gate: int

    @property
    def total_time(self) -> float:
        return float(np.sum(self.durations))


def action_dimension(gate_count: int) -> int:
    if gate_count < 1:
        raise ValueError("gate_count must be positive")
    return 3 * int(gate_count) + 1


def _point_in_polygon(point: np.ndarray, vertices: np.ndarray) -> bool:
    x, y = point
    inside = False
    j = len(vertices) - 1
    for i in range(len(vertices)):
        xi, yi = vertices[i]
        xj, yj = vertices[j]
        crosses = (yi > y) != (yj > y)
        if crosses and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _boundary_polygon(window, samples_per_segment: int) -> np.ndarray:
    points = []
    nodes = np.linspace(0.0, 1.0, samples_per_segment, endpoint=False)
    for segment in window.boundary:
        points.extend(np.asarray(segment.evaluate(float(u)), dtype=float) for u in nodes)
    return np.asarray(points)


def _decode(raw_action: np.ndarray, gate_count: int, bounds: PlanBounds) -> tuple[np.ndarray, np.ndarray]:
    action = np.asarray(raw_action, dtype=float)
    expected = action_dimension(gate_count)
    if action.shape != (expected,) or not np.all(np.isfinite(action)):
        raise ValueError(f"action must be a finite vector of shape ({expected},)")
    action = np.clip(action, -1.0, 1.0)
    local_points = action[: 2 * gate_count].reshape(gate_count, 2) * bounds.local_half_width
    unit_times = 0.5 * (action[2 * gate_count :] + 1.0)
    durations = bounds.minimum_duration + unit_times * (bounds.maximum_duration - bounds.minimum_duration)
    return local_points, durations


def build_plan(
    problem: SIPProblem,
    start_state: BoundaryState,
    finish_state: BoundaryState,
    raw_action: np.ndarray,
    bounds: PlanBounds | None = None,
    *,
    start_time: float = 0.0,
    next_gate: int = 0,
) -> PlanProposal:
    """Build the full nominal plan.

    Curve sampling here is only an early traversal-membership rejection.  It
    never grants safety; the shield still has to certify the original curves.
    """

    settings = bounds or PlanBounds()
    gate_count = len(problem.order)
    if not 0 <= next_gate < gate_count:
        raise ValueError("next_gate must identify a remaining gate")
    all_local_points, all_durations = _decode(raw_action, gate_count, settings)
    local_points = all_local_points[next_gate:]
    durations = all_durations[next_gate:]
    traversal_times = start_time + np.cumsum(durations[:-1])
    world_points = []
    for local_position, window_index in enumerate(problem.order[next_gate:]):
        window = problem.windows[window_index]
        local = local_points[local_position]
        polygon = _boundary_polygon(window, settings.boundary_samples_per_segment)
        if not _point_in_polygon(local, polygon):
            raise InvalidTraversalPoint(
                f"policy point for order position {next_gate + local_position} is outside the aperture"
            )
        center, rotation, scale = window.state_at(float(traversal_times[local_position]))
        world_points.append(center + rotation @ np.array([scale * local[0], scale * local[1], 0.0]))
    minco = MincoSnap(start_state, finish_state, np.asarray(world_points), durations)
    trajectory = PolynomialTrajectory(
        np.asarray(minco.durations, dtype=float),
        np.asarray(minco.coefficients, dtype=float),
    )
    return PlanProposal(
        trajectory=trajectory,
        local_points=local_points,
        world_points=np.asarray(world_points),
        durations=np.asarray(durations),
        traversal_times=np.asarray(traversal_times),
        start_time=float(start_time),
        next_gate=int(next_gate),
    )
