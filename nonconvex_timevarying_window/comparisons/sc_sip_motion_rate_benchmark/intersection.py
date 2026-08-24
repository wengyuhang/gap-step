"""Fail-closed continuous physical-intersection checker.

This is deliberately distinct from the positive-clearance certificate.  It
proves a frame point is strictly inside the oriented cuboid, or proves every
covered cell strictly outside it.  Exact contact remains unresolved.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any

import numpy as np

from nonconvex_timevarying_window.sip_dynatogt.constraints import point_flatness
from nonconvex_timevarying_window.sip_dynatogt.intervals import (
    FlatnessIndeterminate, boundary_interval, boundary_parameter_spans, ctx,
    flatness_interval, global_time_interval, interval_ball, iv_add, iv_matvec,
    iv_transpose, require_flint, window_state_interval,
)
from nonconvex_timevarying_window.sip_dynatogt.model import PolynomialTrajectory, SIPConfig, SIPProblem


class IntersectionStatus(str, Enum):
    PHYSICAL_INTERSECTION_CONFIRMED = "PHYSICAL_INTERSECTION_CONFIRMED"
    NO_INTERSECTION_CERTIFIED = "NO_INTERSECTION_CERTIFIED"
    INTERSECTION_UNRESOLVED = "INTERSECTION_UNRESOLVED"
    NUMERICAL_FAILURE = "NUMERICAL_FAILURE"


@dataclass(frozen=True)
class IntersectionWitness:
    trajectory_segment: int
    normalized_time: float
    window_index: int
    boundary_segment: int
    boundary_parameter: float
    global_time: float
    body_local_boundary_point: tuple[float, float, float]
    axis_interior_margins: tuple[float, float, float]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class IntersectionResult:
    status: IntersectionStatus
    reason: str
    precision_bits: int
    checked_cells: int
    maximum_depth: int
    witness: IntersectionWitness | None = None

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["status"] = self.status.value
        return data


@dataclass(frozen=True)
class _Cell:
    window: int
    boundary: int
    trajectory: int
    tlo: float
    thi: float
    ulo: float
    uhi: float
    depth: int = 0


def _split(cell: _Cell) -> tuple[_Cell, _Cell]:
    if cell.thi - cell.tlo >= cell.uhi - cell.ulo:
        middle = (cell.tlo + cell.thi) / 2.0
        return (_Cell(cell.window, cell.boundary, cell.trajectory, cell.tlo, middle, cell.ulo, cell.uhi, cell.depth + 1), _Cell(cell.window, cell.boundary, cell.trajectory, middle, cell.thi, cell.ulo, cell.uhi, cell.depth + 1))
    middle = (cell.ulo + cell.uhi) / 2.0
    return (_Cell(cell.window, cell.boundary, cell.trajectory, cell.tlo, cell.thi, cell.ulo, middle, cell.depth + 1), _Cell(cell.window, cell.boundary, cell.trajectory, cell.tlo, cell.thi, middle, cell.uhi, cell.depth + 1))


def _witness(problem: SIPProblem, trajectory: PolynomialTrajectory, config: SIPConfig, cell: _Cell) -> IntersectionWitness | None:
    tau = (cell.tlo + cell.thi) / 2.0
    u = (cell.ulo + cell.uhi) / 2.0
    flat = point_flatness(trajectory, cell.trajectory, tau, config)
    window = problem.windows[cell.window]
    center, rotation, scale = window.state_at(sum(trajectory.durations[:cell.trajectory]) + trajectory.durations[cell.trajectory] * tau)
    q = np.asarray(window.boundary[cell.boundary].evaluate(u), dtype=float)
    point = center + rotation @ np.asarray((scale * q[0], scale * q[1], 0.0))
    z = np.asarray(flat.rotation, dtype=float).T @ (point - flat.position)
    margins = np.asarray(config.body.half_extents) - np.abs(z)
    if np.all(margins > 0.0):
        return IntersectionWitness(cell.trajectory, tau, cell.window, cell.boundary, u, float(sum(trajectory.durations[:cell.trajectory]) + trajectory.durations[cell.trajectory] * tau), tuple(float(x) for x in z), tuple(float(x) for x in margins))
    return None


def certify_physical_intersection(problem: SIPProblem, value: Any, config: SIPConfig) -> IntersectionResult:
    """Classify strict frame/cuboid intersection on original continuous curves."""
    try:
        require_flint()
        trajectory = value if isinstance(value, PolynomialTrajectory) else PolynomialTrajectory.from_minco(value)
        if trajectory.num_segments != len(problem.windows) + 1:
            raise ValueError("trajectory/window count mismatch")
    except Exception as error:
        return IntersectionResult(IntersectionStatus.NUMERICAL_FAILURE, str(error), 0, 0, 0)
    last: IntersectionResult | None = None
    for bits in config.precision_bits:
        old_precision = int(ctx.prec)
        ctx.prec = int(bits)
        checked = depth = 0
        stack = [
            _Cell(wi, bi, si, 0.0, 1.0, lo, hi)
            for si in reversed(range(trajectory.num_segments))
            for wi in reversed(range(len(problem.windows)))
            for bi, boundary in reversed(list(enumerate(problem.windows[wi].boundary)))
            for lo, hi in reversed(boundary_parameter_spans(boundary))
        ]
        try:
            while stack:
                if checked >= config.max_cells:
                    last = IntersectionResult(IntersectionStatus.INTERSECTION_UNRESOLVED, "intersection cell budget exhausted", int(bits), checked, depth)
                    break
                cell = stack.pop()
                checked += 1
                depth = max(depth, cell.depth)
                tau = interval_ball(cell.tlo, cell.thi)
                u = interval_ball(cell.ulo, cell.uhi)
                window = problem.windows[cell.window]
                try:
                    flat = flatness_interval(trajectory, cell.trajectory, tau, config)
                    center, rotation, scale = window_state_interval(window, global_time_interval(trajectory, cell.trajectory, tau))
                    q = boundary_interval(window.boundary[cell.boundary], u)
                    y = iv_add(center, iv_matvec(rotation, [scale * q[0], scale * q[1], interval_ball(0.0, 0.0)]))
                    z = iv_matvec(iv_transpose(flat.rotation), [y[i] - flat.position[i] for i in range(3)])
                    signed = [abs(z[i]) - interval_ball(float(config.body.half_extents[i]), float(config.body.half_extents[i])) for i in range(3)]
                except FlatnessIndeterminate:
                    signed = None
                if signed is not None and all(item < 0 for item in signed):
                    witness = _witness(problem, trajectory, config, cell)
                    if witness is not None:
                        return IntersectionResult(IntersectionStatus.PHYSICAL_INTERSECTION_CONFIRMED, "an original boundary cell is strictly inside the oriented cuboid", int(bits), checked, depth, witness)
                if signed is not None and any(item > 0 for item in signed):
                    continue
                witness = _witness(problem, trajectory, config, cell)
                if witness is not None:
                    return IntersectionResult(IntersectionStatus.PHYSICAL_INTERSECTION_CONFIRMED, "direct original-primitive witness is strictly inside the oriented cuboid", int(bits), checked, depth, witness)
                if cell.depth >= config.max_depth or ((cell.thi - cell.tlo) <= config.min_time_width and (cell.uhi - cell.ulo) <= config.min_boundary_width):
                    last = IntersectionResult(IntersectionStatus.INTERSECTION_UNRESOLVED, "intersection sign remains undecidable at subdivision limit", int(bits), checked, depth)
                    break
                stack.extend(reversed(_split(cell)))
            else:
                return IntersectionResult(IntersectionStatus.NO_INTERSECTION_CERTIFIED, "every original boundary cell is strictly exterior to the oriented cuboid", int(bits), checked, depth)
        except Exception as error:
            last = IntersectionResult(IntersectionStatus.NUMERICAL_FAILURE, f"intersection interval evaluation failed: {type(error).__name__}: {error}", int(bits), checked, depth)
        finally:
            ctx.prec = old_precision
        if last is not None and last.status is not IntersectionStatus.INTERSECTION_UNRESOLVED:
            return last
    assert last is not None
    return last


__all__ = ["IntersectionResult", "IntersectionStatus", "IntersectionWitness", "certify_physical_intersection"]

