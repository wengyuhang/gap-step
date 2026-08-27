"""Plane-pruned rigorous certificate for fixed-plane windows."""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, replace
import math
from typing import Any

import numpy as np

from nonconvex_timevarying_window.sip_dynatogt.certificate import certify as sip_certify
from nonconvex_timevarying_window.sip_dynatogt.constraints import safety_residual_value
from nonconvex_timevarying_window.sip_dynatogt.intervals import (
    FlatnessIndeterminate,
    boundary_parameter_spans,
    ctx,
    exact_ball,
    flatness_interval,
    interval_ball,
    iv_dot,
    safety_residual_interval,
)
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateResult,
    CertificateStatus,
    PolynomialTrajectory,
    SIPConfig,
    SIPProblem,
    Witness,
)

from .model import PlanarRSConfig, PlanarRSSIPWindow


@dataclass(frozen=True)
class _PlaneCell:
    segment: int
    window: int
    lo: float
    hi: float
    depth: int = 0


def _merge(spans: list[tuple[float, float]]) -> tuple[tuple[float, float], ...]:
    if not spans:
        return ()
    ordered = sorted(spans)
    output = [ordered[0]]
    for lo, hi in ordered[1:]:
        if lo <= output[-1][1] + 1e-15:
            output[-1] = (output[-1][0], max(output[-1][1], hi))
        else:
            output.append((lo, hi))
    return tuple(output)


def _provider(settings: PlanarRSConfig, statistics: dict[str, float] | None = None):
    def spans(
        problem: SIPProblem, traj: PolynomialTrajectory, config: SIPConfig, budget: int
    ):
        pending = [
            _PlaneCell(segment, window, 0.0, 1.0)
            for segment in reversed(range(traj.num_segments))
            for window in reversed(range(len(problem.windows)))
        ]
        retained: dict[tuple[int, int], list[tuple[float, float]]] = defaultdict(list)
        checked = maximum_depth = 0
        while pending:
            if checked >= budget:
                return (
                    {},
                    checked,
                    maximum_depth,
                    "interval-cell budget exhausted during fixed-plane pruning",
                )
            cell = pending.pop()
            checked += 1
            maximum_depth = max(maximum_depth, cell.depth)
            window = problem.windows[cell.window]
            assert isinstance(window, PlanarRSSIPWindow)
            separated = False
            try:
                flat = flatness_interval(
                    traj, cell.segment, interval_ball(cell.lo, cell.hi), config
                )
                normal = [exact_ball(float(v)) for v in window.fixed_normal]
                center = [exact_ball(float(v)) for v in window.center0]
                signed = iv_dot(
                    normal, [flat.position[i] - center[i] for i in range(3)]
                )
                support = exact_ball(0.0)
                for axis, extent in enumerate(config.body.half_extents):
                    support += exact_ball(float(extent)) * abs(
                        iv_dot(normal, [flat.rotation[row][axis] for row in range(3)])
                    )
                gap = abs(signed) - support - exact_ball(config.clearance)
                separated = gap > 0
            except FlatnessIndeterminate:
                separated = False
            if separated:
                if statistics is not None:
                    statistics["minimum_plane_gap"] = min(
                        statistics.get("minimum_plane_gap", float("inf")),
                        max(0.0, float(gap.lower())),
                    )
                continue
            width = cell.hi - cell.lo
            if (
                cell.depth < settings.plane_prune_max_depth
                and width > settings.plane_prune_min_time_width
            ):
                mid = (cell.lo + cell.hi) / 2.0
                pending.append(
                    _PlaneCell(cell.segment, cell.window, mid, cell.hi, cell.depth + 1)
                )
                pending.append(
                    _PlaneCell(cell.segment, cell.window, cell.lo, mid, cell.depth + 1)
                )
            else:
                retained[(cell.segment, cell.window)].append((cell.lo, cell.hi))
        return (
            {key: _merge(value) for key, value in retained.items()},
            checked,
            maximum_depth,
            None,
        )

    return spans


def _validate(problem: SIPProblem) -> str | None:
    if not problem.windows or any(
        not isinstance(window, PlanarRSSIPWindow) for window in problem.windows
    ):
        return "planar_rs_dynatogt requires windows created by make_planar_problem()"
    return None


def _finite_counterexamples(
    problem: SIPProblem,
    trajectory: PolynomialTrajectory,
    config: SIPConfig,
    spans: dict[tuple[int, int], tuple[tuple[float, float], ...]],
) -> tuple[Witness, ...]:
    """Find obvious violations cheaply; absence is never treated as proof."""

    found: list[Witness] = []
    ordered = sorted(
        spans.items(),
        key=lambda item: (
            0 if item[0][0] in (item[0][1], item[0][1] + 1) else 1,
            item[0],
        ),
    )
    for (segment, window_index), time_spans in ordered:
        window = problem.windows[window_index]
        for tlo, thi in time_spans:
            # Plane pruning already makes these spans short.  Include both
            # endpoints so dyadic contacts are not hidden between samples.
            for tau in np.linspace(tlo, thi, 9):
                for boundary_index, boundary in enumerate(window.boundary):
                    for ulo, uhi in boundary_parameter_spans(boundary):
                        for u in np.linspace(ulo, uhi, 17):
                            provisional = Witness(
                                "safety",
                                segment,
                                float(tau),
                                0.0,
                                window_index,
                                boundary_index,
                                float(u),
                                "plane-coarse",
                            )
                            try:
                                residual = safety_residual_value(
                                    problem, trajectory, provisional, config
                                )
                            except Exception:
                                continue
                            if residual > config.violation_tolerance:
                                # A floating-point sample only proposes the
                                # point.  VIOLATED is returned only after a
                                # zero-width Arb evaluation proves its sign.
                                rigorous = safety_residual_interval(
                                    window,
                                    boundary,
                                    trajectory,
                                    segment,
                                    exact_ball(float(tau)),
                                    exact_ball(float(u)),
                                    config,
                                )
                                if not rigorous > 0:
                                    continue
                                found.append(
                                    Witness(
                                        "safety",
                                        segment,
                                        float(tau),
                                        float(residual),
                                        window_index,
                                        boundary_index,
                                        float(u),
                                        "arb-point",
                                    )
                                )
                                # One strict point is already enough to reject
                                # this candidate and seed the next SIP round.
                                return tuple(found)
    found.sort(key=lambda item: item.residual, reverse=True)
    output: list[Witness] = []
    keys = set()
    for witness in found:
        if witness.key() not in keys:
            output.append(witness)
            keys.add(witness.key())
        if len(output) >= config.max_witnesses_per_iteration:
            break
    return tuple(output)


def certify(
    problem: SIPProblem, trajectory: Any, config: PlanarRSConfig | None = None
) -> CertificateResult:
    settings = config or PlanarRSConfig()
    error = _validate(problem)
    if error is not None:
        return CertificateResult(
            CertificateStatus.NUMERICAL_FAILURE, error, 0, 0, 0, None, None
        )
    try:
        traj = (
            trajectory
            if isinstance(trajectory, PolynomialTrajectory)
            else PolynomialTrajectory.from_minco(trajectory)
        )
        cached: dict[int, tuple] = {}
        statistics: dict[str, float] = {}
        base_provider = _provider(settings, statistics)

        def provider(p, t, c, b):
            precision = int(ctx.prec)
            if precision not in cached:
                cached[precision] = base_provider(p, t, c, b)
            return cached[precision]

        old_precision = int(ctx.prec)
        ctx.prec = int(settings.sip.precision_bits[0])
        try:
            spans, checked, depth, plane_error = provider(
                problem, traj, settings.sip, settings.sip.max_cells
            )
            witnesses = (
                ()
                if plane_error is not None
                else _finite_counterexamples(problem, traj, settings.sip, spans)
            )
        finally:
            ctx.prec = old_precision
        if witnesses:
            return CertificateResult(
                CertificateStatus.VIOLATED,
                "finite points in rigorously retained plane-contact spans expose whole-body violations",
                int(settings.sip.precision_bits[0]),
                checked,
                depth,
                None,
                None,
                witnesses,
            )
    except Exception as exc:
        return CertificateResult(
            CertificateStatus.NUMERICAL_FAILURE,
            f"fixed-plane prescreen failed closed: {type(exc).__name__}: {exc}",
            0,
            0,
            0,
            None,
            None,
        )
    report = sip_certify(problem, traj, settings.sip, _safety_span_provider=provider)
    if report.status is CertificateStatus.CERTIFIED_FEASIBLE:
        gap = statistics.get("minimum_plane_gap", float("inf"))
        plane_margin = (settings.sip.clearance + gap) ** 2 - settings.sip.clearance**2
        curve_margin = report.minimum_safety_squared_margin
        combined = min(
            plane_margin,
            float("inf") if curve_margin is None else curve_margin,
        )
        report = replace(
            report,
            minimum_safety_squared_margin=None
            if not math.isfinite(combined)
            else combined,
        )
    return report


__all__ = ["certify"]
