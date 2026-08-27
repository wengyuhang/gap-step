"""SIP exchange solver using the fixed-plane certificate."""

from __future__ import annotations

from typing import Any

from nonconvex_timevarying_window.sip_dynatogt.solver import solve as sip_solve
from nonconvex_timevarying_window.sip_dynatogt.model import Witness

from .certificate import certify
from .model import PlanarRSConfig


def _initial_witnesses(problem, segments, config):
    """Small finite seed; the certifier supplies every omitted constraint."""

    witnesses = []
    for segment in range(segments):
        witnesses.extend(
            Witness("dynamic", segment, tau, 0.0, source="initial")
            for tau in config.initial_nodes
        )
    for crossing_index, window_index in enumerate(problem.order):
        window = problem.windows[window_index]
        for boundary_index in range(len(window.boundary)):
            witnesses.extend(
                Witness(
                    "safety",
                    crossing_index,
                    1.0,
                    0.0,
                    window_index,
                    boundary_index,
                    u,
                    "initial-crossing",
                )
                for u in config.initial_nodes
            )
    return tuple(witnesses)


def solve(problem, config: PlanarRSConfig | None = None, **kwargs: Any):
    settings = config or PlanarRSConfig()
    return sip_solve(
        problem,
        settings.sip,
        _certifier=lambda p, t, _: certify(p, t, settings),
        _initial_witness_provider=_initial_witnesses,
        **kwargs,
    )


__all__ = ["solve"]
