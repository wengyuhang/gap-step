"""Scenario adapters for the interpolated RotSync method."""

from __future__ import annotations

from dataclasses import replace
from typing import Mapping

import numpy as np

from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import (
    DEFAULT_BODY,
    DEFAULT_RHO,
    REALISTIC_BODY,
    REALISTIC_RHO,
    REALISTIC_SHAPE_SCALES,
    RotSyncScenario,
    build_formal_scenarios,
    build_multi_scenarios,
    build_realistic_extreme_scenario,
    build_smoke_scenario,
    preprocess_shape_catalog,
    scenario_difficulty_metrics,
)
from nonconvex_timevarying_window.sc_dynatogt.collision import CuboidBody
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessedGate


def build_oblique_smoke_scenario(
    catalog: Mapping[str, PreprocessedGate] | None = None,
    *,
    rho: float = DEFAULT_RHO,
    body: CuboidBody = DEFAULT_BODY,
) -> RotSyncScenario:
    """Asymmetric L-window verification case with distinct optimized inputs."""

    scenario = build_smoke_scenario(catalog, rho=rho, body=body)
    altitude = float(scenario.windows[0].center[2])
    return replace(
        scenario,
        name="single_L_oblique",
        start_state=BoundaryState(np.asarray((-4.5, -0.8, altitude))),
        goal_state=BoundaryState(np.asarray((4.5, 0.8, altitude))),
        description=(
            "Oblique single-L verification case for distinct entry/exit SC inputs."
        ),
    )


__all__ = [
    "DEFAULT_BODY",
    "DEFAULT_RHO",
    "REALISTIC_BODY",
    "REALISTIC_RHO",
    "REALISTIC_SHAPE_SCALES",
    "RotSyncScenario",
    "build_formal_scenarios",
    "build_multi_scenarios",
    "build_oblique_smoke_scenario",
    "build_realistic_extreme_scenario",
    "build_smoke_scenario",
    "preprocess_shape_catalog",
    "scenario_difficulty_metrics",
]
