from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from mdg.config import MDGConfig
from mdg.dynamic_gate import EndpointState, Scenario, SplineSeries
from mdg.gate_shapes import GATE_TYPES


def static_gate(kind: str = "l_shape", *, gate_id: int = 0, center=(2.0, 0.0, 1.0)):
    times = np.array((0.0, 4.0))
    gate_type = GATE_TYPES[kind]
    return gate_type(
        gate_id=gate_id,
        name=f"G{gate_id + 1}_{kind}",
        center_profile=SplineSeries(times, np.tile(np.asarray(center), (2, 1))),
        rpy_profile=SplineSeries(
            times, np.tile(np.array((0.0, np.pi / 2.0, 0.0)), (2, 1))
        ),
        scale_profile=SplineSeries(times, np.ones(2)),
        deformation_profile=SplineSeries(times, np.zeros(2)),
        boundary_samples=96,
    )


@pytest.fixture
def fast_config():
    base = MDGConfig()
    return replace(
        base,
        disks=replace(base.disks, grid_resolution=0.10, max_disks_per_gate=3),
        tracking=replace(
            base.tracking,
            gate_sample_dt=0.20,
            validation_dt=0.05,
        ),
        graph=replace(base.graph, dt_coarse=0.20, dt_fine=0.05),
        scenario=replace(base.scenario, planning_horizon=8.0, curve_boundary_samples=96),
        backend=replace(
            base.backend,
            max_iterations=8,
            samples_per_segment=5,
            validation_samples_per_segment=17,
            max_lazy_repairs=0,
        ),
        runtime=replace(
            base.runtime,
            workers=1,
            save_video=False,
            video_fps=4,
            video_duration=0.5,
        ),
    )


@pytest.fixture
def one_gate_scenario():
    gate = static_gate()
    start = EndpointState(
        np.array((0.0, 0.0, 1.0)),
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
    )
    return Scenario(
        "one_gate",
        0,
        4.0,
        (gate,),
        (0,),
        start,
        "low",
        0.0,
    )

