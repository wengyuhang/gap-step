from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.avs_ppo.hardest_comparison import (
    HardestComparisonAVSEnvironment,
    HardestTrackConfig,
)


def test_hardest_track_uses_original_cuboid_and_six_windows() -> None:
    environment = HardestComparisonAVSEnvironment(HardestTrackConfig())
    assert environment.problem.name == "wide_scrambled_fast_closed_loop_6"
    assert len(environment.problem.order) == 6
    assert np.allclose(environment.body.half_extents, (0.26504, 0.26504, 0.05890))
    assert np.isclose(environment.required_clearance, 0.015)
    assert environment.observe().shape == (environment.observation_dim,)


def test_nominal_recovery_completes_hardest_track_without_violation() -> None:
    environment = HardestComparisonAVSEnvironment(HardestTrackConfig())
    done = False
    info = {}
    while not done:
        _, _, terminated, truncated, info = environment.step(
            environment.recovery_action,
            mask=np.ones(environment.action_dim, dtype=bool),
        )
        done = terminated or truncated
    assert info["success"]
    assert info["safety_violations"] == 0
    assert info["gates_crossed"] == 6
    assert len(environment.crossing_records) == 6
    assert all(record["inside"] for record in environment.crossing_records)

