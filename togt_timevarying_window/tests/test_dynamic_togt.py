from __future__ import annotations

import numpy as np

from togt_timevarying_window.environment import demo_track
from togt_timevarying_window.planner import DynamicTOGTPlanner, PlannerConfig


def test_dynamic_gate_changes_geometry():
    track = demo_track(dynamic=True)
    gate = track.gates[1]
    poly0 = gate.polygon_at(0.0)
    poly1 = gate.polygon_at(1.0)
    assert not np.allclose(poly0, poly1)


def test_planner_returns_gate_feasible_dynamic_trajectory():
    track = demo_track(dynamic=True)
    traj = DynamicTOGTPlanner(PlannerConfig(max_time=70.0, max_speed=2.35, wait_steps=8, gate_samples_per_axis=1)).plan(track)
    assert traj is not None
    assert len(traj.gate_times) == len(track.gates)
    assert traj.times == sorted(traj.times)
    for idx, (t, point) in enumerate(zip(traj.gate_times, traj.gate_points)):
        assert track.gates[idx].contains(point, t)


def test_static_and_dynamic_tracks_are_distinct():
    static = demo_track(dynamic=False)
    dynamic = demo_track(dynamic=True)
    planner = DynamicTOGTPlanner(PlannerConfig(max_time=70.0, max_speed=2.35, wait_steps=8, gate_samples_per_axis=1))
    static_traj = planner.plan(static)
    dynamic_traj = planner.plan(dynamic)
    assert static_traj is not None
    assert dynamic_traj is not None
    assert static_traj.gate_times != dynamic_traj.gate_times or not np.allclose(static_traj.gate_points, dynamic_traj.gate_points)
