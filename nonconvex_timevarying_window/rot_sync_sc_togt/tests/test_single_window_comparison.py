from dataclasses import replace

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.rot_sync_sc_togt.single_window_comparison import (
    FixedWaypointObjective, audit,
)
from nonconvex_timevarying_window.rot_sync_sc_togt.optimizer import RotSyncOptimizationConfig
from nonconvex_timevarying_window.rot_sync_sc_togt.scenarios import build_smoke_scenario


@pytest.fixture(scope='module')
def scenario():
    return build_smoke_scenario()


def test_fixed_waypoint_changes_world_target_with_time_but_not_local_point(scenario):
    objective = FixedWaypointObjective(scenario, RotSyncOptimizationConfig())
    x = objective.initial_guess()
    first = objective.forward(x)
    x[0] += .35
    second = objective.forward(x)
    window = scenario.windows[0]
    assert Polygon(window.safe_polygon).covers(Point(objective.fixed_q))
    assert np.array_equal(first.local_points, second.local_points)
    assert not np.allclose(first.trajectory.evaluate(first.crossing_times[0]),
                           second.trajectory.evaluate(second.crossing_times[0]))
    for forward in [first, second]:
        t = forward.crossing_times[0]
        assert forward.trajectory.num_segments == 2
        assert not hasattr(forward.trajectory, 'sync_segments')
        assert np.allclose(forward.trajectory.evaluate(t), window.world_point(objective.fixed_q, t))
        for instant, state in [(0, scenario.start_state),
                               (forward.trajectory.total_time, scenario.goal_state)]:
            assert np.allclose(np.stack([forward.trajectory.evaluate(instant, d)
                                        for d in range(4)]), state.matrix, atol=1e-8)


def test_common_audit_rejects_dynamic_violation_even_with_legal_waypoint(scenario):
    config = RotSyncOptimizationConfig()
    objective = FixedWaypointObjective(scenario, config)
    forward = objective.forward(objective.initial_guess())
    strict = replace(config, dynamic_limits=replace(config.dynamic_limits, max_velocity=.01))
    report, data = audit(scenario, forward, strict, dt=.02)
    assert report['q_in_safe_region']
    assert report['crossing_error'] < 1e-8
    assert report['ordered_once']
    assert not report['checks']['velocity']
    assert not report['trajectory_validation_pass']
    assert np.max(np.diff(data['time'])) <= .02000001
