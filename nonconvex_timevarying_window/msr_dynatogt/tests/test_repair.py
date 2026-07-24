from __future__ import annotations

from dataclasses import replace

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import constraint_extrema
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import JointTOGTObjective
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import k_from_durations
from nonconvex_timevarying_window.msr_dynatogt.config import (
    FeasibilityConfig,
    InitializationConfig,
    MSRConfig,
)
from nonconvex_timevarying_window.msr_dynatogt.feasibility_repair import (
    check_feasibility,
    repair_candidate,
    result_from_x,
)


def test_uniform_time_dilation_reduces_dynamic_peaks():
    start = BoundaryState([-1.0, 0.0, 1.4])
    finish = BoundaryState([1.0, 0.0, 1.4])
    waypoint = [[0.0, 1.4, 1.4]]
    fast = MincoSnap(start, finish, waypoint, [0.35, 0.35])
    slow = MincoSnap(start, finish, waypoint, [1.4, 1.4])
    fast_extrema = constraint_extrema(fast, samples_per_segment=65)
    slow_extrema = constraint_extrema(slow, samples_per_segment=65)
    assert max(slow_extrema["max_rotor_thrust"]) < max(
        fast_extrema["max_rotor_thrust"]
    )
    assert slow_extrema["max_body_rate_xy"] < fast_extrema["max_body_rate_xy"]
    assert slow_extrema["max_velocity"] < fast_extrema["max_velocity"]


def test_repair_preserves_order_and_window_legality(static_scenario):
    config = MSRConfig.for_suite("smoke", repair_mode="uniform")
    config = replace(
        config,
        optimization=replace(config.optimization, max_iterations=2),
        initialization=InitializationConfig(
            random_starts=0, turn_aware_starts=0, dispersed_starts=0
        ),
        feasibility=FeasibilityConfig(samples_per_segment=33),
        repair=replace(
            config.repair,
            maximum_scale=12.0,
            binary_iterations=4,
            reoptimization_penalty_multiplier=8.0,
        ),
    )
    objective = JointTOGTObjective(static_scenario.track, config.optimization)
    base = objective.forward(objective.initial_guess())
    fast_durations = base.durations * 0.12
    fast_x = np.concatenate(
        (k_from_durations(fast_durations), base.d.reshape(-1))
    )
    fast = result_from_x(static_scenario.track, fast_x, config)
    before = check_feasibility(static_scenario.track, fast, config)
    assert before.window_order_legal and before.window_internal_legal
    assert not before.sampled_dynamic_limits_satisfied
    repaired = repair_candidate(
        static_scenario.track, fast, config, original_report=before
    )
    assert repaired.triggered
    assert repaired.succeeded
    assert repaired.feasibility.window_order_legal
    assert repaired.feasibility.window_internal_legal
    assert repaired.feasibility.sampled_dynamic_limits_satisfied
    assert repaired.after_max_rotor_thrust < repaired.before_max_rotor_thrust
