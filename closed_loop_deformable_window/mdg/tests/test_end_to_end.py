from dataclasses import replace

import numpy as np

from mdg.dynamic_gate import SplineSeries
from mdg.experiments import formal_tasks
from mdg.planner import MDGPlanner
from mdg.scenario_generator import generate_scenario


def test_formal_matrix_contains_exactly_2090_method_runs():
    tasks = formal_tasks()
    assert len(tasks) == 2090
    assert sum(task["experiment"] == "E1" for task in tasks) == 750
    assert sum(task["experiment"] == "E5" for task in tasks) == 300


def test_small_end_to_end_is_deterministic(fast_config):
    config = replace(
        fast_config,
        scenario=replace(fast_config.scenario, planning_horizon=12.0),
    )
    first_scenario = generate_scenario(
        config, seed=3, gate_count=1, difficulty="low", closed_ratio=0.0
    )
    second_scenario = generate_scenario(
        config, seed=3, gate_count=1, difficulty="low", closed_ratio=0.0
    )
    first = MDGPlanner(config).plan(first_scenario, method="mdg_center")
    second = MDGPlanner(config).plan(second_scenario, method="mdg_center")
    assert first.graph_coarse is not None
    assert second.graph_coarse is not None
    assert first.graph_coarse.selected_node_ids == second.graph_coarse.selected_node_ids
    assert first.disc_tracks.keys() == second.disc_tracks.keys()


def test_planner_reports_window_without_open_opportunity(
    fast_config, one_gate_scenario
):
    gate = one_gate_scenario.gates[0]
    gate.scale_profile = SplineSeries(
        np.array((0.0, one_gate_scenario.horizon)),
        np.array((0.12, 0.12)),
    )

    result = MDGPlanner(fast_config).plan(one_gate_scenario, method="mdg_free")

    assert not result.success
    assert result.failure_reason == "a gate has no valid disc track"
    assert result.disc_tracks == {0: []}
