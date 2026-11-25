import xml.etree.ElementTree as ET

import numpy as np
import pytest

from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import matrix, scenario
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import matrix_report
from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing.run import make_planner
from convex_timevarying_window.comparisons.论文实验.实验2.matched_full_matrix.togt import matched_objective, initial_vector
from convex_timevarying_window.comparisons.论文实验.实验2.wang2023.rollout import Reference, Settings, model_config


def test_fifty_records_and_independent_global_and_local_budgets():
    assert len(matrix.LEVELS) == 10
    assert matrix.TOTAL == 50
    assert sum(matrix.STARTS.values())*len(matrix.LEVELS) == 330
    assert matrix.STARTS['wang2023'] == 1
    assert Settings().temporal_scales == (1., .8, 1.2, 1.5)


@pytest.mark.parametrize('level', scenario.LEVELS)
def test_every_condition_uses_same_height_rule_in_world_and_planning(tmp_path, level):
    track, _ = scenario.build_case(scenario.Case(level=level))
    world = ET.parse(scenario.write_world(track, scenario.ASSETS, tmp_path))
    models = [m for m in world.findall('world/model') if m.get('name', '').startswith('gate_')]
    assert len(models) == 7
    for w, model in zip(track.windows, models):
        plugin = model.find("plugin[@name='paper_experiment::MixedGateMotion']")
        length = float(plugin.findtext('length'))
        assert length == w.motion.length
        if w.motion.kind == 'swing':
            assert length == 2.20
        for t in (.031, .117):
            np.testing.assert_allclose(np.asarray(scenario.pose(w, t)[0]).ravel(), w.state_at(t)[0], atol=1e-12)


@pytest.mark.parametrize('center', [False, True])
def test_high_swing_togt_uses_identical_cold_curves_and_original_objective(center):
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    initializer = make_planner(track, True)
    ours = make_planner(track, not center)
    obj = matched_objective(track, center, initializer)
    for index, seed in enumerate(ours.seeds()):
        initial = initial_vector(obj, initializer, index)
        forward = obj.forward(initial)
        actual = forward.trajectory.coefficients*forward.durations[:, None, None]**np.arange(8)[None, :, None]
        expected = np.asarray(ours.curve_values(seed)).ravel().reshape(8, 3, 8).transpose(0, 2, 1)
        np.testing.assert_allclose(actual, expected, atol=1e-8, rtol=0.)
        np.testing.assert_allclose(obj.value_and_gradient(initial)[0], ours.objective(seed)[0], rtol=1e-8, atol=1e-4)
        if center:
            for w, at, point in zip(track.windows, forward.traversal_times, forward.waypoints):
                np.testing.assert_allclose(point, w.state_at(at)[0], atol=1e-10)


def test_wang_dynamic_center_equality_uses_the_new_swing_at_absolute_time():
    from dataclasses import replace
    from 复现.wang2023_verified import Planner, x500
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    settings = replace(Settings(), compile_functions=False)
    reference = Reference(track, settings, scenario.ASSETS/'meshes')
    gate = 1
    window = track.windows[gate]
    planner = Planner([lambda t: scenario.pose(window, t)[0]], model_config(settings),
        flatness_function=x500.flatness, yaw=lambda t: 0*t+np.pi/2)
    head = np.zeros((3, 4)); head[:, 0] = track.start
    now = .71
    initial = reference.seed(planner, head, now, gate, 1.2)
    trajectory, _, _ = planner.unpack(initial, planner.parameters(head, reference.tails[gate], now))
    at = now+trajectory.segment_durations[0]
    np.testing.assert_allclose(trajectory.sample(at), window.state_at(at)[0], atol=1e-8)
    assert window.motion.length == 2.20


def test_five_fresh_methods_share_the_comparable_table_clock():
    rows = [dict(level='a75_f6', variant=m, passed=True, outcome='clear', time_s=float(20+i),
        measurement_definition='actual_clock_and_measurement_reach_full_reference_T')
        for i, m in enumerate(matrix.METHODS)]
    rows[-1]['time_s'] = 19.
    table = '\n'.join(matrix_report.table(rows))
    assert '**19.000**' in table
    assert '†' not in table
