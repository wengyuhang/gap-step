import numpy as np
from .. import run


def test_dynamic_diagnostic_separates_motor_reserve_and_physical_limit():
    audit = dict(passed=True, maximum_body_rates_radps=[6.575507, 3.3, .8],
        maximum_motor_n=7.78, minimum_motor_n=.7)
    row = dict(passed=True, **run.diagnostic(audit))
    assert not row['dense_rate_within_planning_limits']
    assert not row['dense_motor_within_planning_reserve']
    assert row['dense_motor_within_physical_limits']
    assert not run.time_comparable(row)
    row['dense_rate_within_planning_limits'] = True
    assert run.time_comparable(row)
    row['passed'] = False
    assert not run.time_comparable(row)


def test_center_uses_same_solver_and_own_analytic_seeds():
    track, _ = run.scenario.build_case(run.scenario.Case(level='static'))
    p = run.Planner(track, False, pose_function=run.scenario.pose)
    from ...independent_repair.smooth_augmented import Planner as Base
    baseline = Base(track, False, pose_function=run.scenario.pose)
    assert p.nvar == 71
    assert len(p.seeds()) == 4
    for actual, expected in zip(p.seeds(), baseline.seeds()):
        np.testing.assert_array_equal(actual, expected)


def test_retained_sources_and_full_input_matrix_are_identical():
    retained = run.retained(run.sources())
    assert len(retained) == 25
    assert sum(m == 'ours' for l, m in retained) == 5
    assert sum(m.startswith('dynamic_togt') for l, m in retained) == 20
    assert all(v['all_source_contents_unchanged'] and v['target_scene_equal'] for v in retained.values())


def test_static_inherited_assessment_preserves_both_evidence_scopes(tmp_path):
    import shutil
    key = 'static', 'dynamic_togt'
    evidence = run.retained(run.sources())[key]
    path = tmp_path/'plans'/'static_dynamic_togt'
    shutil.copytree(evidence['source_directory'], path)
    (tmp_path/'assessments').mkdir()
    run.save(path/'matrix_reuse.json', evidence)
    row = run.assess(tmp_path, *key)
    assert row['passed'] and row['reused_plan']
    assert 'Bernstein' in row['evidence']
    assert '5ms' in row['dynamics_evidence']
