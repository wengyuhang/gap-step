import importlib
import json
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_formal_case_list_matches_frozen_protocol():
    entry = importlib.import_module('convex_timevarying_window.comparisons.论文实验.实验1.run_translation')
    from convex_timevarying_window.comparisons.single_gate_swing.run import CASES
    protocol = json.loads((ROOT/'protocol.json').read_text())
    assert entry.planned_cases(protocol) == CASES
    assert len(CASES) == 17
    assert entry.ROOT/'convex_timevarying_window' == ROOT.parents[3]/'convex_timevarying_window'


def test_analysis_rejects_incomplete_batches(tmp_path):
    analysis = importlib.import_module('convex_timevarying_window.comparisons.论文实验.实验1.analyze_translation')
    (tmp_path/'status.json').write_text(json.dumps(dict(state='running', sources_unchanged=True)))
    with pytest.raises(RuntimeError, match='incomplete'):
        analysis.analyze(tmp_path)


def test_pendulum_is_not_yet_a_formal_collision_comparison():
    protocol = json.loads((ROOT/'protocol.json').read_text())
    assert not protocol['pendulum']['formal_flights_allowed_before_validation']
    assert protocol['pendulum']['mix_absolute_times_with_x500_translation'] is False
