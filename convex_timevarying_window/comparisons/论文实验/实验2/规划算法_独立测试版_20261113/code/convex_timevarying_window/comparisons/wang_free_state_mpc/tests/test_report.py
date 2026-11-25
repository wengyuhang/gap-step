import json
from convex_timevarying_window.comparisons.wang_free_state_mpc import report


def test_legacy_adapter_does_not_change_original_evidence(tmp_path,monkeypatch):
    base = tmp_path/'convex'
    method = base/'comparisons/motion_rate_racing'
    run = method/'results/local_sweep_frozen_20260918/ours_rate1_phase0_attempt1'
    run.mkdir(parents=True)
    original = json.dumps(dict(motion_rate=1.,phase=0.,layout=0.,config={'radius':.38}))
    (run/'result.json').write_text(original)
    (run/'flight.ulg').write_bytes(b'test log bytes')
    (method/'PROTOCOL.md').write_text('Frozen frequency-only matrix, amplitudes unchanged.')
    monkeypatch.setattr(report,'BASE',base)
    view,evidence = report.legacy_metadata_view(run,tmp_path/'new_batch')
    normalized = json.loads((view/'result.json').read_text())
    assert normalized['translation_scale'] == normalized['rotation_scale'] == 1.
    assert (run/'result.json').read_text() == original
    assert (view/'flight.ulg').resolve() == run/'flight.ulg'
    assert evidence['added_fields'] == {'translation_scale':1.,'rotation_scale':1.}
    assert report.legacy_metadata_view(run,tmp_path/'new_batch')[0] == view


def test_never_guess_missing_amplitudes_for_other_matrices(tmp_path,monkeypatch):
    monkeypatch.setattr(report,'BASE',tmp_path)
    assert report.legacy_metadata_view(tmp_path/'unrelated_run',tmp_path/'batch') is None
