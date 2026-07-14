import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.scenarios import build_canonical_scenario


def _fast_config():
    return PreprocessingConfig(
        vertex_counts=(32,),
        sc_fit_options={"quadrature_order": 32, "max_nfev": 500},
    )


def test_canonical_scenario_preserves_fixed_one_pass_order():
    scenario = build_canonical_scenario(mode="static", preprocessing_config=_fast_config(), gate_count=3)
    assert scenario.track.order == (0, 1, 2)
    assert [window.name for window in scenario.track.windows] == ["L", "U", "star"]
    assert len(scenario.preprocessed_gates) == 3
    for gate, window in zip(scenario.preprocessed_gates, scenario.track.windows):
        np.testing.assert_allclose(gate.safe_polygon, window.safe_polygon)
        point = window.to_point(np.zeros(2), 1.0)
        assert window.contains(point, 1.0)


def test_motion_modes_enable_only_documented_components():
    static = build_canonical_scenario(mode="static", preprocessing_config=_fast_config(), gate_count=1)
    translation = build_canonical_scenario(mode="translation", preprocessing_config=_fast_config(), gate_count=1)
    full = build_canonical_scenario(mode="full", preprocessing_config=_fast_config(), gate_count=1)
    t = 1.3
    static_state = static.track.windows[0].state_at(t)
    translation_state = translation.track.windows[0].state_at(t)
    full_state = full.track.windows[0].state_at(t)
    np.testing.assert_allclose(static_state[3], 0.0)
    np.testing.assert_allclose(static_state[4], 0.0)
    assert static_state[5] == 0.0
    assert np.linalg.norm(translation_state[3]) > 0.0
    np.testing.assert_allclose(translation_state[4], 0.0)
    assert translation_state[5] == 0.0
    assert np.linalg.norm(full_state[3]) > 0.0
    assert np.linalg.norm(full_state[4]) > 0.0
    assert abs(full_state[5]) > 0.0
