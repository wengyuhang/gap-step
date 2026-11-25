import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.boundary import DenseBoundary
from nonconvex_timevarying_window.sc_dynatogt.diverse_demo import main as diverse_demo_main
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.scenarios import (
    PAPER_REFERENCE_GATE_ORDER,
    build_boundary_scenario,
    build_canonical_scenario,
    build_diverse_scenario,
)


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


def test_general_boundary_scenario_accepts_an_ordered_custom_shape_list():
    square = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    diamond = np.array([[0.0, -1.2], [1.2, 0.0], [0.0, 1.2], [-1.2, 0.0]])
    scenario = build_boundary_scenario(
        (
            ("square", DenseBoundary(square, square, (0, 1, 2, 3))),
            ("diamond", DenseBoundary(diamond, diamond, (0, 1, 2, 3))),
        ),
        mode="static",
        preprocessing_config=PreprocessingConfig(
            vertex_counts=(4,),
            offset_distance=0.1,
            min_safe_area=0.1,
            sc_fit_options={"quadrature_order": 32, "max_nfev": 300},
        ),
        spacing=3.0,
        name="unit_custom",
    )
    assert scenario.track.name == "unit_custom_static_2"
    assert scenario.track.order == (0, 1)
    assert [window.name for window in scenario.track.windows] == ["square", "diamond"]
    np.testing.assert_allclose([window.center0[0] for window in scenario.track.windows], [-1.5, 1.5])
    for window in scenario.track.windows:
        assert window.physical_boundary_at(0.0) is not None
        assert window.contains(window.to_point(np.zeros(2), 0.0), 0.0)


def test_general_boundary_scenario_accepts_explicit_3d_poses_and_motion_scale():
    square = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    centers = np.array([[-4.0, -2.0, 1.0], [5.0, 3.0, 6.0]])
    angles = np.array([[0.1, 1.4, -0.2], [-0.3, 1.7, 0.4]])
    scenario = build_boundary_scenario(
        (
            ("first", DenseBoundary(square, square, (0, 1, 2, 3))),
            ("second", DenseBoundary(square, square, (0, 1, 2, 3))),
        ),
        mode="full",
        preprocessing_config=PreprocessingConfig(
            vertex_counts=(4,),
            offset_distance=0.1,
            min_safe_area=0.1,
            sc_fit_options={"quadrature_order": 32, "max_nfev": 300},
        ),
        centers=centers,
        angles=angles,
        motion_scale=2.5,
    )
    np.testing.assert_allclose(
        [window.center0 for window in scenario.track.windows], centers
    )
    np.testing.assert_allclose(
        [window.angles0 for window in scenario.track.windows], angles
    )
    np.testing.assert_allclose(scenario.track.start, [-7.5, -2.0, 1.0])
    np.testing.assert_allclose(scenario.track.goal, [8.5, 3.0, 6.0])
    np.testing.assert_allclose(
        scenario.track.windows[0].motion.translation_amplitude,
        2.5 * np.array([0.12, 0.28, 0.20]),
    )
    assert scenario.track.windows[0].motion.scale_amplitude == 0.3
    # The requested 0.1 m is a world-space lower bound.  With scale in
    # [0.7, 1.3], preprocessing must use a 0.1 / 0.7 local inset.
    assert scenario.preprocessed_gates[0].safe_region.distance == pytest.approx(0.1 / 0.7)
    assert scenario.track.windows[0].required_world_clearance == pytest.approx(0.1)
    assert scenario.track.windows[0].world_clearance_lower_bound(0.0) >= 0.1


def test_general_boundary_scenario_accepts_a_closed_endpoint_override():
    square = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    endpoint = np.array([-6.0, 0.0, 2.0])
    scenario = build_boundary_scenario(
        (("square", DenseBoundary(square, square, (0, 1, 2, 3))),),
        mode="static",
        preprocessing_config=PreprocessingConfig(
            vertex_counts=(4,),
            offset_distance=0.1,
            min_safe_area=0.1,
            sc_fit_options={"quadrature_order": 32, "max_nfev": 300},
        ),
        centers=np.array([[3.0, 4.0, 2.0]]),
        start=endpoint,
        goal=endpoint,
    )
    np.testing.assert_allclose(scenario.track.start, endpoint)
    np.testing.assert_allclose(scenario.track.goal, endpoint)


def test_diverse_scenario_uses_polygon_smooth_and_mixed_catalog(monkeypatch):
    captured = {}

    def record(definitions, **kwargs):
        captured["names"] = [name for name, _ in definitions]
        captured["kwargs"] = kwargs
        return "sentinel"

    monkeypatch.setattr(
        "nonconvex_timevarying_window.sc_dynatogt.scenarios.build_boundary_scenario",
        record,
    )
    result = build_diverse_scenario(mode="translation", spacing=2.7)
    assert result == "sentinel"
    assert captured["names"] == ["L", "U", "star", "limacon", "wavy", "line_bezier"]
    assert captured["kwargs"]["mode"] == "translation"
    assert captured["kwargs"]["spacing"] == 2.7
    assert captured["kwargs"]["motion_scale"] == 3.5
    centers = captured["kwargs"]["centers"]
    start = captured["kwargs"]["start"]
    goal = captured["kwargs"]["goal"]
    assert centers.shape == (6, 3)
    np.testing.assert_allclose(start, goal)
    np.testing.assert_allclose(start, [-16.0, 4.0, 3.2])
    assert np.ptp(centers[:, 0]) > 30.0
    assert np.ptp(centers[:, 1]) > 28.0
    assert np.ptp(centers[:, 2]) > 4.5
    route = np.vstack((start, centers, goal))
    assert np.min(np.linalg.norm(np.diff(route, axis=0), axis=1)) > 14.0
    assert captured["kwargs"]["name"] == "diverse_paper_irregular_closed"
    assert PAPER_REFERENCE_GATE_ORDER == (
        "Gate1", "Gate2", "Gate3", "Gate4", "Gate6", "Gate7"
    )


def test_diverse_spacious_layout_retains_previous_open_3d_arrangement(monkeypatch):
    captured = {}

    def record(_definitions, **kwargs):
        captured.update(kwargs)
        return "spacious"

    monkeypatch.setattr(
        "nonconvex_timevarying_window.sc_dynatogt.scenarios.build_boundary_scenario",
        record,
    )
    assert build_diverse_scenario(layout="spacious", motion_scale=2.5) == "spacious"
    np.testing.assert_allclose(
        captured["centers"],
        [
            [-10.0, -3.5, 1.5],
            [-6.2, 0.5, 4.8],
            [-2.2, 3.6, 2.5],
            [2.0, 1.2, 5.8],
            [6.0, -3.3, 3.7],
            [10.2, 0.8, 1.8],
        ],
    )
    assert captured["start"] is None
    assert captured["goal"] is None
    assert captured["motion_scale"] == 2.5


def test_diverse_compact_layout_retains_x_axis_arrangement(monkeypatch):
    captured = {}

    def record(_definitions, **kwargs):
        captured.update(kwargs)
        return "compact"

    monkeypatch.setattr(
        "nonconvex_timevarying_window.sc_dynatogt.scenarios.build_boundary_scenario",
        record,
    )
    assert build_diverse_scenario(layout="compact", motion_scale=1.3) == "compact"
    assert captured["centers"] is None
    assert captured["angles"] is None
    assert captured["motion_scale"] == 1.3
    assert captured["start"] is None
    assert captured["goal"] is None


def test_diverse_demo_defaults_to_a_timestamped_paper_irregular_run(monkeypatch, tmp_path):
    captured = {}
    output = tmp_path / "results/demos/runs/20260717_120000_paper_irregular_full"

    def fake_run(output, **kwargs):
        captured["output"] = output
        captured.update(kwargs)
        return {"window_names": [], "passed": True}

    monkeypatch.setattr(
        "nonconvex_timevarying_window.sc_dynatogt.diverse_demo.run_diverse_demo",
        fake_run,
    )
    monkeypatch.setattr(
        "nonconvex_timevarying_window.sc_dynatogt.diverse_demo.timestamped_run_directory",
        lambda *args, **kwargs: output,
    )
    assert diverse_demo_main([]) == 0
    assert captured["output"] == output
    assert captured["layout"] == "paper_irregular"
    assert captured["motion_scale"] == 3.5
