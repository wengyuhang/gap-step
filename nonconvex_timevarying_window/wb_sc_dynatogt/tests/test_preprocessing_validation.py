from __future__ import annotations

import json
import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.boundary import DenseBoundary
from nonconvex_timevarying_window.wb_sc_dynatogt.config import WBSCOptimizationConfig
from nonconvex_timevarying_window.wb_sc_dynatogt.optimizer import optimize_track
from nonconvex_timevarying_window.wb_sc_dynatogt.preprocessing import (
    WBPreprocessedGate,
    WBPreprocessingConfig,
    preprocess_boundary,
)
from nonconvex_timevarying_window.wb_sc_dynatogt.validation import validate_legacy_sphere


def test_zero_inset_preprocessing_persistence(tmp_path) -> None:
    vertices = np.array([[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]])
    boundary = DenseBoundary(vertices, vertices, tuple(range(4)))
    config = WBPreprocessingConfig(vertex_counts=(16, 32), sc_fit_options={"quadrature_order": 24})
    gate = preprocess_boundary(boundary, name="square", config=config)
    gate.save(tmp_path / "gate")
    restored = WBPreprocessedGate.load(tmp_path / "gate")
    assert np.allclose(restored.candidate_polygon, gate.sampled_boundary.vertices)
    manifest = json.loads((tmp_path / "gate" / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["fixed_inward_offset"] == 0.0
    assert manifest["safety_model"] == "online_pose_dependent_oriented_cuboid"


def test_cuboid_validation_checks_prescribed_crossing(static_track) -> None:
    config = WBSCOptimizationConfig(
        max_iterations=1,
        samples_per_segment=2,
    )
    result = optimize_track(static_track, config)
    assert result.safety_report is not None
    assert result.safety_report.checked_time_count == 1
    assert result.safety_report.safe
    assert "continuous-time collision certificate" in result.safety_report.sampling_statement


def test_result_persistence_contains_attitude_safety(static_track) -> None:
    config = WBSCOptimizationConfig(
        max_iterations=1,
        samples_per_segment=2,
    )
    result = optimize_track(static_track, config)
    payload = result.to_dict()
    assert len(payload["yaw_waypoints"]) == 1
    assert np.isclose(payload["collider"]["maximum_radius"], 0.300)
    assert payload["safety_report"]["checked_time_count"] == 1
    assert len(payload["safety_report"]["checks"][0]["local_corners"]) == 8


def test_legacy_sphere_uses_its_own_0315_inset(static_track) -> None:
    config = WBSCOptimizationConfig(max_iterations=1, samples_per_segment=2)
    result = optimize_track(static_track, config, body_scale=0.0)
    report = validate_legacy_sphere(static_track, result)
    assert report.checked_time_count == 1
    assert report.safe
    assert report.minimum_clearance > 0.0
    assert "0.315 m local gate inset" in report.sampling_statement
