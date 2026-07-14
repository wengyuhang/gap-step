from __future__ import annotations

import json

import numpy as np
import pytest
from shapely.geometry import Point, Polygon

from nonconvex_timevarying_window.sc_dynatogt import preprocessing
from nonconvex_timevarying_window.sc_dynatogt.boundary import (
    ADAPTIVE_VERTEX_COUNTS,
    DenseBoundary,
    Line,
    validate_polygon,
)
from nonconvex_timevarying_window.sc_dynatogt.offset import DEFAULT_INWARD_OFFSET
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    e1_boundaries,
    l_shape_boundary,
    main,
    preprocess_boundary,
)


@pytest.fixture(scope="module")
def small_l_artifact() -> PreprocessedGate:
    # Straight sides are represented exactly at any count; the reduced count is
    # solely a unit-test speed control.  Production defaults are checked below.
    config = PreprocessingConfig(
        vertex_counts=(32,),
        sc_fit_options={"quadrature_order": 32, "max_nfev": 500},
    )
    return preprocess_boundary(l_shape_boundary(), name="l_test", config=config)


def test_production_config_uses_fixed_document_values() -> None:
    config = PreprocessingConfig()
    assert config.vertex_counts == (256, 512, 1024, 2048, 3200)
    assert config.vertex_counts == ADAPTIVE_VERTEX_COUNTS
    assert config.offset_distance == pytest.approx(0.315)
    assert config.offset_distance == DEFAULT_INWARD_OFFSET
    assert config.boundary_tolerance == pytest.approx(0.005)
    assert config.concavity_tolerance == pytest.approx(0.003)


def test_e1_catalog_contains_all_required_boundary_families() -> None:
    catalog = e1_boundaries()
    assert tuple(catalog) == (
        "l_shape",
        "u_shape",
        "five_point_star",
        "limacon",
        "wavy",
        "line_bezier_mixed",
    )
    for boundary in catalog.values():
        result = validate_polygon(boundary.vertices, require_ccw=True)
        assert result.valid, result.errors
        assert len(boundary.vertices) >= 3
    assert len(catalog["l_shape"].corners) == 6
    assert len(catalog["u_shape"].corners) == 8
    assert len(catalog["five_point_star"].corners) == 10
    assert len(catalog["limacon"].corners) == 0
    assert len(catalog["wavy"].corners) == 0
    assert 0 < len(catalog["line_bezier_mixed"].corners) < len(catalog["line_bezier_mixed"].vertices)


def test_complete_pipeline_uses_chang_offset_and_sc(small_l_artifact: PreprocessedGate) -> None:
    artifact = small_l_artifact
    assert artifact.name == "l_test"
    assert artifact.sampled_boundary.m == 32
    assert artifact.sampled_boundary.report is not None
    assert artifact.sampled_boundary.report.accepted
    assert artifact.sampled_boundary.report.corners_preserved
    assert artifact.safe_region.distance == pytest.approx(0.315)
    assert artifact.safe_region.diagnostics.valid
    assert artifact.sc_map.n_vertices == len(artifact.safe_polygon)
    point = artifact.sc_map.evaluate((0.0, 0.0))
    assert np.all(np.isfinite(point))
    assert Polygon(artifact.safe_polygon).contains(Point(point))


def test_preprocessed_artifact_round_trip(tmp_path, small_l_artifact: PreprocessedGate) -> None:
    output = small_l_artifact.save(tmp_path / "l_gate")
    assert output == tmp_path / "l_gate"
    assert (output / "manifest.json").is_file()
    assert (output / "geometry.npz").is_file()
    assert (output / "sc_map.npz").is_file()
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["format_version"] == 1
    assert manifest["name"] == "l_test"

    restored = PreprocessedGate.load(output)
    assert restored.name == small_l_artifact.name
    assert restored.config.vertex_counts == (32,)
    assert np.array_equal(restored.dense_boundary.vertices, small_l_artifact.dense_boundary.vertices)
    assert np.array_equal(restored.sampled_boundary.corner_mask, small_l_artifact.sampled_boundary.corner_mask)
    assert np.allclose(restored.safe_polygon, small_l_artifact.safe_polygon, rtol=0.0, atol=1.0e-12)
    for z in ((0.0, 0.0), (0.2, -0.1), (-0.3, 0.25)):
        assert np.allclose(restored.sc_map.evaluate(z), small_l_artifact.sc_map.evaluate(z), atol=1.0e-12)


def test_segment_and_csv_entry_points_construct_dense_boundary(monkeypatch, tmp_path) -> None:
    captured: list[tuple[DenseBoundary, str]] = []

    def record(boundary, *, name, config=None, corners=None):
        assert isinstance(boundary, DenseBoundary)
        captured.append((boundary, name))
        return "sentinel"

    monkeypatch.setattr(preprocessing, "preprocess_boundary", record)
    segments = [
        Line((0.0, 0.0), (2.0, 0.0)),
        Line((2.0, 0.0), (2.0, 2.0)),
        Line((2.0, 2.0), (0.0, 2.0)),
        Line((0.0, 2.0), (0.0, 0.0)),
    ]
    assert preprocessing.preprocess_segments(segments, name="segments") == "sentinel"
    assert captured[-1][1] == "segments"
    assert len(captured[-1][0].corners) == 4

    path = tmp_path / "gate.csv"
    path.write_text("x,y\n0,0\n2,0\n2,2\n0,2\n", encoding="utf-8")
    assert preprocessing.preprocess_csv(path, corner_indices=(0, 1, 2, 3)) == "sentinel"
    assert captured[-1][1] == "gate"
    assert len(captured[-1][0].corners) == 4


def test_config_rejects_invalid_vertex_ladder() -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        PreprocessingConfig(vertex_counts=(256, 128))


def test_preprocessing_cli_writes_portable_artifact(tmp_path) -> None:
    output = tmp_path / "cli_gate"
    assert main([
        "--shape", "l_shape", "--outdir", str(output),
        "--vertex-counts", "32", "--quadrature-order", "32",
    ]) == 0
    assert (output / "manifest.json").is_file()
    assert PreprocessedGate.load(output).name == "l_shape"
