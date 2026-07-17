from __future__ import annotations

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt
import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.boundary import DenseBoundary
from nonconvex_timevarying_window.sc_dynatogt.environment import (
    MotionProfile,
    SCDynamicWindow,
    SCWindowTrack,
)
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.optimizer import OptimizationResult
from nonconvex_timevarying_window.sc_dynatogt.preprocessing import (
    PreprocessedGate,
    PreprocessingConfig,
    line_bezier_mixed_boundary,
    load_preprocessed_gate,
    preprocess_boundary,
)
from nonconvex_timevarying_window.sc_dynatogt.visualization import (
    _quadrotor_basis,
    export_dynamic_window_gif,
    export_trajectory_csv,
    plot_preprocessing,
    plot_trajectory,
)


@pytest.fixture(scope="module")
def visualization_case() -> tuple[PreprocessedGate, SCWindowTrack, MincoSnap, OptimizationResult]:
    vertices = np.asarray(
        [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)],
        dtype=float,
    )
    dense = DenseBoundary(vertices, vertices, (0, 1, 2, 3))
    gate = preprocess_boundary(
        dense,
        name="visual_square",
        config=PreprocessingConfig(
            vertex_counts=(4,),
            offset_distance=0.1,
            min_safe_area=0.1,
            sc_fit_options={"quadrature_order": 32, "max_nfev": 300},
        ),
    )
    window = SCDynamicWindow(
        name="moving_square",
        sc_map=gate.sc_map,
        safe_polygon=gate.safe_polygon,
        center0=np.asarray((0.0, 0.0, 0.8)),
        angles0=np.asarray((0.0, 0.0, 0.0)),
        motion=MotionProfile(
            translation_amplitude=np.asarray((0.04, 0.08, 0.05)),
            rotation_amplitude=np.asarray((0.08, 0.04, 0.03)),
            scale_amplitude=0.05,
            translation_period=4.0,
            rotation_period=5.0,
            scale_period=6.0,
        ),
        physical_boundary=gate.dense_boundary.vertices,
    )
    start = np.asarray((-1.4, 0.0, 0.8))
    goal = np.asarray((1.4, 0.0, 0.8))
    crossing_time = np.asarray((1.0,))
    waypoint = window.to_point(np.zeros(2), float(crossing_time[0]))
    trajectory = MincoSnap(
        BoundaryState(start),
        BoundaryState(goal),
        waypoint[None, :],
        np.asarray((1.0, 1.0)),
    )
    track = SCWindowTrack(
        "visualization_test",
        start=start,
        goal=goal,
        windows=(window,),
        order=(0,),
    )
    result = OptimizationResult(
        success=True,
        status=0,
        message="unit-test fixture",
        objective=1.0,
        iterations=1,
        evaluations=1,
        gradient_inf_norm=0.0,
        x=np.zeros(5),
        k=np.zeros(2),
        d=np.zeros((1, 2)),
        durations=np.asarray((1.0, 1.0)),
        traversal_times=crossing_time,
        waypoints=waypoint[None, :],
        local_points=np.zeros((1, 2)),
        trajectory=trajectory,
        constraint_extrema={},
        full_time_gradient=True,
    )
    return gate, track, trajectory, result


def test_preprocessing_png_contains_all_pipeline_layers(
    tmp_path, visualization_case
) -> None:
    gate, _, _, _ = visualization_case
    figures_before = tuple(plt.get_fignums())
    output = plot_preprocessing(
        gate,
        tmp_path / "nested" / "preprocessing.png",
        radial_count=3,
        angular_count=6,
        samples_per_line=24,
        dpi=70,
    )
    assert output.is_file()
    assert output.stat().st_size > 2_000
    assert output.read_bytes().startswith(b"\x89PNG")
    assert tuple(plt.get_fignums()) == figures_before


def test_loaded_256_sample_mixed_gate_preprocessing_png(tmp_path) -> None:
    gate = preprocess_boundary(
        line_bezier_mixed_boundary(),
        name="line_bezier_mixed",
        config=PreprocessingConfig(
            vertex_counts=(256,),
            sc_fit_options={"quadrature_order": 32},
        ),
    )
    artifact = gate.save(tmp_path / "mixed_artifact")
    loaded = load_preprocessed_gate(artifact)
    figures_before = tuple(plt.get_fignums())
    output = plot_preprocessing(
        loaded,
        artifact / "preprocessing.png",
        samples_per_line=40,
        dpi=70,
    )
    assert output.is_file()
    assert output.stat().st_size > 5_000
    assert output.read_bytes().startswith(b"\x89PNG")
    assert tuple(plt.get_fignums()) == figures_before


def test_trajectory_png_accepts_optimization_result_and_closes_figure(
    tmp_path, visualization_case
) -> None:
    _, track, _, result = visualization_case
    figures_before = tuple(plt.get_fignums())
    output = plot_trajectory(
        track,
        result,
        tmp_path / "trajectory.png",
        num_samples=41,
        dpi=70,
    )
    assert output.is_file()
    assert output.stat().st_size > 2_000
    assert output.read_bytes().startswith(b"\x89PNG")
    assert tuple(plt.get_fignums()) == figures_before


def test_scene_visualization_uses_physical_boundary_not_safe_inset(
    tmp_path, visualization_case, monkeypatch
) -> None:
    _, track, _, result = visualization_case

    def reject_safe_polygon(_time: float) -> np.ndarray:
        raise AssertionError("the scene renderer must not request the safe inset")

    monkeypatch.setattr(track.windows[0], "polygon_at", reject_safe_polygon)
    output = plot_trajectory(
        track,
        result,
        tmp_path / "physical_only.png",
        num_samples=21,
        dpi=50,
    )
    assert output.is_file()
    assert output.stat().st_size > 2_000


def test_quadrotor_body_frame_is_orthonormal_and_tilts_with_acceleration() -> None:
    basis = _quadrotor_basis(
        np.asarray((3.0, 1.0, 0.2)),
        np.asarray((2.0, -1.0, 0.5)),
    )
    np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1.0e-12)
    assert np.linalg.det(basis) == pytest.approx(1.0)
    assert not np.allclose(basis[:, 2], np.asarray((0.0, 0.0, 1.0)))

    aligned = _quadrotor_basis(
        np.asarray((1.0, 0.0, 0.0)),
        np.asarray((1.0, 0.0, -9.81)),
    )
    assert np.all(np.isfinite(aligned))
    np.testing.assert_allclose(aligned.T @ aligned, np.eye(3), atol=1.0e-12)


def test_window_exposes_physical_and_safe_boundaries_at_the_same_pose(
    visualization_case,
) -> None:
    gate, track, _, result = visualization_case
    window = track.windows[0]
    time = float(result.traversal_times[0])
    physical = window.physical_boundary_at(time)
    safe = window.polygon_at(time)
    assert physical is not None
    assert physical.shape == (len(gate.dense_boundary.vertices), 3)
    assert safe.shape == (len(gate.safe_polygon), 3)

    center, basis, scale, *_ = window.state_at(time)
    np.testing.assert_allclose(
        physical,
        center[None, :] + (basis @ (scale * gate.dense_boundary.vertices).T).T,
    )
    np.testing.assert_allclose(
        safe,
        center[None, :] + (basis @ (scale * gate.safe_polygon).T).T,
    )


def test_trajectory_csv_accepts_minco_and_has_derivative_columns(
    tmp_path, visualization_case
) -> None:
    _, _, trajectory, _ = visualization_case
    output = export_trajectory_csv(
        trajectory,
        tmp_path / "tables" / "trajectory.csv",
        num_samples=9,
    )
    assert output.is_file()
    assert output.stat().st_size > 200
    header = output.read_text(encoding="utf-8").splitlines()[0]
    assert header == "time,px,py,pz,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz,cx,cy,cz"
    values = np.loadtxt(output, delimiter=",", skiprows=1)
    assert values.shape == (9, 19)
    np.testing.assert_allclose(
        values[[0, -1], 1:4],
        [trajectory.start_state.position, trajectory.end_state.position],
        atol=2.0e-12,
    )


def test_dynamic_window_gif_is_nonempty_and_closes_every_frame(
    tmp_path, visualization_case
) -> None:
    _, track, _, result = visualization_case
    figures_before = tuple(plt.get_fignums())
    output = export_dynamic_window_gif(
        track,
        result,
        tmp_path / "animation" / "dynamic.gif",
        num_frames=4,
        trajectory_samples=31,
        fps=4.0,
        dpi=50,
    )
    assert output.is_file()
    assert output.stat().st_size > 2_000
    assert output.read_bytes().startswith((b"GIF87a", b"GIF89a"))
    assert len(imageio.mimread(output)) == 4
    assert tuple(plt.get_fignums()) == figures_before
