from __future__ import annotations

import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt import simulation_render
from nonconvex_timevarying_window.sc_dynatogt.simulation_render import (
    SimulationRenderConfig,
    _look_at,
    _resample_closed_boundary,
)


def test_dense_render_boundary_is_reduced_and_true_corners_survive() -> None:
    edge = np.linspace(-1.0, 1.0, 101, endpoint=False)
    boundary = np.vstack(
        (
            np.column_stack((edge, np.full_like(edge, -1.0))),
            np.column_stack((np.full_like(edge, 1.0), edge)),
            np.column_stack((-edge, np.full_like(edge, 1.0))),
            np.column_stack((np.full_like(edge, -1.0), -edge)),
        )
    )
    sampled = _resample_closed_boundary(boundary, maximum_segments=32)
    assert len(sampled) <= 32
    for corner in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
        assert np.linalg.norm(sampled - np.asarray(corner), axis=1).min() < 1.0e-10


def test_look_at_returns_rigid_camera_pose_facing_target() -> None:
    eye = np.asarray((3.0, -4.0, 2.0))
    target = np.asarray((1.0, 2.0, 1.0))
    pose = _look_at(eye, target)
    np.testing.assert_allclose(pose[:3, 3], eye)
    np.testing.assert_allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1.0e-12)
    expected = (target - eye) / np.linalg.norm(target - eye)
    np.testing.assert_allclose(-pose[:3, 2], expected, atol=1.0e-12)
    assert np.linalg.det(pose[:3, :3]) == pytest.approx(1.0)


@pytest.mark.parametrize(
    "kwargs",
    (
        {"width": 200},
        {"frame_count": 1},
        {"fps": 0.0},
        {"gate_tube_radius": 0.0},
        {"maximum_gate_segments": 8},
        {"field_of_view_degrees": 15.0},
    ),
)
def test_simulation_render_config_rejects_invalid_values(kwargs) -> None:
    with pytest.raises(ValueError):
        SimulationRenderConfig(**kwargs)


def test_simulation_render_cli_uses_independent_default_output(monkeypatch) -> None:
    captured = {}

    def fake_render(summary, output, *, config):
        captured.update(summary=summary, output=output, config=config)
        return {"overview_png": "preview.png"}

    monkeypatch.setattr(simulation_render, "render_diverse_summary", fake_render)
    assert simulation_render.main(["--no-video", "--frames", "2"]) == 0
    assert str(captured["output"]).endswith(
        "results/diverse_paper_irregular_closed_airsim_style"
    )
    assert captured["config"].render_video is False
