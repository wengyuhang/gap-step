from __future__ import annotations

import subprocess
import sys

import numpy as np

from togt_timevarying_window.environment import DEFAULT_ORDER, canonical_track
from togt_timevarying_window.optimizer import DynaTOGTConfig, DynaTOGTOptimizer


def test_dynamic_window_changes_center_orientation_and_scale():
    """验证动态窗口的中心、姿态和尺度都会随时间变化。"""
    track = canonical_track()
    window = track.windows[1]
    assert not np.allclose(window.center_at(0.0), window.center_at(1.0))
    assert not np.allclose(window.angles_at(0.0), window.angles_at(1.0))
    assert not np.allclose(window.scale_at(0.0), window.scale_at(1.0))


def test_unconstrained_mapping_stays_inside_dynamic_window():
    """验证无约束变量映射出的穿越点始终位于动态窗口内部。"""
    track = canonical_track()
    window = track.windows[2]
    for z in [np.asarray([0.0, 0.0]), np.asarray([1.8, -1.2]), np.asarray([-2.0, 2.0])]:
        point, local = window.point_from_unconstrained(z, 2.3, dynamic=True)
        assert window.contains(point, 2.3, dynamic=True)
        assert window.local_margin(local, 2.3, dynamic=True) >= -1e-7


def test_dynatogt_keeps_default_specified_order_and_crosses_windows():
    """验证 ordered_dynamic 保持默认指定顺序，并真实穿过每个动态窗口。"""
    track = canonical_track()
    plan = DynaTOGTOptimizer(DynaTOGTConfig(max_iter=12)).solve(track, mode="ordered_dynamic")
    assert plan.success
    assert plan.order == DEFAULT_ORDER
    assert plan.chosen_order == ["G1", "G6", "G3", "G2", "G5", "G4"]
    for idx, t, point in zip(plan.order, plan.crossing_times, plan.crossing_points):
        assert track.windows[idx].contains(point, float(t), dynamic=True)


def test_dynatogt_allows_repeated_arbitrary_window_sequence():
    """验证 DynaTOGT 支持任意顺序和重复穿越同一个窗口。"""
    track = canonical_track()
    repeated_order = (0, 5, 0, 2, 1, 4, 3, 1)
    plan = DynaTOGTOptimizer(DynaTOGTConfig(max_iter=8)).solve(track, mode="ordered_dynamic", order=repeated_order)
    assert plan.success
    assert plan.order == repeated_order
    assert plan.chosen_order == ["G1", "G6", "G1", "G3", "G2", "G5", "G4", "G2"]
    assert len(plan.crossing_times) == len(repeated_order)
    assert np.all(np.diff(plan.crossing_times) > 0.0)
    for idx, t, point in zip(plan.order, plan.crossing_times, plan.crossing_points):
        assert track.windows[idx].contains(point, float(t), dynamic=True)


def test_polynomial_trajectory_hits_crossing_points_exactly():
    """验证 Hermite 轨迹在穿越时刻精确经过优化器给出的穿越点。"""
    track = canonical_track()
    plan = DynaTOGTOptimizer(DynaTOGTConfig(max_iter=8)).solve(track, mode="ordered_dynamic")
    for t, point in zip(plan.crossing_times, plan.crossing_points):
        assert np.allclose(plan.trajectory.position_at(float(t)), point, atol=1e-6)


def test_demo_export_and_experiment_smoke(tmp_path):
    """验证 demo、导出演示和 smoke 实验三个命令行入口能正常运行并产出文件。"""
    demo = subprocess.run(
        [sys.executable, "-m", "togt_timevarying_window.demo", "--scenario", "canonical", "--mode", "ordered_dynamic", "--max-iter", "8"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert "success=True" in demo.stdout
    export_dir = tmp_path / "demo"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "togt_timevarying_window.export_demo",
            "--scenario",
            "canonical",
            "--mode",
            "ordered_dynamic",
            "--outdir",
            str(export_dir),
            "--frames",
            "2",
            "--max-iter",
            "8",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    assert list((export_dir / "trajectories").glob("*.csv"))
    assert list((export_dir / "figures").glob("*.png"))
    assert list((export_dir / "gifs").glob("*.gif"))
    result_dir = tmp_path / "results"
    subprocess.run(
        [sys.executable, "-m", "togt_timevarying_window.experiments", "--suite", "smoke", "--outdir", str(result_dir), "--frames", "2"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert (result_dir / "smoke" / "summary.csv").is_file()
