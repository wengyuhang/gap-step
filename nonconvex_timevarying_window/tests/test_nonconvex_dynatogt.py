from __future__ import annotations

import subprocess
import sys

import numpy as np

from nonconvex_timevarying_window.environment import DEFAULT_ORDER, canonical_track
from nonconvex_timevarying_window.geometry import ChartAtlas, make_region, triangle_area
from nonconvex_timevarying_window.optimizer import AtlasDynaTOGTConfig, AtlasDynaTOGTOptimizer


def test_nonconvex_point_contains_rejects_notch():
    """凹陷区域内的缺口点必须被判为窗口外。"""
    region = make_region("u_notch")
    assert region.contains(np.asarray([0.0, -0.35]))
    assert not region.contains(np.asarray([0.0, 0.30]))
    assert region.margin(np.asarray([0.0, -0.35])) > 0.0
    assert region.margin(np.asarray([0.0, 0.30])) < 0.0


def test_ear_clipping_area_and_centroids():
    """三角剖分面积应覆盖原非凸区域，三角重心应位于区域内。"""
    for kind in ["crescent", "u_notch", "starfish", "l_shape", "wavy_bean", "asymmetric_gear"]:
        region = make_region(kind)
        tris = region.triangles
        assert len(tris) >= 1
        assert np.isclose(sum(triangle_area(tri) for tri in tris), region.area, rtol=1e-7, atol=1e-7)
        assert all(region.contains(tri.mean(axis=0), tol=1e-7) for tri in tris)


def test_chart_mapping_stays_inside_and_has_positive_barycentric_weights():
    """任意无约束变量都应映射到对应 chart 和非凸区域内部。"""
    region = make_region("starfish")
    atlas = ChartAtlas.from_region(region)
    for chart_id in [0, len(atlas.charts) // 2, len(atlas.charts) - 1]:
        chart = atlas.charts[chart_id]
        for z in [np.asarray([0.0, 0.0]), np.asarray([2.0, -1.0]), np.asarray([-2.5, 1.5])]:
            local = chart.point_from_z(z)
            bary = chart.barycentric_from_z(z)
            assert np.all(bary > 0.0)
            assert chart.contains(local)
            assert region.contains(local)


def test_atlas_covers_region_samples():
    """区域内部采样点应至少落入一个 chart。"""
    region = make_region("wavy_bean")
    atlas = ChartAtlas.from_region(region)
    for point in region.sample_points(samples_per_axis=4):
        assert atlas.contains_in_chart(atlas.chart_for_point(point), point)


def test_atlas_dynatogt_keeps_order_and_crosses_nonconvex_windows():
    """主算法应保持默认指定顺序，并穿越真实动态非凸窗口。"""
    track = canonical_track()
    plan = AtlasDynaTOGTOptimizer(AtlasDynaTOGTConfig(max_iter=2, chart_multistarts=1)).solve(track, mode="ordered_dynamic")
    assert plan.success
    assert plan.order == DEFAULT_ORDER
    assert len(plan.chart_ids) == len(DEFAULT_ORDER)
    for idx, chart_id, t, point, local in zip(plan.order, plan.chart_ids, plan.crossing_times, plan.crossing_points, plan.crossing_locals):
        window = track.windows[idx]
        assert window.contains(point, float(t), dynamic=True)
        assert window.local_margin(local, float(t), dynamic=True) > 0.0
        assert window.chart_contains(chart_id, local, float(t), dynamic=True)


def test_atlas_dynatogt_allows_repeated_order():
    """主算法应支持重复穿越任意窗口序列。"""
    track = canonical_track()
    repeated = (0, 5, 0, 2, 1, 4, 3, 1)
    plan = AtlasDynaTOGTOptimizer(AtlasDynaTOGTConfig(max_iter=1, chart_multistarts=1)).solve(track, mode="ordered_dynamic", order=repeated)
    assert plan.success
    assert plan.order == repeated
    assert len(plan.crossing_times) == len(repeated)
    assert np.all(np.diff(plan.crossing_times) > 0.0)


def test_experiment_smoke_cli_outputs_files(tmp_path):
    """验证唯一 CLI 入口 experiments 可运行并产出结果文件。"""
    result_dir = tmp_path / "results"
    subprocess.run(
        [sys.executable, "-m", "nonconvex_timevarying_window.experiments", "--suite", "smoke", "--outdir", str(result_dir), "--frames", "2"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert (result_dir / "smoke" / "summary.csv").is_file()
    assert list((result_dir / "smoke" / "trajectories").glob("*.csv"))
    assert list((result_dir / "smoke" / "figures").glob("*.png"))
    assert list((result_dir / "smoke" / "gifs").glob("*.gif"))
