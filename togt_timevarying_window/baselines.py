from __future__ import annotations

from dataclasses import replace

import numpy as np

from .environment import WindowTrack
from .optimizer import DynaTOGTConfig, DynaTOGTOptimizer, DynaTOGTPlan, plan_metrics


def solve_baseline(name: str, track: WindowTrack, config: DynaTOGTConfig | None = None) -> DynaTOGTPlan:
    """按基线名称调用对应求解策略。

    DynaTOGT 是完整动态连续优化；DiscreteDynamic 只使用离散 warm start；StaticTOGT
    在静态窗口上规划再用动态窗口评估；WaypointCenter 只穿越窗口中心；ShuffledDynaTOGT
    用 permutation 搜索做顺序对照。
    """
    optimizer = DynaTOGTOptimizer(config)
    if name == "DynaTOGT":
        return optimizer.solve(track, mode="ordered_dynamic", optimize=True)
    if name == "DiscreteDynamic":
        return optimizer.solve(track, mode="ordered_dynamic", optimize=False)
    if name == "StaticTOGT":
        plan = optimizer.solve(track, mode="static", optimize=True)
        return replace(plan, success=evaluate_dynamic_success(plan, track), message="static_planned_dynamic_eval")
    if name == "WaypointCenter":
        return waypoint_center_plan(track, optimizer)
    if name == "ShuffledDynaTOGT":
        return optimizer.solve(track, mode="shuffled_dynamic", optimize=True)
    raise ValueError(f"unknown baseline: {name}")


def waypoint_center_plan(track: WindowTrack, optimizer: DynaTOGTOptimizer) -> DynaTOGTPlan:
    """生成“只追窗口中心”的简单 waypoint 基线。

    该基线不优化窗口内部穿越点，而是把每次穿越点固定为当前窗口中心。它用于展示
    DynaTOGT 相比中心点 waypoint 能更好利用窗口几何空间。
    """
    order = tuple(track.order)
    durations = []
    point = track.start
    t = 0.0
    for idx in order:
        center = track.windows[idx].center_at(t, dynamic=True)
        duration = max(np.linalg.norm(center - point) / optimizer.config.max_speed, optimizer.config.min_segment_time)
        durations.append(float(duration))
        t += duration
        point = track.windows[idx].center_at(t, dynamic=True)
    durations.append(max(np.linalg.norm(track.goal - point) / optimizer.config.max_speed, optimizer.config.min_segment_time))
    z = np.zeros(2 * len(order), dtype=np.float64)
    x = np.concatenate([np.asarray(durations), z])
    plan = optimizer._decode(track, order, x, dynamic=True, mode="ordered_dynamic", optimization_time=0.0, message="waypoint_center")
    return replace(plan, success=evaluate_dynamic_success(plan, track), message="waypoint_center")


def evaluate_dynamic_success(plan: DynaTOGTPlan, track: WindowTrack) -> bool:
    """用真实动态窗口重新判断计划是否穿越成功。

    静态基线可能在静态几何上满足约束，但在窗口真实运动后失败；因此实验统计统一调用
    这个函数做动态评估。`plane_tol=0.08` 给数值轨迹和基线近似留出少量容忍。
    """
    for idx, t, point in zip(plan.order, plan.crossing_times, plan.crossing_points):
        if not track.windows[idx].contains(point, float(t), dynamic=True, plane_tol=0.08):
            return False
    return True


def baseline_metrics(name: str, plan: DynaTOGTPlan, track: WindowTrack, scenario: str) -> dict[str, object]:
    """为一个基线计划生成 summary.csv 所需的指标字典。

    除了通用 `plan_metrics`，这里还追加 baseline/scenario 字段，并重新计算动态成功状态
    与考虑平面误差后的最小窗口裕度。
    """
    metrics = plan_metrics(plan, track, dynamic_eval=True)
    metrics["baseline"] = name
    metrics["scenario"] = scenario
    metrics["success"] = evaluate_dynamic_success(plan, track)
    margins = []
    for idx, t, point in zip(plan.order, plan.crossing_times, plan.crossing_points):
        local, plane_error = track.windows[idx].point_to_local(point, float(t), dynamic=True)
        margins.append(track.windows[idx].local_margin(local, float(t), dynamic=True) - plane_error)
    metrics["min_gate_margin"] = float(min(margins)) if margins else 0.0
    return metrics
