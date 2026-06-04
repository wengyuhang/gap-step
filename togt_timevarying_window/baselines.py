from __future__ import annotations

from dataclasses import replace

import numpy as np

from .environment import WindowTrack
from .optimizer import DynaTOGTConfig, DynaTOGTOptimizer, DynaTOGTPlan, plan_metrics


def solve_baseline(name: str, track: WindowTrack, config: DynaTOGTConfig | None = None) -> DynaTOGTPlan:
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
    for idx, t, point in zip(plan.order, plan.crossing_times, plan.crossing_points):
        if not track.windows[idx].contains(point, float(t), dynamic=True, plane_tol=0.08):
            return False
    return True


def baseline_metrics(name: str, plan: DynaTOGTPlan, track: WindowTrack, scenario: str) -> dict[str, object]:
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
