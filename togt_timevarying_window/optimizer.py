from __future__ import annotations

import time
from dataclasses import dataclass
from heapq import heappop, heappush
from math import ceil, inf
from typing import Literal

import numpy as np
from scipy.optimize import minimize

from .environment import DEFAULT_ORDER, WindowTrack
from .geometry import path_length
from .trajectory import PolynomialTrajectory, build_trajectory

PlannerMode = Literal["static", "ordered_dynamic", "shuffled_dynamic"]


@dataclass(frozen=True)
class DynaTOGTConfig:
    """DynaTOGT 优化器的超参数集合。

    这些参数同时控制离散 warm start、连续 L-BFGS-B 优化和轨迹可飞性惩罚。
    默认值偏向演示稳定性：先找到可穿越解，再用速度/加速度/jerk 指标约束轨迹质量。
    """

    max_speed: float = 2.0
    max_acceleration: float = 4.5
    max_jerk: float = 45.0
    min_segment_time: float = 0.35
    max_segment_time: float = 12.0
    samples_per_axis: int = 3
    wait_steps: int = 8
    discrete_dt: float = 0.25
    beam_width: int = 256
    time_weight: float = 1.0
    length_weight: float = 0.03
    acceleration_weight: float = 0.045
    jerk_weight: float = 0.002
    violation_weight: float = 40.0
    margin_weight: float = 0.05
    max_iter: int = 90


@dataclass
class DynaTOGTPlan:
    """优化完成后返回的完整计划结果。

    计划中同时保存穿越序列、每次穿越的时间/世界点/局部点、Hermite 轨迹、总代价和
    验证状态。可视化、CSV 导出和实验统计都直接消费这个对象。
    """

    mode: PlannerMode
    order: tuple[int, ...]
    success: bool
    crossing_times: np.ndarray
    crossing_points: np.ndarray
    crossing_locals: np.ndarray
    key_times: np.ndarray
    key_points: np.ndarray
    trajectory: PolynomialTrajectory
    total_cost: float
    optimization_time: float
    message: str = "ok"

    @property
    def chosen_order(self) -> list[str]:
        """把内部 0-based 窗口索引转换成展示用的 `G1/G2/...` 名称列表。"""
        return [f"G{i + 1}" for i in self.order]

    @property
    def path_length(self) -> float:
        """返回连续轨迹采样得到的路径长度。"""
        return self.trajectory.path_length

    @property
    def duration(self) -> float:
        """返回整条轨迹总时长。"""
        return self.trajectory.duration


class DynaTOGTOptimizer:
    """DynaTOGT 求解器。

    求解流程是：选择穿越顺序 -> 离散搜索 warm start -> L-BFGS-B 连续优化 ->
    解码为 Hermite 轨迹 -> 验证每个穿越点是否位于对应动态窗口 `G_i(t_i)` 内。
    """

    def __init__(self, config: DynaTOGTConfig | None = None):
        """创建优化器，并在未传入配置时使用默认超参数。"""
        self.config = DynaTOGTConfig() if config is None else config

    def solve(self, track: WindowTrack, mode: PlannerMode = "ordered_dynamic", order: tuple[int, ...] | None = None, optimize: bool = True) -> DynaTOGTPlan:
        """求解一个窗口穿越任务。

        `mode="static"` 会冻结窗口运动，用作静态 TOGT 对照；`ordered_dynamic` 按给定顺序
        穿越动态窗口；`shuffled_dynamic` 会先用 beam search 找一个一次性 permutation。
        `optimize=False` 时只返回离散 warm start，作为 DiscreteDynamic 基线。
        """
        start_time = time.perf_counter()
        dynamic = mode != "static"
        chosen_order = self._select_order(track, mode, order)
        warm = self._warm_start(track, chosen_order, dynamic=dynamic)
        if not optimize:
            plan = self._decode(track, chosen_order, warm, dynamic=dynamic, mode=mode, optimization_time=time.perf_counter() - start_time, message="discrete_only")
            return plan
        bounds = self._bounds(len(chosen_order))
        result = minimize(
            lambda x: self._objective(track, chosen_order, x, dynamic=dynamic),
            warm,
            method="L-BFGS-B",
            bounds=bounds,
            options={"maxiter": self.config.max_iter, "ftol": 1e-5, "maxls": 20},
        )
        x = result.x if result.success or np.isfinite(result.fun) else warm
        return self._decode(track, chosen_order, x, dynamic=dynamic, mode=mode, optimization_time=time.perf_counter() - start_time, message=str(result.message))

    def _select_order(self, track: WindowTrack, mode: PlannerMode, order: tuple[int, ...] | None) -> tuple[int, ...]:
        """根据模式选择实际使用的窗口索引序列。

        ordered/static 模式保留用户指定顺序，并允许重复窗口；shuffled_dynamic 模式用于
        一次性排列搜索对照，因此会忽略固定顺序并调用 `_beam_order`。
        """
        fixed = tuple(track.order if order is None else order)
        self._validate_order(track, fixed)
        if mode != "shuffled_dynamic":
            return fixed
        return self._beam_order(track)

    def _validate_order(self, track: WindowTrack, order: tuple[int, ...]) -> None:
        """检查穿越序列是否非空且索引都在窗口列表范围内。"""
        if not order:
            raise ValueError("order must contain at least one window")
        invalid = [idx for idx in order if idx < 0 or idx >= len(track.windows)]
        if invalid:
            raise ValueError(f"order contains invalid window indices {invalid}; valid range is 0..{len(track.windows) - 1}")

    def _beam_order(self, track: WindowTrack) -> tuple[int, ...]:
        """为 shuffled_dynamic 基线搜索一个窗口 permutation。

        该搜索只允许每个窗口出现一次，使用近似旅行时间作为代价，并用 beam width 控制
        队列规模。它是对照实验，不代表 ordered_dynamic 的主要任务语义。
        """
        count = len(track.windows)
        start = (0.0, 0, -1, tuple(), 0.0, track.start)
        queue = [start]
        serial = 0
        best: tuple[float, tuple[int, ...]] | None = None
        while queue:
            cost, _, last, order, t, point = heappop(queue)
            if len(order) == count:
                final = cost + np.linalg.norm(track.goal - point) / self.config.max_speed
                if best is None or final < best[0]:
                    best = (float(final), order)
                continue
            for idx in range(count):
                if idx in order:
                    continue
                center = track.windows[idx].center_at(t, dynamic=True)
                travel = float(np.linalg.norm(center - point) / self.config.max_speed)
                serial += 1
                heappush(queue, (cost + travel + 0.05 * abs(idx - last), serial, idx, order + (idx,), t + travel, center))
            if len(queue) > self.config.beam_width:
                queue = sorted(queue, key=lambda item: item[0])[: self.config.beam_width]
        return best[1] if best is not None else tuple(range(count))

    def _warm_start(self, track: WindowTrack, order: tuple[int, ...], dynamic: bool) -> np.ndarray:
        """构造连续优化的初始变量向量。

        变量格式为 `[每段持续时间..., 每个窗口的局部 z_u/z_v...]`。本函数先按离散时间
        步枚举未来若干候选穿越时刻和窗口内采样点，选择局部最短的可行候选，再把局部点
        近似反变换成优化变量 `z`。
        """
        durations = []
        locals_ = []
        point = track.start
        current_step = 0
        for gate_idx in order:
            window = track.windows[gate_idx]
            best = (inf, None, None, None)
            effective_speed = max(0.65 * self.config.max_speed, 1e-6)
            min_steps = max(1, int(ceil(np.linalg.norm(window.center_at(current_step * self.config.discrete_dt, dynamic=dynamic) - point) / (effective_speed * self.config.discrete_dt))))
            for step in range(current_step + min_steps, current_step + min_steps + self.config.wait_steps + 1):
                t = step * self.config.discrete_dt
                for candidate, local in window.candidates(t, samples_per_axis=self.config.samples_per_axis, dynamic=dynamic):
                    duration = max((step - current_step) * self.config.discrete_dt, self.config.min_segment_time)
                    score = duration + 0.04 * np.linalg.norm(candidate - point)
                    if score < best[0]:
                        best = (score, step, candidate, local)
            step = int(best[1])
            durations.append(max((step - current_step) * self.config.discrete_dt, self.config.min_segment_time))
            locals_.append(np.asarray(best[3], dtype=np.float64))
            point = np.asarray(best[2], dtype=np.float64)
            current_step = step
        final_duration = max(np.linalg.norm(track.goal - point) / max(0.65 * self.config.max_speed, 1e-6), self.config.min_segment_time)
        durations.append(final_duration)
        z_values = []
        crossing_times = np.cumsum(np.asarray(durations, dtype=np.float64))[:-1]
        for gate_idx, local, crossing_t in zip(order, locals_, crossing_times):
            polygon = track.windows[gate_idx].local_polygon_at(float(crossing_t), dynamic=dynamic)
            scale = np.maximum((polygon.max(axis=0) - polygon.min(axis=0)) * 0.5 * 0.94, 1e-6)
            center = polygon.mean(axis=0)
            z_values.extend(np.arctanh(np.clip((local - center) / scale, -0.92, 0.92)).tolist())
        return np.asarray(durations + z_values, dtype=np.float64)

    def _bounds(self, gate_count: int) -> list[tuple[float, float]]:
        """生成 L-BFGS-B 的变量上下界。

        前 `gate_count + 1` 个变量是段时长，后续变量是每个窗口穿越点的局部无约束坐标。
        """
        duration_bounds = [(self.config.min_segment_time, self.config.max_segment_time)] * (gate_count + 1)
        z_bounds = [(-2.2, 2.2)] * (2 * gate_count)
        return duration_bounds + z_bounds

    def _decode(
        self,
        track: WindowTrack,
        order: tuple[int, ...],
        x: np.ndarray,
        dynamic: bool,
        mode: PlannerMode,
        optimization_time: float,
        message: str,
    ) -> DynaTOGTPlan:
        """把优化变量解码为 `DynaTOGTPlan`。

        解码会根据累计持续时间得到每次穿越时刻，再用窗口的动态局部映射生成穿越点，
        最后把起点、穿越点、终点拼成关键点并构造 Hermite 连续轨迹。
        """
        durations = np.asarray(x[: len(order) + 1], dtype=np.float64)
        crossing_times = np.cumsum(durations)[:-1]
        z_values = np.asarray(x[len(order) + 1 :], dtype=np.float64).reshape(len(order), 2)
        points = []
        locals_ = []
        for gate_idx, t, z in zip(order, crossing_times, z_values):
            point, local = track.windows[gate_idx].point_from_unconstrained(z, float(t), dynamic=dynamic)
            points.append(point)
            locals_.append(local)
        key_times = np.concatenate([[0.0], np.cumsum(durations)])
        key_points = np.vstack([track.start, np.asarray(points), track.goal])
        trajectory = build_trajectory(key_times, key_points)
        cost = self._objective(track, order, x, dynamic=dynamic)
        success, msg = self._validate_plan(track, order, crossing_times, np.asarray(points), np.asarray(locals_), trajectory, dynamic=dynamic)
        return DynaTOGTPlan(
            mode=mode,
            order=order,
            success=success,
            crossing_times=crossing_times,
            crossing_points=np.asarray(points),
            crossing_locals=np.asarray(locals_),
            key_times=key_times,
            key_points=key_points,
            trajectory=trajectory,
            total_cost=float(cost),
            optimization_time=optimization_time,
            message=msg if not success else message,
        )

    def _objective(self, track: WindowTrack, order: tuple[int, ...], x: np.ndarray, dynamic: bool) -> float:
        """计算连续优化目标函数。

        目标由总时间、路径长度、最大加速度、平均 jerk、速度/加速度/jerk 超限惩罚以及
        窗口边界裕度惩罚组成。窗口包含性主要通过变量映射保证，目标里的 margin 项用于
        鼓励穿越点远离边框。
        """
        durations = np.asarray(x[: len(order) + 1], dtype=np.float64)
        if np.any(durations <= 0.0):
            return 1e9
        plan = self._decode_no_cost(track, order, x, dynamic=dynamic)
        traj = plan[0]
        locals_ = plan[1]
        crossing_times = plan[2]
        accel_excess = max(0.0, traj.max_acceleration - self.config.max_acceleration)
        jerk_excess = max(0.0, traj.mean_jerk - self.config.max_jerk)
        speed_excess = max(0.0, traj.max_speed - self.config.max_speed * 1.25)
        min_margin = min(track.windows[i].local_margin(local, float(t), dynamic=dynamic) for i, local, t in zip(order, locals_, crossing_times))
        margin_penalty = max(0.0, 0.04 - min_margin)
        return float(
            self.config.time_weight * durations.sum()
            + self.config.length_weight * traj.path_length
            + self.config.acceleration_weight * traj.max_acceleration
            + self.config.jerk_weight * traj.mean_jerk
            + self.config.violation_weight * (accel_excess**2 + 0.2 * jerk_excess**2 + speed_excess**2)
            + self.config.margin_weight * margin_penalty**2
        )

    def _decode_no_cost(self, track: WindowTrack, order: tuple[int, ...], x: np.ndarray, dynamic: bool) -> tuple[PolynomialTrajectory, np.ndarray, np.ndarray]:
        """轻量解码优化变量，供目标函数内部反复调用。

        与 `_decode` 相比，这里不递归计算 cost、不做完整验证，也不构造结果 dataclass，
        以降低 L-BFGS-B 多次评估目标时的开销。
        """
        durations = np.asarray(x[: len(order) + 1], dtype=np.float64)
        crossing_times = np.cumsum(durations)[:-1]
        z_values = np.asarray(x[len(order) + 1 :], dtype=np.float64).reshape(len(order), 2)
        points = []
        locals_ = []
        for gate_idx, t, z in zip(order, crossing_times, z_values):
            p, local = track.windows[gate_idx].point_from_unconstrained(z, float(t), dynamic=dynamic)
            points.append(p)
            locals_.append(local)
        key_times = np.concatenate([[0.0], np.cumsum(durations)])
        key_points = np.vstack([track.start, np.asarray(points), track.goal])
        return build_trajectory(key_times, key_points, samples_per_second=24.0), np.asarray(locals_), crossing_times

    def _validate_plan(
        self,
        track: WindowTrack,
        order: tuple[int, ...],
        crossing_times: np.ndarray,
        crossing_points: np.ndarray,
        crossing_locals: np.ndarray,
        trajectory: PolynomialTrajectory,
        dynamic: bool,
    ) -> tuple[bool, str]:
        """验证计划中每个穿越点是否真正落在对应动态窗口内。

        这里检查的是几何成功条件：点在窗口平面上，并且局部坐标位于当前多边形内部。
        轨迹动力学质量已经通过目标函数的速度/加速度/jerk 指标体现。
        """
        del trajectory
        for gate_idx, t, point, local in zip(order, crossing_times, crossing_points, crossing_locals):
            if not track.windows[gate_idx].contains(point, float(t), dynamic=dynamic, plane_tol=1e-4):
                return False, f"{track.windows[gate_idx].name} not traversed"
            if track.windows[gate_idx].local_margin(local, float(t), dynamic=dynamic) < -1e-6:
                return False, f"{track.windows[gate_idx].name} outside margin"
        return True, "ok"


def plan_metrics(plan: DynaTOGTPlan, track: WindowTrack, dynamic_eval: bool = True) -> dict[str, float | str | int | bool]:
    """把计划对象整理成实验汇总表的一行指标。

    `dynamic_eval=True` 表示即使某个计划来自静态规划，也按真实动态窗口重新计算裕度，
    这样 StaticTOGT、WaypointCenter 和 DynaTOGT 可以在同一标准下比较。
    """
    margins = [
        track.windows[idx].local_margin(local, float(t), dynamic=dynamic_eval)
        for idx, local, t in zip(plan.order, plan.crossing_locals, plan.crossing_times)
    ]
    return {
        "mode": plan.mode,
        "order": "->".join(plan.chosen_order),
        "success": bool(plan.success),
        "duration": plan.duration,
        "path_length": plan.path_length,
        "total_cost": plan.total_cost,
        "min_gate_margin": float(min(margins)) if margins else 0.0,
        "max_speed": plan.trajectory.max_speed,
        "max_acceleration": plan.trajectory.max_acceleration,
        "mean_jerk": plan.trajectory.mean_jerk,
        "optimization_time": plan.optimization_time,
    }
