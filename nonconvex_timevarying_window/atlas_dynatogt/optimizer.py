from __future__ import annotations

import time
from dataclasses import dataclass
from math import ceil
from typing import Literal

import numpy as np
from scipy.optimize import minimize

from .environment import NonConvexWindowTrack
from .geometry import path_length

PlannerMode = Literal["ordered_dynamic"]


@dataclass
class PolynomialTrajectory:
    """Hermite 多项式轨迹采样结果。

    Hermite 插值只需要关键点和关键点切向速度。这里用相邻关键点差分估计切向速度，
    再用三次多项式连接每一段；轨迹会经过所有窗口穿越点。
    """

    key_times: np.ndarray
    key_points: np.ndarray
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    jerks: np.ndarray
    yaws: np.ndarray

    @property
    def duration(self) -> float:
        """总时长。"""
        return float(self.key_times[-1])

    @property
    def path_length(self) -> float:
        """采样折线长度，作为真实曲线长度的近似。"""
        return path_length(self.positions)

    @property
    def max_speed(self) -> float:
        """最大速度模长。"""
        return float(np.linalg.norm(self.velocities, axis=1).max())

    @property
    def max_acceleration(self) -> float:
        """最大加速度模长。"""
        return float(np.linalg.norm(self.accelerations, axis=1).max())

    @property
    def mean_jerk(self) -> float:
        """平均 jerk 模长；jerk 是加速度对时间的一阶导。"""
        return float(np.linalg.norm(self.jerks, axis=1).mean())


def build_trajectory(key_times: np.ndarray, key_points: np.ndarray, samples_per_second: float = 40.0) -> PolynomialTrajectory:
    """由关键点构造连续轨迹并密集采样。

    `np.gradient` 用有限差分从位置采样依次估计速度、加速度和 jerk。
    """
    key_times = np.asarray(key_times, dtype=np.float64)
    key_points = np.asarray(key_points, dtype=np.float64)
    count = max(2, int(np.ceil(key_times[-1] * samples_per_second)) + 1)
    times = np.linspace(0.0, float(key_times[-1]), count)
    positions = np.asarray([hermite_position(key_times, key_points, t) for t in times])
    velocities = np.gradient(positions, times, axis=0, edge_order=1)
    accelerations = np.gradient(velocities, times, axis=0, edge_order=1)
    jerks = np.gradient(accelerations, times, axis=0, edge_order=1)
    yaws = np.arctan2(velocities[:, 1], velocities[:, 0] + 1e-9)
    return PolynomialTrajectory(key_times, key_points, times, positions, velocities, accelerations, jerks, yaws)


def hermite_position(key_times: np.ndarray, key_points: np.ndarray, t: float) -> np.ndarray:
    """分段三次 Hermite 插值。

    公式为：
    p(u)=h00*p0+h10*h*m0+h01*p1+h11*h*m1
    其中 u=(t-t0)/(t1-t0)，h 是本段时间长度，m0/m1 是两端切向速度。
    """
    idx = int(np.searchsorted(key_times, t, side="right") - 1)
    idx = int(np.clip(idx, 0, len(key_times) - 2))
    t0 = float(key_times[idx])
    t1 = float(key_times[idx + 1])
    h = max(t1 - t0, 1e-9)
    u = float(np.clip((t - t0) / h, 0.0, 1.0))
    p0 = key_points[idx]
    p1 = key_points[idx + 1]
    m0 = 0.55 * _tangent(key_times, key_points, idx)
    m1 = 0.55 * _tangent(key_times, key_points, idx + 1)
    h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
    h10 = u**3 - 2.0 * u**2 + u
    h01 = -2.0 * u**3 + 3.0 * u**2
    h11 = u**3 - u**2
    return h00 * p0 + h10 * h * m0 + h01 * p1 + h11 * h * m1


def _tangent(times: np.ndarray, points: np.ndarray, idx: int) -> np.ndarray:
    """用相邻点差分估计关键点切向速度。"""
    if idx == 0:
        return (points[1] - points[0]) / max(times[1] - times[0], 1e-9)
    if idx == len(points) - 1:
        return (points[-1] - points[-2]) / max(times[-1] - times[-2], 1e-9)
    return (points[idx + 1] - points[idx - 1]) / max(times[idx + 1] - times[idx - 1], 1e-9)


@dataclass(frozen=True)
class AtlasDynaTOGTConfig:
    """优化超参数。代价为 time + length + dynamics violation + boundary margin penalty。"""

    max_speed: float = 2.0
    max_acceleration: float = 4.5
    max_jerk: float = 45.0
    min_segment_time: float = 0.35
    max_segment_time: float = 12.0
    samples_per_chart: int = 1
    wait_steps: int = 8
    discrete_dt: float = 0.25
    chart_multistarts: int = 2
    chart_candidates_per_gate: int = 5
    time_weight: float = 1.0
    length_weight: float = 0.03
    acceleration_weight: float = 0.045
    jerk_weight: float = 0.002
    violation_weight: float = 40.0
    margin_weight: float = 0.08
    max_iter: int = 80


@dataclass
class AtlasDynaTOGTPlan:
    """一次规划结果：固定 chart ids，连续变量给出时间和 chart 内点。"""

    mode: PlannerMode
    order: tuple[int, ...]
    chart_ids: tuple[int, ...]
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
        """展示用 `G1 -> G6 ...`。"""
        return [f"G{i + 1}" for i in self.order]

    @property
    def path_length(self) -> float:
        """轨迹长度。"""
        return self.trajectory.path_length

    @property
    def duration(self) -> float:
        """总飞行时间。"""
        return self.trajectory.duration


class AtlasDynaTOGTOptimizer:
    """非凸 atlas 优化器。

    优化变量 x=[durations, z_0, ..., z_n]。chart_id 是离散选择，由 warm start
    beam 搜索给出；L-BFGS-B 只优化连续时间和 softmax chart 坐标。
    """

    def __init__(self, config: AtlasDynaTOGTConfig | None = None):
        """创建优化器。"""
        self.config = AtlasDynaTOGTConfig() if config is None else config

    def solve(self, track: NonConvexWindowTrack, mode: PlannerMode = "ordered_dynamic", order: tuple[int, ...] | None = None) -> AtlasDynaTOGTPlan:
        """求解一个场景。"""
        if mode != "ordered_dynamic":
            raise ValueError("only ordered_dynamic is supported")
        t0 = time.perf_counter()
        dynamic = True
        order = self._select_order(track, order)
        starts = self._warm_starts(track, order, dynamic)
        bounds, best = self._bounds(len(order)), None
        for x0, charts in starts:
            res = minimize(
                lambda x, c=charts: self._objective(track, order, c, x, dynamic),
                x0,
                method="L-BFGS-B",
                bounds=bounds,
                options={"maxiter": self.config.max_iter, "ftol": 1e-5, "maxls": 20},
            )
            x = res.x if res.success or np.isfinite(res.fun) else x0
            plan = self._decode(track, order, charts, x, dynamic, mode, time.perf_counter() - t0, str(res.message))
            if best is None or (plan.success and not best.success) or (plan.success == best.success and plan.total_cost < best.total_cost):
                best = plan
        assert best is not None
        return best

    def _select_order(self, track: NonConvexWindowTrack, order: tuple[int, ...] | None) -> tuple[int, ...]:
        """保留用户指定顺序；默认使用场景中的固定顺序。"""
        fixed = tuple(track.order if order is None else order)
        if not fixed or any(i < 0 or i >= len(track.windows) for i in fixed):
            raise ValueError(f"invalid order {fixed}")
        return fixed

    def _warm_starts(self, track: NonConvexWindowTrack, order: tuple[int, ...], dynamic: bool) -> list[tuple[np.ndarray, tuple[int, ...]]]:
        """离散 warm start：枚举未来时间步和 chart 候选，保留若干低分状态。"""
        states = [(0.0, [], [], [], track.start, 0)]
        speed = max(0.65 * self.config.max_speed, 1e-6)
        for gate_idx in order:
            nxt = []
            for score, ds, charts, locals_, point, step0 in states:
                w = track.windows[gate_idx]
                distance = np.linalg.norm(w.center_at(step0 * self.config.discrete_dt, dynamic) - point)
                min_step = max(1, int(ceil(distance / (speed * self.config.discrete_dt))))
                for step in range(step0 + min_step, step0 + min_step + self.config.wait_steps + 1):
                    t = step * self.config.discrete_dt
                    for cid, world, local in w.candidates(t, self.config.samples_per_chart, dynamic):
                        dt = max((step - step0) * self.config.discrete_dt, self.config.min_segment_time)
                        path_cost = 0.04 * np.linalg.norm(world - point)
                        margin_bonus = 0.03 * max(0.0, w.local_margin(local, t, dynamic))
                        s = score + dt + path_cost - margin_bonus
                        nxt.append((s, ds + [float(dt)], charts + [int(cid)], locals_ + [local], world, step))
            states = sorted(nxt, key=lambda x: x[0])[: self.config.chart_multistarts * self.config.chart_candidates_per_gate]
            states = states[: self.config.chart_multistarts]
        return [self._state_to_x(track, order, state, speed, dynamic) for state in states]

    def _state_to_x(self, track: NonConvexWindowTrack, order: tuple[int, ...], state, speed: float, dynamic: bool) -> tuple[np.ndarray, tuple[int, ...]]:
        """beam 状态 -> L-BFGS-B 初始向量。"""
        _, ds, charts, locals_, point, _ = state
        ds = ds + [float(max(np.linalg.norm(track.goal - point) / speed, self.config.min_segment_time))]
        times, z = np.cumsum(np.asarray(ds))[:-1], []
        for gate, cid, local, t in zip(order, charts, locals_, times):
            z += track.windows[gate].z_from_local(cid, local, float(t), dynamic).tolist()
        return np.asarray(ds + z, dtype=np.float64), tuple(charts)

    def _bounds(self, n: int) -> list[tuple[float, float]]:
        """前 n+1 个变量是时间段，后 2n 个变量是 chart logits。"""
        return [(self.config.min_segment_time, self.config.max_segment_time)] * (n + 1) + [(-4.0, 4.0)] * (2 * n)

    def _unpack(self, track: NonConvexWindowTrack, order: tuple[int, ...], charts: tuple[int, ...], x: np.ndarray, dynamic: bool, sps: float = 40.0):
        """x -> crossing_times, locals, world points, Hermite trajectory。"""
        ds = np.asarray(x[: len(order) + 1], dtype=np.float64)
        times = np.cumsum(ds)[:-1]
        zvals = np.asarray(x[len(order) + 1 :], dtype=np.float64).reshape(len(order), 2)
        points = []
        locals_ = []
        for gate, chart, z, t in zip(order, charts, zvals, times):
            point, local = track.windows[gate].point_from_chart_z(chart, z, float(t), dynamic)
            points.append(point)
            locals_.append(local)
        pts = np.asarray(points)
        locals_arr = np.asarray(locals_)
        key_times = np.concatenate([[0.0], np.cumsum(ds)])
        key_points = np.vstack([track.start, pts, track.goal])
        return times, pts, locals_arr, key_times, key_points, build_trajectory(key_times, key_points, samples_per_second=sps)

    def _objective(self, track: NonConvexWindowTrack, order: tuple[int, ...], charts: tuple[int, ...], x: np.ndarray, dynamic: bool) -> float:
        """J=sum dt + w_l L + w_a a_max + w_j jerk + violation + margin。"""
        if np.any(np.asarray(x[: len(order) + 1]) <= 0):
            return 1e9
        times, _, locals_, _, _, traj = self._unpack(track, order, charts, x, dynamic, sps=24.0)
        acc_excess = max(0.0, traj.max_acceleration - self.config.max_acceleration)
        jerk_excess = max(0.0, traj.mean_jerk - self.config.max_jerk)
        speed_excess = max(0.0, traj.max_speed - 1.25 * self.config.max_speed)
        excess = acc_excess**2 + 0.2 * jerk_excess**2 + speed_excess**2
        margin = min(track.windows[g].local_margin(q, float(t), dynamic) for g, q, t in zip(order, locals_, times))
        cost = self.config.time_weight * np.sum(x[: len(order) + 1])
        cost += self.config.length_weight * traj.path_length
        cost += self.config.acceleration_weight * traj.max_acceleration
        cost += self.config.jerk_weight * traj.mean_jerk
        cost += self.config.violation_weight * excess
        cost += self.config.margin_weight * max(0.0, 0.035 - margin) ** 2
        return float(cost)

    def _decode(self, track: NonConvexWindowTrack, order: tuple[int, ...], charts: tuple[int, ...], x: np.ndarray, dynamic: bool, mode: PlannerMode, optimization_time: float, message: str) -> AtlasDynaTOGTPlan:
        """连续变量 -> 对外计划对象。"""
        times, pts, locals_, key_times, key_points, traj = self._unpack(track, order, charts, x, dynamic)
        ok, msg = self._validate_plan(track, order, charts, times, pts, locals_, dynamic)
        cost = self._objective(track, order, charts, x, dynamic)
        final_message = message if ok else msg
        return AtlasDynaTOGTPlan(mode, order, charts, ok, times, pts, locals_, key_times, key_points, traj, cost, optimization_time, final_message)

    def _validate_plan(self, track: NonConvexWindowTrack, order: tuple[int, ...], charts: tuple[int, ...], times: np.ndarray, pts: np.ndarray, locals_: np.ndarray, dynamic: bool) -> tuple[bool, str]:
        """几何成功：contains=True，margin>0，且 local 仍在指定 chart 内。"""
        for gate, cid, t, p, q in zip(order, charts, times, pts, locals_):
            w = track.windows[gate]
            in_window = w.contains(p, float(t), dynamic, 1e-4)
            has_margin = w.local_margin(q, float(t), dynamic) > 0
            in_chart = w.chart_contains(cid, q, float(t), dynamic)
            if not (in_window and has_margin and in_chart):
                return False, f"{w.name} failed non-convex crossing"
        return True, "ok"


def plan_metrics(plan: AtlasDynaTOGTPlan, track: NonConvexWindowTrack, dynamic_eval: bool = True) -> dict[str, float | str | int | bool]:
    """计划 -> summary.csv 指标。"""
    margins = [track.windows[i].local_margin(q, float(t), dynamic=dynamic_eval) for i, q, t in zip(plan.order, plan.crossing_locals, plan.crossing_times)]
    return {
        "mode": plan.mode,
        "order": "->".join(plan.chosen_order),
        "chart_ids": "->".join(str(i) for i in plan.chart_ids),
        "success": bool(plan.success),
        "duration": plan.duration,
        "path_length": plan.path_length,
        "total_cost": plan.total_cost,
        "min_boundary_margin": float(min(margins)) if margins else 0.0,
        "max_speed": plan.trajectory.max_speed,
        "max_acceleration": plan.trajectory.max_acceleration,
        "mean_jerk": plan.trajectory.mean_jerk,
        "optimization_time": plan.optimization_time,
    }
