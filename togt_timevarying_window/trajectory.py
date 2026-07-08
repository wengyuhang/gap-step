from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import path_length


@dataclass
class PolynomialTrajectory:
    """Hermite 多项式轨迹的采样结果。

    `key_times/key_points` 是轨迹必须精确经过的关键时刻和位置；其它数组是在连续轨迹上
    密集采样得到的位置、速度、加速度、jerk 和 yaw，用于导出、可视化和优化指标。
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
        """返回轨迹结束时间，也就是最后一个关键时刻。"""
        return float(self.key_times[-1])

    @property
    def path_length(self) -> float:
        """返回采样轨迹的折线长度。"""
        return path_length(self.positions)

    @property
    def max_speed(self) -> float:
        """返回采样点上的最大速度模长。"""
        return float(np.linalg.norm(self.velocities, axis=1).max())

    @property
    def max_acceleration(self) -> float:
        """返回采样点上的最大加速度模长。"""
        return float(np.linalg.norm(self.accelerations, axis=1).max())

    @property
    def mean_jerk(self) -> float:
        """返回采样点上的平均 jerk 模长，作为轨迹平滑性指标。"""
        return float(np.linalg.norm(self.jerks, axis=1).mean())

    def position_at(self, t: float) -> np.ndarray:
        """查询任意时刻的 Hermite 插值位置。

        测试会用它确认轨迹在每个 `crossing_time` 精确命中优化器给出的穿越点。
        """
        return hermite_position(self.key_times, self.key_points, t)


def build_trajectory(key_times: np.ndarray, key_points: np.ndarray, samples_per_second: float = 40.0) -> PolynomialTrajectory:
    """根据关键点构造连续 Hermite 轨迹并进行密集采样。

    本项目没有复刻原 TOGT 的 MINCO 轨迹，而是使用轻量 Hermite 插值：关键点保证精确
    命中，速度/加速度/jerk 通过数值梯度估计，足以支撑动态窗口任务原型的比较实验。
    """
    key_times = np.asarray(key_times, dtype=np.float64)
    key_points = np.asarray(key_points, dtype=np.float64)
    sample_count = max(2, int(np.ceil(key_times[-1] * samples_per_second)) + 1)
    times = np.linspace(0.0, float(key_times[-1]), sample_count)
    positions = np.asarray([hermite_position(key_times, key_points, t) for t in times], dtype=np.float64)
    velocities = np.gradient(positions, times, axis=0, edge_order=1)
    accelerations = np.gradient(velocities, times, axis=0, edge_order=1)
    jerks = np.gradient(accelerations, times, axis=0, edge_order=1)
    yaws = np.arctan2(velocities[:, 1], velocities[:, 0] + 1e-9)
    return PolynomialTrajectory(key_times, key_points, times, positions, velocities, accelerations, jerks, yaws)


def hermite_position(key_times: np.ndarray, key_points: np.ndarray, t: float) -> np.ndarray:
    """计算时刻 `t` 的分段三次 Hermite 插值位置。

    函数先定位 `t` 所在的关键点区间，再用相邻关键点估计切向速度，最后应用标准 Hermite
    基函数得到平滑位置。`t` 超出区间时会被夹到首尾段。
    """
    idx = int(np.searchsorted(key_times, t, side="right") - 1)
    idx = int(np.clip(idx, 0, len(key_times) - 2))
    t0 = float(key_times[idx])
    t1 = float(key_times[idx + 1])
    h = max(t1 - t0, 1e-9)
    u = float(np.clip((t - t0) / h, 0.0, 1.0))
    p0 = key_points[idx]
    p1 = key_points[idx + 1]
    tangent_scale = 0.55
    m0 = tangent_scale * _tangent(key_times, key_points, idx)
    m1 = tangent_scale * _tangent(key_times, key_points, idx + 1)
    h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
    h10 = u**3 - 2.0 * u**2 + u
    h01 = -2.0 * u**3 + 3.0 * u**2
    h11 = u**3 - u**2
    return h00 * p0 + h10 * h * m0 + h01 * p1 + h11 * h * m1


def _tangent(times: np.ndarray, points: np.ndarray, idx: int) -> np.ndarray:
    """估计 Hermite 插值在第 `idx` 个关键点处的切向速度。

    首尾点使用单侧差分，中间点使用跨过当前点的中心差分，以获得较平滑的段间连接。
    """
    if idx == 0:
        return (points[1] - points[0]) / max(times[1] - times[0], 1e-9)
    if idx == len(points) - 1:
        return (points[-1] - points[-2]) / max(times[-1] - times[-2], 1e-9)
    return (points[idx + 1] - points[idx - 1]) / max(times[idx + 1] - times[idx - 1], 1e-9)
