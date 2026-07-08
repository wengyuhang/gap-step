from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np

from .geometry import Shape2D, ShapeKind, convex_margin, local_from_unconstrained, rotation_matrix, sample_polygon

DEFAULT_ORDER = (0, 5, 2, 1, 4, 3)


@dataclass(frozen=True)
class MotionProfile:
    """描述单个窗口随时间变化的运动曲线。

    三类变化分别是平移、姿态旋转和二维局部尺度变化。每类变化都用正弦/余弦函数
    生成平滑周期运动，并可通过 `enabled_*` 开关单独关闭，用于构造 translation_only、
    rotation_only、scale_only 等消融场景。
    """

    translation_amp: np.ndarray
    rotation_amp: np.ndarray
    scale_amp: np.ndarray
    period: float = 7.0
    phase: float = 0.0
    enabled_translation: bool = True
    enabled_rotation: bool = True
    enabled_scale: bool = True

    def translation(self, t: float) -> np.ndarray:
        """返回时刻 `t` 的三维平移偏移量。

        当平移关闭时返回零向量；否则按 `translation_amp` 指定的三个方向振幅生成
        周期位移。该偏移会叠加到窗口初始中心 `center0` 上。
        """
        if not self.enabled_translation:
            return np.zeros(3, dtype=np.float64)
        return self.translation_amp * math.sin(2.0 * math.pi * t / self.period + self.phase)

    def rotation(self, t: float) -> np.ndarray:
        """返回时刻 `t` 的 yaw/pitch/roll 姿态偏移。

        三个角速度比例略有不同，避免所有轴完全同步，从而产生更明显的时变姿态。
        返回值会叠加到窗口初始角 `yaw0/pitch0/roll0`。
        """
        if not self.enabled_rotation:
            return np.zeros(3, dtype=np.float64)
        angle = 2.0 * math.pi * t / (self.period * 0.91) + self.phase
        return self.rotation_amp * np.asarray([math.sin(angle), math.cos(angle * 0.83), math.sin(angle * 1.21)], dtype=np.float64)

    def scale(self, t: float) -> np.ndarray:
        """返回时刻 `t` 的局部二维缩放系数。

        缩放只作用于窗口平面内的两个轴，法向不缩放。关闭缩放时返回 `[1, 1]`，
        表示使用原始形状尺寸。
        """
        if not self.enabled_scale:
            return np.ones(2, dtype=np.float64)
        angle = 2.0 * math.pi * t / (self.period * 1.13) + self.phase
        return np.asarray([1.0 + self.scale_amp[0] * math.sin(angle), 1.0 + self.scale_amp[1] * math.cos(angle)], dtype=np.float64)


@dataclass
class DynamicWindow:
    """一个会随时间移动、旋转、缩放的三维动态窗口。

    窗口几何由二维 `Shape2D` 加三维姿态组成：局部平面内的点 `(u, v)` 会通过当前
    时刻的窗口基向量映射到世界坐标。优化器选择的是局部点和穿越时间，本类负责在
    `G_i(t_i)` 中完成局部/世界坐标转换与包含性验证。
    """

    name: str
    shape: Shape2D
    center0: np.ndarray
    yaw0: float
    pitch0: float
    roll0: float
    motion: MotionProfile

    def center_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """返回窗口在时刻 `t` 的世界坐标中心。

        `dynamic=False` 用于静态 TOGT 对照，此时忽略运动曲线，只返回初始中心。
        """
        return self.center0 + (self.motion.translation(t) if dynamic else 0.0)

    def angles_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """返回窗口在时刻 `t` 的 yaw/pitch/roll 姿态角。

        动态模式下把运动曲线产生的姿态偏移叠加到初始姿态上；静态模式下保持初始姿态。
        """
        base = np.asarray([self.yaw0, self.pitch0, self.roll0], dtype=np.float64)
        return base + (self.motion.rotation(t) if dynamic else 0.0)

    def scale_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """返回窗口在时刻 `t` 的局部二维缩放比例。"""
        return self.motion.scale(t) if dynamic else np.ones(2, dtype=np.float64)

    def local_polygon_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """返回时刻 `t` 的窗口局部二维多边形。

        这里仍在窗口自身平面内，只应用尺度变化；后续 `polygon_at` 才会映射到三维世界。
        """
        return self.shape.polygon() * self.scale_at(t, dynamic=dynamic)[None, :]

    def basis_at(self, t: float, dynamic: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """返回窗口当前法向、局部 u 轴和局部 v 轴的世界坐标方向。

        本项目约定局部 x 轴是窗口法向，局部 y/z 轴是窗口平面内两个坐标轴。
        """
        yaw, pitch, roll = self.angles_at(t, dynamic=dynamic)
        rot = rotation_matrix(float(yaw), float(pitch), float(roll))
        normal = rot @ np.asarray([1.0, 0.0, 0.0])
        u_axis = rot @ np.asarray([0.0, 1.0, 0.0])
        v_axis = rot @ np.asarray([0.0, 0.0, 1.0])
        return normal, u_axis, v_axis

    def point_from_local(self, local: np.ndarray, t: float, dynamic: bool = True) -> np.ndarray:
        """把窗口局部二维点 `(u, v)` 映射为三维世界坐标点。"""
        _, u_axis, v_axis = self.basis_at(t, dynamic=dynamic)
        return self.center_at(t, dynamic=dynamic) + local[0] * u_axis + local[1] * v_axis

    def point_from_unconstrained(self, z: np.ndarray, t: float, dynamic: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """把优化变量 `z` 映射为窗口内的世界点和局部点。

        这是 DynaTOGT 动态几何约束的关键：优化器无需直接处理复杂不等式约束，
        而是通过局部变量映射天然得到位于 `G_i(t)` 内部的穿越点。
        """
        poly = self.local_polygon_at(t, dynamic=dynamic)
        local = local_from_unconstrained(z, poly)
        return self.point_from_local(local, t, dynamic=dynamic), local

    def polygon_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """返回窗口在时刻 `t` 的三维世界坐标多边形顶点。"""
        local = self.local_polygon_at(t, dynamic=dynamic)
        return np.asarray([self.point_from_local(p, t, dynamic=dynamic) for p in local], dtype=np.float64)

    def candidates(self, t: float, samples_per_axis: int = 3, dynamic: bool = True) -> list[tuple[np.ndarray, np.ndarray]]:
        """生成时刻 `t` 可用于离散 warm start 的候选穿越点。

        返回列表中每个元素都是 `(世界坐标点, 局部坐标点)`，方便优化器同时记录路径距离
        和后续转回无约束变量。
        """
        poly = self.local_polygon_at(t, dynamic=dynamic)
        locals_ = sample_polygon(poly, samples_per_axis=samples_per_axis)
        return [(self.point_from_local(local, t, dynamic=dynamic), local) for local in locals_]

    def contains(self, point: np.ndarray, t: float, dynamic: bool = True, plane_tol: float = 1e-4) -> bool:
        """判断世界坐标点是否位于时刻 `t` 的窗口区域内。

        验证分两步：先检查点到窗口平面的法向距离是否小于 `plane_tol`，再检查局部点
        是否落在当前缩放后的凸多边形内。
        """
        local, plane_error = self.point_to_local(point, t, dynamic=dynamic)
        if plane_error > plane_tol:
            return False
        return self.local_margin(local, t, dynamic=dynamic) >= -1e-7

    def point_to_local(self, point: np.ndarray, t: float, dynamic: bool = True) -> tuple[np.ndarray, float]:
        """把世界坐标点投影回窗口局部坐标，并返回平面误差。

        返回的 `local` 是点在窗口平面两个轴上的投影坐标；`plane_error` 是沿窗口法向
        的绝对距离，用于判断是否真正穿过窗口平面。
        """
        center = self.center_at(t, dynamic=dynamic)
        normal, u_axis, v_axis = self.basis_at(t, dynamic=dynamic)
        rel = np.asarray(point, dtype=np.float64) - center
        local = np.asarray([np.dot(rel, u_axis), np.dot(rel, v_axis)], dtype=np.float64)
        return local, abs(float(np.dot(rel, normal)))

    def local_margin(self, local: np.ndarray, t: float, dynamic: bool = True) -> float:
        """计算局部点到当前窗口边界的有符号安全裕度。"""
        return convex_margin(local, self.local_polygon_at(t, dynamic=dynamic))


@dataclass
class WindowTrack:
    """一个完整穿越任务：起点、终点、窗口集合和指定穿越顺序。"""

    name: str
    start: np.ndarray
    goal: np.ndarray
    windows: list[DynamicWindow]
    order: tuple[int, ...] = DEFAULT_ORDER

    def ordered_windows(self, order: Iterable[int] | None = None) -> list[DynamicWindow]:
        """按给定顺序返回窗口对象列表。

        `order=None` 时使用场景默认顺序；传入显式序列时允许重复窗口索引，因此可表达
        `G1 -> G6 -> G1` 这样的重复穿越任务。
        """
        idxs = self.order if order is None else tuple(order)
        return [self.windows[i] for i in idxs]


def make_window(idx: int, center: tuple[float, float, float], kind: ShapeKind, motion_scale: float = 1.0, motion_flags: tuple[bool, bool, bool] = (True, True, True)) -> DynamicWindow:
    """按索引和形状快速构造一个动态窗口。

    该工厂函数集中设置窗口的初始姿态、周期、相位和运动振幅。不同 `idx` 会得到不同
    相位/周期，使 canonical 场景中的多个窗口不会同步运动。
    """
    amp_t = motion_scale * np.asarray([0.22 * math.sin(idx + 0.7), 0.45 + 0.06 * (idx % 3), 0.28 + 0.04 * (idx % 2)], dtype=np.float64)
    amp_r = motion_scale * np.asarray([0.28, 0.18, 0.34], dtype=np.float64)
    amp_s = motion_scale * np.asarray([0.18, -0.16], dtype=np.float64)
    return DynamicWindow(
        name=f"G{idx + 1}",
        shape=Shape2D(kind=kind, size=(1.65, 1.12), radius=0.86),
        center0=np.asarray(center, dtype=np.float64),
        yaw0=0.28 * math.sin(idx * 0.9),
        pitch0=0.10 * math.cos(idx * 0.6),
        roll0=0.16 * math.sin(idx * 0.5),
        motion=MotionProfile(
            translation_amp=amp_t,
            rotation_amp=amp_r,
            scale_amp=amp_s,
            period=6.0 + 0.45 * idx,
            phase=0.55 * idx,
            enabled_translation=motion_flags[0],
            enabled_rotation=motion_flags[1],
            enabled_scale=motion_flags[2],
        ),
    )


def canonical_track(motion_scale: float = 1.45, motion_flags: tuple[bool, bool, bool] = (True, True, True), name: str = "canonical_6") -> WindowTrack:
    """构造默认的六窗口 canonical 场景。

    该场景包含六种窗口形状，并使用默认顺序 `G1 -> G6 -> G3 -> G2 -> G5 -> G4`。
    `motion_flags` 可关闭某类动态变化，用于生成消融场景。
    """
    centers = [
        (2.0, -0.3, 1.3),
        (8.4, 3.3, 2.7),
        (6.0, -3.0, 1.1),
        (12.4, -2.2, 3.2),
        (10.3, 2.0, 1.0),
        (4.4, 3.6, 2.5),
    ]
    kinds: list[ShapeKind] = ["rectangle", "circle", "pentagon", "slanted_quadrilateral", "hexagon", "triangle"]
    windows = [make_window(i, c, k, motion_scale=motion_scale, motion_flags=motion_flags) for i, (c, k) in enumerate(zip(centers, kinds))]
    return WindowTrack(name=name, start=np.asarray([0.0, -1.4, 1.2]), goal=np.asarray([14.4, 1.4, 1.9]), windows=windows)


def random_track(seed: int, count: int = 6) -> WindowTrack:
    """根据随机种子生成一个随机窗口轨迹场景。

    窗口大致沿 x 方向排列，但 y/z 坐标、运动幅度和穿越顺序随机，用于 default
    实验套件中的泛化对照。
    """
    rng = np.random.default_rng(seed)
    kinds: list[ShapeKind] = ["rectangle", "circle", "triangle", "pentagon", "hexagon", "slanted_quadrilateral"]
    centers = []
    for i in range(count):
        centers.append((2.0 + 2.0 * i + rng.uniform(-0.6, 0.6), rng.uniform(-3.5, 3.5), rng.uniform(0.9, 3.2)))
    windows = [make_window(i, centers[i], kinds[i % len(kinds)], motion_scale=float(rng.uniform(0.6, 1.3))) for i in range(count)]
    order = tuple(rng.permutation(count).tolist())
    return WindowTrack(name=f"random_{seed}", start=np.asarray([0.0, -1.0, 1.2]), goal=np.asarray([2.0 + 2.0 * count, 0.8, 1.8]), windows=windows, order=order)


def make_scenario(name: str) -> WindowTrack:
    """按场景名称创建 `WindowTrack`。

    支持 canonical、单一运动类型消融、快/慢动态和 `random_<seed>`。CLI 入口都通过
    该函数把字符串参数转换成可优化的场景对象。
    """
    if name == "canonical":
        return canonical_track()
    if name == "translation_only":
        return canonical_track(motion_flags=(True, False, False), name=name)
    if name == "rotation_only":
        return canonical_track(motion_flags=(False, True, False), name=name)
    if name == "scale_only":
        return canonical_track(motion_flags=(False, False, True), name=name)
    if name == "slow_dynamic":
        return canonical_track(motion_scale=0.45, name=name)
    if name == "fast_dynamic":
        return canonical_track(motion_scale=1.55, name=name)
    if name.startswith("random_"):
        return random_track(int(name.split("_", 1)[1]))
    raise ValueError(f"unknown scenario: {name}")
