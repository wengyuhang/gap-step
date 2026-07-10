from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import ChartAtlas, NonConvexRegion, ShapeKind, TriangleChart, make_region, rotation_matrix

DEFAULT_ORDER = (0, 5, 2, 1, 4, 3)
KINDS: list[ShapeKind] = ["crescent", "u_notch", "starfish", "l_shape", "wavy_bean", "asymmetric_gear"]
CENTERS = [(2.0, -0.3, 1.3), (8.4, 3.3, 2.7), (6.0, -3.0, 1.1), (12.4, -2.2, 3.2), (10.3, 2.0, 1.0), (4.4, 3.6, 2.5)]


@dataclass(frozen=True)
class MotionProfile:
    """窗口运动公式。

    center(t)=center0+A_t sin(2*pi*t/T+phi)
    angles(t)=angles0+A_r [sin a, cos .87a, sin 1.17a]
    scale(t)=1+A_s [sin b, cos b]
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
        """三维平移偏移。"""
        if not self.enabled_translation:
            return np.zeros(3)
        phase = 2 * math.pi * t / self.period + self.phase
        return self.translation_amp * math.sin(phase)

    def rotation(self, t: float) -> np.ndarray:
        """yaw/pitch/roll 偏移。"""
        if not self.enabled_rotation:
            return np.zeros(3)
        a = 2 * math.pi * t / (self.period * 0.93) + self.phase
        return self.rotation_amp * np.asarray([math.sin(a), math.cos(0.87 * a), math.sin(1.17 * a)])

    def scale(self, t: float) -> np.ndarray:
        """局部 u/v 缩放。"""
        if not self.enabled_scale:
            return np.ones(2)
        a = 2 * math.pi * t / (self.period * 1.11) + self.phase
        scale_u = 1 + self.scale_amp[0] * math.sin(a)
        scale_v = 1 + self.scale_amp[1] * math.cos(a)
        return np.asarray([scale_u, scale_v])


@dataclass
class NonConvexDynamicWindow:
    """三维动态窗口：P(t,q)=center(t)+u*u_axis(t)+v*v_axis(t)，q=(u,v) in Omega(t)。"""

    name: str
    region: NonConvexRegion
    center0: np.ndarray
    yaw0: float
    pitch0: float
    roll0: float
    motion: MotionProfile

    def center_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """窗口中心。"""
        return self.center0 + (self.motion.translation(t) if dynamic else 0.0)

    def angles_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """窗口姿态角。"""
        return np.asarray([self.yaw0, self.pitch0, self.roll0]) + (self.motion.rotation(t) if dynamic else 0.0)

    def scale_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """局部缩放。"""
        return self.motion.scale(t) if dynamic else np.ones(2)

    def region_at(self, t: float, dynamic: bool = True) -> NonConvexRegion:
        """Omega(t)=diag(scale(t)) Omega_0。"""
        return self.region.scaled(self.scale_at(t, dynamic))

    def basis_at(self, t: float, dynamic: bool = True) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """局部 x 为法向，局部 y/z 为窗口平面 u/v 轴。"""
        r = rotation_matrix(*map(float, self.angles_at(t, dynamic)))
        normal = r @ [1.0, 0.0, 0.0]
        u_axis = r @ [0.0, 1.0, 0.0]
        v_axis = r @ [0.0, 0.0, 1.0]
        return normal, u_axis, v_axis

    def atlas_at(self, t: float, dynamic: bool = True) -> ChartAtlas:
        """缩放 chart 顶点即可得到 Omega(t) 的 atlas。"""
        s = self.scale_at(t, dynamic)
        return ChartAtlas.from_triangles(self.region_at(t, dynamic), self.region.triangles * s[None, None, :])

    def _chart(self, chart_id: int, t: float, dynamic: bool) -> TriangleChart:
        """取当前缩放后的 chart。"""
        return TriangleChart(self.region.triangles[int(chart_id)] * self.scale_at(t, dynamic)[None, :])

    def point_from_local(self, local: np.ndarray, t: float, dynamic: bool = True) -> np.ndarray:
        """局部点 -> 世界点。"""
        _, u, v = self.basis_at(t, dynamic)
        return self.center_at(t, dynamic) + local[0] * u + local[1] * v

    def point_from_chart_z(self, chart_id: int, z: np.ndarray, t: float, dynamic: bool = True) -> tuple[np.ndarray, np.ndarray]:
        """chart 变量 z -> 局部点 -> 世界点。"""
        local = self._chart(chart_id, t, dynamic).point_from_z(z)
        return self.point_from_local(local, t, dynamic), local

    def polygon_at(self, t: float, dynamic: bool = True) -> np.ndarray:
        """世界坐标边界采样。"""
        return np.asarray([self.point_from_local(p, t, dynamic) for p in self.region_at(t, dynamic).vertices])

    def candidates(self, t: float, max_per_chart: int = 2, dynamic: bool = True) -> list[tuple[int, np.ndarray, np.ndarray]]:
        """warm start 候选 `(chart_id, world, local)`。"""
        return [(cid, self.point_from_local(local, t, dynamic), local) for cid, local, _ in self.atlas_at(t, dynamic).candidates(max_per_chart)]

    def point_to_local(self, point: np.ndarray, t: float, dynamic: bool = True) -> tuple[np.ndarray, float]:
        """世界点 -> 局部投影；plane_error=|rel·normal|。"""
        n, u, v = self.basis_at(t, dynamic)
        rel = np.asarray(point) - self.center_at(t, dynamic)
        return np.asarray([np.dot(rel, u), np.dot(rel, v)]), abs(float(np.dot(rel, n)))

    def contains(self, point: np.ndarray, t: float, dynamic: bool = True, plane_tol: float = 1e-4) -> bool:
        """点是否在窗口平面内且局部点属于 Omega(t)。"""
        local, err = self.point_to_local(point, t, dynamic)
        return err <= plane_tol and self.region_at(t, dynamic).contains(local, 1e-7)

    def local_margin(self, local: np.ndarray, t: float, dynamic: bool = True) -> float:
        """局部边界裕度。"""
        return self.region_at(t, dynamic).margin(local)

    def chart_contains(self, chart_id: int, local: np.ndarray, t: float, dynamic: bool = True) -> bool:
        """点是否仍在对应三角 chart。"""
        return 0 <= chart_id < len(self.region.triangles) and self._chart(chart_id, t, dynamic).contains(local)

    def z_from_local(self, chart_id: int, local: np.ndarray, t: float, dynamic: bool = True) -> np.ndarray:
        """局部候选点 -> softmax logits 初值。"""
        return self._chart(chart_id, t, dynamic).z_from_point(local)

    def reference_local(self, t: float, dynamic: bool = True) -> tuple[int, np.ndarray]:
        """代表性内部点及其 chart。"""
        local = self.region_at(t, dynamic).best_interior_point()
        return self.atlas_at(t, dynamic).chart_for_point(local), local


@dataclass
class NonConvexWindowTrack:
    """起点、终点、窗口列表和穿越顺序。"""

    name: str
    start: np.ndarray
    goal: np.ndarray
    windows: list[NonConvexDynamicWindow]
    order: tuple[int, ...] = DEFAULT_ORDER

def make_window(idx: int, center: tuple[float, float, float], kind: ShapeKind, motion_scale: float = 1.0, motion_flags: tuple[bool, bool, bool] = (True, True, True)) -> NonConvexDynamicWindow:
    """构造单个动态非凸窗口。"""
    region = make_region(kind)
    translation_amp = motion_scale * np.asarray([0.20 * math.sin(idx + 0.9), 0.42 + 0.05 * (idx % 3), 0.25 + 0.04 * (idx % 2)])
    rotation_amp = motion_scale * np.asarray([0.25, 0.16, 0.31])
    scale_amp = motion_scale * np.asarray([0.16, -0.14])
    motion = MotionProfile(translation_amp, rotation_amp, scale_amp, 6.2 + 0.42 * idx, 0.53 * idx, *motion_flags)
    return NonConvexDynamicWindow(
        name=f"G{idx + 1}",
        region=region,
        center0=np.asarray(center),
        yaw0=0.24 * math.sin(idx * 0.8),
        pitch0=0.09 * math.cos(idx * 0.5),
        roll0=0.15 * math.sin(idx * 0.6),
        motion=motion,
    )


def canonical_track(motion_scale: float = 1.35, motion_flags: tuple[bool, bool, bool] = (True, True, True), name: str = "canonical_nonconvex_6") -> NonConvexWindowTrack:
    """六窗口 canonical 场景。"""
    windows = [make_window(i, c, k, motion_scale, motion_flags) for i, (c, k) in enumerate(zip(CENTERS, KINDS))]
    return NonConvexWindowTrack(name, np.asarray([0.0, -1.4, 1.2]), np.asarray([14.4, 1.4, 1.9]), windows)


def random_track(seed: int, count: int = 6) -> NonConvexWindowTrack:
    """随机非凸窗口场景。"""
    rng = np.random.default_rng(seed)
    centers = []
    for i in range(count):
        centers.append((2 + 2 * i + rng.uniform(-0.6, 0.6), rng.uniform(-3.5, 3.5), rng.uniform(0.9, 3.2)))
    windows = []
    for i, center in enumerate(centers):
        windows.append(make_window(i, center, KINDS[i % len(KINDS)], float(rng.uniform(0.55, 1.25))))
    return NonConvexWindowTrack(f"random_{seed}", np.asarray([0.0, -1.0, 1.2]), np.asarray([2.0 + 2.0 * count, 0.8, 1.8]), windows, tuple(rng.permutation(count).tolist()))


def make_scenario(name: str) -> NonConvexWindowTrack:
    """CLI 场景入口。"""
    flags = {"translation_only": (True, False, False), "rotation_only": (False, True, False), "scale_only": (False, False, True)}
    if name == "canonical":
        return canonical_track()
    if name in flags:
        return canonical_track(motion_flags=flags[name], name=name)
    if name in {"slow_dynamic", "fast_dynamic"}:
        return canonical_track(motion_scale=0.45 if name == "slow_dynamic" else 1.55, name=name)
    if name.startswith("random_"):
        return random_track(int(name.split("_", 1)[1]))
    raise ValueError(f"unknown non-convex scenario: {name}")
