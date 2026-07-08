from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal

import numpy as np

ShapeKind = Literal["rectangle", "circle", "triangle", "pentagon", "hexagon", "slanted_quadrilateral"]


def rot2(theta: float) -> np.ndarray:
    """生成二维平面旋转矩阵。

    该函数目前主要作为几何工具保留：输入角度 `theta`，返回把二维向量逆时针旋转
    `theta` 弧度的 2x2 矩阵。DynaTOGT 主流程里的窗口姿态使用三维
    `rotation_matrix`，但二维旋转在后续扩展局部形状变换时仍然有用。
    """
    c = math.cos(theta)
    s = math.sin(theta)
    return np.asarray([[c, -s], [s, c]], dtype=np.float64)


def rotation_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """根据 yaw/pitch/roll 生成三维旋转矩阵。

    窗口的局部坐标系定义为：局部 x 轴是窗口法向，局部 y/z 轴张成窗口平面。
    这里按 `Rz(yaw) @ Ry(pitch) @ Rx(roll)` 的顺序组合姿态，用于把局部坐标轴
    映射到世界坐标系。
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class Shape2D:
    """窗口在自身局部平面内的二维形状定义。

    `kind` 决定使用矩形、圆近似多边形、三角形等形状；`size` 用于矩形和斜四边形，
    `radius` 用于正多边形/圆形近似。所有顶点都位于以局部原点为中心的二维平面内。
    """

    kind: ShapeKind
    size: tuple[float, float] = (1.6, 1.1)
    radius: float = 0.82

    def polygon(self, resolution: int = 32) -> np.ndarray:
        """返回该形状的凸多边形顶点。

        DynaTOGT 后续的“点是否在窗口内”“安全裕度”“采样候选点”都统一基于凸多边形。
        圆形会被离散成 `resolution` 边形；其它正多边形使用固定顶点数。
        """
        if self.kind == "rectangle":
            w, h = self.size
            return np.asarray([[-w / 2, -h / 2], [w / 2, -h / 2], [w / 2, h / 2], [-w / 2, h / 2]], dtype=np.float64)
        if self.kind == "slanted_quadrilateral":
            w, h = self.size
            return np.asarray([[-0.55 * w, -0.50 * h], [0.36 * w, -0.50 * h], [0.58 * w, 0.50 * h], [-0.36 * w, 0.50 * h]], dtype=np.float64)
        vertices = {"triangle": 3, "pentagon": 5, "hexagon": 6}.get(self.kind, max(16, resolution))
        angles = np.linspace(0.0, 2.0 * math.pi, vertices, endpoint=False) + math.pi / 2.0
        return np.stack([self.radius * np.cos(angles), self.radius * np.sin(angles)], axis=1).astype(np.float64)


def local_from_unconstrained(z: np.ndarray, polygon: np.ndarray, shrink: float = 0.72) -> np.ndarray:
    """把无约束优化变量映射到窗口凸多边形内部。

    L-BFGS-B 优化的是两个近似无约束的局部变量 `z`。这里先用 `tanh` 把它们压到
    [-1, 1]，再按多边形包围盒缩放到窗口中心附近。如果候选点因为斜四边形等原因
    落到多边形外，就沿中心到候选点方向二分回缩，保证穿越点仍在窗口内部。
    """

    poly = np.asarray(polygon, dtype=np.float64)
    center = poly.mean(axis=0)
    direction = np.tanh(np.asarray(z, dtype=np.float64))
    lo = poly.min(axis=0)
    hi = poly.max(axis=0)
    candidate = center + 0.5 * shrink * direction * (hi - lo)
    if point_in_convex_polygon(candidate, poly, margin=1e-9):
        return candidate
    return center + shrink * (candidate - center) * max_scale_inside(center, candidate, poly)


def max_scale_inside(center: np.ndarray, point: np.ndarray, polygon: np.ndarray) -> float:
    """求从中心指向候选点时仍位于多边形内的最大缩放比例。

    `local_from_unconstrained` 用它把越界候选点拉回窗口内部。因为窗口形状均为凸多边形，
    从中心到任意方向的线段与内部区域的交集是连续区间，二分搜索可以稳定找到边界。
    """
    lo = 0.0
    hi = 1.0
    for _ in range(32):
        mid = 0.5 * (lo + hi)
        p = center + mid * (point - center)
        if point_in_convex_polygon(p, polygon, margin=1e-9):
            lo = mid
        else:
            hi = mid
    return lo


def point_in_convex_polygon(point: np.ndarray, polygon: np.ndarray, margin: float = 0.0) -> bool:
    """判断二维点是否位于凸多边形内。

    通过检查点相对每条有向边的叉积符号是否一致来判断内外关系。`margin` 用于给
    数值误差留容忍度，避免点刚好落在边界时因为浮点误差被误判为外部。
    """
    p = np.asarray(point, dtype=np.float64)
    poly = np.asarray(polygon, dtype=np.float64)
    sign = 0.0
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        edge = b - a
        rel = p - a
        cross = float(edge[0] * rel[1] - edge[1] * rel[0])
        if abs(cross) <= margin:
            continue
        if sign == 0.0:
            sign = 1.0 if cross > 0.0 else -1.0
        elif sign * cross < -margin:
            return False
    return True


def convex_margin(point: np.ndarray, polygon: np.ndarray) -> float:
    """计算点到凸多边形边界的有符号安全裕度。

    返回值近似等于点到最靠近边的距离：正值表示在窗口内部，0 表示在边界上，
    负值表示在窗口外部。该指标会写入 CSV，也会进入优化目标作为边界安全惩罚。
    """
    p = np.asarray(point, dtype=np.float64)
    poly = np.asarray(polygon, dtype=np.float64)
    signs = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        edge = b - a
        rel = p - a
        cross = float(edge[0] * rel[1] - edge[1] * rel[0])
        signs.append(cross / max(np.linalg.norm(edge), 1e-9))
    signs_arr = np.asarray(signs)
    if np.mean(signs_arr) < 0.0:
        signs_arr = -signs_arr
    return float(np.min(signs_arr))


def sample_polygon(polygon: np.ndarray, samples_per_axis: int = 3, shrink: float = 0.88) -> np.ndarray:
    """在凸多边形内部生成一组离散候选局部点。

    warm start 阶段需要快速枚举“可能穿越窗口的位置”。这里组合中心点、收缩后的顶点
    和包围盒网格点，并去重，得到小而稳定的候选集合。
    """
    poly = np.asarray(polygon, dtype=np.float64)
    center = poly.mean(axis=0)
    candidates = [center]
    for v in poly:
        candidates.append(center + shrink * (v - center))
    lo = poly.min(axis=0)
    hi = poly.max(axis=0)
    for x in np.linspace(lo[0], hi[0], samples_per_axis):
        for y in np.linspace(lo[1], hi[1], samples_per_axis):
            p = center + shrink * (np.asarray([x, y]) - center)
            if point_in_convex_polygon(p, poly, margin=1e-8):
                candidates.append(p)
    unique: list[np.ndarray] = []
    for p in candidates:
        if not any(np.linalg.norm(p - old) < 1e-7 for old in unique):
            unique.append(p)
    return np.asarray(unique, dtype=np.float64)


def path_length(points: np.ndarray | list[np.ndarray]) -> float:
    """计算折线路径长度。

    输入可以是轨迹采样点数组或关键点列表；函数把相邻点欧氏距离相加，用于轨迹
    指标和优化目标中的路径长度项。
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
