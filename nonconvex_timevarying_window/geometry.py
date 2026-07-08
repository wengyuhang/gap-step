from __future__ import annotations

import math
from dataclasses import dataclass
from functools import cached_property
from typing import Literal

import numpy as np

ShapeKind = Literal["crescent", "u_notch", "starfish", "l_shape", "wavy_bean", "asymmetric_gear"]


def rotation_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """三维姿态矩阵：R=Rz(yaw)Ry(pitch)Rx(roll)，把窗口局部坐标轴转到世界系。"""
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    rz = np.asarray([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    ry = np.asarray([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rx = np.asarray([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    return rz @ ry @ rx


def signed_area(vertices: np.ndarray) -> float:
    """鞋带公式：A=1/2 sum_i (x_i y_{i+1}-y_i x_{i+1})；A>0 表示逆时针。"""
    p = np.asarray(vertices, dtype=np.float64)
    return float(0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1)))


def polygon_area(vertices: np.ndarray) -> float:
    """简单多边形面积。"""
    return abs(signed_area(vertices))


def ensure_ccw(vertices: np.ndarray) -> np.ndarray:
    """统一为逆时针顶点，便于用叉积判断凸耳朵。"""
    p = np.asarray(vertices, dtype=np.float64)
    return p[::-1].copy() if signed_area(p) < 0 else p.copy()


def distance_to_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """点到线段距离：u=clip(((p-a)·(b-a))/||b-a||^2,0,1)，d=||p-(a+u(b-a))||。"""
    p, a, b = map(lambda x: np.asarray(x, dtype=np.float64), (point, a, b))
    ab, den = b - a, float(np.dot(b - a, b - a))
    if den <= 1e-15:
        return float(np.linalg.norm(p - a))
    return float(np.linalg.norm(p - (a + np.clip(np.dot(p - a, ab) / den, 0.0, 1.0) * ab)))


def distance_to_boundary(point: np.ndarray, vertices: np.ndarray) -> float:
    """点到非凸边界的最小线段距离。"""
    poly = np.asarray(vertices, dtype=np.float64)
    return float(min(distance_to_segment(point, a, b) for a, b in zip(poly, np.roll(poly, -1, axis=0))))


def point_in_polygon(point: np.ndarray, vertices: np.ndarray, tol: float = 1e-9) -> bool:
    """射线法：水平射线交边界次数为奇数则在内；边界距离 <= tol 也算在内。"""
    p, poly = np.asarray(point, dtype=np.float64), np.asarray(vertices, dtype=np.float64)
    if distance_to_boundary(p, poly) <= tol:
        return True
    inside, x, y = False, float(p[0]), float(p[1])
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        if (a[1] > y) != (b[1] > y) and abs(b[1] - a[1]) > 1e-15:
            x_cross = (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
            if x < x_cross:
                inside = not inside
    return inside


def boundary_margin(point: np.ndarray, vertices: np.ndarray) -> float:
    """有符号裕度 m(p)：区域内为 +dist(p,boundary)，区域外为 -dist(p,boundary)。"""
    d = distance_to_boundary(point, vertices)
    return d if point_in_polygon(point, vertices) else -d


def barycentric_coordinates(point: np.ndarray, triangle: np.ndarray) -> np.ndarray:
    """重心坐标：p=b0*a+b1*b+b2*c，且 b0+b1+b2=1。"""
    p = np.asarray(point, dtype=np.float64)
    a, b, c = np.asarray(triangle, dtype=np.float64)
    v0, v1, v2 = b - a, c - a, p - a
    den = float(v0[0] * v1[1] - v0[1] * v1[0])
    if abs(den) <= 1e-15:
        return np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    b1 = float((v2[0] * v1[1] - v2[1] * v1[0]) / den)
    b2 = float((v0[0] * v2[1] - v0[1] * v2[0]) / den)
    return np.asarray([1.0 - b1 - b2, b1, b2], dtype=np.float64)


def point_in_triangle(point: np.ndarray, triangle: np.ndarray, tol: float = 1e-9) -> bool:
    """重心坐标均 >=0 表示点在三角形内。"""
    return bool(np.all(barycentric_coordinates(point, triangle) >= -tol))


def triangle_area(triangle: np.ndarray) -> float:
    """三角形面积。"""
    return polygon_area(np.asarray(triangle, dtype=np.float64))


def _cross(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    """cross(b-a,c-a)，正值表示 a->b->c 左转。"""
    return float((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))


def _intersects(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray, eps: float = 1e-10) -> bool:
    """线段相交检测，用于排除穿出边界的候选对角线。"""
    def on_segment(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> bool:
        """q 是否在线段 p-r 上。"""
        in_x = min(p[0], r[0]) - eps <= q[0] <= max(p[0], r[0]) + eps
        in_y = min(p[1], r[1]) - eps <= q[1] <= max(p[1], r[1]) + eps
        return in_x and in_y and abs(_cross(p, q, r)) <= eps

    o1, o2, o3, o4 = _cross(a, b, c), _cross(a, b, d), _cross(c, d, a), _cross(c, d, b)
    strictly_cross = o1 * o2 < -eps and o3 * o4 < -eps
    touches = on_segment(a, c, b) or on_segment(a, d, b) or on_segment(c, a, d) or on_segment(c, b, d)
    return strictly_cross or touches


def _diagonal_ok(poly: np.ndarray, i: int, j: int) -> bool:
    """合法对角线：中点在区域内，且不与任何非相邻边相交。"""
    a, b, n = poly[i], poly[j], len(poly)
    return point_in_polygon(0.5 * (a + b), poly, 1e-8) and not any(k not in (i, j) and (k + 1) % n not in (i, j) and _intersects(a, b, poly[k], poly[(k + 1) % n]) for k in range(n))


def ear_clip_triangulate(vertices: np.ndarray) -> np.ndarray:
    """Ear clipping 三角剖分。

    候选耳朵 (a,b,c) 需要：1) cross(b-a,c-a)>0；2) a-c 是内部对角线；
    3) 没有其它顶点严格落在三角形 abc 内。失败时用 Delaunay 候选过滤兜底。
    """
    poly, ids, tris, guard = ensure_ccw(vertices), list(range(len(vertices))), [], 0
    while len(ids) > 3 and guard < len(poly) * len(poly):
        guard += 1
        for pos, idx in enumerate(list(ids)):
            prev_i, next_i = ids[(pos - 1) % len(ids)], ids[(pos + 1) % len(ids)]
            tri = np.asarray([poly[prev_i], poly[idx], poly[next_i]], dtype=np.float64)
            if _cross(*tri) > 1e-10 and _diagonal_ok(poly, prev_i, next_i) and not any(o not in (prev_i, idx, next_i) and np.all(barycentric_coordinates(poly[o], tri) > 1e-10) for o in ids):
                tris.append(tri)
                ids.pop(pos)
                break
        else:
            return _delaunay_fallback(poly)
    tris.append(np.asarray([poly[i] for i in ids], dtype=np.float64))
    return np.asarray([t for t in tris if triangle_area(t) > 1e-12], dtype=np.float64)


def _delaunay_fallback(poly: np.ndarray) -> np.ndarray:
    """兜底：Delaunay 候选三角形中过滤重心/边中点都在区域内的部分。"""
    from scipy.spatial import Delaunay

    tris = []
    for simplex in Delaunay(poly).simplices:
        tri = np.asarray(poly[simplex], dtype=np.float64)
        probes = [tri.mean(axis=0), *((tri + np.roll(tri, -1, axis=0)) * 0.5)]
        if triangle_area(tri) > 1e-12 and all(point_in_polygon(p, poly, 1e-8) for p in probes):
            tris.append(tri if signed_area(tri) > 0 else tri[[0, 2, 1]])
    area = sum(triangle_area(t) for t in tris)
    if tris and abs(area - polygon_area(poly)) <= 5e-3 * max(1.0, polygon_area(poly)):
        return np.asarray(tris, dtype=np.float64)
    raise ValueError("triangulation failed; polygon may be self-intersecting")


@dataclass(frozen=True)
class NonConvexRegion:
    """无洞简单区域；光滑任意形状先采样成此边界多边形。"""

    vertices: np.ndarray

    def __post_init__(self) -> None:
        """统一边界方向。"""
        object.__setattr__(self, "vertices", ensure_ccw(self.vertices))

    @property
    def area(self) -> float:
        """区域面积。"""
        return polygon_area(self.vertices)

    @cached_property
    def triangles(self) -> np.ndarray:
        """缓存三角剖分；动态缩放只缩放三角顶点，不重剖分。"""
        return ear_clip_triangulate(self.vertices)

    def scaled(self, scale: np.ndarray) -> "NonConvexRegion":
        """Omega(t)=diag(s_u,s_v) Omega_0。"""
        return NonConvexRegion(self.vertices * np.asarray(scale, dtype=np.float64)[None, :])

    def contains(self, point: np.ndarray, tol: float = 1e-9) -> bool:
        """点是否在区域内。"""
        return point_in_polygon(point, self.vertices, tol=tol)

    def margin(self, point: np.ndarray) -> float:
        """点到边界的有符号安全裕度。"""
        return boundary_margin(point, self.vertices)

    def best_interior_point(self) -> np.ndarray:
        """选三角形重心中裕度最大的点，作为 waypoint 中心。"""
        return np.asarray(max((t.mean(axis=0) for t in self.triangles), key=self.margin), dtype=np.float64)

    def sample_points(self, samples_per_axis: int = 5) -> np.ndarray:
        """内部采样点，用于测试 atlas 覆盖。"""
        pts = [self.best_interior_point(), *(t.mean(axis=0) for t in self.triangles)]
        lo, hi = self.vertices.min(axis=0), self.vertices.max(axis=0)
        pts += [np.asarray([x, y]) for x in np.linspace(lo[0], hi[0], samples_per_axis) for y in np.linspace(lo[1], hi[1], samples_per_axis)]
        out: list[np.ndarray] = []
        for p in pts:
            if self.contains(p, 1e-8) and not any(np.linalg.norm(p - q) < 1e-7 for q in out):
                out.append(p)
        return np.asarray(out, dtype=np.float64)


def softmax3(z: np.ndarray) -> np.ndarray:
    """把二维自由变量变成三角形重心坐标。

    公式：b = softmax([z0, z1, 0])。
    softmax 的结果满足 b0,b1,b2 都大于 0，且 b0+b1+b2=1，
    所以用 b0*a+b1*b+b2*c 得到的点一定在三角形内部。
    """
    logits = np.asarray([float(z[0]), float(z[1]), 0.0], dtype=np.float64)
    logits = logits - logits.max()
    exp = np.exp(logits)
    return exp / exp.sum()


@dataclass(frozen=True)
class TriangleChart:
    """单个三角形 chart。

    前向映射：phi(z)=sum_k softmax([z0,z1,0])_k * v_k。
    L-BFGS-B 优化 z，phi(z) 自动落在该三角形内。
    """

    triangle: np.ndarray

    def point_from_z(self, z: np.ndarray) -> np.ndarray:
        """无约束变量 z -> 三角形内部点。"""
        bary = softmax3(z)
        triangle = np.asarray(self.triangle, dtype=np.float64)
        return bary @ triangle

    def barycentric_from_z(self, z: np.ndarray) -> np.ndarray:
        """查看 z 对应的重心坐标。"""
        return softmax3(z)

    def contains(self, point: np.ndarray, tol: float = 1e-8) -> bool:
        """点是否位于这个三角 chart。"""
        return point_in_triangle(point, self.triangle, tol=tol)

    def z_from_point(self, point: np.ndarray, eps: float = 1e-5) -> np.ndarray:
        """warm start 近似逆映射。

        若点的重心坐标为 b，则 softmax([log(b0/b2), log(b1/b2), 0]) 会还原 b。
        eps 用来避免候选点太靠近三角形边界时出现 log(0)。
        """
        bary = barycentric_coordinates(point, self.triangle)
        bary = np.clip(bary, eps, 1.0)
        bary = bary / bary.sum()
        return np.asarray([np.log(bary[0] / bary[2]), np.log(bary[1] / bary[2])], dtype=np.float64)


@dataclass(frozen=True)
class ChartAtlas:
    """一个非凸区域的所有三角 chart。"""

    region: NonConvexRegion
    charts: tuple[TriangleChart, ...]

    @classmethod
    def from_region(cls, region: NonConvexRegion) -> "ChartAtlas":
        """由区域的三角剖分创建 atlas。"""
        charts = []
        for triangle in region.triangles:
            charts.append(TriangleChart(triangle))
        return cls(region=region, charts=tuple(charts))

    @classmethod
    def from_triangles(cls, region: NonConvexRegion, triangles: np.ndarray) -> "ChartAtlas":
        """动态缩放后复用原三角连接。"""
        charts = []
        for triangle in np.asarray(triangles, dtype=np.float64):
            charts.append(TriangleChart(triangle))
        return cls(region=region, charts=tuple(charts))

    def contains_in_chart(self, chart_id: int, point: np.ndarray, tol: float = 1e-8) -> bool:
        """点是否在指定 chart 内。"""
        if chart_id < 0 or chart_id >= len(self.charts):
            return False
        return self.charts[int(chart_id)].contains(point, tol=tol)

    def chart_for_point(self, point: np.ndarray) -> int:
        """找到包含点的第一个 chart；边界误差时退到重心裕度最大的 chart。"""
        for chart_id, chart in enumerate(self.charts):
            if chart.contains(point, tol=1e-8):
                return chart_id
        margins = []
        for chart in self.charts:
            margins.append(self.region.margin(chart.triangle.mean(axis=0)))
        return int(np.argmax(margins))

    def candidates(self, max_per_chart: int = 3) -> list[tuple[int, np.ndarray, np.ndarray]]:
        """生成 `(chart_id, local_point, z)` 候选集合。"""
        z_bank = [
            np.asarray([0.0, 0.0], dtype=np.float64),
            np.asarray([1.1, -0.8], dtype=np.float64),
            np.asarray([-0.8, 1.1], dtype=np.float64),
            np.asarray([-0.9, -0.9], dtype=np.float64),
        ][: max(1, max_per_chart)]
        out = []
        for chart_id, chart in enumerate(self.charts):
            for z in z_bank:
                local = chart.point_from_z(z)
                if self.region.contains(local, tol=1e-8):
                    out.append((chart_id, local, z.copy()))
        return out


def _radial(samples: int, radius_fn) -> NonConvexRegion:
    """极坐标边界采样：v_k=r(theta_k)[cos theta_k, sin theta_k]。"""
    a = np.linspace(0.0, 2.0 * math.pi, samples, endpoint=False)
    r = np.asarray([radius_fn(x) for x in a], dtype=np.float64)
    return NonConvexRegion(np.stack([r * np.cos(a), r * np.sin(a)], axis=1))


def make_region(kind: ShapeKind, samples: int = 24) -> NonConvexRegion:
    """非凸窗口工厂；光滑形状用边界采样多边形近似。"""
    polys = {
        "u_notch": [[-0.82, -0.56], [0.82, -0.56], [0.82, 0.56], [0.36, 0.56], [0.36, -0.10], [-0.36, -0.10], [-0.36, 0.56], [-0.82, 0.56]],
        "l_shape": [[-0.78, -0.58], [0.82, -0.58], [0.82, -0.15], [-0.15, -0.15], [-0.15, 0.58], [-0.78, 0.58]],
    }
    if kind in polys:
        return NonConvexRegion(np.asarray(polys[kind], dtype=np.float64))
    if kind == "crescent":
        return _radial(samples, _crescent_radius)
    if kind == "starfish":
        return _radial(samples, _starfish_radius)
    if kind == "wavy_bean":
        return _radial(samples, _wavy_bean_radius)
    if kind == "asymmetric_gear":
        return _radial(samples, _gear_radius)
    raise ValueError(f"unknown non-convex shape kind: {kind}")


def _crescent_radius(a: float) -> float:
    """月牙形的极径函数 r(theta)。"""
    return 0.70 + 0.24 * math.sin(a - 0.1) - 0.18 * math.cos(2.0 * a + 0.4)


def _starfish_radius(a: float) -> float:
    """海星形的极径函数，高频 sin 项制造凹凸。"""
    return 0.70 + 0.20 * math.sin(5.0 * a + 0.2) + 0.08 * math.cos(2.0 * a)


def _wavy_bean_radius(a: float) -> float:
    """豆形窗口的极径函数。"""
    return 0.68 + 0.18 * math.sin(a - 0.5) + 0.13 * math.sin(3.0 * a + 0.8)


def _gear_radius(a: float) -> float:
    """不对称齿轮形窗口的极径函数。"""
    return 0.68 + 0.15 * math.sin(7.0 * a) + 0.09 * math.cos(4.0 * a + 0.3)


def path_length(points: np.ndarray | list[np.ndarray]) -> float:
    """折线长度：L=sum_i ||p_{i+1}-p_i||。"""
    pts = np.asarray(points, dtype=np.float64)
    return 0.0 if len(pts) < 2 else float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
