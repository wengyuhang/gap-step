"""Paper-style static and animated visualization for Experiment B."""

from __future__ import annotations

from pathlib import Path
import tempfile

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
from matplotlib.patches import Polygon as PolygonPatch, Rectangle  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402
from shapely.geometry import MultiPoint  # noqa: E402

from .geometry import cuboid_world_vertices
from .stress_case import WORLD_CLEARANCE, MethodSnapshot, StressCase


_OLD = "#d55e00"
_OURS = "#0072b2"
_FRAME = "#8c8c8c"

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def _closed(points: np.ndarray) -> np.ndarray:
    return np.vstack((points, points[0]))


def _draw_cuboid(axis, vertices: np.ndarray, color: str, alpha: float = 0.24) -> None:
    faces = (
        (0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
        (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5),
    )
    axis.add_collection3d(Poly3DCollection(
        [[vertices[index] for index in face] for face in faces],
        facecolor=color, edgecolor=color, linewidth=0.8, alpha=alpha,
    ))


def _draw_local_panel(axis, case: StressCase, snapshot: MethodSnapshot) -> None:
    frame = case.frame_at(snapshot.time)
    boundary = frame.boundary_world_2d(case.boundary_local)
    inset = case.world_inset_at(snapshot.time)
    axis.add_patch(PolygonPatch(boundary, closed=True, facecolor="#f4f4f4", edgecolor="black", linewidth=2.0, label="physical aperture"))
    axis.plot(*_closed(inset).T, linestyle="--", color="#7f7f7f", linewidth=1.5, label="fixed-world 0.315 m inset")
    for component in snapshot.metrics.intersection_components:
        axis.add_patch(PolygonPatch(component, closed=True, facecolor="#55a868", edgecolor="#2f6f3e", alpha=0.58, label="section inside gate"))
    for component in snapshot.metrics.outside_components:
        axis.add_patch(PolygonPatch(
            component, closed=True, facecolor="#ef3b2c", edgecolor="#8c1d20",
            linewidth=2.0, hatch="////", alpha=0.96, zorder=7, label="outside area E",
        ))
    section = _closed(snapshot.section.vertices_2d)
    axis.plot(section[:, 0], section[:, 1], color="#5e3c99", linewidth=2.0, label="true UAV section")
    axis.scatter(*snapshot.center_plane, marker="x", s=58, linewidth=2.0, color="black", zorder=8, label="UAV center")
    y = snapshot.center_plane[1]
    edge_x = 2.0 * frame.scale
    axis.annotate("", (edge_x, y + 0.24), (edge_x - snapshot.boundary_distance, y + 0.24), arrowprops=dict(arrowstyle="<->", color="#333333"))
    axis.text(edge_x - 0.5 * snapshot.boundary_distance, y + 0.29, f"d={snapshot.boundary_distance:.3f} m", ha="center", fontsize=8)
    axis.annotate("", (snapshot.center_plane[0] + snapshot.support_half_width, y - 0.27), (snapshot.center_plane[0], y - 0.27), arrowprops=dict(arrowstyle="<->", color="#5e3c99"))
    axis.text(snapshot.center_plane[0] + 0.5 * snapshot.support_half_width, y - 0.42, f"h_B={snapshot.support_half_width:.3f} m", ha="center", fontsize=8, color="#5e3c99")
    margin = WORLD_CLEARANCE
    facts = (
        f"m_world={margin:.3f} m\n"
        f"A={snapshot.metrics.section_area:.5f} m2\n"
        f"C={snapshot.metrics.intersection_area:.5f} m2\n"
        f"E={snapshot.metrics.outside_area:.3e} m2"
    )
    axis.text(0.02, 0.98, facts, transform=axis.transAxes, va="top", fontsize=8,
              bbox=dict(boxstyle="round", facecolor="white", alpha=0.88, edgecolor="#bbbbbb"))
    if snapshot.metrics.whole_body_collision and snapshot.metrics.outside_components:
        outside = snapshot.metrics.outside_components[0]
        tip = np.mean(outside, axis=0)
        axis.annotate(
            "outside E", xy=tip, xytext=(tip[0] - 0.22, tip[1] + 0.12),
            color="#a50f15", fontweight="bold", fontsize=8,
            arrowprops=dict(arrowstyle="->", color="#a50f15", linewidth=1.5),
            zorder=10,
        )
    status = "COLLISION" if snapshot.metrics.whole_body_collision else "WHOLE-BODY SAFE"
    axis.set_title(f"{snapshot.method}: {status}", color=_OLD if snapshot.method.startswith("Old") else _OURS, fontweight="bold")
    # Experiment B is a local geometric explanation.  Both methods use these
    # exact same limits, while the 3-D panel retains the complete L aperture.
    edge_x = 2.0 * frame.scale
    axis.set_xlim(edge_x - 0.72, edge_x + 0.10)
    axis.set_ylim(snapshot.center_plane[1] - 0.43, snapshot.center_plane[1] + 0.43)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("gate-plane u [m]")
    axis.set_ylabel("gate-plane v [m]")
    axis.grid(alpha=0.18)


def _draw_simple_local_panel(axis, case: StressCase, snapshot: MethodSnapshot) -> None:
    """A presentation-first front view with one unambiguous takeaway."""

    frame = case.frame_at(snapshot.time)
    boundary = frame.boundary_world_2d(case.boundary_local)
    inset = case.world_inset_at(snapshot.time)
    axis.add_patch(PolygonPatch(
        boundary, closed=True, facecolor="#f7f7f7", edgecolor="#202020",
        linewidth=4.0, label="物理门框",
    ))
    axis.plot(
        *_closed(inset).T, linestyle="--", color="#7f7f7f", linewidth=2.0,
        label="固定世界裕度 0.315 m",
    )
    for component in snapshot.metrics.intersection_components:
        axis.add_patch(PolygonPatch(
            component, closed=True, facecolor="#74c476", edgecolor="#238b45",
            alpha=0.72,
        ))
    for component in snapshot.metrics.outside_components:
        axis.add_patch(PolygonPatch(
            component, closed=True, facecolor="#ef3b2c", edgecolor="#99000d",
            linewidth=2.0, alpha=1.0,
        ))
    if len(snapshot.section.vertices_2d) >= 3:
        section = _closed(snapshot.section.vertices_2d)
        axis.plot(section[:, 0], section[:, 1], color="#54278f", linewidth=2.6)
    axis.scatter(
        *snapshot.center_plane, marker="x", s=72, linewidth=2.5,
        color="black", zorder=10,
    )

    if snapshot.metrics.whole_body_collision:
        if snapshot.metrics.outside_components:
            contact = np.mean(snapshot.metrics.outside_components[0], axis=0)
        else:
            contact = snapshot.section.vertices_2d[np.argmax(snapshot.section.vertices_2d[:, 0])]
        axis.scatter(
            *contact, marker="X", s=220, color="#cb181d", edgecolor="white",
            linewidth=1.3, zorder=15,
        )
        axis.annotate(
            "首次接触\n立即停止", xy=contact,
            xytext=(contact[0] - 0.20, contact[1] - 0.20),
            fontsize=11, fontweight="bold", color="#a50f15", ha="center",
            arrowprops=dict(arrowstyle="->", color="#a50f15", linewidth=2.0),
        )
        title = "旧方法：中心穿越前发生碰撞"
        title_color = _OLD
        takeaway = (
            f"t_col = {snapshot.time:.3f} s\n"
            f"此时中心尚离窗口平面 {snapshot.center_normal_distance:.3f} m\n"
            f"越界面积 E = {snapshot.metrics.outside_area:.1e} m²"
        )
    else:
        axis.text(
            0.96, 0.08, "整机安全", transform=axis.transAxes,
            ha="right", va="bottom", fontsize=13, fontweight="bold",
            color="#006d2c",
        )
        title = "新方法：同一时刻安全"
        title_color = _OURS
        takeaway = (
            f"t = {snapshot.time:.3f} s\n"
            f"完整截面位于开口内\n"
            f"越界面积 E = 0"
        )
    axis.text(
        0.03, 0.97, takeaway, transform=axis.transAxes, va="top", fontsize=10,
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", alpha=0.92, edgecolor="#bbbbbb"),
    )
    axis.set_title(title, color=title_color, fontweight="bold", fontsize=14)
    edge_x = 2.0 * frame.scale
    axis.set_xlim(edge_x - 0.78, edge_x + 0.12)
    axis.set_ylim(snapshot.center_plane[1] - 0.48, snapshot.center_plane[1] + 0.48)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("窗口平面横向 u [m]")
    axis.set_ylabel("窗口平面纵向 v [m]")
    axis.grid(alpha=0.16)


def _draw_3d_crossing_closeup(axis, case: StressCase, snapshot: MethodSnapshot) -> None:
    """Show the actual 3-D cuboid straddling the gate plane at t*."""

    frame = case.frame_at(snapshot.time)
    transform = np.column_stack((frame.basis, frame.normal))

    def local3(points_world: np.ndarray) -> np.ndarray:
        return (np.asarray(points_world, dtype=float) - frame.center) @ transform

    color = _OLD if snapshot.method.startswith("Old") else _OURS
    boundary_2d = frame.boundary_world_2d(case.boundary_local)
    inset_2d = case.world_inset_at(snapshot.time)
    boundary_3d = np.column_stack((boundary_2d, np.zeros(len(boundary_2d))))
    inset_3d = np.column_stack((inset_2d, np.zeros(len(inset_2d))))
    closed_boundary = _closed(boundary_3d)
    closed_inset = _closed(inset_3d)

    # The zero-thickness physical frame lies in n=0.  Draw it heavily so the
    # red section outside the aperture visibly crosses the frame boundary.
    axis.plot(
        closed_boundary[:, 0], closed_boundary[:, 1], closed_boundary[:, 2],
        color="#202020", linewidth=6.0, solid_capstyle="round",
    )
    axis.plot(
        closed_boundary[:, 0], closed_boundary[:, 1], closed_boundary[:, 2],
        color=_FRAME, linewidth=3.2, solid_capstyle="round",
    )
    axis.plot(
        closed_inset[:, 0], closed_inset[:, 1], closed_inset[:, 2],
        color="#777777", linestyle="--", linewidth=1.2,
    )

    body_vertices_world = cuboid_world_vertices(
        case.cuboid, snapshot.center_world, snapshot.rotation
    )
    body_vertices_local = local3(body_vertices_world)
    _draw_cuboid(axis, body_vertices_local, color, alpha=0.20)

    for component in snapshot.metrics.intersection_components:
        polygon = np.column_stack((component, np.zeros(len(component))))
        axis.add_collection3d(Poly3DCollection(
            [polygon], facecolor="#55a868", edgecolor="#2f6f3e",
            linewidth=1.2, alpha=0.78,
        ))
    for component in snapshot.metrics.outside_components:
        polygon = np.column_stack((component, np.zeros(len(component))))
        axis.add_collection3d(Poly3DCollection(
            [polygon], facecolor="#ef3b2c", edgecolor="#8c1d20",
            linewidth=2.2, alpha=1.0,
        ))
        center = np.mean(polygon, axis=0)
        axis.scatter(
            *center, marker="X", s=230, color="#b30000", edgecolor="white",
            linewidth=1.2, depthshade=False, zorder=20,
        )
        axis.text(
            center[0] - 0.18, center[1] + 0.08, center[2] + 0.08,
            "FRAME / BODY\nINTERSECTION", color="#b30000",
            fontsize=7, fontweight="bold", ha="center",
        )

    center_local = local3(snapshot.center_world[None, :])[0]
    axis.scatter(*center_local, marker="x", s=55, linewidth=2.0, color="black", depthshade=False)
    # A translucent patch makes the gate plane orientation visible without
    # pretending the solid frame fills the aperture.
    edge_x = 2.0 * frame.scale
    y_center = snapshot.center_plane[1]
    plane_patch = np.array([
        [edge_x - 0.68, y_center - 0.40, 0.0],
        [edge_x + 0.08, y_center - 0.40, 0.0],
        [edge_x + 0.08, y_center + 0.40, 0.0],
        [edge_x - 0.68, y_center + 0.40, 0.0],
    ])
    axis.add_collection3d(Poly3DCollection(
        [plane_patch], facecolor="#9ecae1", edgecolor="none", alpha=0.08,
    ))

    facts = (
        f"|center-plane|={snapshot.center_normal_distance:.4f} m\n"
        f"A={snapshot.metrics.section_area:.5f} m2\n"
        f"E={snapshot.metrics.outside_area:.3e} m2"
    )
    axis.text2D(
        0.02, 0.96, facts, transform=axis.transAxes, va="top", fontsize=7,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.86, edgecolor="#bbbbbb"),
    )
    status = (
        f"3-D COLLISION AT t_col={snapshot.time:.3f}s"
        if snapshot.metrics.whole_body_collision
        else f"3-D SAFE AT t={snapshot.time:.3f}s"
    )
    axis.set_title(status, color=color, fontweight="bold", fontsize=10)
    axis.set_xlim(edge_x - 0.68, edge_x + 0.08)
    axis.set_ylim(y_center - 0.40, y_center + 0.40)
    axis.set_zlim(-0.23, 0.23)
    axis.set_xlabel("u [m]", fontsize=8)
    axis.set_ylabel("v [m]", fontsize=8)
    axis.set_zlabel("plane normal n [m]", fontsize=8)
    axis.tick_params(labelsize=7)
    axis.view_init(elev=24, azim=-56)
    axis.set_box_aspect((1.15, 1.0, 0.72))


def _draw_simple_body_panel(axis, case: StressCase, snapshot: MethodSnapshot) -> None:
    """Show the complete cuboid and gate plane with minimal annotations."""

    frame = case.frame_at(snapshot.time)
    transform = np.column_stack((frame.basis, frame.normal))

    def local3(points_world: np.ndarray) -> np.ndarray:
        return (np.asarray(points_world, dtype=float) - frame.center) @ transform

    color = _OLD if snapshot.method.startswith("Old") else _OURS
    boundary_2d = frame.boundary_world_2d(case.boundary_local)
    boundary_3d = np.column_stack((boundary_2d, np.zeros(len(boundary_2d))))
    closed_boundary = _closed(boundary_3d)
    axis.plot(
        closed_boundary[:, 0], closed_boundary[:, 1], closed_boundary[:, 2],
        color="#202020", linewidth=6.0, solid_capstyle="round",
    )
    axis.plot(
        closed_boundary[:, 0], closed_boundary[:, 1], closed_boundary[:, 2],
        color=_FRAME, linewidth=3.2, solid_capstyle="round",
    )

    edge_x = 2.0 * frame.scale
    y_center = snapshot.center_plane[1]
    plane_patch = np.array([
        [edge_x - 0.72, y_center - 0.43, 0.0],
        [edge_x + 0.10, y_center - 0.43, 0.0],
        [edge_x + 0.10, y_center + 0.43, 0.0],
        [edge_x - 0.72, y_center + 0.43, 0.0],
    ])
    axis.add_collection3d(Poly3DCollection(
        [plane_patch], facecolor="#9ecae1", edgecolor="none", alpha=0.10,
    ))

    vertices_world = cuboid_world_vertices(
        case.cuboid, snapshot.center_world, snapshot.rotation
    )
    vertices_local = local3(vertices_world)
    _draw_cuboid(axis, vertices_local, color, alpha=0.30)
    for component in snapshot.metrics.intersection_components:
        polygon = np.column_stack((component, np.zeros(len(component))))
        axis.add_collection3d(Poly3DCollection(
            [polygon], facecolor="#55a868", edgecolor="#238b45",
            linewidth=1.2, alpha=0.70,
        ))

    center = local3(snapshot.center_world[None, :])[0]
    projection = center.copy()
    projection[2] = 0.0
    axis.scatter(*center, marker="o", s=45, color="black", depthshade=False, zorder=20)
    axis.plot(
        [center[0], projection[0]], [center[1], projection[1]],
        [center[2], projection[2]], color="black", linestyle=":", linewidth=2.0,
    )

    if snapshot.metrics.whole_body_collision:
        outside = snapshot.metrics.outside_components[0]
        contact = np.array([*np.mean(outside, axis=0), 0.0])
        axis.scatter(
            *contact, marker="X", s=230, color="#cb181d", edgecolor="white",
            linewidth=1.2, depthshade=False, zorder=25,
        )
        status = "机体碰到门框  →  立即停止"
        title = "旧方法：完整机体发生碰撞"
        status_color = "#a50f15"
    else:
        status = "完整机体与门框无交叠"
        title = "新方法：完整机体安全"
        status_color = "#006d2c"

    axis.text2D(
        0.02, 0.96,
        f"中心到窗口平面 = {snapshot.center_normal_distance:.3f} m\n"
        "（两种方法此时进度相同）",
        transform=axis.transAxes, va="top", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.90, edgecolor="#bbbbbb"),
    )
    axis.text2D(
        0.50, 0.13, status, transform=axis.transAxes, ha="center",
        fontsize=11, fontweight="bold", color=status_color,
    )
    axis.set_title(title, color=color, fontweight="bold", fontsize=13)
    axis.set_xlim(edge_x - 0.72, edge_x + 0.10)
    axis.set_ylim(y_center - 0.43, y_center + 0.43)
    axis.set_zlim(-0.24, 0.24)
    axis.set_xlabel("窗口横向 u [m]", fontsize=8)
    axis.set_ylabel("窗口纵向 v [m]", fontsize=8)
    axis.set_zlabel("法向 n [m]", fontsize=8)
    axis.tick_params(labelsize=7)
    axis.view_init(elev=24, azim=-56)
    axis.set_box_aspect((1.15, 1.0, 0.72))


def _draw_side_view(axis, case: StressCase, snapshot: MethodSnapshot) -> None:
    """Orthographic u-n view: whole body plus its actual plane intersection."""

    frame = case.frame_at(snapshot.time)
    transform = np.column_stack((frame.basis, frame.normal))
    vertices_world = cuboid_world_vertices(
        case.cuboid, snapshot.center_world, snapshot.rotation
    )
    vertices_local = (vertices_world - frame.center) @ transform
    body_projection = np.column_stack((vertices_local[:, 0], vertices_local[:, 2]))
    hull = MultiPoint(body_projection).convex_hull
    hull_points = np.asarray(hull.exterior.coords, dtype=float)
    color = _OLD if snapshot.method.startswith("Old") else _OURS

    edge_x = 2.0 * frame.scale
    x_left = edge_x - 0.82
    x_right = edge_x + 0.20
    axis.axhline(0.0, color="#6baed6", linewidth=2.0, alpha=0.75)
    axis.text(x_left + 0.02, 0.018, "窗口平面 n=0", color="#2171b5", fontsize=9)
    axis.plot(
        [edge_x, x_right], [0.0, 0.0], color="#202020", linewidth=11,
        solid_capstyle="butt", zorder=8,
    )
    axis.plot(
        [edge_x, x_right], [0.0, 0.0], color=_FRAME, linewidth=6,
        solid_capstyle="butt", zorder=9,
    )
    axis.text(edge_x + 0.09, 0.025, "门框", ha="center", fontsize=10, fontweight="bold")

    axis.fill(
        hull_points[:, 0], hull_points[:, 1], facecolor=color,
        edgecolor=color, linewidth=2.2, alpha=0.28, zorder=4,
    )
    center_local = (snapshot.center_world - frame.center) @ transform
    nose_world = (
        snapshot.center_world
        + case.cuboid.half_extents[0] * snapshot.rotation[:, 0]
    )
    nose_local = (nose_world - frame.center) @ transform
    axis.annotate(
        "机头先穿", xy=(nose_local[0], nose_local[2]),
        xytext=(center_local[0] - 0.10, center_local[2] - 0.10),
        color=color, fontsize=10, fontweight="bold",
        arrowprops=dict(arrowstyle="->", color=color, linewidth=2.0),
        zorder=12,
    )
    axis.scatter(center_local[0], center_local[2], color="black", s=42, zorder=12)
    axis.annotate(
        "", xy=(center_local[0], 0.0), xytext=(center_local[0], center_local[2]),
        arrowprops=dict(arrowstyle="<->", color="black", linewidth=1.5),
    )

    section_u = snapshot.section.vertices_2d[:, 0]
    section_left = float(np.min(section_u))
    section_right = float(np.max(section_u))
    safe_right = min(section_right, edge_x)
    axis.plot(
        [section_left, safe_right], [0.0, 0.0], color="#238b45",
        linewidth=7, solid_capstyle="butt", zorder=11,
    )
    if snapshot.metrics.whole_body_collision:
        axis.plot(
            [edge_x, section_right], [0.0, 0.0], color="#cb181d",
            linewidth=8, solid_capstyle="butt", zorder=13,
        )
        axis.scatter(
            edge_x, 0.0, marker="X", s=190, color="#cb181d",
            edgecolor="white", linewidth=1.2, zorder=14,
        )
        axis.annotate(
            "机体与门框接触\n立即停止", xy=(edge_x, 0.0),
            xytext=(edge_x - 0.22, 0.19), ha="center", color="#a50f15",
            fontsize=10, fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#a50f15", linewidth=1.8),
        )
        title = "旧方法侧视图：平面交线进入门框"
    else:
        axis.annotate(
            "机体穿过开口\n未接触门框", xy=(section_right, 0.0),
            xytext=(section_right - 0.16, 0.19), ha="center", color="#006d2c",
            fontsize=10, fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#006d2c", linewidth=1.8),
        )
        title = "新方法侧视图：平面交线留在开口内"

    side = "前" if center_local[2] < 0.0 else "后"
    axis.text(
        0.02, 0.05,
        f"中心仍在窗口平面{side}方 {abs(center_local[2]):.3f} m\n"
        "机体：0.530 × 0.530 × 0.118 m（正方形底面，扁平机身）",
        transform=axis.transAxes, fontsize=10,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.90, edgecolor="#bbbbbb"),
    )
    axis.set_title(title, color=color, fontweight="bold", fontsize=13)
    axis.set_xlim(x_left, x_right)
    axis.set_ylim(-0.31, 0.31)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("窗口横向 u [m]")
    axis.set_ylabel("窗口法向 n [m]")
    axis.grid(alpha=0.18)


def _draw_compact_side_state(axis, case: StressCase, snapshot: MethodSnapshot) -> None:
    """Complete-body orthographic side state for the static timeline."""

    frame = case.frame_at(snapshot.time)
    transform = np.column_stack((frame.basis, frame.normal))
    vertices_world = cuboid_world_vertices(
        case.cuboid, snapshot.center_world, snapshot.rotation
    )
    vertices_local = (vertices_world - frame.center) @ transform
    projection = np.column_stack((vertices_local[:, 0], vertices_local[:, 2]))
    hull = np.asarray(MultiPoint(projection).convex_hull.exterior.coords, dtype=float)
    color = _OLD if snapshot.method.startswith("Old") else _OURS
    edge_x = 2.0 * frame.scale

    center = (snapshot.center_world - frame.center) @ transform
    nose_world = snapshot.center_world + case.cuboid.half_extents[0] * snapshot.rotation[:, 0]
    nose = (nose_world - frame.center) @ transform

    x_min = min(float(np.min(hull[:, 0])), edge_x) - 0.16
    x_max = max(float(np.max(hull[:, 0])), edge_x + 0.12) + 0.12
    n_min = min(float(np.min(hull[:, 1])), 0.0) - 0.14
    n_max = max(float(np.max(hull[:, 1])), 0.0) + 0.14
    axis.axhline(0.0, color="#6baed6", linewidth=2.0, alpha=0.85)
    axis.plot(
        [edge_x, edge_x + 0.12], [0.0, 0.0], color="#202020",
        linewidth=10, solid_capstyle="butt", zorder=8,
    )
    axis.plot(
        [edge_x, edge_x + 0.12], [0.0, 0.0], color=_FRAME,
        linewidth=5, solid_capstyle="butt", zorder=9,
    )
    axis.fill(
        hull[:, 0], hull[:, 1], facecolor=color, edgecolor=color,
        linewidth=2.2, alpha=0.34, zorder=4,
    )
    axis.scatter(center[0], center[2], color="black", s=30, zorder=12)
    axis.annotate(
        "机头", xy=(nose[0], nose[2]), xytext=(center[0], center[2]),
        color=color, fontsize=8, fontweight="bold",
        arrowprops=dict(arrowstyle="->", color=color, linewidth=1.8),
        zorder=13,
    )

    if len(snapshot.section.vertices_2d) >= 3:
        section_u = snapshot.section.vertices_2d[:, 0]
        section_left = float(np.min(section_u))
        section_right = float(np.max(section_u))
        axis.plot(
            [section_left, min(section_right, edge_x)], [0.0, 0.0],
            color="#238b45", linewidth=6, solid_capstyle="butt", zorder=11,
        )
        if snapshot.metrics.whole_body_collision:
            axis.plot(
                [edge_x, max(edge_x, section_right)], [0.0, 0.0],
                color="#cb181d", linewidth=7, solid_capstyle="butt", zorder=13,
            )
            axis.scatter(
                edge_x, 0.0, marker="X", s=145, color="#cb181d",
                edgecolor="white", linewidth=1.0, zorder=14,
            )
            axis.text(
                0.04, 0.90, "碰撞", transform=axis.transAxes,
                color="#b30000", fontweight="bold", fontsize=10,
            )
        else:
            axis.text(
                0.04, 0.90, "整机安全", transform=axis.transAxes,
                color="#006d2c", fontweight="bold", fontsize=9,
            )
    else:
        axis.text(
            0.04, 0.90, "未与窗口平面相交", transform=axis.transAxes,
            color="#555555", fontsize=8,
        )

    axis.text(edge_x + 0.02, 0.025, "门框", fontsize=8, fontweight="bold")
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(n_min, n_max)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("u [m]", fontsize=8)
    axis.set_ylabel("n [m]", fontsize=8)
    axis.tick_params(labelsize=7)
    axis.grid(alpha=0.18)


def _draw_scene(
    axis,
    case: StressCase,
    time: float,
    method: str,
    *,
    continue_after_collision: bool = False,
    compact: bool = False,
) -> None:
    color = _OLD if method == "Old-0.315" else _OURS
    frame = case.frame_at(time)
    boundary_world = frame.plane_to_world(frame.boundary_world_2d(case.boundary_local))
    closed = _closed(boundary_world)
    axis.plot(closed[:, 0], closed[:, 1], closed[:, 2], color="#343a40", linewidth=6.0, solid_capstyle="round")
    axis.plot(closed[:, 0], closed[:, 1], closed[:, 2], color=_FRAME, linewidth=3.5, solid_capstyle="round")

    planned_times = np.linspace(0.0, case.total_time, 121)
    planned = np.asarray(
        [case.planned_position(method, float(value)) for value in planned_times]
    )
    axis.plot(
        planned[:, 0], planned[:, 1], planned[:, 2], color=color,
        linestyle="--", linewidth=1.2, alpha=0.32, label="计划路线（未执行部分）",
    )

    executed_end = (
        time
        if continue_after_collision or method != "Old-0.315"
        else min(time, case.collision_time)
    )
    trail_times = np.linspace(
        0.0, executed_end, max(2, int(90 * executed_end / case.total_time) + 1)
    )
    position_at = case.planned_position if continue_after_collision else case.trajectory_position
    trail = np.asarray([position_at(method, float(value)) for value in trail_times])
    axis.plot(
        trail[:, 0], trail[:, 1], trail[:, 2], color=color,
        linewidth=3.0,
        label="原计划轨迹回放" if continue_after_collision else "实际执行轨迹",
    )
    center = position_at(method, time)
    body_time = (
        time
        if continue_after_collision or method != "Old-0.315"
        else min(time, case.collision_time)
    )
    rotation = case.body_rotation(method, body_time)
    vertices = cuboid_world_vertices(case.cuboid, center, rotation)
    _draw_cuboid(axis, vertices, color, alpha=0.38)
    axis.scatter(*center, color=color, s=28)
    forward = rotation[:, 0]
    nose = center + case.cuboid.half_extents[0] * forward
    axis.quiver(
        *center, *forward, length=case.cuboid.half_extents[0],
        color=color, linewidth=2.2, arrow_length_ratio=0.24,
    )
    if not compact:
        axis.text(*nose, "  机头", color=color, fontsize=8, fontweight="bold")
    if method == "Old-0.315" and continue_after_collision:
        collision_center = case.planned_position(method, case.collision_time)
        current = case.snapshot(method, time, executed=False)
        if time >= case.collision_time and not current.metrics.whole_body_collision:
            axis.scatter(
                *collision_center, marker="X", s=150, color="#b30000",
                edgecolor="white", linewidth=1.2, zorder=12,
            )
            if not compact:
                axis.text(
                    collision_center[0], collision_center[1], collision_center[2] + 0.30,
                    "首次碰撞点\n回放中忽略终止",
                    color="#b30000", fontweight="bold", ha="center", fontsize=8,
                )
        if current.metrics.whole_body_collision:
            axis.scatter(
                *center, marker="X", s=190, color="#b30000",
                edgecolor="white", linewidth=1.2, zorder=13,
            )
            if compact:
                axis.text2D(
                    0.04, 0.90, "当前碰撞",
                    transform=axis.transAxes, color="#b30000",
                    fontweight="bold", fontsize=10,
                )
            else:
                axis.text(
                    center[0], center[1], center[2] + 0.48,
                    "当前机体碰撞\n仍沿原计划轨迹继续",
                    color="#b30000", fontweight="bold", ha="center", fontsize=9,
                )
    elif method == "Old-0.315" and time >= case.collision_time:
        axis.scatter(*center, marker="X", s=180, color="#b30000", edgecolor="white", linewidth=1.2, zorder=12)
        axis.text(
            center[0], center[1], center[2] + 0.40,
            f"碰撞并立即停止  t={case.collision_time:.3f} s\n未到达名义中心穿越点",
            color="#b30000", fontweight="bold", ha="center", fontsize=9,
        )
    axis.scatter(*case.start, marker="*", s=100, color="#222222", label="共同起点 = 终点")
    if not compact:
        axis.text(*case.start, "  起点/终点", fontsize=8)
    axis.set_xlim(-5.2, 4.0)
    axis.set_ylim(-4.3, 3.5)
    axis.set_zlim(-0.4, 4.2)
    axis.set_xlabel("" if compact else "x [m]")
    axis.set_ylabel("" if compact else "y [m]")
    axis.set_zlabel("" if compact else "z [m]")
    axis.view_init(elev=24, azim=-58)
    axis.set_box_aspect((1.45, 1.2, 0.75))
    if not compact:
        axis.legend(loc="upper left", fontsize=7)
    if method == "Old-0.315":
        if continue_after_collision:
            status = "忽略碰撞终止，完整回放原计划轨迹"
            title = f"旧方法：{status}（t={time:.2f} s）"
        elif time >= case.collision_time:
            title = f"旧方法：{case.collision_time:.3f} s 碰撞后停止（画面 t={time:.2f} s）"
        else:
            title = f"旧方法：正在接近动态窗口（t={time:.2f} s）"
    else:
        status = "安全穿越后返回终点" if time >= case.crossing_time else "正在安全接近"
        title = f"新方法：{status}（t={time:.2f} s）"
    axis.set_title(title, color=color, fontweight="bold")


def _render_full_planned_frame(case: StressCase, time: float, path: Path, *, dpi: int) -> None:
    """Counterfactual replay in which Old follows its plan after collision."""

    figure = plt.figure(figsize=(13.8, 5.8))
    old_axis = figure.add_subplot(1, 2, 1, projection="3d")
    ours_axis = figure.add_subplot(1, 2, 2, projection="3d")
    _draw_scene(
        old_axis, case, time, "Old-0.315", continue_after_collision=True
    )
    _draw_scene(
        ours_axis, case, time, "Ours", continue_after_collision=True
    )
    figure.suptitle(
        "完整穿越计划轨迹回放（反事实诊断图）\n"
        "旧方法碰撞后仍按原计划继续；此图不表示真实物理执行",
        fontsize=14, y=0.98, fontweight="bold", color="#8c1d20",
    )
    figure.subplots_adjust(left=0.04, right=0.98, bottom=0.08, top=0.83, wspace=0.12)
    figure.savefig(path, dpi=dpi, facecolor="white")
    plt.close(figure)


def _render_full_planned_timeline(case: StressCase, path: Path, *, dpi: int) -> None:
    """One static 4x5 montage pairing 3-D and full-body side views."""

    scan = np.linspace(case.crossing_time, min(case.total_time, case.crossing_time + 3.0), 301)
    exit_time = float(scan[-1])
    seen_contact = False
    for instant in scan:
        touching = any(
            case.snapshot(method, float(instant), executed=False).section.area > 1.0e-12
            for method in ("Old-0.315", "Ours")
        )
        seen_contact = seen_contact or touching
        if seen_contact and not touching:
            exit_time = float(instant)
            break

    instants = (
        max(0.0, case.collision_time - 0.55),
        case.collision_time,
        case.crossing_time,
        exit_time,
        case.total_time,
    )
    method_stages = {
        "Old-0.315": ("接近窗口", "首次碰撞", "中心穿越", "离开窗口", "返回终点"),
        "Ours": ("接近窗口", "同一时刻安全", "中心穿越", "离开窗口", "返回终点"),
    }
    figure = plt.figure(figsize=(20.0, 13.6))
    for method_index, method in enumerate(("Old-0.315", "Ours")):
        for column, (stage, instant) in enumerate(zip(method_stages[method], instants)):
            scene_axis = figure.add_subplot(
                4, 5, 2 * method_index * 5 + column + 1, projection="3d"
            )
            _draw_scene(
                scene_axis, case, float(instant), method,
                continue_after_collision=True, compact=True,
            )
            scene_axis.set_title("")
            frame = case.frame_at(float(instant))
            shift = frame.center - case.center0
            yaw = np.degrees(np.arctan2(frame.basis[1, 0], frame.basis[0, 0]))
            scene_axis.text2D(
                0.02, 0.03,
                f"门框变化：Δp=({shift[0]:+.2f},{shift[1]:+.2f},{shift[2]:+.2f}) m\n"
                f"缩放 s={frame.scale:.3f}，旋转角={yaw:+.1f}°",
                transform=scene_axis.transAxes, fontsize=7, va="bottom",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.88,
                          edgecolor="#aaaaaa"),
            )

            side_axis = figure.add_subplot(
                4, 5, (2 * method_index + 1) * 5 + column + 1
            )
            snapshot = case.snapshot(method, float(instant), executed=False)
            _draw_compact_side_state(side_axis, case, snapshot)
            if method_index == 0 and column == 1:
                status = "碰撞，但图中继续"
                title_color = "#b30000"
            elif method_index == 0 and column > 1:
                status = "已忽略碰撞终止"
                title_color = _OLD
            else:
                status = stage
                title_color = _OLD if method_index == 0 else _OURS
            scene_axis.set_title(
                f"{stage}\nt={instant:.3f} s\n{status}",
                color=title_color, fontweight="bold", fontsize=10,
            )
            scene_axis.tick_params(labelsize=5, pad=0)
            side_axis.tick_params(labelsize=6, pad=1)

    figure.text(
        0.008, 0.785, "旧方法\n3D",
        color=_OLD, fontsize=12, fontweight="bold", va="center", ha="left",
    )
    figure.text(
        0.008, 0.585, "旧方法\n完整机体\n侧视",
        color=_OLD, fontsize=12, fontweight="bold", va="center", ha="left",
    )
    figure.text(
        0.008, 0.370, "新方法\n3D",
        color=_OURS, fontsize=12, fontweight="bold", va="center", ha="left",
    )
    figure.text(
        0.008, 0.165, "新方法\n完整机体\n侧视",
        color=_OURS, fontsize=12, fontweight="bold", va="center", ha="left",
    )
    figure.suptitle(
        "完整穿越过程：3D 与完整机体侧视同步对照\n"
        "（旧方法忽略碰撞终止，继续沿原规划轨迹）",
        fontsize=16, fontweight="bold", color="#6b1d1d", y=0.975,
    )
    figure.subplots_adjust(
        left=0.065, right=0.995, bottom=0.045, top=0.89,
        wspace=0.16, hspace=0.30,
    )
    figure.savefig(path, dpi=dpi, facecolor="white")
    plt.close(figure)


def _render_frame(case: StressCase, time: float, path: Path, *, dpi: int) -> None:
    old = case.snapshot("Old-0.315", case.collision_time, executed=False)
    ours = case.snapshot("Ours", case.collision_time, executed=False)
    figure = plt.figure(figsize=(13.8, 8.3))
    grid = figure.add_gridspec(2, 2, height_ratios=(1.15, 1.0))
    old_scene = figure.add_subplot(grid[0, 0], projection="3d")
    ours_scene = figure.add_subplot(grid[0, 1], projection="3d")
    old_axis = figure.add_subplot(grid[1, 0])
    ours_axis = figure.add_subplot(grid[1, 1])
    _draw_scene(old_scene, case, time, "Old-0.315")
    _draw_scene(ours_scene, case, time, "Ours")
    _draw_side_view(old_axis, case, old)
    _draw_side_view(ours_axis, case, ours)
    figure.suptitle(
        "实验 B：名义中心穿越时是安全的，但动态窗口会在此前撞到整机\n"
        f"{case.collision_time:.3f} s 碰撞并停止  →  {case.crossing_time:.3f} s 名义中心穿越点（旧方法未到达）"
        f"  |  世界安全裕度始终为 {WORLD_CLEARANCE:.3f} m",
        fontsize=13, y=0.985, fontweight="bold",
    )
    figure.subplots_adjust(left=0.055, right=0.985, bottom=0.07, top=0.88, wspace=0.18, hspace=0.25)
    figure.savefig(path, dpi=dpi, facecolor="white")
    plt.close(figure)


def _render_3d_diagnostic(case: StressCase, path: Path, *, dpi: int) -> None:
    """Preserve the spatial contact proof outside the presentation-first GIF."""

    old = case.snapshot("Old-0.315", case.collision_time, executed=False)
    ours = case.snapshot("Ours", case.collision_time, executed=False)
    figure = plt.figure(figsize=(11.8, 5.2))
    old_axis = figure.add_subplot(1, 2, 1, projection="3d")
    ours_axis = figure.add_subplot(1, 2, 2, projection="3d")
    _draw_3d_crossing_closeup(old_axis, case, old)
    _draw_3d_crossing_closeup(ours_axis, case, ours)
    figure.suptitle(
        f"三维接触证据：t={case.collision_time:.3f} s，旧方法整机与移动窗口边界相交",
        fontsize=13, fontweight="bold",
    )
    figure.subplots_adjust(left=0.02, right=0.98, bottom=0.07, top=0.88, wspace=0.10)
    figure.savefig(path, dpi=dpi, facecolor="white")
    plt.close(figure)


def _render_body_model_figure(case: StressCase, path: Path, *, dpi: int) -> None:
    """Document the flat square-footprint box independently of gate views."""

    length, width, height = 2.0 * case.cuboid.half_extents
    figure, axes = plt.subplots(1, 2, figsize=(10.5, 4.4))
    top, side = axes
    top.add_patch(Rectangle(
        (-0.5 * length, -0.5 * width), length, width,
        facecolor="#9ecae1", edgecolor=_OURS, linewidth=2.5, alpha=0.65,
    ))
    top.arrow(
        0.0, 0.0, 0.40 * length, 0.0, width=0.008,
        head_width=0.055, head_length=0.055, color=_OURS,
        length_includes_head=True,
    )
    top.text(0.08, 0.025, "机头 +x", color=_OURS, fontweight="bold")
    top.set_title(f"俯视：正方形底面 {length:.3f} × {width:.3f} m", fontweight="bold")
    top.set_xlabel("机体前向 x [m]")
    top.set_ylabel("机体横向 y [m]")
    top.set_aspect("equal", adjustable="box")
    top.set_xlim(-0.36, 0.36)
    top.set_ylim(-0.36, 0.36)
    top.grid(alpha=0.2)

    side.add_patch(Rectangle(
        (-0.5 * length, -0.5 * height), length, height,
        facecolor="#fdae6b", edgecolor=_OLD, linewidth=2.5, alpha=0.68,
    ))
    side.arrow(
        0.0, 0.0, 0.40 * length, 0.0, width=0.004,
        head_width=0.035, head_length=0.055, color=_OLD,
        length_includes_head=True,
    )
    side.text(0.08, 0.018, "机头 +x", color=_OLD, fontweight="bold")
    side.set_title(f"侧视：机身高度仅 {height:.3f} m", fontweight="bold")
    side.set_xlabel("机体前向 x [m]")
    side.set_ylabel("机体高度 z [m]")
    side.set_aspect("equal", adjustable="box")
    side.set_xlim(-0.36, 0.36)
    side.set_ylim(-0.18, 0.18)
    side.grid(alpha=0.2)
    figure.suptitle("四旋翼整机包络：扁平正方形底面长方体", fontsize=14, fontweight="bold")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.91))
    figure.savefig(path, dpi=dpi, facecolor="white")
    plt.close(figure)


def _render_cross_section_figure(
    case: StressCase,
    path: Path,
    *,
    time: float,
    title: str,
    dpi: int,
) -> None:
    """Keep the exact 2-D area evidence as a separate companion figure."""

    old = case.snapshot("Old-0.315", time, executed=False)
    ours = case.snapshot("Ours", time, executed=False)
    figure, axes = plt.subplots(1, 2, figsize=(10.8, 4.8))
    _draw_local_panel(axes[0], case, old)
    _draw_local_panel(axes[1], case, ours)
    figure.suptitle(
        title,
        fontsize=13,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    figure.savefig(path, dpi=dpi, facecolor="white")
    plt.close(figure)


def render_counterexample(
    case: StressCase,
    output_directory: str | Path,
    *,
    frames: int = 56,
    fps: int = 12,
) -> dict[str, Path]:
    root = Path(output_directory)
    figures = root / "figures"
    media = root / "media"
    figures.mkdir(parents=True, exist_ok=True)
    media.mkdir(parents=True, exist_ok=True)
    snapshot_path = figures / "counterexample_t_star.png"
    _render_frame(case, case.collision_time, snapshot_path, dpi=150)
    cross_section_path = figures / "cross_section_t_star.png"
    _render_cross_section_figure(
        case,
        cross_section_path,
        time=case.collision_time,
        title=f"Exact sections at dynamic collision t_col={case.collision_time:.3f}s",
        dpi=160,
    )
    diagnostic_3d_path = figures / "collision_3d_diagnostic.png"
    _render_3d_diagnostic(case, diagnostic_3d_path, dpi=170)
    body_model_path = figures / "body_model_dimensions.png"
    _render_body_model_figure(case, body_model_path, dpi=170)
    full_planned_timeline_path = figures / "full_planned_timeline.png"
    _render_full_planned_timeline(case, full_planned_timeline_path, dpi=145)
    nominal_path = figures / "nominal_center_crossing_safe.png"
    _render_cross_section_figure(
        case,
        nominal_path,
        time=case.crossing_time,
        title=f"Nominal center crossing is safe at t_i={case.crossing_time:.3f}s",
        dpi=160,
    )
    gif_path = media / "dynamic_counterexample.gif"
    with tempfile.TemporaryDirectory(prefix="exact_area_b_") as temporary:
        directory = Path(temporary)
        paths = []
        for index, instant in enumerate(np.linspace(0.0, case.total_time, frames)):
            path = directory / f"frame_{index:04d}.png"
            _render_frame(case, float(instant), path, dpi=92)
            paths.append(path)
        # The Pillow-backed GIF writer expects milliseconds (values below one
        # are truncated to a zero-delay animation).
        with imageio.get_writer(gif_path, mode="I", duration=1000.0 / fps, loop=0, subrectangles=False) as writer:
            for path in paths:
                writer.append_data(imageio.imread(path))
    return {
        "snapshot_overview": snapshot_path,
        "cross_section_2d": cross_section_path,
        "collision_3d_diagnostic": diagnostic_3d_path,
        "body_model_dimensions": body_model_path,
        "full_planned_timeline": full_planned_timeline_path,
        "nominal_crossing_safe": nominal_path,
        "animation": gif_path,
    }


__all__ = ["render_counterexample"]
