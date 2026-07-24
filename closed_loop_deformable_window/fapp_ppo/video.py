"""Render a successful FAPP-PPO rollout as an H.264 MP4."""

from __future__ import annotations

import argparse
from pathlib import Path

import imageio_ffmpeg
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .environment import ClosedLoopWindowEnv
from .evaluate import load_policy, run_episode
from .geometry import signed_margin


Array = np.ndarray

plt.rcParams.update(
    {
        "font.sans-serif": ["Microsoft YaHei", "Noto Sans CJK SC", "DejaVu Sans"],
        "axes.unicode_minus": False,
    }
)


def _closed_curve(points: Array) -> Array:
    return np.vstack((points, points[0]))


def _world_boundary(window_state) -> Array:
    local = np.column_stack(
        (window_state.boundary, np.zeros(len(window_state.boundary)))
    )
    return window_state.center[None, :] + local @ window_state.rotation.T


def _safe_vertices(window_state) -> Array:
    if window_state.safe_polygon.is_empty:
        return np.zeros((3, 2), dtype=float)
    return np.asarray(window_state.safe_polygon.exterior.coords[:-1], dtype=float)


def render_video(
    checkpoint: str | Path,
    output: str | Path,
    *,
    stage: str = "full",
    seed: int = 40_000,
    device: str = "auto",
    fps: int = 25,
    dpi: int = 130,
) -> dict:
    """Run one deterministic episode and encode the complete dynamic scene."""

    model, config, resolved_device, _ = load_policy(checkpoint, device)
    environment = ClosedLoopWindowEnv(
        config.environment, config.quadrotor, stage=stage, seed=seed
    )
    result, trajectory = run_episode(
        model,
        environment,
        resolved_device,
        seed=seed,
        record_trajectory=True,
    )
    if not trajectory:
        raise RuntimeError("rollout produced no frames")

    initial = environment.scenario.initial_state
    initial_row = {
        "time": 0.0,
        "x": float(initial.position[0]),
        "y": float(initial.position[1]),
        "z": float(initial.position[2]),
        "vx": float(initial.velocity[0]),
        "vy": float(initial.velocity[1]),
        "vz": float(initial.velocity[2]),
        "progress_index": 0,
        "action": [0.0] * 4,
        "rotation": initial.rotation.tolist(),
        "body_rate": initial.body_rate.tolist(),
    }
    frames = [initial_row, *trajectory]
    positions = np.asarray(
        [[frame["x"], frame["y"], frame["z"]] for frame in frames], dtype=float
    )
    speeds = np.asarray(
        [
            np.linalg.norm([frame["vx"], frame["vy"], frame["vz"]])
            for frame in frames
        ],
        dtype=float,
    )
    total_windows = len(environment.scenario.order)
    route_extent = config.environment.route_radius + 1.8

    figure = plt.figure(figsize=(14.0, 7.2), facecolor="#f7f7f5")
    overview = figure.add_subplot(1, 2, 1, projection="3d")
    local_view = figure.add_subplot(1, 2, 2)
    figure.subplots_adjust(left=0.03, right=0.98, bottom=0.08, top=0.90, wspace=0.16)

    overview.set_xlim(-route_extent, route_extent)
    overview.set_ylim(-route_extent, route_extent)
    overview.set_zlim(0.25, 3.0)
    overview.set_xlabel("x 坐标 [m]")
    overview.set_ylabel("y 坐标 [m]")
    overview.set_zlabel("z 坐标 [m]")
    overview.set_box_aspect((1.0, 1.0, 0.42))
    overview.view_init(elev=27.0, azim=-58.0)
    overview.plot(
        positions[:, 0],
        positions[:, 1],
        positions[:, 2],
        color="#90a4ae",
        linewidth=1.0,
        linestyle="--",
        alpha=0.55,
        label="完整轨迹",
    )
    overview.scatter(
        *initial.position,
        color="#2e7d32",
        edgecolor="white",
        linewidth=0.8,
        s=70,
        label="起点 / 终点",
        zorder=8,
    )
    trail, = overview.plot([], [], [], color="#1565c0", linewidth=2.8, label="已飞轨迹")
    drone_point, = overview.plot(
        [], [], [], marker="o", color="#111111", markersize=6, linestyle=""
    )
    arm_x, = overview.plot([], [], [], color="#212121", linewidth=3.0)
    arm_y, = overview.plot([], [], [], color="#616161", linewidth=3.0)
    body_z, = overview.plot([], [], [], color="#d32f2f", linewidth=2.0)
    overview.legend(loc="upper left", framealpha=0.92, fontsize=8)

    gate_lines = []
    gate_faces = []
    for window in environment.scenario.windows:
        state = window.state(0.0)
        world = _world_boundary(state)
        closed = _closed_curve(world)
        line, = overview.plot(
            closed[:, 0], closed[:, 1], closed[:, 2], color="#ef6c00", linewidth=1.6
        )
        face = Poly3DCollection(
            [world],
            facecolor="#ffb74d",
            edgecolor="none",
            alpha=0.10,
        )
        overview.add_collection3d(face)
        gate_lines.append(line)
        gate_faces.append(face)

    local_view.set_xlim(-1.65, 1.65)
    local_view.set_ylim(-1.65, 1.65)
    local_view.set_aspect("equal", adjustable="box")
    local_view.set_xlabel("窗口局部 x [m]")
    local_view.set_ylabel("窗口局部 y [m]")
    local_view.grid(color="#cfd8dc", linewidth=0.6, alpha=0.8)
    physical_patch = PolyCollection(
        [np.zeros((3, 2))],
        facecolor="#ffb74d",
        edgecolor="#e65100",
        linewidth=2.0,
        alpha=0.30,
    )
    safe_patch = PolyCollection(
        [np.zeros((3, 2))],
        facecolor="#4db6ac",
        edgecolor="#00796b",
        linewidth=1.8,
        alpha=0.38,
    )
    local_view.add_collection(physical_patch)
    local_view.add_collection(safe_patch)
    local_drone, = local_view.plot(
        [], [], marker="o", color="#1565c0", markeredgecolor="white", markersize=9
    )
    local_center, = local_view.plot(
        [0.0], [0.0], marker="+", color="#424242", markersize=10, linestyle=""
    )
    local_note = local_view.text(
        0.03,
        0.97,
        "",
        transform=local_view.transAxes,
        ha="left",
        va="top",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "white", "alpha": 0.90},
    )
    return_note = local_view.text(
        0.5,
        0.50,
        "",
        transform=local_view.transAxes,
        ha="center",
        va="center",
        fontsize=17,
        color="#2e7d32",
        weight="bold",
    )
    hud = figure.text(
        0.5,
        0.965,
        "",
        ha="center",
        va="top",
        fontsize=14,
        weight="bold",
        color="#263238",
    )
    footer = figure.text(
        0.5,
        0.025,
        "橙色：物理非凸窗口    青色：安全内缩区    蓝色：无人机投影",
        ha="center",
        va="bottom",
        fontsize=9,
        color="#455a64",
    )

    crossing_times = [
        float(record["time"]) for record in environment.crossing_records
    ]
    crossing_artists: list = []

    def update(frame_index: int):
        frame = frames[frame_index]
        time = float(frame["time"])
        progress = int(frame["progress_index"])
        position = positions[frame_index]
        rotation = np.asarray(frame["rotation"], dtype=float)

        trail.set_data_3d(
            positions[: frame_index + 1, 0],
            positions[: frame_index + 1, 1],
            positions[: frame_index + 1, 2],
        )
        drone_point.set_data_3d([position[0]], [position[1]], [position[2]])
        arm_length = 0.24
        x_tip = rotation @ np.array([arm_length, 0.0, 0.0])
        y_tip = rotation @ np.array([0.0, arm_length, 0.0])
        z_tip = rotation @ np.array([0.0, 0.0, 0.34])
        arm_x.set_data_3d(
            [position[0] - x_tip[0], position[0] + x_tip[0]],
            [position[1] - x_tip[1], position[1] + x_tip[1]],
            [position[2] - x_tip[2], position[2] + x_tip[2]],
        )
        arm_y.set_data_3d(
            [position[0] - y_tip[0], position[0] + y_tip[0]],
            [position[1] - y_tip[1], position[1] + y_tip[1]],
            [position[2] - y_tip[2], position[2] + y_tip[2]],
        )
        body_z.set_data_3d(
            [position[0], position[0] + z_tip[0]],
            [position[1], position[1] + z_tip[1]],
            [position[2], position[2] + z_tip[2]],
        )

        for window_index, window in enumerate(environment.scenario.windows):
            state = window.state(time)
            world = _world_boundary(state)
            closed = _closed_curve(world)
            gate_lines[window_index].set_data_3d(
                closed[:, 0], closed[:, 1], closed[:, 2]
            )
            gate_faces[window_index].set_verts([world])
            if window_index < progress:
                color, face_color, width, alpha = "#2e7d32", "#81c784", 1.4, 0.08
            elif window_index == progress and progress < total_windows:
                color, face_color, width, alpha = "#c62828", "#ef5350", 2.8, 0.20
            else:
                color, face_color, width, alpha = "#ef6c00", "#ffb74d", 1.6, 0.10
            gate_lines[window_index].set_color(color)
            gate_lines[window_index].set_linewidth(width)
            gate_faces[window_index].set_facecolor(face_color)
            gate_faces[window_index].set_alpha(alpha)

        while len(crossing_artists) < sum(time >= crossing for crossing in crossing_times):
            crossing_index = len(crossing_artists)
            nearest = int(
                np.argmin(
                    np.abs(
                        np.asarray([candidate["time"] for candidate in frames])
                        - crossing_times[crossing_index]
                    )
                )
            )
            marker = overview.scatter(
                *positions[nearest],
                marker="*",
                color="#43a047",
                edgecolor="white",
                linewidth=0.6,
                s=80,
                zorder=10,
            )
            crossing_artists.append(marker)

        if progress < total_windows:
            window_index = environment.scenario.order[progress]
            window = environment.scenario.windows[window_index]
            window_state = window.state(time)
            physical_patch.set_verts([window_state.boundary])
            safe_patch.set_verts([_safe_vertices(window_state)])
            local = window_state.world_to_local(position)
            margin = signed_margin(window_state.safe_polygon, local[:2])
            margin_text = (
                f"{margin:+.3f} m" if np.isfinite(margin) else "无安全区"
            )
            interval = window.next_opportunity(time)
            if window.is_passable_state(window_state):
                gate_status = "开放"
                gate_color = "#00796b"
            else:
                gate_status = "闭合"
                gate_color = "#c62828"
            if interval is None:
                timing_note = (
                    "连续闭合过程"
                    if window.is_passable_state(window_state)
                    else "后续无开放机会"
                )
            elif time < interval[0]:
                if window.is_passable_state(window_state):
                    has_previous = any(
                        end < time for _, end in window.planned_opportunities
                    )
                    timing_note = (
                        "连续闭合过程"
                        if has_previous
                        else f"连续打开（{interval[0] - time:.2f} s 后完全开放）"
                    )
                else:
                    timing_note = f"距完全开放      {interval[0] - time:.2f} s"
            else:
                timing_note = f"距开始闭合      {max(0.0, interval[1] - time):.2f} s"
            local_drone.set_data([local[0]], [local[1]])
            local_drone.set_visible(True)
            local_center.set_visible(True)
            return_note.set_text("")
            local_view.set_title(
                f"下一个窗口：{window.name}   （{progress + 1}/{total_windows}）",
                fontsize=13,
                weight="bold",
            )
            local_note.set_text(
                f"状态            {gate_status}\n"
                f"{timing_note}\n"
                f"距窗口平面      {local[2]:+.3f} m\n"
                f"安全裕度        {margin_text}\n"
                f"安全面积        {window_state.safe_polygon.area:.3f} m²"
            )
            local_note.set_color(gate_color)
        else:
            physical_patch.set_verts([np.zeros((3, 2))])
            safe_patch.set_verts([np.zeros((3, 2))])
            local_drone.set_visible(False)
            local_center.set_visible(False)
            position_error = float(np.linalg.norm(position - initial.position))
            velocity_error = float(speeds[frame_index])
            rate_error = float(np.linalg.norm(frame["body_rate"]))
            local_view.set_title("闭环返回阶段", fontsize=13, weight="bold")
            local_note.set_text(
                f"位置误差        {position_error:.3f} m\n"
                f"速度误差        {velocity_error:.3f} m/s\n"
                f"角速度误差      {rate_error:.3f} rad/s"
            )
            local_note.set_color("#263238")
            return_note.set_text("返回初始完整状态")

        status = "成功" if frame_index == len(frames) - 1 and result["success"] else "飞行中"
        hud.set_text(
            f"FAPP-PPO · 独立时变窗口 · t={time:05.2f} s · "
            f"速度={speeds[frame_index]:.2f} m/s · 已通过={progress}/{total_windows} · {status}"
        )
        return (
            trail,
            drone_point,
            arm_x,
            arm_y,
            body_z,
            physical_patch,
            safe_patch,
            local_drone,
            local_note,
            return_note,
            hud,
            footer,
            *gate_lines,
            *gate_faces,
            *crossing_artists,
        )

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    matplotlib.rcParams["animation.ffmpeg_path"] = imageio_ffmpeg.get_ffmpeg_exe()
    animation = FuncAnimation(
        figure,
        update,
        frames=len(frames),
        interval=1000.0 / fps,
        blit=False,
        repeat=False,
    )
    writer = FFMpegWriter(
        fps=fps,
        codec="libx264",
        bitrate=4200,
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
        metadata={
            "title": "FAPP-PPO closed-loop deformable-window traversal",
            "artist": "Codex",
            "comment": f"stage={stage}, seed={seed}, success={result['success']}",
        },
    )
    animation.save(output_path, writer=writer, dpi=dpi)
    plt.close(figure)
    return {
        **result,
        "video": str(output_path),
        "frames": len(frames),
        "fps": fps,
        "duration_seconds": len(frames) / fps,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument(
        "--output",
        default="closed_loop_deformable_window/fapp_ppo/results/fapp_ppo_rollout.mp4",
    )
    parser.add_argument(
        "--stage",
        choices=("static", "moving", "deforming", "full"),
        default="full",
    )
    parser.add_argument("--seed", type=int, default=40_000)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--fps", type=int, default=25)
    parser.add_argument("--dpi", type=int, default=130)
    args = parser.parse_args()
    result = render_video(
        args.checkpoint,
        args.output,
        stage=args.stage,
        seed=args.seed,
        device=args.device,
        fps=args.fps,
        dpi=args.dpi,
    )
    print(
        f"已保存 {result['video']} | success={result['success']} | "
        f"crossings={result['crossings']} | duration={result['duration_seconds']:.2f}s"
    )


if __name__ == "__main__":
    main()
