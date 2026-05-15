from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from PIL import Image, ImageDraw

from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, GraphObs


Point = tuple[float, float]
Polygon = list[Point]


@dataclass(frozen=True)
class WallRect:
    xmin: float
    ymin: float
    xmax: float
    ymax: float
    name: str = "wall"

    def polygon(self) -> Polygon:
        return [(self.xmin, self.ymin), (self.xmax, self.ymin), (self.xmax, self.ymax), (self.xmin, self.ymax)]


@dataclass(frozen=True)
class ApertureWindow:
    name: str
    start: Point
    end: Point
    control: Point | tuple[Point, ...] | None
    thickness: float
    gap_sizes: tuple[float, ...]
    gap_centers: tuple[float, ...]
    cell: tuple[int, int]
    orientation: str
    kind_id: int

    def state(self, phase: int, period: int) -> dict[str, Any]:
        points = self._curve_points()
        distances = _polyline_distances(points)
        total = max(distances[-1], 1e-6)
        idx = phase % len(self.gap_sizes)
        gap_size = max(0.08, float(self.gap_sizes[idx]))
        gap_center = float(np.clip(self.gap_centers[idx], 0.12, 0.88))
        lo = max(0.0, gap_center * total - gap_size / 2.0)
        hi = min(total, gap_center * total + gap_size / 2.0)
        left = _sub_polyline(points, distances, 0.0, lo)
        gap = _sub_polyline(points, distances, lo, hi)
        right = _sub_polyline(points, distances, hi, total)
        obstacles = []
        if len(left) >= 2:
            obstacles.append(_polyline_thick_polygon(left, self.thickness))
        if len(right) >= 2:
            obstacles.append(_polyline_thick_polygon(right, self.thickness))
        opening = _polyline_thick_polygon(gap, self.thickness * 1.35) if len(gap) >= 2 else []
        mid = _point_at_distance(points, distances, 0.5 * (lo + hi))
        return {
            "name": self.name,
            "kind": "aperture_window",
            "opening": opening,
            "obstacles": obstacles,
            "gap_width": float(hi - lo),
            "safe": bool(hi - lo > 2.15 * 0.23),
            "label_pos": (mid[0] + 0.10, mid[1] - 0.42),
            "gap_midpoint": mid,
            "total_length": float(total),
            "gap_center": float(gap_center),
            "cell": self.cell,
            "orientation": self.orientation,
            "kind_id": self.kind_id,
        }

    def _curve_points(self) -> list[Point]:
        if self.control is None:
            return [self.start, self.end]
        if isinstance(self.control, tuple) and self.control and isinstance(self.control[0], tuple):
            return [self.start, *self.control, self.end]
        pts: list[Point] = []
        control = self.control
        assert isinstance(control, tuple)
        for u in np.linspace(0.0, 1.0, 18):
            a = (1.0 - u) * (1.0 - u)
            b = 2.0 * (1.0 - u) * u
            c = u * u
            pts.append(
                (
                    float(a * self.start[0] + b * control[0] + c * self.end[0]),
                    float(a * self.start[1] + b * control[1] + c * self.end[1]),
                )
            )
        return pts


_STAGE_CONFIG: dict[str, dict[str, Any]] = {
    "C1": {"windows": 0, "braid": 70, "widen": 0.45, "goal_frac": 0.05, "min_spacing": 5, "gap": (0.75, 0.95), "curves": (0,)},
    "C1_5": {"windows": 0, "braid": 56, "widen": 0.30, "goal_frac": 0.10, "min_spacing": 5, "gap": (0.68, 0.92), "curves": (0,)},
    "C2": {"windows": 1, "braid": 46, "widen": 0.20, "goal_frac": 0.18, "min_spacing": 4, "gap": (0.58, 0.90), "curves": (0,)},
    "C2A": {"windows": 3, "braid": 42, "widen": 0.16, "goal_frac": 0.30, "min_spacing": 4, "gap": (0.56, 0.88), "curves": (0,)},
    "C2B": {"windows": 6, "braid": 36, "widen": 0.10, "goal_frac": 0.45, "min_spacing": 4, "gap": (0.48, 0.84), "curves": (0, 1)},
    "C3": {"windows": 6, "braid": 32, "widen": 0.08, "goal_frac": 0.54, "min_spacing": 3, "gap": (0.42, 0.82), "curves": (0, 1, 2)},
    "C3S70": {"windows": 0, "braid": 30, "widen": 0.06, "goal_frac": 0.70, "min_spacing": 3, "gap": (0.52, 0.84), "curves": (0,)},
    "C3S85": {"windows": 0, "braid": 26, "widen": 0.03, "goal_frac": 0.85, "min_spacing": 3, "gap": (0.52, 0.84), "curves": (0,)},
    "C3S100": {"windows": 0, "braid": 22, "widen": 0.0, "goal_frac": 1.0, "min_spacing": 3, "gap": (0.52, 0.84), "curves": (0,)},
    "C3_5": {"windows": 4, "braid": 30, "widen": 0.06, "goal_frac": 0.62, "min_spacing": 3, "gap": (0.70, 0.95), "curves": (0, 1)},
    "C4": {"windows": 6, "braid": 28, "widen": 0.04, "goal_frac": 0.70, "min_spacing": 3, "gap": (0.65, 0.92), "curves": (0, 1, 2)},
    "C4A": {"windows": 6, "braid": 27, "widen": 0.03, "goal_frac": 0.78, "min_spacing": 3, "gap": (0.65, 0.92), "curves": (0, 1, 2, 3)},
    "C4B": {"windows": 8, "braid": 27, "widen": 0.03, "goal_frac": 0.78, "min_spacing": 3, "gap": (0.65, 0.92), "curves": (0, 1, 2, 3)},
    "C4C": {"windows": 8, "braid": 27, "widen": 0.03, "goal_frac": 0.78, "min_spacing": 3, "gap": (0.62, 0.90), "curves": (0, 1, 2, 3)},
    "C4D": {"windows": 8, "braid": 27, "widen": 0.03, "goal_frac": 0.78, "min_spacing": 3, "gap": (0.58, 0.88), "curves": (0, 1, 2, 3)},
    "C4E0": {"windows": 8, "braid": 28, "widen": 0.04, "goal_frac": 0.84, "min_spacing": 3, "gap": (0.62, 0.90), "curves": (0, 1, 2, 3)},
    "C4E1": {"windows": 10, "braid": 28, "widen": 0.04, "goal_frac": 0.84, "min_spacing": 3, "gap": (0.58, 0.88), "curves": (0, 1, 2, 3)},
    "C4E": {"windows": 10, "braid": 28, "widen": 0.04, "goal_frac": 0.90, "min_spacing": 3, "gap": (0.58, 0.88), "curves": (0, 1, 2, 3)},
    "C4F": {"windows": 10, "braid": 28, "widen": 0.04, "goal_frac": 1.0, "min_spacing": 3, "gap": (0.52, 0.84), "curves": (0, 1, 2, 3)},
    "C4_5": {"windows": 12, "braid": 28, "widen": 0.04, "goal_frac": 1.0, "min_spacing": 3, "gap": (0.46, 0.84), "force_narrow": 0.20, "curves": (0, 1, 2, 3)},
    "C5": {"windows": 6, "braid": 28, "widen": 0.04, "goal_frac": 1.0, "min_spacing": 2, "gap": (0.72, 0.96), "force_narrow": 0.0, "curves": (0, 1, 2, 3)},
}

_SPLIT_OFFSETS = {"train": 0, "id_test": 100_000, "ood_window_test": 200_000, "ood_maze_test": 300_000}


class TimeVaryingWindowMazeEnv(gym.Env):
    """Generated 2D maze family with continuous moving aperture windows."""

    metadata = {"render_modes": ["rgb_array"], "render_fps": 8}

    def __init__(self, config: dict[str, Any] | None = None):
        super().__init__()
        self.config = {} if config is None else dict(config)
        self.stage_name = str(self.config.get("stage_name", "C5"))
        self.split = str(self.config.get("split", "train"))
        self.layout_seed = int(self.config.get("layout_seed", self.config.get("seed", 17)))
        self.return_graph_obs = bool(self.config.get("return_graph_obs", False))
        self.graph_mode = str(self.config.get("graph_mode", "compact"))
        self.graph_max_cells = int(self.config.get("graph_max_cells", 128))
        self.fixed_layout = bool(self.config.get("fixed_layout", False))
        self.width = float(self.config.get("width", 35.0))
        self.height = float(self.config.get("height", 19.0))
        self.period = int(self.config.get("period", 8))
        self.max_steps = int(self.config.get("max_steps", 320))
        self.robot_radius = float(self.config.get("robot_radius", 0.23))
        self.max_step = float(self.config.get("max_step", 0.42))
        self.max_acc = 1.0
        self.render_width = int(self.config.get("render_width", 980))
        self.render_height = int(self.config.get("render_height", 540))
        self.show_reference_path = bool(self.config.get("show_reference_path", True))
        self.start = np.array(self.config.get("start", [1.5, 17.5]), dtype=np.float32)
        self.full_goal = np.array(self.config.get("goal", [33.5, 1.5]), dtype=np.float32)
        self.goal = self.full_goal.copy()
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.max_window_features = 24
        self.observation_space = spaces.Dict(
            {
                "agent": spaces.Box(0.0, 1.0, shape=(2,), dtype=np.float32),
                "goal": spaces.Box(0.0, 1.0, shape=(2,), dtype=np.float32),
                "phase": spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32),
                "window_gaps": spaces.Box(0.0, 4.0, shape=(self.max_window_features,), dtype=np.float32),
            }
        )
        self._episode_index = 0
        self._current_layout_seed = -1
        self._cell_nodes: dict[tuple[int, int], int] = {}
        self._path_remaining_cache: dict[tuple[int, int], float] = {}
        self._reference_remaining: list[float] = []
        self._free_cells: list[tuple[int, int]] = []
        self._regenerate_layout(self.layout_seed)
        self.pos = self.start.copy()
        self.t = 0
        self.step_count = 0
        self.last_action = np.zeros(2, dtype=np.float32)
        self.trajectory: list[np.ndarray] = []
        self.collision_points: list[np.ndarray] = []

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        options = {} if options is None else options
        if "stage_name" in options:
            self.stage_name = str(options["stage_name"])
        if "split" in options:
            self.split = str(options["split"])
        super().reset(seed=seed)
        if seed is not None:
            self._episode_index = 0
            layout_seed = self._layout_seed_from_episode(int(seed), self.split)
        elif self.fixed_layout:
            layout_seed = self._current_layout_seed
        else:
            self._episode_index += 1
            layout_seed = self._layout_seed_from_episode(self.layout_seed + self._episode_index * 9973, self.split)
        if layout_seed != self._current_layout_seed:
            self._regenerate_layout(layout_seed)
        self.t = int(options.get("phase", options.get("phase_offset", 0))) % self.period
        self.step_count = 0
        self.pos = self.start.copy()
        self.last_action = np.zeros(2, dtype=np.float32)
        self.trajectory = [self.pos.copy()]
        self.collision_points = []
        return self._obs(), self._info(False, False, False, "")

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        norm = float(np.linalg.norm(action))
        if norm > 1.0:
            action = action / norm
        old_pos = self.pos.copy()
        old_route = self._route_distance(old_pos)
        new_pos = old_pos + action * self.max_step
        collision_type, point = self._swept_collision(old_pos, new_pos, self.t)
        collision = bool(collision_type)
        if collision:
            self.collision_points.append(point.copy())
        else:
            self.pos = new_pos.astype(np.float32)
        success = bool(np.linalg.norm(self.goal - self.pos) <= 0.45 and not collision)
        self.step_count += 1
        self.t = (self.t + 1) % self.period
        if not collision and not success:
            collision_type = self._point_collision_type(self.pos, self.t)
            if collision_type:
                collision = True
                self.collision_points.append(self.pos.copy())
        terminated = bool(success or collision)
        truncated = bool(self.step_count >= self.max_steps and not terminated)

        new_route = self._route_distance(self.pos)
        progress_delta = 0.0 if collision else float(np.clip(old_route - new_route, -0.8, 0.8))
        nearest = self._nearest_obstacle_summary(self.pos, self.t)
        risk = max(0.0, self.robot_radius + 0.12 - nearest["dynamic_clearance"])
        wall_risk = max(0.0, self.robot_radius + 0.22 - nearest["wall_clearance"])
        center_offset = self._nearest_reference_offset(self.pos)
        route_direction = self._next_reference_direction(old_pos)
        near_closed = max(0.0, 1.2 - nearest["next_gap_distance"]) * (1.0 - nearest["next_gap_safe"])
        gap_vec = np.array([nearest["next_gap_future_dx"], nearest["next_gap_future_dy"]], dtype=np.float32)
        gap_norm = float(np.linalg.norm(gap_vec))
        gap_dir = gap_vec / gap_norm if gap_norm > 1e-6 else np.zeros(2, dtype=np.float32)
        near_open = max(0.0, 1.2 - nearest["next_gap_distance"]) * nearest["next_gap_safe"]
        near_future_blocked = near_open * (1.0 - nearest["next_gap_future_safe"])
        window_focus = max(near_closed, near_open)
        # Near a moving aperture, the geometric route centerline can point into a panel.
        # Fade it out there and let the live gap midpoint dominate the local behavior.
        center_scale = max(0.15, 1.0 - 0.85 * min(window_focus, 1.0))
        center_penalty = -0.28 * center_scale * min(float(np.linalg.norm(center_offset)), 1.0)
        gap_alignment_reward = (
            1.45 * near_open * nearest["next_gap_future_safe"] * float(np.dot(action, gap_dir))
            if not collision
            else 0.0
        )
        alignment_scale = 0.15 if near_closed > 0.0 else max(0.20, 1.0 - 0.80 * min(near_open, 1.0))
        alignment_reward = 0.45 * alignment_scale * float(np.dot(action, route_direction)) if not collision else 0.0
        closed_window_penalty = -0.35 * near_closed
        closing_window_penalty = -0.50 * near_future_blocked
        wall_penalty = -0.35 * wall_risk
        reward = -0.005 + 1.20 * progress_delta + alignment_reward + gap_alignment_reward - 0.002 * float(np.linalg.norm(action)) - 0.12 * risk + closed_window_penalty + closing_window_penalty + wall_penalty + center_penalty
        if success:
            reward += 30.0
        if collision:
            reward = self._collision_reward()
        if truncated:
            reward -= 2.0
        self.last_action = action.astype(np.float32)
        self.trajectory.append(self.pos.copy())
        info = self._info(success, collision, truncated, collision_type)
        info.update(
            {
                "progress_delta": progress_delta,
                "progress_reward": 1.20 * progress_delta,
                "risk_penalty": -0.12 * risk,
                "wall_penalty": wall_penalty,
                "center_penalty": center_penalty,
                "closed_window_penalty": closed_window_penalty,
                "closing_window_penalty": closing_window_penalty,
                "alignment_reward": alignment_reward,
                "gap_alignment_reward": gap_alignment_reward,
                "action_norm": float(np.linalg.norm(action)),
                "route_distance": float(new_route),
                **nearest,
            }
        )
        return self._obs(), float(reward), terminated, truncated, info

    def window_state(self, t: int | None = None) -> list[dict[str, Any]]:
        phase = self.t if t is None else int(t) % self.period
        states = [window.state(phase, self.period) for window in self.windows]
        for state in states:
            state["safe"] = bool(state["gap_width"] > 2.15 * self.robot_radius)
        return states

    def render(self):
        img = Image.new("RGB", (self.render_width, self.render_height), (248, 248, 248))
        draw = ImageDraw.Draw(img)
        self._draw_grid(draw)
        for wall in self.static_walls:
            self._draw_polygon(draw, wall.polygon(), fill=(32, 32, 32), outline=(16, 16, 16))

        states = self.window_state(self.t)
        overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
        overlay_draw = ImageDraw.Draw(overlay)
        for state in states:
            self._draw_polygon(overlay_draw, state["opening"], fill=(75, 125, 220, 120), outline=(45, 90, 190, 220))
        img = Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")
        draw = ImageDraw.Draw(img)
        for state in states:
            for obstacle in state["obstacles"]:
                self._draw_polygon(draw, obstacle, fill=(24, 24, 24), outline=(8, 8, 8))
            self._draw_window_label(draw, state)

        if self.show_reference_path:
            self._draw_dashed_path(draw, self.reference_path, fill=(245, 85, 95), width=2)
        for p in self.trajectory:
            self._draw_disk(draw, p, 0.08, fill=(235, 85, 45))
        for p in self.collision_points:
            self._draw_disk(draw, p, 0.34, fill=(255, 215, 0), outline=(30, 30, 30))
        self._draw_disk(draw, self.start, 0.32, fill=(30, 170, 65), outline=(255, 255, 255))
        self._draw_disk(draw, self.goal, 0.32, fill=(220, 35, 45), outline=(255, 255, 255))
        self._draw_disk(draw, self.pos, self.robot_radius, fill=(235, 110, 40), outline=(20, 20, 20))
        draw.text((14, 8), f"{self.stage_name} seed={self._current_layout_seed}  t={self.t}", fill=(20, 20, 20))
        return np.asarray(img, dtype=np.uint8)

    def _layout_seed_from_episode(self, seed: int, split: str) -> int:
        return int(seed + _SPLIT_OFFSETS.get(split, 0))

    def _stage_config(self) -> dict[str, Any]:
        return dict(_STAGE_CONFIG.get(self.stage_name, _STAGE_CONFIG["C5"]))

    def _regenerate_layout(self, seed: int) -> None:
        stage_cfg = self._stage_config()
        for attempt in range(32):
            rng = np.random.default_rng(seed + attempt * 7919)
            self.goal = self.full_goal.copy()
            self.maze_grid = self._build_maze_grid(rng, int(stage_cfg["braid"]), float(stage_cfg.get("widen", 0.0)))
            self.static_walls = self._walls_from_grid()
            self.reference_path = self._reference_path_from_maze()
            if len(self.reference_path) < 2:
                continue
            self._apply_stage_goal(float(stage_cfg.get("goal_frac", 1.0)))
            self._build_reference_remaining()
            self._free_cells = [(r, c) for r in range(self.maze_grid.shape[0]) for c in range(self.maze_grid.shape[1]) if self.maze_grid[r, c] == 0]
            self.windows = self._build_windows(rng, stage_cfg)
            self._rebuild_cell_index()
            self._current_layout_seed = seed
            return
        raise RuntimeError(f"Could not generate a legal window maze for seed={seed}, stage={self.stage_name}")

    def _apply_stage_goal(self, goal_fraction: float) -> None:
        if goal_fraction >= 0.999 or len(self.reference_path) < 2:
            self.goal = self.full_goal.copy()
            return
        target_idx = int(np.clip(round((len(self.reference_path) - 1) * goal_fraction), 1, len(self.reference_path) - 1))
        target = self.reference_path[target_idx]
        self.goal = np.array([target[0], target[1]], dtype=np.float32)
        self.reference_path = self.reference_path[: target_idx + 1]

    def _build_maze_grid(self, rng: np.random.Generator, braid_count: int, widen_prob: float) -> np.ndarray:
        rows, cols = int(self.height), int(self.width)
        grid = np.ones((rows, cols), dtype=np.int8)
        start = (rows - 2, 1)
        goal = (1, cols - 2)
        stack = [start]
        grid[start] = 0
        visited = {start}
        dirs = [(0, 2), (0, -2), (2, 0), (-2, 0)]
        while stack:
            r, c = stack[-1]
            neighbors = []
            shuffled = dirs[:]
            rng.shuffle(shuffled)
            for dr, dc in shuffled:
                nr, nc = r + dr, c + dc
                if 1 <= nr < rows - 1 and 1 <= nc < cols - 1 and (nr, nc) not in visited:
                    neighbors.append((nr, nc, dr, dc))
            if not neighbors:
                stack.pop()
                continue
            nr, nc, dr, dc = neighbors[0]
            grid[r + dr // 2, c + dc // 2] = 0
            grid[nr, nc] = 0
            visited.add((nr, nc))
            stack.append((nr, nc))

        for _ in range(max(0, braid_count)):
            r = int(rng.integers(2, rows - 2))
            c = int(rng.integers(2, cols - 2))
            if grid[r, c] == 1:
                free_neighbors = sum(grid[r + dr, c + dc] == 0 for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)))
                if free_neighbors >= 2:
                    grid[r, c] = 0
        if widen_prob > 0.0:
            widened = grid.copy()
            for r in range(1, rows - 1):
                for c in range(1, cols - 1):
                    if grid[r, c] == 1 and rng.random() < widen_prob:
                        free_neighbors = sum(grid[r + dr, c + dc] == 0 for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)))
                        if free_neighbors >= 1:
                            widened[r, c] = 0
            grid = widened
        grid[rows - 2, 1] = 0
        grid[1, cols - 2] = 0
        return grid

    def _walls_from_grid(self) -> list[WallRect]:
        walls: list[WallRect] = []
        for r in range(self.maze_grid.shape[0]):
            for c in range(self.maze_grid.shape[1]):
                if self.maze_grid[r, c] == 1:
                    walls.append(WallRect(float(c), float(r), float(c + 1), float(r + 1), f"wall_{r}_{c}"))
        return walls

    def _build_windows(self, rng: np.random.Generator, stage_cfg: dict[str, Any]) -> tuple[ApertureWindow, ...]:
        target_count = int(stage_cfg["windows"])
        if target_count <= 0:
            return tuple()
        min_spacing = int(stage_cfg["min_spacing"])
        candidates: list[tuple[int, int, str]] = []
        rows, cols = self.maze_grid.shape
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if self.maze_grid[r, c] != 0:
                    continue
                if abs(c - 1) + abs(r - (rows - 2)) < 7 or abs(c - (cols - 2)) + abs(r - 1) < 7:
                    continue
                if self.maze_grid[r - 1, c] == 1 and self.maze_grid[r + 1, c] == 1:
                    candidates.append((r, c, "vertical"))
                if self.maze_grid[r, c - 1] == 1 and self.maze_grid[r, c + 1] == 1:
                    candidates.append((r, c, "horizontal"))
        if self.split == "ood_window_test":
            rng.shuffle(candidates)
            candidates = sorted(candidates, key=lambda x: (x[1] + 3 * x[0] + (x[2] == "vertical")) % 11)
        else:
            rng.shuffle(candidates)

        selected: list[tuple[int, int, str]] = []
        spacing = min_spacing
        while len(selected) < target_count and spacing >= 1:
            for r, c, orient in candidates:
                if (r, c, orient) in selected:
                    continue
                if all(abs(r - rr) + abs(c - cc) >= spacing for rr, cc, _ in selected):
                    selected.append((r, c, orient))
                if len(selected) >= target_count:
                    break
            spacing -= 1

        windows: list[ApertureWindow] = []
        curves = tuple(int(v) for v in stage_cfg["curves"])
        gap_lo, gap_hi = stage_cfg["gap"]
        for idx, (r, c, orient) in enumerate(selected[:target_count]):
            kind_id = int(rng.choice(curves))
            if self.split == "ood_window_test" and len(curves) > 1:
                kind_id = int(curves[(idx + 2) % len(curves)])
            start, end, control = self._window_geometry(rng, r, c, orient, kind_id)
            gap_sizes = rng.uniform(float(gap_lo), float(gap_hi), size=self.period)
            gap_centers = rng.uniform(0.34, 0.66, size=self.period)
            gap_sizes[idx % self.period] = float(gap_lo)
            gap_sizes[(idx + 3) % self.period] = float(gap_hi)
            force_narrow = float(stage_cfg.get("force_narrow", 0.0))
            should_force_narrow = force_narrow > 0.0 and (force_narrow >= 1.0 or rng.random() < force_narrow)
            if should_force_narrow or self.split == "ood_window_test":
                narrow_idx = (idx + int(rng.integers(0, self.period))) % self.period
                open_idx = (narrow_idx + int(rng.integers(2, 5))) % self.period
                gap_sizes[narrow_idx] = max(0.08, float(gap_lo) * 0.75)
                gap_sizes[open_idx] = min(0.86, float(gap_hi) + 0.14)
                jitter = rng.normal(0.0, 0.06 if self.stage_name == "C5" else 0.035, size=self.period)
                gap_centers = np.clip(gap_centers + jitter, 0.24, 0.76)
            windows.append(
                ApertureWindow(
                    _window_name(idx),
                    start,
                    end,
                    control,
                    0.20,
                    tuple(float(x) for x in gap_sizes),
                    tuple(float(x) for x in gap_centers),
                    (r, c),
                    orient,
                    kind_id,
                )
            )
        return tuple(windows)

    def _window_geometry(
        self, rng: np.random.Generator, r: int, c: int, orient: str, kind_id: int
    ) -> tuple[Point, Point, Point | tuple[Point, ...] | None]:
        if orient == "vertical":
            start, end = (c + 0.5, float(r)), (c + 0.5, float(r + 1))
            if kind_id == 0:
                control = None
            elif kind_id == 1:
                control = (c + float(rng.uniform(0.22, 0.78)), r + 0.5)
            elif kind_id == 2:
                control = ((c + 0.72, r + 0.32), (c + 0.28, r + 0.68))
            else:
                amp = float(rng.choice([-1.0, 1.0]) * rng.uniform(0.18, 0.28))
                control = (c + 0.5 + amp, r + 0.5)
        else:
            start, end = (float(c), r + 0.5), (float(c + 1), r + 0.5)
            if kind_id == 0:
                control = None
            elif kind_id == 1:
                control = (c + 0.5, r + float(rng.uniform(0.22, 0.78)))
            elif kind_id == 2:
                control = ((c + 0.32, r + 0.72), (c + 0.68, r + 0.28))
            else:
                amp = float(rng.choice([-1.0, 1.0]) * rng.uniform(0.18, 0.28))
                control = (c + 0.5, r + 0.5 + amp)
        return start, end, control

    def _reference_path_from_maze(self) -> list[Point]:
        start = (int(self.start[1]), int(self.start[0]))
        goal = (int(self.goal[1]), int(self.goal[0]))
        queue = deque([start])
        parent: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        while queue:
            r, c = queue.popleft()
            if (r, c) == goal:
                break
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (r + dr, c + dc)
                if nb in parent:
                    continue
                if 0 <= nb[0] < self.maze_grid.shape[0] and 0 <= nb[1] < self.maze_grid.shape[1] and self.maze_grid[nb] == 0:
                    parent[nb] = (r, c)
                    queue.append(nb)
        if goal not in parent:
            return []
        rev = []
        cur: tuple[int, int] | None = goal
        while cur is not None:
            rev.append((cur[1] + 0.5, cur[0] + 0.5))
            cur = parent[cur]
        rev.reverse()
        return rev

    def _build_reference_remaining(self) -> None:
        self._reference_remaining = [0.0 for _ in self.reference_path]
        self._reference_cell_index = {(int(p[1]), int(p[0])): idx for idx, p in enumerate(self.reference_path)}
        total = 0.0
        for idx in range(len(self.reference_path) - 2, -1, -1):
            a, b = self.reference_path[idx], self.reference_path[idx + 1]
            total += float(np.hypot(b[0] - a[0], b[1] - a[1]))
            self._reference_remaining[idx] = total
        self._path_remaining_cache = {}

    def _route_distance(self, pos: np.ndarray) -> float:
        if len(self.reference_path) < 2:
            return float(np.linalg.norm(self.goal - pos))
        p = (float(pos[0]), float(pos[1]))
        best = float("inf")
        best_remaining = float("inf")
        for idx, (a, b) in enumerate(zip(self.reference_path, self.reference_path[1:])):
            ax, ay = a
            bx, by = b
            ab = np.array([bx - ax, by - ay], dtype=np.float32)
            ap = np.array([p[0] - ax, p[1] - ay], dtype=np.float32)
            denom = float(np.dot(ab, ab))
            alpha = float(np.clip(np.dot(ap, ab) / max(denom, 1e-8), 0.0, 1.0))
            closest = np.array([ax, ay], dtype=np.float32) + alpha * ab
            dist = float(np.linalg.norm(np.array(p, dtype=np.float32) - closest))
            remaining = self._reference_remaining[idx] - alpha * float(np.sqrt(max(denom, 0.0)))
            score = dist + 0.03 * remaining
            if score < best:
                best = score
                best_remaining = remaining + dist
        return float(best_remaining)

    def _next_reference_direction(self, pos: np.ndarray) -> np.ndarray:
        if len(self.reference_path) < 2:
            vec = self.goal - pos
        else:
            p = np.asarray(pos, dtype=np.float32)
            best_idx = 0
            best_dist = float("inf")
            for idx, (a, b) in enumerate(zip(self.reference_path, self.reference_path[1:])):
                dist = _point_segment_distance((float(p[0]), float(p[1])), a, b)
                if dist < best_dist:
                    best_idx = idx
                    best_dist = dist
            target_idx = min(best_idx + 1, len(self.reference_path) - 1)
            target = np.asarray(self.reference_path[target_idx], dtype=np.float32)
            if float(np.linalg.norm(target - p)) < 0.18 and target_idx + 1 < len(self.reference_path):
                target = np.asarray(self.reference_path[target_idx + 1], dtype=np.float32)
            vec = target - p
        norm = float(np.linalg.norm(vec))
        return (vec / norm).astype(np.float32) if norm > 1e-6 else np.zeros(2, dtype=np.float32)

    def _nearest_reference_offset(self, pos: np.ndarray) -> np.ndarray:
        if len(self.reference_path) < 2:
            return np.zeros(2, dtype=np.float32)
        p = np.asarray(pos, dtype=np.float32)
        best_dist = float("inf")
        best_offset = np.zeros(2, dtype=np.float32)
        for a, b in zip(self.reference_path, self.reference_path[1:]):
            a_arr = np.asarray(a, dtype=np.float32)
            b_arr = np.asarray(b, dtype=np.float32)
            seg = b_arr - a_arr
            alpha = float(np.clip(np.dot(p - a_arr, seg) / max(float(np.dot(seg, seg)), 1e-8), 0.0, 1.0))
            closest = a_arr + alpha * seg
            offset = closest - p
            dist = float(np.linalg.norm(offset))
            if dist < best_dist:
                best_dist = dist
                best_offset = offset
        return best_offset.astype(np.float32)

    def _rebuild_cell_index(self) -> None:
        self._cell_nodes = {}
        for idx, cell in enumerate(self._free_cells):
            self._cell_nodes[cell] = idx

    def _graph_cells(
        self,
        agent_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        path_cells: set[tuple[int, int]],
    ) -> list[tuple[int, int]]:
        if self.graph_mode == "full":
            return list(self._free_cells)
        cells: set[tuple[int, int]] = set(path_cells)
        cells.add(agent_cell)
        cells.add(goal_cell)
        for window in self.windows:
            cells.add(window.cell)
            r, c = window.cell
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (r + dr, c + dc)
                if nb in self._cell_nodes:
                    cells.add(nb)
        ar, ac = agent_cell
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                nb = (ar + dr, ac + dc)
                if nb in self._cell_nodes:
                    cells.add(nb)
        valid = [cell for cell in cells if cell in self._cell_nodes]
        if len(valid) <= self.graph_max_cells:
            return sorted(valid, key=lambda cell: self._cell_route_remaining(cell))
        protected = {agent_cell, goal_cell, *(window.cell for window in self.windows)}
        protected = {cell for cell in protected if cell in self._cell_nodes}
        ranked = sorted(valid, key=lambda cell: (cell not in protected, self._cell_route_remaining(cell)))
        selected = ranked[: self.graph_max_cells]
        for cell in protected:
            if cell not in selected:
                selected[-1] = cell
        return sorted(set(selected), key=lambda cell: self._cell_route_remaining(cell))

    def _dynamic_obstacles(self, t: int) -> list[Polygon]:
        obstacles: list[Polygon] = []
        for state in self.window_state(t):
            obstacles.extend(state["obstacles"])
        return obstacles

    def _point_collision_type(self, pos: np.ndarray, t: int) -> str:
        if pos[0] < self.robot_radius or pos[1] < self.robot_radius or pos[0] > self.width - self.robot_radius or pos[1] > self.height - self.robot_radius:
            return "wall"
        if self._circle_intersects_static_wall(pos):
            return "wall"
        for poly in self._dynamic_obstacles(t):
            if _circle_intersects_polygon(pos, self.robot_radius, poly):
                return "window"
        return ""

    def _circle_intersects_static_wall(self, pos: np.ndarray) -> bool:
        x, y = float(pos[0]), float(pos[1])
        rmin = max(0, int(np.floor(y - self.robot_radius - 0.02)))
        rmax = min(self.maze_grid.shape[0] - 1, int(np.floor(y + self.robot_radius + 0.02)))
        cmin = max(0, int(np.floor(x - self.robot_radius - 0.02)))
        cmax = min(self.maze_grid.shape[1] - 1, int(np.floor(x + self.robot_radius + 0.02)))
        for rr in range(rmin, rmax + 1):
            for cc in range(cmin, cmax + 1):
                if self.maze_grid[rr, cc] == 1 and _circle_intersects_rect_cell(pos, self.robot_radius, rr, cc):
                    return True
        return False

    def _swept_collision(self, old_pos: np.ndarray, new_pos: np.ndarray, t: int) -> tuple[str, np.ndarray]:
        distance = float(np.linalg.norm(new_pos - old_pos))
        samples = max(2, int(np.ceil(distance / 0.025)))
        for idx in range(samples + 1):
            alpha = idx / samples
            p = old_pos * (1.0 - alpha) + new_pos * alpha
            collision_type = self._point_collision_type(p, t)
            if collision_type:
                return collision_type, p.astype(np.float32)
        return "", new_pos.astype(np.float32)

    def _obs(self):
        if self.return_graph_obs:
            return self._graph_obs()
        phase = 2.0 * np.pi * (self.t % self.period) / self.period
        gaps = np.zeros((self.max_window_features,), dtype=np.float32)
        raw_gaps = [state["gap_width"] for state in self.window_state(self.t)]
        gaps[: min(len(raw_gaps), self.max_window_features)] = raw_gaps[: self.max_window_features]
        return {
            "agent": np.array([self.pos[0] / self.width, self.pos[1] / self.height], dtype=np.float32),
            "goal": np.array([self.goal[0] / self.width, self.goal[1] / self.height], dtype=np.float32),
            "phase": np.array([np.sin(phase), np.cos(phase)], dtype=np.float32),
            "window_gaps": gaps,
        }

    def _graph_obs(self) -> GraphObs:
        phase_angle = 2.0 * np.pi * (self.t % self.period) / self.period
        route_dist = self._route_distance(self.pos)
        direction = self._next_reference_direction(self.pos)
        center_offset = self._nearest_reference_offset(self.pos)
        nearest = self._nearest_obstacle_summary(self.pos, self.t)
        states = self.window_state(self.t)
        safe_frac = float(np.mean([s["safe"] for s in states])) if states else 1.0
        stage_scalar = self._stage_scalar()

        global_features = np.zeros((GLOBAL_FEATURE_DIM,), dtype=np.float32)
        global_features[0:2] = [self.pos[0] / self.width, self.pos[1] / self.height]
        global_features[2:4] = self.last_action
        global_features[4:6] = [self.goal[0] / self.width, self.goal[1] / self.height]
        global_features[6:8] = [np.sin(phase_angle), np.cos(phase_angle)]
        global_features[8] = route_dist / max(self.width + self.height, 1.0)
        global_features[9] = self.step_count / max(1, self.max_steps)
        global_features[10] = len(self.windows) / max(1, self.max_window_features)
        global_features[11] = stage_scalar
        global_features[12] = min(nearest["dynamic_clearance"], 4.0) / 4.0
        global_features[13] = min(nearest["next_gap_width"], 1.2) / 1.2
        global_features[14] = nearest["next_gap_safe"]
        global_features[15] = min(nearest["wall_clearance"], 4.0) / 4.0
        global_features[16:18] = direction
        global_features[18:20] = np.clip(center_offset, -1.0, 1.0)
        global_features[20:22] = np.clip(
            np.array([nearest["next_gap_dx"], nearest["next_gap_dy"]], dtype=np.float32) / 4.0,
            -1.0,
            1.0,
        )
        global_features[22] = min(nearest["next_gap_width"], 1.2) / 1.2
        global_features[23] = nearest["next_gap_safe"]
        global_features[24:26] = 0.0

        node_features: list[np.ndarray] = []
        node_type: list[int] = []
        agent_cell = (int(np.floor(self.pos[1])), int(np.floor(self.pos[0])))
        goal_cell = (int(np.floor(self.goal[1])), int(np.floor(self.goal[0])))
        path_cells = {(int(p[1]), int(p[0])) for p in self.reference_path}
        graph_cells = self._graph_cells(agent_cell, goal_cell, path_cells)
        local_cell_nodes: dict[tuple[int, int], int] = {}
        for r, c in graph_cells:
            local_cell_nodes[(r, c)] = len(node_features)
            center = np.array([c + 0.5, r + 0.5], dtype=np.float32)
            feat = np.zeros((NODE_FEATURE_DIM,), dtype=np.float32)
            feat[0:2] = [center[0] / self.width, center[1] / self.height]
            feat[2:4] = (center - self.pos) / np.array([self.width, self.height], dtype=np.float32)
            feat[4:6] = (self.goal - center) / np.array([self.width, self.height], dtype=np.float32)
            feat[6] = min(float(np.linalg.norm(center - self.pos)), 10.0) / 10.0
            feat[7] = min(float(np.linalg.norm(center - self.goal)), 20.0) / 20.0
            feat[8] = self._cell_route_remaining((r, c)) / max(self.width + self.height, 1.0)
            feat[9] = 1.0 if (r, c) in path_cells else 0.0
            feat[10:12] = [np.sin(phase_angle), np.cos(phase_angle)]
            feat[13] = 1.0 if (r, c) == goal_cell else 0.0
            feat[14] = 1.0 if (r, c) == agent_cell else 0.0
            feat[15] = 1.0
            feat[16:20] = self._wall_adjacency(r, c)
            feat[20] = stage_scalar
            node_features.append(feat)
            node_type.append(0)

        window_node_start = len(node_features)
        for idx, (window, state) in enumerate(zip(self.windows, states)):
            mid = np.array(state["gap_midpoint"], dtype=np.float32)
            feat = np.zeros((NODE_FEATURE_DIM,), dtype=np.float32)
            feat[0:2] = [mid[0] / self.width, mid[1] / self.height]
            feat[2:4] = (mid - self.pos) / np.array([self.width, self.height], dtype=np.float32)
            feat[4:6] = (self.goal - mid) / np.array([self.width, self.height], dtype=np.float32)
            feat[6] = min(float(np.linalg.norm(mid - self.pos)), 10.0) / 10.0
            feat[7] = min(float(np.linalg.norm(mid - self.goal)), 20.0) / 20.0
            feat[8] = min(float(state["gap_width"]), 1.2) / 1.2
            feat[9] = 1.0 if state["safe"] else 0.0
            feat[10:12] = [np.sin(phase_angle), np.cos(phase_angle)]
            feat[12] = self._time_to_safe(idx) / max(1, self.period)
            feat[15] = 0.0
            feat[16] = 1.0 if window.orientation == "vertical" else 0.0
            feat[17] = 1.0 if window.orientation == "horizontal" else 0.0
            feat[18] = window.kind_id / 3.0
            feat[19] = state["gap_center"]
            feat[20] = stage_scalar
            feat[21] = min(float(state["total_length"]), 1.8) / 1.8
            feat[22] = min(float(np.max(window.gap_sizes)), 1.2) / 1.2
            feat[23] = min(float(np.min(window.gap_sizes)), 1.2) / 1.2
            feat[24] = 1.0 if idx == int(nearest["next_window_index"]) else 0.0
            future_states = [window.state(self.t + offset, self.period) for offset in range(1, 5)]
            feat[25:29] = [min(float(s["gap_width"]), 1.2) / 1.2 for s in future_states]
            feat[29:32] = [float(s["gap_center"]) for s in future_states[:3]]
            node_features.append(feat)
            node_type.append(1)

        edge_index: list[tuple[int, int]] = []
        edge_features: list[np.ndarray] = []
        for r, c in graph_cells:
            src = local_cell_nodes[(r, c)]
            self._add_edge(edge_index, edge_features, src, src, 0.0, 0.0, 0)
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (r + dr, c + dc)
                if nb in local_cell_nodes:
                    dst = local_cell_nodes[nb]
                    self._add_edge(edge_index, edge_features, src, dst, float(dc), float(dr), 1)
        for idx, window in enumerate(self.windows):
            wnode = window_node_start + idx
            cell_node = local_cell_nodes.get(window.cell)
            if cell_node is None:
                continue
            self._add_edge(edge_index, edge_features, wnode, cell_node, 0.0, 0.0, 2)
            self._add_edge(edge_index, edge_features, cell_node, wnode, 0.0, 0.0, 3)
            self._add_edge(edge_index, edge_features, wnode, wnode, 0.0, 0.0, 0)

        if edge_index:
            edge_arr = np.asarray(edge_index, dtype=np.int64).T
            edge_feat_arr = np.asarray(edge_features, dtype=np.float32)
        else:
            edge_arr = np.zeros((2, 0), dtype=np.int64)
            edge_feat_arr = np.zeros((0, EDGE_FEATURE_DIM), dtype=np.float32)
        return GraphObs(
            global_features=global_features,
            node_features=np.asarray(node_features, dtype=np.float32),
            node_type=np.asarray(node_type, dtype=np.int64),
            edge_index=edge_arr,
            edge_features=edge_feat_arr,
        )

    def _add_edge(
        self,
        edge_index: list[tuple[int, int]],
        edge_features: list[np.ndarray],
        src: int,
        dst: int,
        dx: float,
        dy: float,
        edge_kind: int,
    ) -> None:
        feat = np.zeros((EDGE_FEATURE_DIM,), dtype=np.float32)
        feat[0] = dx
        feat[1] = dy
        feat[2] = float(np.hypot(dx, dy))
        feat[3 + min(edge_kind, 4)] = 1.0
        edge_index.append((src, dst))
        edge_features.append(feat)

    def _cell_route_remaining(self, cell: tuple[int, int]) -> float:
        if cell in self._path_remaining_cache:
            return self._path_remaining_cache[cell]
        p = np.array([cell[1] + 0.5, cell[0] + 0.5], dtype=np.float32)
        dist = self._route_distance(p)
        self._path_remaining_cache[cell] = dist
        return dist

    def _wall_adjacency(self, r: int, c: int) -> np.ndarray:
        rows, cols = self.maze_grid.shape
        out = np.zeros((4,), dtype=np.float32)
        for idx, (dr, dc) in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1))):
            rr, cc = r + dr, c + dc
            out[idx] = 1.0 if rr < 0 or cc < 0 or rr >= rows or cc >= cols or self.maze_grid[rr, cc] == 1 else 0.0
        return out

    def _time_to_safe(self, window_idx: int) -> float:
        for offset in range(self.period):
            if self.windows[window_idx].state(self.t + offset, self.period)["safe"]:
                return float(offset)
        return float(self.period)

    def _stage_scalar(self) -> float:
        order = ["C1", "C1_5", "C2", "C2A", "C2B", "C3", "C3S70", "C3S85", "C3S100", "C3_5", "C4", "C4A", "C4B", "C4C", "C4D", "C4E0", "C4E1", "C4E", "C4F", "C4_5", "C5"]
        return order.index(self.stage_name) / max(1, len(order) - 1) if self.stage_name in order else 1.0

    def _collision_reward(self) -> float:
        if self.stage_name in {"C1", "C1_5", "C2"}:
            return -8.0
        if self.stage_name in {"C2A", "C2B", "C3", "C3S70", "C3S85", "C3S100"}:
            return -18.0
        return -40.0

    def _nearest_obstacle_summary(self, pos: np.ndarray, t: int) -> dict[str, float]:
        wall_clearance = self._wall_clearance_grid(pos)
        states = self.window_state(t)
        dynamic_clearance = min((_polygon_distance(pos, poly) for s in states for poly in s["obstacles"]), default=4.0)
        nearest_gap_distance = 8.0
        nearest_gap_width = 1.2
        nearest_gap_safe = 1.0
        nearest_gap_dx = 0.0
        nearest_gap_dy = 0.0
        next_gap_distance = 8.0
        next_gap_width = 1.2
        next_gap_safe = 1.0
        next_gap_dx = 0.0
        next_gap_dy = 0.0
        next_gap_future_width = 1.2
        next_gap_future_safe = 1.0
        next_gap_future_dx = 0.0
        next_gap_future_dy = 0.0
        next_window_index = -1
        next_ahead_delta = float("inf")
        closed_window_distance = 8.0
        future_open_fraction = 1.0
        future_min_gap = 1.2
        for idx, (window, state) in enumerate(zip(self.windows, states)):
            mid = np.asarray(state["gap_midpoint"], dtype=np.float32)
            dist = float(np.linalg.norm(pos - mid))
            if dist < nearest_gap_distance:
                nearest_gap_distance = dist
                nearest_gap_width = float(state["gap_width"])
                nearest_gap_safe = 1.0 if state["safe"] else 0.0
                nearest_gap_dx = float(mid[0] - pos[0])
                nearest_gap_dy = float(mid[1] - pos[1])
                future = [window.state(t + k, self.period)["gap_width"] for k in range(self.period)]
                future_open_fraction = float(np.mean([g > 2.15 * self.robot_radius for g in future]))
                future_min_gap = float(np.min(future))
            if not state["safe"]:
                closed_window_distance = min(closed_window_distance, dist)
            path_idx = self._reference_cell_index.get(window.cell)
            if path_idx is not None:
                window_remaining = self._reference_remaining[path_idx]
                ahead_delta = self._route_distance(pos) - window_remaining
                if ahead_delta >= -0.5 and ahead_delta < next_ahead_delta:
                    next_ahead_delta = ahead_delta
                    next_gap_distance = dist
                    next_gap_width = float(state["gap_width"])
                    next_gap_safe = 1.0 if state["safe"] else 0.0
                    next_gap_dx = float(mid[0] - pos[0])
                    next_gap_dy = float(mid[1] - pos[1])
                    future_state = window.state(t + 1, self.period)
                    next_gap_future_width = float(future_state["gap_width"])
                    next_gap_future_safe = 1.0 if next_gap_future_width > 2.15 * self.robot_radius else 0.0
                    future_mid = np.asarray(future_state["gap_midpoint"], dtype=np.float32)
                    next_gap_future_dx = float(future_mid[0] - pos[0])
                    next_gap_future_dy = float(future_mid[1] - pos[1])
                    next_window_index = idx
        if next_ahead_delta == float("inf"):
            next_gap_distance = nearest_gap_distance
            next_gap_width = nearest_gap_width
            next_gap_safe = nearest_gap_safe
            next_gap_dx = nearest_gap_dx
            next_gap_dy = nearest_gap_dy
            if states:
                nearest_idx = min(
                    range(len(states)),
                    key=lambda idx: float(np.linalg.norm(pos - np.asarray(states[idx]["gap_midpoint"], dtype=np.float32))),
                )
                future_state = self.windows[nearest_idx].state(t + 1, self.period)
                next_gap_future_width = float(future_state["gap_width"])
                next_gap_future_safe = 1.0 if next_gap_future_width > 2.15 * self.robot_radius else 0.0
                future_mid = np.asarray(future_state["gap_midpoint"], dtype=np.float32)
                next_gap_future_dx = float(future_mid[0] - pos[0])
                next_gap_future_dy = float(future_mid[1] - pos[1])
                next_window_index = nearest_idx
        return {
            "wall_clearance": float(wall_clearance),
            "dynamic_clearance": float(dynamic_clearance),
            "nearest_gap_distance": float(nearest_gap_distance),
            "nearest_gap_width": float(nearest_gap_width),
            "nearest_gap_safe": float(nearest_gap_safe),
            "nearest_gap_dx": float(nearest_gap_dx),
            "nearest_gap_dy": float(nearest_gap_dy),
            "next_gap_distance": float(next_gap_distance),
            "next_gap_width": float(next_gap_width),
            "next_gap_safe": float(next_gap_safe),
            "next_gap_dx": float(next_gap_dx),
            "next_gap_dy": float(next_gap_dy),
            "next_gap_future_width": float(next_gap_future_width),
            "next_gap_future_safe": float(next_gap_future_safe),
            "next_gap_future_dx": float(next_gap_future_dx),
            "next_gap_future_dy": float(next_gap_future_dy),
            "next_window_index": float(next_window_index),
            "closed_window_distance": float(closed_window_distance),
            "future_open_fraction": float(future_open_fraction),
            "future_min_gap": float(future_min_gap),
        }

    def _wall_clearance_grid(self, pos: np.ndarray, search_radius: float = 3.0) -> float:
        x, y = float(pos[0]), float(pos[1])
        rmin = max(0, int(np.floor(y - search_radius)))
        rmax = min(self.maze_grid.shape[0] - 1, int(np.floor(y + search_radius)))
        cmin = max(0, int(np.floor(x - search_radius)))
        cmax = min(self.maze_grid.shape[1] - 1, int(np.floor(x + search_radius)))
        best = search_radius
        for rr in range(rmin, rmax + 1):
            for cc in range(cmin, cmax + 1):
                if self.maze_grid[rr, cc] == 1:
                    best = min(best, _point_rect_cell_distance(pos, rr, cc))
        return float(best)

    def _info(self, success: bool, collision: bool, truncated: bool, collision_type: str) -> dict[str, Any]:
        return {
            "t": int(self.t),
            "step": int(self.step_count),
            "stage": self.stage_name,
            "split": self.split,
            "layout_seed": int(self._current_layout_seed),
            "pos": self.pos.copy(),
            "goal": self.goal.copy(),
            "success": bool(success),
            "collision": bool(collision),
            "timeout": bool(truncated),
            "collision_type": collision_type,
            "wall_collision": collision_type == "wall",
            "window_collision": collision_type == "window",
            "closed_gate_collision": collision_type == "window",
            "boundary_collision": collision_type == "wall",
        }

    def _world_to_px(self, p: Point | np.ndarray) -> tuple[int, int]:
        x, y = float(p[0]), float(p[1])
        return int(round(x / self.width * self.render_width)), int(round(y / self.height * self.render_height))

    def _draw_polygon(self, draw: ImageDraw.ImageDraw, poly: Polygon, fill, outline=None) -> None:
        if len(poly) >= 3:
            draw.polygon([self._world_to_px(p) for p in poly], fill=fill, outline=outline)

    def _draw_disk(self, draw: ImageDraw.ImageDraw, pos: Point | np.ndarray, radius: float, fill, outline=None) -> None:
        cx, cy = self._world_to_px(pos)
        rx = radius / self.width * self.render_width
        ry = radius / self.height * self.render_height
        draw.ellipse((cx - rx, cy - ry, cx + rx, cy + ry), fill=fill, outline=outline)

    def _draw_grid(self, draw: ImageDraw.ImageDraw) -> None:
        for x in np.arange(0.0, self.width + 1e-6, 1.0):
            px, _ = self._world_to_px((x, 0.0))
            draw.line((px, 0, px, self.render_height), fill=(226, 226, 226), width=1)
        for y in np.arange(0.0, self.height + 1e-6, 1.0):
            _, py = self._world_to_px((0.0, y))
            draw.line((0, py, self.render_width, py), fill=(226, 226, 226), width=1)

    def _draw_window_label(self, draw: ImageDraw.ImageDraw, state: dict[str, Any]) -> None:
        x, y = self._world_to_px(state["label_pos"])
        text = str(state["name"])
        color = (45, 100, 190) if state["safe"] else (105, 105, 105)
        draw.rounded_rectangle((x - 10, y - 14, x + 18, y + 12), radius=3, fill=color, outline=(245, 245, 245), width=2)
        draw.text((x - 3, y - 12), text, fill=(255, 255, 255))

    def _draw_dashed_path(self, draw: ImageDraw.ImageDraw, points: list[Point], fill, width: int) -> None:
        px_points = [self._world_to_px(p) for p in points]
        dash, gap = 12.0, 8.0
        for a, b in zip(px_points, px_points[1:]):
            ax, ay = a
            bx, by = b
            seg = np.array([bx - ax, by - ay], dtype=np.float32)
            length = float(np.linalg.norm(seg))
            if length <= 1e-6:
                continue
            direction = seg / length
            cursor = 0.0
            while cursor < length:
                end = min(length, cursor + dash)
                p0 = np.array([ax, ay], dtype=np.float32) + direction * cursor
                p1 = np.array([ax, ay], dtype=np.float32) + direction * end
                draw.line((*p0, *p1), fill=fill, width=width)
                cursor = end + gap


def _polyline_thick_polygon(points: list[Point], thickness: float) -> Polygon:
    left: list[Point] = []
    right: list[Point] = []
    for idx, p in enumerate(points):
        if idx == 0:
            q = points[1]
            tangent = (q[0] - p[0], q[1] - p[1])
        elif idx == len(points) - 1:
            q = points[idx - 1]
            tangent = (p[0] - q[0], p[1] - q[1])
        else:
            p0, p1 = points[idx - 1], points[idx + 1]
            tangent = (p1[0] - p0[0], p1[1] - p0[1])
        length = max(1e-6, float(np.hypot(*tangent)))
        nx, ny = -tangent[1] / length * thickness / 2.0, tangent[0] / length * thickness / 2.0
        left.append((p[0] + nx, p[1] + ny))
        right.append((p[0] - nx, p[1] - ny))
    return left + list(reversed(right))


def _polyline_distances(points: list[Point]) -> list[float]:
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + float(np.hypot(b[0] - a[0], b[1] - a[1])))
    return distances


def _point_at_distance(points: list[Point], distances: list[float], d: float) -> Point:
    d = float(np.clip(d, 0.0, distances[-1]))
    for idx in range(len(points) - 1):
        if distances[idx] <= d <= distances[idx + 1]:
            seg_len = max(1e-6, distances[idx + 1] - distances[idx])
            alpha = (d - distances[idx]) / seg_len
            a, b = points[idx], points[idx + 1]
            return (float(a[0] * (1.0 - alpha) + b[0] * alpha), float(a[1] * (1.0 - alpha) + b[1] * alpha))
    return points[-1]


def _sub_polyline(points: list[Point], distances: list[float], start: float, end: float) -> list[Point]:
    if end - start <= 1e-5:
        return []
    out = [_point_at_distance(points, distances, start)]
    for point, d in zip(points[1:-1], distances[1:-1]):
        if start < d < end:
            out.append(point)
    out.append(_point_at_distance(points, distances, end))
    cleaned: list[Point] = []
    for p in out:
        if not cleaned or float(np.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1])) > 1e-4:
            cleaned.append(p)
    return cleaned


def _window_name(idx: int) -> str:
    letters = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    if idx < len(letters):
        return letters[idx]
    return f"W{idx + 1}"


def _circle_intersects_polygon(center: np.ndarray, radius: float, poly: Polygon) -> bool:
    p = (float(center[0]), float(center[1]))
    if len(poly) < 3:
        return False
    if _point_in_polygon(p, poly):
        return True
    for a, b in zip(poly, poly[1:] + poly[:1]):
        if _point_segment_distance(p, a, b) <= radius:
            return True
    return False


def _polygon_distance(center: np.ndarray, poly: Polygon) -> float:
    p = (float(center[0]), float(center[1]))
    if len(poly) < 3:
        return 4.0
    if _point_in_polygon(p, poly):
        return 0.0
    return min((_point_segment_distance(p, a, b) for a, b in zip(poly, poly[1:] + poly[:1])), default=4.0)


def _circle_intersects_rect_cell(center: np.ndarray, radius: float, row: int, col: int) -> bool:
    closest_x = float(np.clip(center[0], col, col + 1.0))
    closest_y = float(np.clip(center[1], row, row + 1.0))
    dx = float(center[0]) - closest_x
    dy = float(center[1]) - closest_y
    return bool(dx * dx + dy * dy <= radius * radius)


def _point_rect_cell_distance(center: np.ndarray, row: int, col: int) -> float:
    closest_x = float(np.clip(center[0], col, col + 1.0))
    closest_y = float(np.clip(center[1], row, row + 1.0))
    dx = float(center[0]) - closest_x
    dy = float(center[1]) - closest_y
    return float(np.hypot(dx, dy))


def _point_segment_distance(p: Point, a: Point, b: Point) -> float:
    px, py = p
    ax, ay = a
    bx, by = b
    ab = np.array([bx - ax, by - ay], dtype=np.float32)
    ap = np.array([px - ax, py - ay], dtype=np.float32)
    denom = float(np.dot(ab, ab))
    if denom <= 1e-8:
        return float(np.linalg.norm(ap))
    t = float(np.clip(np.dot(ap, ab) / denom, 0.0, 1.0))
    closest = np.array([ax, ay], dtype=np.float32) + t * ab
    return float(np.linalg.norm(np.array([px, py], dtype=np.float32) - closest))


def _point_in_polygon(p: Point, poly: Polygon) -> bool:
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_intersect = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1
            if x < x_intersect:
                inside = not inside
    return inside
