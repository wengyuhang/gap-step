from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, GraphObs


FREE = 0
WALL = 1
DYN = 2

PASSAGE_STAGE_ORDER = ("C1", "C2", "C3", "C4", "C5")


@dataclass(frozen=True)
class PassageSpec:
    name: str
    col: int
    rows: tuple[int, ...]
    offset: int
    open_width: int
    closed_phases: tuple[int, ...]

    def active_rows(self, phase: int, period: int) -> tuple[int, ...]:
        local = (phase + self.offset) % period
        if local in self.closed_phases:
            return tuple()
        # Hold each aperture row for two phases so a continuous agent can enter
        # and leave the bottleneck before it turns back into wall.
        center = self.rows[(local // 2) % len(self.rows)]
        half = max(0, self.open_width - 1)
        active = [r for r in range(center - half, center + half + 1) if r in self.rows]
        return tuple(active)


@dataclass(frozen=True)
class PassageLayout:
    base_grid: np.ndarray
    passages: tuple[PassageSpec, ...]
    start_cell: tuple[int, int]
    goal_cell: tuple[int, int]
    spine_rows: tuple[int, ...]


class TimeVaryingPassageMazeEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 8}

    def __init__(self, config: dict[str, Any] | None = None):
        super().__init__()
        self.config = {} if config is None else dict(config)
        self.stage_name = str(self.config.get("stage_name", self.config.get("difficulty", "C5")))
        self.split = str(self.config.get("split", "train"))
        self.period = int(self.config.get("period", 8))
        self.max_steps = int(self.config.get("max_steps", 240))
        self.robot_radius = float(self.config.get("robot_radius", 0.20))
        self.max_step = float(self.config.get("max_step", 1.08))
        self.render_size = int(self.config.get("render_size", 760))
        self.reward_goal = float(self.config.get("reward_goal", 12.0))
        self.reward_collision = float(self.config.get("reward_collision", -12.0))
        self.reward_timeout = float(self.config.get("reward_timeout", -2.0))
        self.reward_step = float(self.config.get("reward_step", -0.015))
        self.reward_progress = float(self.config.get("reward_progress", 0.08))
        self.reward_wait = float(self.config.get("reward_wait", -0.004))
        self.return_graph_obs = bool(self.config.get("return_graph_obs", True))

        self.rows = int(self.config.get("rows", 19))
        self.cols = int(self.config.get("cols", 35))
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Dict({})

        self.layout_seed = 0
        self.layout = self._generate_layout(np.random.default_rng(0), self.stage_name, self.split)
        self.base_grid = self.layout.base_grid
        self.passages = self.layout.passages
        self._dynamic_cells = self._build_dynamic_cells()
        self._phase_grids = self._build_phase_grids()
        self.start_cell = self.layout.start_cell
        self.goal_cell = self.layout.goal_cell
        self.start = self._cell_center(self.start_cell)
        self.goal = self._cell_center(self.goal_cell)
        self.pos = self.start.copy()
        self.t = 0
        self.step_count = 0
        self.trajectory: list[np.ndarray] = []
        self.collision_points: list[np.ndarray] = []
        self._last_progress = self._goal_distance(self.pos)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        options = {} if options is None else dict(options)
        self.stage_name = str(options.get("stage_name", self.stage_name))
        self.split = str(options.get("split", self.split))
        base_seed = int(self.config.get("seed", 0) if seed is None else seed)
        self.layout_seed = self._split_seed(base_seed, self.split)
        rng = np.random.default_rng(self.layout_seed)
        self.layout = self._generate_layout(rng, self.stage_name, self.split)
        self.base_grid = self.layout.base_grid
        self.passages = self.layout.passages
        self._dynamic_cells = self._build_dynamic_cells()
        self._phase_grids = self._build_phase_grids()
        self.start_cell = self.layout.start_cell
        self.goal_cell = self.layout.goal_cell
        self.start = self._cell_center(self.start_cell)
        self.goal = self._cell_center(self.goal_cell)
        phase_offset = int(options.get("phase_offset", self.layout_seed % self.period))
        self.t = phase_offset % self.period
        self.step_count = 0
        self.pos = self.start.copy()
        self.trajectory = [self.pos.copy()]
        self.collision_points = []
        self._last_progress = self._goal_distance(self.pos)
        return self._obs(), self._info(False, False, False, "")

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        norm = float(np.linalg.norm(action))
        if norm > 1.0:
            action = action / norm
            norm = 1.0
        old_pos = self.pos.copy()
        grid = self.maze_at_time(self.t)
        candidate = old_pos + action * self.max_step
        collision_type, collision_point = self._swept_collision(old_pos, candidate, grid)
        collision = bool(collision_type)
        if collision:
            self.collision_points.append(collision_point.copy())
        else:
            self.pos = candidate.astype(np.float32)

        success = bool(self._goal_distance(self.pos) <= 0.48 and not collision)
        self.step_count += 1
        self.t = (self.t + 1) % self.period
        if not collision and not success:
            post_type = self._point_collision_type(self.pos, self.maze_at_time(self.t))
            if post_type:
                collision_type = post_type
                collision = True
                self.collision_points.append(self.pos.copy())
        terminated = bool(success or collision)
        truncated = bool(self.step_count >= self.max_steps and not terminated)

        progress = self._goal_distance(self.pos)
        reward = self.reward_step + self.reward_progress * (self._last_progress - progress)
        if norm < 0.05:
            reward += self.reward_wait
        if success:
            reward += self.reward_goal
        if collision:
            reward += self.reward_collision
        if truncated:
            reward += self.reward_timeout
        self._last_progress = progress
        self.trajectory.append(self.pos.copy())
        return self._obs(), float(reward), terminated, truncated, self._info(success, collision, truncated, collision_type)

    def _obs(self):
        if self.return_graph_obs:
            return self.graph_obs()
        phase = 2.0 * np.pi * (self.t % self.period) / self.period
        return {
            "grid": self.maze_at_time(self.t),
            "agent": np.array([self.pos[0] / self.cols, self.pos[1] / self.rows], dtype=np.float32),
            "goal": np.array([self.goal[0] / self.cols, self.goal[1] / self.rows], dtype=np.float32),
            "phase": np.array([np.sin(phase), np.cos(phase)], dtype=np.float32),
        }

    def maze_at_time(self, t: int) -> np.ndarray:
        return self._phase_grids[int(t) % self.period]

    def graph_obs(self, prior_action: np.ndarray | None = None) -> GraphObs:
        grid = self.maze_at_time(self.t)
        phase = 2.0 * np.pi * (self.t % self.period) / self.period
        prior = np.zeros(2, dtype=np.float32) if prior_action is None else np.asarray(prior_action, dtype=np.float32)
        prior_norm = float(np.linalg.norm(prior))
        if prior_norm > 1.0:
            prior = prior / prior_norm
        global_features = np.zeros(GLOBAL_FEATURE_DIM, dtype=np.float32)
        global_features[0:2] = [self.pos[0] / self.cols, self.pos[1] / self.rows]
        global_features[4:6] = [self.goal[0] / self.cols, self.goal[1] / self.rows]
        global_features[6:8] = [np.sin(phase), np.cos(phase)]
        global_features[8] = self._goal_distance(self.pos) / np.hypot(self.rows, self.cols)
        global_features[9] = self.step_count / max(1, self.max_steps)
        global_features[10] = len(self.passages) / 5.0
        global_features[11] = PASSAGE_STAGE_ORDER.index(self.stage_name) / max(1, len(PASSAGE_STAGE_ORDER) - 1) if self.stage_name in PASSAGE_STAGE_ORDER else 1.0
        global_features[16:18] = prior
        global_features[19] = float(prior_norm < 0.05)
        global_features[24:26] = np.clip(prior, -0.95, 0.95)

        cells: list[tuple[int, int]] = []
        for r in range(self.rows):
            for c in range(self.cols):
                if grid[r, c] != WALL or (r, c) in self._dynamic_cells:
                    cells.append((r, c))
        cell_to_idx = {cell: idx for idx, cell in enumerate(cells)}
        node_features = np.zeros((len(cells), NODE_FEATURE_DIM), dtype=np.float32)
        node_type = np.zeros((len(cells),), dtype=np.int64)
        agent_cell = self.cell_from_pos(self.pos)
        for idx, (r, c) in enumerate(cells):
            value = int(grid[r, c])
            center = self._cell_center((r, c))
            node_features[idx, 0:2] = [c / max(1, self.cols - 1), r / max(1, self.rows - 1)]
            node_features[idx, 2:4] = [(center[0] - self.pos[0]) / self.cols, (center[1] - self.pos[1]) / self.rows]
            node_features[idx, 4:6] = [(self.goal[0] - center[0]) / self.cols, (self.goal[1] - center[1]) / self.rows]
            node_features[idx, 7] = float(value == FREE)
            node_features[idx, 8] = float(value == DYN)
            node_features[idx, 9] = float(value == WALL and self._is_dynamic_cell((r, c)))
            node_features[idx, 10] = self._next_open_steps((r, c)) / self.period
            node_features[idx, 11] = self._open_fraction((r, c))
            node_features[idx, 12] = float((r, c) == self.start_cell)
            node_features[idx, 13] = float((r, c) == self.goal_cell)
            node_features[idx, 14] = float((r, c) == agent_cell)
            node_features[idx, 15] = float(self._is_dynamic_cell((r, c)))
            node_type[idx] = 2 if value == DYN else (1 if self._is_dynamic_cell((r, c)) else 0)

        edges: list[tuple[int, int]] = []
        edge_features: list[np.ndarray] = []
        dirs = ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))
        for cell, src in cell_to_idx.items():
            r, c = cell
            for dr, dc in dirs:
                nb = (r + dr, c + dc)
                if nb not in cell_to_idx:
                    continue
                dst = cell_to_idx[nb]
                edges.append((src, dst))
                ef = np.zeros(EDGE_FEATURE_DIM, dtype=np.float32)
                ef[0:2] = [dc, dr]
                ef[2] = float(dr == 0 and dc == 0)
                ef[3] = float(self.passable(nb, self.t))
                ef[4] = float(self.passable(nb, self.t + 1))
                ef[5] = self._next_open_steps(nb) / self.period
                edge_features.append(ef)
        edge_index = np.asarray(edges, dtype=np.int64).T if edges else np.zeros((2, 0), dtype=np.int64)
        edge_arr = np.asarray(edge_features, dtype=np.float32) if edge_features else np.zeros((0, EDGE_FEATURE_DIM), dtype=np.float32)
        return GraphObs(global_features, node_features, node_type, edge_index, edge_arr)

    def render(self):
        grid = self.maze_at_time(self.t)
        cell = max(6, self.render_size // max(self.rows, self.cols))
        h, w = self.rows * cell, self.cols * cell
        img = np.full((h, w, 3), 245, dtype=np.uint8)
        colors = {
            FREE: np.array([250, 250, 250], dtype=np.uint8),
            WALL: np.array([32, 32, 32], dtype=np.uint8),
            DYN: np.array([74, 121, 214], dtype=np.uint8),
        }
        for r in range(self.rows):
            for c in range(self.cols):
                img[r * cell : (r + 1) * cell, c * cell : (c + 1) * cell] = colors[int(grid[r, c])]
        img[::cell, :, :] = np.minimum(img[::cell, :, :] + 35, 255)
        img[:, ::cell, :] = np.minimum(img[:, ::cell, :] + 35, 255)
        for p in self.trajectory:
            rr = int(np.clip(p[1] * cell, 0, h - 1))
            cc = int(np.clip(p[0] * cell, 0, w - 1))
            img[max(0, rr - 1) : min(h, rr + 2), max(0, cc - 1) : min(w, cc + 2)] = np.array([240, 95, 90], dtype=np.uint8)
        for p in self.collision_points:
            self._draw_disk(img, p, cell, np.array([255, 210, 0], dtype=np.uint8), 0.33)
        self._draw_disk(img, self.start, cell, np.array([35, 170, 65], dtype=np.uint8), 0.28)
        self._draw_disk(img, self.goal, cell, np.array([220, 35, 45], dtype=np.uint8), 0.28)
        self._draw_disk(img, self.pos, cell, np.array([235, 85, 45], dtype=np.uint8), self.robot_radius)
        return img

    def passable(self, cell: tuple[int, int], t: int) -> bool:
        r, c = cell
        if not (0 <= r < self.rows and 0 <= c < self.cols):
            return False
        return int(self.maze_at_time(t)[r, c]) in {FREE, DYN}

    def cell_from_pos(self, pos: np.ndarray) -> tuple[int, int]:
        c = int(np.floor(float(pos[0])))
        r = int(np.floor(float(pos[1])))
        return int(np.clip(r, 0, self.rows - 1)), int(np.clip(c, 0, self.cols - 1))

    def _split_seed(self, seed: int, split: str) -> int:
        offsets = {"train": 0, "id_test": 100_000, "ood_phase_test": 200_000, "ood_topology_test": 300_000}
        return int(seed) + offsets.get(split, 400_000)

    def _generate_layout(self, rng: np.random.Generator, stage_name: str, split: str) -> PassageLayout:
        dynamic_count = self._dynamic_count(stage_name, rng, split)
        base = np.ones((self.rows, self.cols), dtype=np.int8)
        boundary_cols = self._boundary_cols(dynamic_count, rng, split)
        left_cols = [1] + [c + 1 for c in boundary_cols]
        right_cols = [c - 1 for c in boundary_cols] + [self.cols - 2]
        start_row = int(rng.integers(self.rows - 5, self.rows - 2))
        goal_row = int(rng.integers(2, 5))
        spine = [start_row]
        for idx in range(1, dynamic_count):
            lo = max(3, min(spine[-1], goal_row) - 4)
            hi = min(self.rows - 4, max(spine[-1], goal_row) + 4)
            spine.append(int(rng.integers(lo, hi + 1)))
        spine.append(goal_row)
        if split == "ood_topology_test":
            spine = [int(np.clip(r + rng.integers(-2, 3), 2, self.rows - 3)) for r in spine]
            spine[-1] = goal_row

        band = 1 if stage_name in {"C4", "C5"} else 2
        for idx, (c0, c1) in enumerate(zip(left_cols, right_cols)):
            row = spine[idx]
            base[max(1, row - band) : min(self.rows - 1, row + band + 1), c0 : c1 + 1] = FREE
            if idx > 0:
                prev = spine[idx - 1]
                mid_c0 = max(c0, c0 + 1)
                mid_c1 = min(c1 + 1, c0 + 4)
                base[min(prev, row) - band : max(prev, row) + band + 1, mid_c0:mid_c1] = FREE
            if idx < dynamic_count:
                nxt = spine[idx + 1]
                mid_c0 = max(c0, c1 - 3)
                mid_c1 = c1 + 1
                base[min(nxt, row) - band : max(nxt, row) + band + 1, mid_c0:mid_c1] = FREE

        passages: list[PassageSpec] = []
        for idx, col in enumerate(boundary_cols):
            a, b = spine[idx], spine[idx + 1]
            rows = tuple(range(max(1, min(a, b) - 1), min(self.rows - 1, max(a, b) + 2)))
            for r in rows:
                base[r, col] = WALL
                base[r, col - 1] = FREE
                base[r, col + 1] = FREE
            closed = tuple() if stage_name in {"C1", "C2"} else ((idx * 2 + 3) % self.period,)
            if stage_name == "C5":
                closed = ((idx * 2 + 2) % self.period, (idx * 2 + 6) % self.period)
            open_width = 2 if stage_name in {"C1", "C2", "C3"} else 1
            offset = int(rng.integers(0, self.period)) if split != "ood_phase_test" else int(rng.integers(0, self.period) + 3)
            passages.append(PassageSpec(chr(ord("A") + idx), int(col), rows, offset, open_width, closed))

        self._add_static_islands(base, rng, stage_name)
        start_cell = (start_row, 2)
        goal_cell = (goal_row, self.cols - 3)
        base[start_cell] = FREE
        base[goal_cell] = FREE
        if dynamic_count == 0:
            # C1 is a static maze: carve the boundary columns instead of dynamic gates.
            for col in boundary_cols:
                for r in range(1, self.rows - 1):
                    if base[r, col - 1] == FREE or base[r, col + 1] == FREE:
                        base[r, col] = FREE
        return PassageLayout(base, tuple(passages if dynamic_count else ()), start_cell, goal_cell, tuple(spine))

    def _dynamic_count(self, stage_name: str, rng: np.random.Generator, split: str) -> int:
        if stage_name == "C1":
            return 0
        if stage_name == "C2":
            return 1
        if stage_name == "C3":
            return 2
        if stage_name == "C4":
            return 3
        if split == "ood_topology_test":
            return int(rng.integers(4, 6))
        return int(rng.integers(3, 6))

    def _boundary_cols(self, dynamic_count: int, rng: np.random.Generator, split: str) -> list[int]:
        if dynamic_count <= 0:
            return [self.cols // 2]
        cols = np.linspace(7, self.cols - 8, dynamic_count, dtype=int).tolist()
        jitter = 2 if split == "ood_topology_test" else 1
        out = []
        last = 4
        for c in cols:
            cc = int(np.clip(c + rng.integers(-jitter, jitter + 1), last + 4, self.cols - 5))
            out.append(cc)
            last = cc
        return out

    def _add_static_islands(self, base: np.ndarray, rng: np.random.Generator, stage_name: str) -> None:
        count = 0
        for _ in range(count):
            free = np.argwhere(base == FREE)
            if len(free) == 0:
                return
            r, c = free[int(rng.integers(0, len(free)))]
            if c < 4 or c > self.cols - 5:
                continue
            if rng.random() < 0.5:
                base[max(1, r - 1) : min(self.rows - 1, r + 1), c : min(self.cols - 1, c + 2)] = WALL
            else:
                base[r : min(self.rows - 1, r + 2), max(1, c - 1) : min(self.cols - 1, c + 1)] = WALL

    def _build_dynamic_cells(self) -> set[tuple[int, int]]:
        return {(r, passage.col) for passage in self.passages for r in passage.rows}

    def _build_phase_grids(self) -> tuple[np.ndarray, ...]:
        grids: list[np.ndarray] = []
        for phase in range(self.period):
            grid = self.base_grid.copy()
            for passage in self.passages:
                for r in passage.active_rows(phase, self.period):
                    grid[r, passage.col] = DYN
            grids.append(grid)
        return tuple(grids)

    def _is_dynamic_cell(self, cell: tuple[int, int]) -> bool:
        return cell in self._dynamic_cells

    def _next_open_steps(self, cell: tuple[int, int]) -> float:
        if not self._is_dynamic_cell(cell):
            return 0.0
        for dt in range(self.period + 1):
            if self.passable(cell, self.t + dt):
                return float(dt)
        return float(self.period)

    def _open_fraction(self, cell: tuple[int, int]) -> float:
        if not self._is_dynamic_cell(cell):
            return 1.0
        return float(np.mean([self.passable(cell, phase) for phase in range(self.period)]))

    def _goal_distance(self, pos: np.ndarray) -> float:
        return float(np.linalg.norm(self.goal - pos))

    def _cell_center(self, cell: tuple[int, int]) -> np.ndarray:
        r, c = cell
        return np.array([c + 0.5, r + 0.5], dtype=np.float32)

    def _swept_collision(self, old_pos: np.ndarray, new_pos: np.ndarray, grid: np.ndarray) -> tuple[str, np.ndarray]:
        distance = float(np.linalg.norm(new_pos - old_pos))
        samples = max(2, int(np.ceil(distance / 0.04)))
        for i in range(samples + 1):
            alpha = i / samples
            p = old_pos * (1.0 - alpha) + new_pos * alpha
            collision_type = self._point_collision_type(p, grid)
            if collision_type:
                return collision_type, p.astype(np.float32)
        return "", new_pos.astype(np.float32)

    def _swept_collision_type(self, old_pos: np.ndarray, new_pos: np.ndarray, grid: np.ndarray) -> str:
        return self._swept_collision(old_pos, new_pos, grid)[0]

    def _point_collision_type(self, pos: np.ndarray, grid: np.ndarray) -> str:
        if pos[0] < self.robot_radius or pos[1] < self.robot_radius or pos[0] > self.cols - self.robot_radius or pos[1] > self.rows - self.robot_radius:
            return "wall"
        c0 = int(np.floor(pos[0] - self.robot_radius))
        c1 = int(np.floor(pos[0] + self.robot_radius))
        r0 = int(np.floor(pos[1] - self.robot_radius))
        r1 = int(np.floor(pos[1] + self.robot_radius))
        for r in range(max(0, r0), min(self.rows - 1, r1) + 1):
            for c in range(max(0, c0), min(self.cols - 1, c1) + 1):
                if int(grid[r, c]) == WALL and self._circle_intersects_cell(pos, self.robot_radius, r, c):
                    return "dynamic_passage" if self._is_dynamic_cell((r, c)) else "wall"
        return ""

    def _info(self, success: bool, collision: bool, truncated: bool, collision_type: str) -> dict[str, Any]:
        return {
            "t": int(self.t),
            "step": int(self.step_count),
            "stage": self.stage_name,
            "split": self.split,
            "layout_seed": int(self.layout_seed),
            "passage_count": int(len(self.passages)),
            "pos": self.pos.copy(),
            "goal": self.goal.copy(),
            "success": bool(success),
            "collision": bool(collision),
            "timeout": bool(truncated),
            "collision_type": collision_type,
            "wall_collision": collision_type == "wall",
            "dynamic_passage_collision": collision_type == "dynamic_passage",
        }

    @staticmethod
    def _circle_intersects_cell(pos: np.ndarray, radius: float, r: int, c: int) -> bool:
        closest_x = np.clip(pos[0], c, c + 1)
        closest_y = np.clip(pos[1], r, r + 1)
        dx = pos[0] - closest_x
        dy = pos[1] - closest_y
        return bool(dx * dx + dy * dy <= radius * radius)

    @staticmethod
    def _draw_disk(img: np.ndarray, pos: np.ndarray, cell: int, color: np.ndarray, radius_cells: float) -> None:
        cx = int(pos[0] * cell)
        cy = int(pos[1] * cell)
        rr = max(2, int(radius_cells * cell))
        h, w = img.shape[:2]
        yy, xx = np.ogrid[:h, :w]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= rr * rr
        img[mask] = color
