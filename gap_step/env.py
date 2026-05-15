from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from gap_step.curriculum import Maze, sample_maze
from gap_step.graph import EDGE_FEATURE_DIM, GLOBAL_FEATURE_DIM, NODE_FEATURE_DIM, GraphObs
from gap_step.utils import circle_intersects_rect, ray_rect_distance, wrap_angle


@dataclass(frozen=True)
class RoadmapEdge:
    to: int
    travel_time: float
    gate_id: int | None = None


class ContinuousMazeEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 20}

    def __init__(self, config: dict[str, Any] | None = None):
        super().__init__()
        config = {} if config is None else dict(config)
        self.config = config
        self.stage_name = str(config.get("stage_name", "C5"))
        self.split = str(config.get("split", "train"))
        self.dt = float(config.get("dt", 0.1))
        self.robot_radius = float(config.get("robot_radius", 0.25))
        self.safe_margin = float(config.get("safe_margin", 0.10))
        self.max_speed = float(config.get("v_max", config.get("max_speed", 2.0)))
        self.max_acc = float(config.get("a_max", config.get("max_acc", 3.0)))
        self.max_steps = int(config.get("max_steps", 500))
        self.goal_radius = float(config.get("goal_radius", 0.45))
        self.num_rays = int(config.get("debug_num_rays", 32))
        self.ray_max_dist_ratio = float(config.get("ray_max_dist_ratio", 0.35))
        self.reward_goal = float(config.get("reward_goal", 20.0))
        self.reward_collision = float(config.get("reward_collision", -20.0))
        self.reward_time = float(config.get("reward_time", -0.01))
        self.reward_action = float(config.get("reward_action", -0.001))
        self.reward_progress = float(config.get("reward_progress", 0.0))
        self.reward_guidance = float(config.get("reward_guidance", 0.0))
        self.reward_timeout = float(config.get("reward_timeout", 0.0))
        self.progress_mode = str(config.get("progress_mode", "none"))
        self.suppress_positive_progress_on_collision = bool(config.get("suppress_positive_progress_on_collision", True))
        self.gate_lookahead_time = float(config.get("gate_lookahead_time", 20.0))
        self.gate_time_resolution = float(config.get("gate_time_resolution", self.dt))
        self.gate_unreachable_cost = float(config.get("gate_unreachable_cost", 1e6))
        self.progress_delta_clip = float(config.get("progress_delta_clip", 0.25))
        self.render_size = int(config.get("render_size", 512))

        self.action_space = spaces.Box(-self.max_acc, self.max_acc, shape=(2,), dtype=np.float32)
        self.observation_space = None
        self.graph_feature_dims = {
            "global": GLOBAL_FEATURE_DIM,
            "node": NODE_FEATURE_DIM,
            "edge": EDGE_FEATURE_DIM,
        }
        self.np_random: np.random.Generator
        self.maze: Maze
        self.S = 15.0
        self.ray_max_dist = self.ray_max_dist_ratio * self.S
        self.pos = np.zeros(2, dtype=np.float32)
        self.vel = np.zeros(2, dtype=np.float32)
        self.goal = np.zeros(2, dtype=np.float32)
        self.t = 0.0
        self.step_count = 0
        self.trajectory: list[np.ndarray] = []
        self._roadmap_nodes: list[np.ndarray] = []
        self._roadmap_edges: list[list[RoadmapEdge]] = []
        self._roadmap_gates: dict[int, Any] = {}
        self._gate_approach_nodes: dict[int, tuple[int, int]] = {}
        self._visibility_blockers: list[dict[str, float]] = []
        self._prev_progress_potential = 0.0
        self._last_progress_potential = 0.0
        self._last_progress_delta = 0.0
        self._last_progress_reward = 0.0
        self._last_guidance_reward = 0.0
        self._last_dynamic_path_wait_time = 0.0
        self._last_dynamic_path_uses_gate = False

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        options = {} if options is None else options
        stage_name = str(options.get("stage_name", self.stage_name))
        split = str(options.get("split", self.split))
        maze_seed = seed if seed is not None else int(self.np_random.integers(0, 2**31 - 1))
        self.maze = sample_maze(stage_name=stage_name, split=split, seed=maze_seed)
        self.S = float(self.maze.S)
        self.ray_max_dist = self.ray_max_dist_ratio * self.S
        self.pos = self.maze.start.copy().astype(np.float32)
        self.goal = self.maze.goal.copy().astype(np.float32)
        self.vel = np.zeros(2, dtype=np.float32)
        self.t = 0.0
        self.step_count = 0
        self.trajectory = [self.pos.copy()]
        self._build_dynamic_geometry_roadmap()
        potential, wait_time, uses_gate = self._progress_potential(self.pos, self.t)
        self._prev_progress_potential = potential
        self._last_progress_potential = potential
        self._last_progress_delta = 0.0
        self._last_progress_reward = 0.0
        self._last_guidance_reward = 0.0
        self._last_dynamic_path_wait_time = wait_time
        self._last_dynamic_path_uses_gate = uses_gate
        return self.get_privileged_obs(), self._info(success=False, collision=False, truncated=False, collision_type="")

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -self.max_acc, self.max_acc)
        old_pos = self.pos.copy()

        self.vel = self.vel + action * self.dt
        speed = float(np.linalg.norm(self.vel))
        if speed > self.max_speed:
            self.vel = self.vel / speed * self.max_speed
        self.pos = self.pos + self.vel * self.dt
        self.t += self.dt
        self.step_count += 1

        collision_type = self._collision_type(old_pos, self.pos)
        collision = bool(collision_type)
        success = bool(np.linalg.norm(self.goal - self.pos) <= self.goal_radius and not collision)
        terminated = bool(success or collision)
        truncated = bool(self.step_count >= self.max_steps and not terminated)

        old_potential_at_current_time, _, _ = self._progress_potential(old_pos, self.t)
        current_potential, wait_time, uses_gate = self._progress_potential(self.pos, self.t)
        progress_delta = 0.0
        if self.reward_progress != 0.0 and self.progress_mode == "dynamic_geometry":
            if (
                old_potential_at_current_time < 0.5 * self.gate_unreachable_cost
                and current_potential < 0.5 * self.gate_unreachable_cost
            ):
                raw_delta = old_potential_at_current_time - current_potential
                progress_delta = float(np.clip(raw_delta, -self.progress_delta_clip, self.progress_delta_clip))
        progress_reward = self.reward_progress * progress_delta
        if collision and self.suppress_positive_progress_on_collision and progress_reward > 0.0:
            progress_delta = 0.0
            progress_reward = 0.0
        if self.reward_guidance != 0.0 and not collision:
            guidance_dir, _, _, _, _, _ = self._progress_guidance(old_pos, self.t)
            speed_scale = max(1e-6, self.max_speed)
            guidance_reward = self.reward_guidance * float(np.dot(self.vel / speed_scale, guidance_dir))
        else:
            guidance_reward = 0.0
        self._prev_progress_potential = current_potential
        self._last_progress_potential = current_potential
        self._last_progress_delta = progress_delta
        self._last_progress_reward = progress_reward
        self._last_guidance_reward = guidance_reward
        self._last_dynamic_path_wait_time = wait_time
        self._last_dynamic_path_uses_gate = uses_gate

        reward = -abs(self.reward_time)
        reward += -abs(self.reward_action) * float(np.dot(action, action))
        reward += progress_reward
        reward += guidance_reward
        if success:
            reward += self.reward_goal
        if collision:
            reward += self.reward_collision
        if truncated:
            reward += self.reward_timeout

        self.trajectory.append(self.pos.copy())
        info = self._info(success=success, collision=collision, truncated=truncated, collision_type=collision_type)
        info["action_norm"] = float(np.linalg.norm(action))
        return self.get_privileged_obs(), float(reward), terminated, truncated, info

    def get_privileged_obs(self) -> GraphObs:
        return self._build_graph_obs()

    def _build_graph_obs(self) -> GraphObs:
        cell_nodes: dict[tuple[int, int], int] = {}
        node_features: list[np.ndarray] = []
        node_type: list[int] = []
        node_positions: list[np.ndarray] = []
        agent_cell = self._cell_from_position(self.pos)
        goal_cell = self.maze.goal_cell

        for r in range(self.maze.rows):
            for c in range(self.maze.cols):
                cell = (r, c)
                center = self._cell_center(cell)
                cell_nodes[cell] = len(node_features)
                node_features.append(self._cell_node_features(cell, center, agent_cell, goal_cell))
                node_type.append(0)
                node_positions.append(center)

        gate_nodes: dict[int, int] = {}
        for gate in self.maze.gates:
            gate_nodes[gate.id] = len(node_features)
            node_features.append(self._gate_node_features(gate))
            node_type.append(1)
            node_positions.append(gate.center.astype(np.float32))

        edge_index: list[tuple[int, int]] = []
        edge_features: list[np.ndarray] = []

        def add_edge(src: int, dst: int, features: np.ndarray) -> None:
            edge_index.append((src, dst))
            edge_features.append(features)

        gate_by_edge = {gate.cell_edge: gate for gate in self.maze.gates}
        for r in range(self.maze.rows):
            for c in range(self.maze.cols):
                cell = (r, c)
                for nxt in ((r + 1, c), (r, c + 1)):
                    nr, nc = nxt
                    if nr >= self.maze.rows or nc >= self.maze.cols:
                        continue
                    edge = _cell_edge(cell, nxt)
                    gate = gate_by_edge.get(edge)
                    if gate is not None:
                        kind = "gate"
                    elif edge in self.maze.open_edges:
                        kind = "open"
                    else:
                        kind = "wall"
                    i, j = cell_nodes[cell], cell_nodes[nxt]
                    add_edge(i, j, self._edge_features(kind, node_positions[i], node_positions[j], gate))
                    add_edge(j, i, self._edge_features(kind, node_positions[j], node_positions[i], gate))

        for gate in self.maze.gates:
            gate_idx = gate_nodes[gate.id]
            for cell in gate.cell_edge:
                cell_idx = cell_nodes[cell]
                add_edge(gate_idx, cell_idx, self._edge_features("gate_cell", node_positions[gate_idx], node_positions[cell_idx], gate))
                add_edge(cell_idx, gate_idx, self._edge_features("gate_cell", node_positions[cell_idx], node_positions[gate_idx], gate))

        for idx, pos in enumerate(node_positions):
            add_edge(idx, idx, self._edge_features("self", pos, pos, None))

        edges = np.asarray(edge_index, dtype=np.int64).T if edge_index else np.zeros((2, 0), dtype=np.int64)
        return GraphObs(
            global_features=self._global_features(),
            node_features=np.asarray(node_features, dtype=np.float32),
            node_type=np.asarray(node_type, dtype=np.int64),
            edge_index=edges,
            edge_features=np.asarray(edge_features, dtype=np.float32),
        )

    def _global_features(self) -> np.ndarray:
        goal_rel = self.goal - self.pos
        goal_dist = float(np.linalg.norm(goal_rel))
        phase = 2.0 * np.pi * (self.t % max(1e-6, self.max_steps * self.dt)) / max(1e-6, self.max_steps * self.dt)
        safe_count = sum(gate.is_safe(self.t, self.robot_radius, self.safe_margin) for gate in self.maze.gates)
        denom = max(1, len(self.maze.gates))
        if self.reward_guidance != 0.0:
            guidance_dir, guidance_potential, guidance_wait, guidance_uses_gate, target_dist, next_gate_wait = self._progress_guidance(
                self.pos, self.t
            )
            closed_gate_dist, closed_gate_wait = self._closed_gate_ahead(guidance_dir, self.t)
            action_prior = self._guidance_action_prior(
                guidance_dir,
                target_dist,
                next_gate_wait,
                closed_gate_dist,
                closed_gate_wait,
            )
        else:
            guidance_dir = np.zeros(2, dtype=np.float32)
            guidance_potential = 0.0
            guidance_wait = 0.0
            guidance_uses_gate = False
            target_dist = 0.0
            next_gate_wait = 0.0
            closed_gate_dist = 1.0
            closed_gate_wait = 0.0
            action_prior = np.zeros(2, dtype=np.float32)
        return np.array(
            [
                self.pos[0] / self.S,
                self.pos[1] / self.S,
                self.vel[0] / self.max_speed,
                self.vel[1] / self.max_speed,
                goal_rel[0] / self.S,
                goal_rel[1] / self.S,
                goal_dist / self.S,
                self.S / 31.0,
                self.maze.rows / 8.0,
                self.maze.cols / 8.0,
                np.sin(phase),
                np.cos(phase),
                self.step_count / max(1, self.max_steps),
                len(self.maze.gates) / 16.0,
                safe_count / denom,
                1.0,
                guidance_dir[0],
                guidance_dir[1],
                np.clip(guidance_potential / max(1e-6, self.max_steps * self.dt), 0.0, 1.0),
                guidance_wait if guidance_uses_gate else -guidance_wait,
                np.clip(target_dist / max(1e-6, self.S), 0.0, 1.0),
                next_gate_wait,
                closed_gate_dist,
                closed_gate_wait,
                action_prior[0],
                action_prior[1],
            ],
            dtype=np.float32,
        )

    def _cell_node_features(
        self,
        cell: tuple[int, int],
        center: np.ndarray,
        agent_cell: tuple[int, int],
        goal_cell: tuple[int, int],
    ) -> np.ndarray:
        rel_agent = center - self.pos
        rel_goal = center - self.goal
        out = np.zeros(NODE_FEATURE_DIM, dtype=np.float32)
        out[:16] = np.array(
            [
                1.0,
                0.0,
                center[0] / self.S,
                center[1] / self.S,
                rel_agent[0] / self.S,
                rel_agent[1] / self.S,
                rel_goal[0] / self.S,
                rel_goal[1] / self.S,
                np.linalg.norm(rel_agent) / self.S,
                np.linalg.norm(rel_goal) / self.S,
                cell[0] / max(1, self.maze.rows - 1),
                cell[1] / max(1, self.maze.cols - 1),
                float(cell == self.maze.start_cell),
                float(cell == goal_cell),
                float(cell == agent_cell),
                float(cell == goal_cell),
            ],
            dtype=np.float32,
        )
        return out

    def _gate_node_features(self, gate) -> np.ndarray:
        rel_agent = gate.center - self.pos
        rel_goal = gate.center - self.goal
        width = gate.width(self.t)
        theta = gate.theta(self.t)
        required_width = 2.0 * self.robot_radius + self.safe_margin
        clearance = (width - required_width) / max(1e-6, gate.slot_width)
        safe = gate.is_safe(self.t, self.robot_radius, self.safe_margin)
        out = np.zeros(NODE_FEATURE_DIM, dtype=np.float32)
        out[:10] = np.array(
            [
                0.0,
                1.0,
                gate.center[0] / self.S,
                gate.center[1] / self.S,
                rel_agent[0] / self.S,
                rel_agent[1] / self.S,
                rel_goal[0] / self.S,
                rel_goal[1] / self.S,
                np.linalg.norm(rel_agent) / self.S,
                np.linalg.norm(rel_goal) / self.S,
            ],
            dtype=np.float32,
        )
        out[16:] = np.array(
            [
                float(gate.orientation == "vertical"),
                float(gate.orientation == "horizontal"),
                gate.slot_width / self.S,
                width / max(1e-6, gate.slot_width),
                np.clip(clearance, -1.0, 1.0),
                float(safe),
                abs(wrap_angle(theta - gate.theta_ref)) / np.pi,
                self._normalize_gate_time(self._gate_wait_until_safe(gate, self.t)),
                self._normalize_gate_time(self._gate_time_until_unsafe(gate, self.t) if safe else 0.0),
                np.sin(theta),
                np.cos(theta),
                gate.d_min / max(1e-6, gate.slot_width),
                gate.d_max / max(1e-6, gate.slot_width),
                gate.omega_d / 2.0,
                gate.theta_amp / np.pi,
                gate.omega_theta / 2.0,
            ],
            dtype=np.float32,
        )
        return out

    def _edge_features(self, kind: str, src: np.ndarray, dst: np.ndarray, gate) -> np.ndarray:
        delta = dst - src
        distance = float(np.linalg.norm(delta))
        direction = delta / distance if distance > 1e-8 else np.zeros(2, dtype=np.float32)
        out = np.zeros(EDGE_FEATURE_DIM, dtype=np.float32)
        type_index = {"self": 0, "wall": 1, "open": 2, "gate": 3, "gate_cell": 4}[kind]
        out[type_index] = 1.0
        out[5:8] = np.array([direction[0], direction[1], distance / max(1e-6, self.S)], dtype=np.float32)
        if gate is None:
            out[8] = 1.0 if kind in {"self", "open"} else 0.0
            return out

        width = gate.width(self.t)
        theta = gate.theta(self.t)
        required_width = 2.0 * self.robot_radius + self.safe_margin
        safe = gate.is_safe(self.t, self.robot_radius, self.safe_margin)
        out[8:] = np.array(
            [
                float(safe),
                self._normalize_gate_time(self._gate_wait_until_safe(gate, self.t)),
                self._normalize_gate_time(self._gate_time_until_unsafe(gate, self.t) if safe else 0.0),
                np.clip((width - required_width) / max(1e-6, gate.slot_width), -1.0, 1.0),
                abs(wrap_angle(theta - gate.theta_ref)) / np.pi,
                float(gate.orientation == "vertical"),
                float(gate.orientation == "horizontal"),
                gate.slot_width / self.S,
                width / max(1e-6, gate.slot_width),
                np.sin(theta),
                np.cos(theta),
                1.0,
            ],
            dtype=np.float32,
        )
        return out

    def _cell_from_position(self, pos: np.ndarray) -> tuple[int, int]:
        geom = self._grid_geometry()
        c = int(np.floor((float(pos[0]) - geom["margin"]) / max(1e-6, geom["cell_w"])))
        r = int(np.floor((float(pos[1]) - geom["margin"]) / max(1e-6, geom["cell_h"])))
        return int(np.clip(r, 0, self.maze.rows - 1)), int(np.clip(c, 0, self.maze.cols - 1))

    def _cell_center(self, cell: tuple[int, int]) -> np.ndarray:
        geom = self._grid_geometry()
        r, c = cell
        return np.array(
            [
                geom["margin"] + (c + 0.5) * geom["cell_w"],
                geom["margin"] + (r + 0.5) * geom["cell_h"],
            ],
            dtype=np.float32,
        )

    def _grid_geometry(self) -> dict[str, float]:
        margin = max(0.70, 0.055 * self.S)
        return {
            "margin": margin,
            "cell_w": (self.S - 2.0 * margin) / self.maze.cols,
            "cell_h": (self.S - 2.0 * margin) / self.maze.rows,
        }

    def _ray_features(self) -> np.ndarray:
        angles = np.linspace(0.0, 2.0 * np.pi, self.num_rays, endpoint=False, dtype=np.float32)
        distances = []
        for angle in angles:
            direction = np.array([np.cos(angle), np.sin(angle)], dtype=np.float32)
            distances.append(self._ray_distance(direction))
        # 射线距离按当前迷宫尺度归一化，便于不同尺寸共享同一输入维度。
        clipped = np.minimum(np.asarray(distances, dtype=np.float32), self.ray_max_dist)
        return np.clip(clipped / self.ray_max_dist, 0.0, 1.0).astype(np.float32)

    def _ray_distance(self, direction: np.ndarray) -> float:
        rects = self._current_obstacle_rects()
        hits = [ray_rect_distance(self.pos, direction, rect) for rect in rects]
        boundary = self._ray_boundary_distance(direction)
        values = [h for h in hits if h is not None]
        values.append(boundary)
        return float(min(values)) if values else self.ray_max_dist

    def _ray_boundary_distance(self, direction: np.ndarray) -> float:
        hits: list[float] = []
        dx, dy = float(direction[0]), float(direction[1])
        if dx > 1e-8:
            hits.append((self.S - self.pos[0]) / dx)
        elif dx < -1e-8:
            hits.append((0.0 - self.pos[0]) / dx)
        if dy > 1e-8:
            hits.append((self.S - self.pos[1]) / dy)
        elif dy < -1e-8:
            hits.append((0.0 - self.pos[1]) / dy)
        return float(min([h for h in hits if h > 1e-8], default=self.ray_max_dist))

    def _current_obstacle_rects(self) -> list[dict[str, float]]:
        rects = list(self.maze.walls)
        for gate in self.maze.gates:
            if not gate.is_safe(self.t, self.robot_radius, self.safe_margin):
                rects.append(gate.slot_rect)
        return rects

    def _build_dynamic_geometry_roadmap(self) -> None:
        self._roadmap_nodes = [self.goal.copy().astype(np.float32)]
        self._roadmap_edges = [[]]
        self._roadmap_gates = {gate.id: gate for gate in self.maze.gates}
        self._gate_approach_nodes = {}
        self._visibility_blockers = self._inflated_visibility_blockers()

        for gate in self.maze.gates:
            p0, p1 = self._gate_approach_points(gate)
            if not (self._is_clear_point(p0) and self._is_clear_point(p1)):
                continue
            i0 = self._add_roadmap_node(p0)
            i1 = self._add_roadmap_node(p1)
            self._gate_approach_nodes[gate.id] = (i0, i1)

        for rect in self._visibility_blockers:
            cx = 0.5 * (rect["xmin"] + rect["xmax"])
            cy = 0.5 * (rect["ymin"] + rect["ymax"])
            for x in (rect["xmin"], rect["xmax"]):
                for y in (rect["ymin"], rect["ymax"]):
                    direction = np.array([np.sign(x - cx), np.sign(y - cy)], dtype=np.float32)
                    point = np.array([x, y], dtype=np.float32) + 0.05 * direction
                    if self._is_clear_point(point):
                        self._add_roadmap_node(point)

        self._roadmap_edges = [[] for _ in self._roadmap_nodes]
        for i in range(len(self._roadmap_nodes)):
            for j in range(i + 1, len(self._roadmap_nodes)):
                if self._visible(self._roadmap_nodes[i], self._roadmap_nodes[j]):
                    self._add_static_edge(i, j, None)

        for gate_id, (i0, i1) in self._gate_approach_nodes.items():
            self._add_static_edge(i0, i1, gate_id)

    def _add_roadmap_node(self, point: np.ndarray) -> int:
        point = point.astype(np.float32)
        key = tuple(np.round(point, 3))
        for idx, existing in enumerate(self._roadmap_nodes):
            if tuple(np.round(existing, 3)) == key:
                return idx
        self._roadmap_nodes.append(point)
        return len(self._roadmap_nodes) - 1

    def _add_static_edge(self, i: int, j: int, gate_id: int | None) -> None:
        travel_time = float(np.linalg.norm(self._roadmap_nodes[j] - self._roadmap_nodes[i]) / max(1e-6, self.max_speed))
        self._roadmap_edges[i].append(RoadmapEdge(j, travel_time, gate_id))
        self._roadmap_edges[j].append(RoadmapEdge(i, travel_time, gate_id))

    def _inflated_visibility_blockers(self) -> list[dict[str, float]]:
        clearance = self.robot_radius + 0.02
        rects = list(self.maze.walls)
        rects.extend(gate.slot_rect for gate in self.maze.gates)
        return [_inflate_rect(rect, clearance) for rect in rects]

    def _gate_approach_points(self, gate) -> tuple[np.ndarray, np.ndarray]:
        offset = 0.5 * gate.wall_thickness + self.robot_radius + 0.06
        if gate.orientation == "vertical":
            return (
                np.array([gate.center[0] - offset, gate.center[1]], dtype=np.float32),
                np.array([gate.center[0] + offset, gate.center[1]], dtype=np.float32),
            )
        return (
            np.array([gate.center[0], gate.center[1] - offset], dtype=np.float32),
            np.array([gate.center[0], gate.center[1] + offset], dtype=np.float32),
        )

    def _is_clear_point(self, point: np.ndarray) -> bool:
        if np.any(point < self.robot_radius) or np.any(point > self.S - self.robot_radius):
            return False
        for rect in self._visibility_blockers:
            if _point_in_rect(point, rect):
                return False
        return True

    def _visible(self, p0: np.ndarray, p1: np.ndarray) -> bool:
        if not (self._point_within_bounds(p0) and self._point_within_bounds(p1)):
            return False
        for rect in self._visibility_blockers:
            if _segment_intersects_rect(p0, p1, rect):
                return False
        return True

    def _point_within_bounds(self, point: np.ndarray) -> bool:
        return bool(np.all(point >= self.robot_radius) and np.all(point <= self.S - self.robot_radius))

    def _progress_potential(self, pos: np.ndarray, t: float) -> tuple[float, float, bool]:
        if self.progress_mode != "dynamic_geometry":
            return 0.0, 0.0, False
        if not self._roadmap_nodes:
            return self.gate_unreachable_cost, 0.0, False

        goal_index = 0
        heap: list[tuple[float, int, float, bool]] = []
        best: dict[int, float] = {}
        current_gate_edges = self._current_gate_edges(pos, t)
        for node_idx, travel_time, wait_time, uses_gate in self._current_visibility_edges(pos):
            total = travel_time
            heapq.heappush(heap, (total, node_idx, wait_time, uses_gate))
        for node_idx, travel_time, wait_time, uses_gate in current_gate_edges:
            total = travel_time + wait_time
            heapq.heappush(heap, (total, node_idx, wait_time, uses_gate))

        while heap:
            cost, node_idx, wait_sum, uses_gate = heapq.heappop(heap)
            if cost >= best.get(node_idx, float("inf")):
                continue
            best[node_idx] = cost
            if node_idx == goal_index:
                return float(cost), float(wait_sum), bool(uses_gate)
            arrival_time = t + cost
            for edge in self._roadmap_edges[node_idx]:
                wait = 0.0
                edge_uses_gate = edge.gate_id is not None
                if edge.gate_id is not None:
                    wait = self._gate_wait_until_safe(self._roadmap_gates[edge.gate_id], arrival_time)
                next_cost = cost + wait + edge.travel_time
                if next_cost < best.get(edge.to, float("inf")):
                    heapq.heappush(heap, (next_cost, edge.to, wait_sum + wait, uses_gate or edge_uses_gate))
        return self.gate_unreachable_cost, 0.0, False

    def _progress_guidance(self, pos: np.ndarray, t: float) -> tuple[np.ndarray, float, float, bool, float, float]:
        if self.progress_mode != "dynamic_geometry":
            return np.zeros(2, dtype=np.float32), 0.0, 0.0, False, 0.0, 0.0
        if not self._roadmap_nodes:
            return np.zeros(2, dtype=np.float32), self.gate_unreachable_cost, 0.0, False, 0.0, 0.0

        goal_index = 0
        heap: list[tuple[float, int, float, bool, float, float, float, float]] = []
        best: dict[int, float] = {}
        initial_edges = self._current_visibility_edges(pos)
        initial_edges.extend(self._current_gate_edges(pos, t))
        for node_idx, travel_time, wait_time, uses_gate in initial_edges:
            direction = self._roadmap_nodes[node_idx] - pos
            distance = float(np.linalg.norm(direction))
            if distance > 1e-8:
                direction = direction / distance
            else:
                direction = np.zeros(2, dtype=np.float32)
            total = travel_time + wait_time
            heapq.heappush(
                heap,
                (total, node_idx, wait_time, uses_gate, float(direction[0]), float(direction[1]), distance, self._normalize_gate_time(wait_time)),
            )

        while heap:
            cost, node_idx, wait_sum, uses_gate, dir_x, dir_y, target_dist, next_gate_wait = heapq.heappop(heap)
            if cost >= best.get(node_idx, float("inf")):
                continue
            best[node_idx] = cost
            if node_idx == goal_index:
                direction = np.array([dir_x, dir_y], dtype=np.float32)
                return direction, float(cost), self._normalize_gate_time(wait_sum), bool(uses_gate), float(target_dist), float(next_gate_wait)
            arrival_time = t + cost
            for edge in self._roadmap_edges[node_idx]:
                wait = 0.0
                edge_uses_gate = edge.gate_id is not None
                if edge.gate_id is not None:
                    wait = self._gate_wait_until_safe(self._roadmap_gates[edge.gate_id], arrival_time)
                next_cost = cost + wait + edge.travel_time
                if next_cost < best.get(edge.to, float("inf")):
                    first_gate_wait = next_gate_wait
                    if first_gate_wait <= 1e-6 and edge_uses_gate:
                        first_gate_wait = self._normalize_gate_time(wait)
                    heapq.heappush(
                        heap,
                        (next_cost, edge.to, wait_sum + wait, uses_gate or edge_uses_gate, dir_x, dir_y, target_dist, first_gate_wait),
                    )
        return np.zeros(2, dtype=np.float32), self.gate_unreachable_cost, 0.0, False, 0.0, 0.0

    def _closed_gate_ahead(self, direction: np.ndarray, t: float) -> tuple[float, float]:
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-8:
            return 1.0, 0.0
        direction = direction / norm
        best_distance = float("inf")
        best_wait = 0.0
        for gate in self.maze.gates:
            if gate.is_safe(t, self.robot_radius, self.safe_margin):
                continue
            rel = gate.center - self.pos
            forward = float(np.dot(rel, direction))
            if forward <= 0.0:
                continue
            lateral = float(abs(rel[0] * direction[1] - rel[1] * direction[0]))
            if lateral > 0.5 * gate.slot_width + self.robot_radius:
                continue
            if forward < best_distance:
                best_distance = forward
                best_wait = self._normalize_gate_time(self._gate_wait_until_safe(gate, t))
        if not np.isfinite(best_distance):
            return 1.0, 0.0
        return float(np.clip(best_distance / max(1e-6, self.S), 0.0, 1.0)), best_wait

    def _guidance_action_prior(
        self,
        direction: np.ndarray,
        target_dist: float,
        next_gate_wait: float,
        closed_gate_dist: float,
        closed_gate_wait: float,
    ) -> np.ndarray:
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-8:
            desired_vel = np.zeros(2, dtype=np.float32)
        else:
            direction = (direction / norm).astype(np.float32)
            target_dist_world = float(target_dist)
            closed_dist_world = float(closed_gate_dist) * self.S
            braking_distance = 0.60 + float(np.dot(self.vel, direction).clip(0.0, self.max_speed)) ** 2 / (2.0 * self.max_acc)
            should_wait = (next_gate_wait > 0.005 and target_dist_world < 1.25) or (
                closed_gate_wait > 0.005 and closed_dist_world < max(1.2, braking_distance)
            )
            if should_wait:
                desired_vel = -0.25 * direction
            else:
                speed = min(self.max_speed, max(0.35, 1.8 * target_dist_world))
                if target_dist_world < 1.25 and next_gate_wait <= 0.005:
                    speed = max(speed, 1.5)
                if closed_dist_world < 1.5 and closed_gate_wait <= 0.005:
                    speed = max(speed, 1.5)
                desired_vel = direction * speed
        accel = (desired_vel - self.vel) / max(1e-6, self.dt)
        return np.clip(accel / max(1e-6, self.max_acc), -0.95, 0.95).astype(np.float32)

    def _current_visibility_edges(self, pos: np.ndarray) -> list[tuple[int, float, float, bool]]:
        edges = []
        if not self._point_within_bounds(pos):
            return edges
        for idx, node in enumerate(self._roadmap_nodes):
            if self._visible(pos, node):
                travel_time = float(np.linalg.norm(node - pos) / max(1e-6, self.max_speed))
                edges.append((idx, travel_time, 0.0, False))
        return edges

    def _current_gate_edges(self, pos: np.ndarray, t: float) -> list[tuple[int, float, float, bool]]:
        edges = []
        for gate in self.maze.gates:
            blocker = _inflate_rect(gate.slot_rect, self.robot_radius + 0.02)
            if not _point_in_rect(pos, blocker):
                continue
            wait = self._gate_wait_until_safe(gate, t)
            for node_idx in self._gate_approach_nodes.get(gate.id, ()):
                travel_time = float(np.linalg.norm(self._roadmap_nodes[node_idx] - pos) / max(1e-6, self.max_speed))
                edges.append((node_idx, travel_time, wait, True))
        return edges

    def _gate_wait_until_safe(self, gate, arrival_time: float) -> float:
        resolution = max(1e-6, self.gate_time_resolution)
        steps = int(np.ceil(self.gate_lookahead_time / resolution))
        for step in range(steps + 1):
            wait = step * resolution
            if gate.is_safe(arrival_time + wait, self.robot_radius, self.safe_margin):
                return float(wait)
        return self.gate_unreachable_cost

    def _gate_time_until_unsafe(self, gate, arrival_time: float) -> float:
        resolution = max(1e-6, self.gate_time_resolution)
        steps = int(np.ceil(self.gate_lookahead_time / resolution))
        for step in range(steps + 1):
            wait = step * resolution
            if not gate.is_safe(arrival_time + wait, self.robot_radius, self.safe_margin):
                return float(wait)
        return self.gate_lookahead_time

    def _normalize_gate_time(self, value: float) -> float:
        if value >= 0.5 * self.gate_unreachable_cost:
            return 1.0
        return float(np.clip(value / max(1e-6, self.gate_lookahead_time), 0.0, 1.0))

    def _collision_type(self, old_pos: np.ndarray, new_pos: np.ndarray) -> str:
        # 先判定落点碰撞，再判定一步运动中是否穿过薄墙或窗口槽位。
        if np.any(new_pos < self.robot_radius) or np.any(new_pos > self.S - self.robot_radius):
            return "boundary"

        for wall in self.maze.walls:
            if circle_intersects_rect(new_pos, self.robot_radius, wall):
                return "wall"
        for gate in self.maze.gates:
            if circle_intersects_rect(new_pos, self.robot_radius, gate.slot_rect) and not self._gate_motion_is_allowed(old_pos, new_pos, gate):
                return "closed_gate"

        segment_hit = self._segment_collision(old_pos, new_pos)
        return segment_hit

    def _segment_collision(self, old_pos: np.ndarray, new_pos: np.ndarray) -> str:
        candidates: list[tuple[float, str]] = []
        delta = new_pos - old_pos
        for segment in self.maze.wall_segments:
            if segment.orientation == "vertical":
                denom = float(delta[0])
                if abs(denom) < 1e-8:
                    continue
                alpha = (segment.coord - float(old_pos[0])) / denom
                axis_cross = float(old_pos[1] + alpha * delta[1])
            else:
                denom = float(delta[1])
                if abs(denom) < 1e-8:
                    continue
                alpha = (segment.coord - float(old_pos[1])) / denom
                axis_cross = float(old_pos[0] + alpha * delta[0])

            if not 0.0 <= alpha <= 1.0:
                continue
            if not segment.span[0] <= axis_cross <= segment.span[1]:
                continue

            hit_type = "wall"
            gates = [g for g in self.maze.gates if g.wall_id == segment.id]
            for gate in gates:
                lo, hi = gate.slot_axis_bounds
                if lo <= axis_cross <= hi:
                    # 只有安全窗口内、且运动方向主要垂直于墙面时，才视作合法穿越。
                    if self._gate_motion_is_allowed(old_pos, new_pos, gate):
                        hit_type = ""
                    else:
                        hit_type = "closed_gate"
                    break
            if hit_type:
                candidates.append((float(alpha), hit_type))
            else:
                candidates.append((float(alpha), ""))
        if not candidates:
            return ""
        candidates.sort(key=lambda item: item[0])
        for _, hit_type in candidates:
            if hit_type:
                return hit_type
        return ""

    def _gate_motion_is_allowed(self, old_pos: np.ndarray, new_pos: np.ndarray, gate) -> bool:
        if not gate.is_safe(self.t, self.robot_radius, self.safe_margin):
            return False

        safe_lo, safe_hi = gate.safe_axis_bounds(self.t, self.robot_radius)
        delta = new_pos - old_pos
        if gate.orientation == "vertical":
            axis_value = float(new_pos[1])
            normal_delta = float(delta[0])
            axis_delta = float(delta[1])
        else:
            axis_value = float(new_pos[0])
            normal_delta = float(delta[1])
            axis_delta = float(delta[0])

        if not safe_lo <= axis_value <= safe_hi:
            return False
        if abs(normal_delta) <= 1e-8:
            return False
        return abs(normal_delta) >= 0.35 * abs(axis_delta)

    def _info(self, success: bool, collision: bool, truncated: bool, collision_type: str) -> dict[str, Any]:
        return {
            "t": self.t,
            "step": self.step_count,
            "S": self.S,
            "pos": self.pos.copy(),
            "vel": self.vel.copy(),
            "goal": self.goal.copy(),
            "ray_max_dist": self.ray_max_dist,
            "success": success,
            "collision": collision,
            "timeout": bool(truncated),
            "collision_type": collision_type,
            "closed_gate_collision": collision_type == "closed_gate",
            "wall_collision": collision_type == "wall",
            "boundary_collision": collision_type == "boundary",
            "progress_potential": self._last_progress_potential,
            "progress_delta": self._last_progress_delta,
            "progress_reward": self._last_progress_reward,
            "guidance_reward": self._last_guidance_reward,
            "dynamic_path_wait_time": self._last_dynamic_path_wait_time,
            "dynamic_path_uses_gate": self._last_dynamic_path_uses_gate,
        }

    def render(self):
        return render_rgb(self)


def _cell_edge(a: tuple[int, int], b: tuple[int, int]) -> tuple[tuple[int, int], tuple[int, int]]:
    return (a, b) if a <= b else (b, a)


def world_to_pixel(pos: np.ndarray, S: float, image_size: int) -> tuple[int, int]:
    x = int(np.clip(pos[0] / S * (image_size - 1), 0, image_size - 1))
    y = int(np.clip((1.0 - pos[1] / S) * (image_size - 1), 0, image_size - 1))
    return x, y


def _draw_disk(img: np.ndarray, cx: int, cy: int, radius_px: int, color: np.ndarray) -> None:
    h, w = img.shape[:2]
    yy, xx = np.ogrid[:h, :w]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius_px**2
    img[mask] = color


def _inflate_rect(rect: dict[str, float], amount: float) -> dict[str, float]:
    return {
        "xmin": float(rect["xmin"] - amount),
        "xmax": float(rect["xmax"] + amount),
        "ymin": float(rect["ymin"] - amount),
        "ymax": float(rect["ymax"] + amount),
    }


def _point_in_rect(point: np.ndarray, rect: dict[str, float]) -> bool:
    return bool(rect["xmin"] <= float(point[0]) <= rect["xmax"] and rect["ymin"] <= float(point[1]) <= rect["ymax"])


def _segment_intersects_rect(p0: np.ndarray, p1: np.ndarray, rect: dict[str, float]) -> bool:
    direction = p1 - p0
    tmin = 0.0
    tmax = 1.0
    for axis, lo_key, hi_key in ((0, "xmin", "xmax"), (1, "ymin", "ymax")):
        d = float(direction[axis])
        origin = float(p0[axis])
        lo = float(rect[lo_key])
        hi = float(rect[hi_key])
        if abs(d) < 1e-8:
            if lo <= origin <= hi:
                continue
            return False
        t1 = (lo - origin) / d
        t2 = (hi - origin) / d
        t_near = min(t1, t2)
        t_far = max(t1, t2)
        tmin = max(tmin, t_near)
        tmax = min(tmax, t_far)
        if tmin > tmax:
            return False
    return bool(tmax >= 1e-8 and tmin <= 1.0 - 1e-8)


def _draw_rect(img: np.ndarray, rect: dict[str, float], S: float, color: np.ndarray) -> None:
    size = img.shape[0]
    x0, y1 = world_to_pixel(np.array([rect["xmin"], rect["ymin"]], dtype=np.float32), S, size)
    x1, y0 = world_to_pixel(np.array([rect["xmax"], rect["ymax"]], dtype=np.float32), S, size)
    img[min(y0, y1) : max(y0, y1) + 1, min(x0, x1) : max(x0, x1) + 1] = color


def _draw_line(img: np.ndarray, p0: tuple[int, int], p1: tuple[int, int], color: np.ndarray, thickness: int = 2) -> None:
    from PIL import Image, ImageDraw

    # 旋转窗口方向线用透明图层绘制，避免斜线像素台阶过重。
    h, w = img.shape[:2]
    scale = 3
    overlay = Image.new("RGBA", (w * scale, h * scale), (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    draw.line(
        [(p0[0] * scale, p0[1] * scale), (p1[0] * scale, p1[1] * scale)],
        fill=tuple(int(v) for v in color) + (255,),
        width=max(1, (2 * thickness + 1) * scale),
    )
    resample = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
    overlay = overlay.resize((w, h), resample)
    base = Image.fromarray(img).convert("RGBA")
    base.alpha_composite(overlay)
    img[:] = np.asarray(base.convert("RGB"), dtype=np.uint8)


def _draw_gate_orientation(img: np.ndarray, env: ContinuousMazeEnv, gate) -> None:
    center_px = world_to_pixel(gate.center, env.S, img.shape[0])
    base_angle = 0.0 if gate.orientation == "vertical" else 0.5 * np.pi
    angle = base_angle + gate.theta(env.t)
    half_len = max(10, int(0.42 * gate.slot_width / env.S * img.shape[0]))
    dx = int(np.cos(angle) * half_len)
    dy = int(-np.sin(angle) * half_len)
    color = np.array([35, 90, 210] if gate.is_safe(env.t, env.robot_radius, env.safe_margin) else [185, 55, 65], dtype=np.uint8)
    _draw_line(img, (center_px[0] - dx, center_px[1] - dy), (center_px[0] + dx, center_px[1] + dy), color, thickness=2)


def render_rgb(env: ContinuousMazeEnv) -> np.ndarray:
    size = env.render_size
    img = np.full((size, size, 3), 246, dtype=np.uint8)
    for wall in env.maze.walls:
        _draw_rect(img, wall, env.S, np.array([35, 38, 42], dtype=np.uint8))
    for gate in env.maze.gates:
        color = np.array([196, 238, 204] if gate.is_safe(env.t, env.robot_radius, env.safe_margin) else [244, 190, 184], dtype=np.uint8)
        _draw_rect(img, gate.slot_rect, env.S, color)
        _draw_gate_orientation(img, env, gate)
    if env.trajectory:
        for p in env.trajectory:
            x, y = world_to_pixel(p, env.S, size)
            img[max(0, y - 1) : min(size, y + 2), max(0, x - 1) : min(size, x + 2)] = np.array([75, 125, 185], dtype=np.uint8)
    sx, sy = world_to_pixel(env.maze.start, env.S, size)
    gx, gy = world_to_pixel(env.goal, env.S, size)
    ax, ay = world_to_pixel(env.pos, env.S, size)
    _draw_disk(img, sx, sy, 5, np.array([80, 120, 215], dtype=np.uint8))
    _draw_disk(img, gx, gy, 8, np.array([35, 145, 85], dtype=np.uint8))
    _draw_disk(img, ax, ay, max(4, int(env.robot_radius / env.S * size)), np.array([230, 90, 55], dtype=np.uint8))
    return img


GapStepEnv = ContinuousMazeEnv
