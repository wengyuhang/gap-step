from __future__ import annotations

from collections import deque
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from gap_step.envs.gate_dynamics import DynamicGates
from gap_step.envs.renderer import render_gray, render_rgb


class GapStepEnv(gym.Env):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 20}

    def __init__(self, config: dict[str, Any]):
        super().__init__()
        self.config = dict(config)
        self.W = float(config.get("W", 12.0))
        self.H = float(config.get("H", 8.0))
        self.dt = float(config.get("dt", 0.05))
        self.max_steps = int(config.get("max_steps", 400))
        self.robot_radius = float(config.get("robot_radius", 0.18))
        self.safe_margin = float(config.get("safe_margin", 0.10))
        self.max_speed = float(config.get("max_speed", 2.0))
        self.max_acc = float(config.get("max_acc", 3.0))
        self.image_size = int(config.get("image_size", 64))
        self.K_obs = int(config.get("K_obs", 4))
        self.wall_x = self.W / 2.0
        self.goal = np.array(config.get("goal", [self.W - 1.0, self.H / 2.0]), dtype=np.float32)
        self.start = np.array(config.get("start", [1.0, self.H / 2.0]), dtype=np.float32)
        self.start_noise = np.array(config.get("start_noise", [0.1, 0.4]), dtype=np.float32)
        self.goal_radius = float(config.get("goal_radius", 0.35))
        self.entry_dist = float(config.get("entry_dist", 0.65))

        self.reward_goal = float(config.get("reward_goal", 10.0))
        self.reward_cross = float(config.get("reward_cross", 3.0))
        self.reward_collision = float(config.get("reward_collision", -10.0))
        self.reward_time = float(config.get("reward_time", -0.01))
        self.reward_progress = float(config.get("reward_progress", 0.1))
        self.reward_action = float(config.get("reward_action", -0.001))

        self.gates = DynamicGates.from_config(config)
        self.num_gates = self.gates.num_gates
        self.action_space = spaces.Box(-self.max_acc, self.max_acc, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Dict(
            {
                "image_stack": spaces.Box(0.0, 1.0, shape=(self.K_obs, self.image_size, self.image_size), dtype=np.float32),
                "proprio": spaces.Box(-np.inf, np.inf, shape=(6,), dtype=np.float32),
            }
        )
        self.np_random: np.random.Generator
        self.frame_stack: deque[np.ndarray] = deque(maxlen=self.K_obs)
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.trajectory: list[np.ndarray] = []
        self.reset()

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        noise = self.np_random.uniform(-self.start_noise, self.start_noise).astype(np.float32)
        self.pos = self.start + noise
        self.pos[1] = np.clip(self.pos[1], self.robot_radius, self.H - self.robot_radius)
        self.vel = np.zeros(2, dtype=np.float32)
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.t = 0.0
        self.step_count = 0
        self.crossed_wall = bool(self.pos[0] > self.wall_x)
        self.crossed_gate = -1
        self.trajectory = [self.pos.copy()]
        self.frame_stack.clear()
        frame = render_gray(self, self.image_size)
        for _ in range(self.K_obs):
            self.frame_stack.append(frame.copy())
        return self._obs(), self._info()

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, -self.max_acc, self.max_acc)
        old_pos = self.pos.copy()
        old_dist = float(np.linalg.norm(self.goal - old_pos))

        self.vel = self.vel + action * self.dt
        speed = float(np.linalg.norm(self.vel))
        if speed > self.max_speed:
            self.vel = self.vel / speed * self.max_speed
        self.pos = self.pos + self.vel * self.dt
        self.t += self.dt
        self.step_count += 1

        collision = self._boundary_collision()
        crossing_success = False
        closed_gate_attempt = False
        crossed_gate_this_step = -1
        if self._crossed_wall_segment(old_pos, self.pos):
            alpha = (self.wall_x - old_pos[0]) / max(1e-6, self.pos[0] - old_pos[0])
            y_cross = float(old_pos[1] + alpha * (self.pos[1] - old_pos[1]))
            gate_idx = self.gates.gate_at_y(y_cross, self.t, self.robot_radius)
            if gate_idx is None:
                collision = True
                closed_gate_attempt = True
            else:
                crossing_success = True
                crossed_gate_this_step = gate_idx
                self.crossed_wall = True
                self.crossed_gate = gate_idx

        new_dist = float(np.linalg.norm(self.goal - self.pos))
        success = bool(new_dist <= self.goal_radius and self.crossed_wall)
        terminated = bool(success or collision)
        truncated = bool(self.step_count >= self.max_steps)

        reward = self.reward_time
        reward += self.reward_progress * (old_dist - new_dist)
        reward += self.reward_action * float(np.dot(action, action))
        if crossing_success:
            reward += self.reward_cross
        if success:
            reward += self.reward_goal
        if collision:
            reward += self.reward_collision

        self.prev_action = action.astype(np.float32)
        self.trajectory.append(self.pos.copy())
        self.frame_stack.append(render_gray(self, self.image_size))

        info = self._info()
        info.update(
            {
                "success": success,
                "collision": collision,
                "crossing_success": crossing_success,
                "closed_gate_attempt": closed_gate_attempt,
                "crossed_gate": crossed_gate_this_step,
            }
        )
        return self._obs(), float(reward), terminated, truncated, info

    def _crossed_wall_segment(self, old_pos: np.ndarray, new_pos: np.ndarray) -> bool:
        return bool((old_pos[0] - self.wall_x) * (new_pos[0] - self.wall_x) <= 0.0 and old_pos[0] != new_pos[0])

    def _boundary_collision(self) -> bool:
        return bool(
            self.pos[0] < self.robot_radius
            or self.pos[0] > self.W - self.robot_radius
            or self.pos[1] < self.robot_radius
            or self.pos[1] > self.H - self.robot_radius
        )

    def _obs(self) -> dict[str, np.ndarray]:
        goal_rel = self.goal - self.pos
        proprio = np.array(
            [self.vel[0], self.vel[1], goal_rel[0], goal_rel[1], self.prev_action[0], self.prev_action[1]],
            dtype=np.float32,
        )
        return {"image_stack": np.stack(list(self.frame_stack), axis=0).astype(np.float32), "proprio": proprio}

    def _info(self) -> dict[str, Any]:
        widths = self.gates.widths(self.t)
        safe = self.gates.safe_flags(self.t)
        return {
            "t": self.t,
            "step": self.step_count,
            "pos": self.pos.copy(),
            "vel": self.vel.copy(),
            "goal": self.goal.copy(),
            "gate_centers": self.gates.centers.copy(),
            "gate_widths": widths.copy(),
            "safe_flags": safe.copy(),
            "crossed_wall": self.crossed_wall,
            "crossed_gate": self.crossed_gate,
        }

    def get_gate_labels(self) -> tuple[np.ndarray, np.ndarray]:
        return self.gates.widths(self.t), self.gates.safe_flags(self.t)

    def get_teacher_state(self) -> dict[str, Any]:
        info = self._info()
        info.update({"W": self.W, "H": self.H, "wall_x": self.wall_x, "max_acc": self.max_acc})
        return info

    def render(self):
        return render_rgb(self, trajectory=self.trajectory)
