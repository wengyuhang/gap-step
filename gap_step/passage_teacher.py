from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

from gap_step.passage_env import TimeVaryingPassageMazeEnv


Cell = tuple[int, int]
State = tuple[int, int, int]


@dataclass
class PassageTeacherConfig:
    replan_horizon: int = 360
    arrival_radius: float = 0.18
    safety_probe: float = 0.02


class TimeExpandedPassageTeacher:
    """Privileged teacher with full map, passage phase, and goal access."""

    def __init__(self, config: dict[str, Any] | PassageTeacherConfig | None = None):
        if config is None:
            self.config = PassageTeacherConfig()
        elif isinstance(config, PassageTeacherConfig):
            self.config = config
        else:
            self.config = PassageTeacherConfig(**config)

    def act(self, env: TimeVaryingPassageMazeEnv) -> np.ndarray:
        cell = env.cell_from_pos(env.pos)
        plan = self.plan(env, cell, env.t, env.max_steps - env.step_count)
        if len(plan) <= 1 and cell != env.goal_cell:
            return np.zeros(2, dtype=np.float32)
        if len(plan) <= 1:
            target = env.goal
        else:
            target_cell = plan[1]
            if target_cell == cell:
                return np.zeros(2, dtype=np.float32)
            target = env._cell_center(target_cell)
            if float(np.linalg.norm(target - env.pos)) <= self.config.arrival_radius and len(plan) > 2:
                target = env._cell_center(plan[2])

        vec = target - env.pos
        norm = float(np.linalg.norm(vec))
        if norm <= 1e-6:
            return np.zeros(2, dtype=np.float32)
        if norm <= env.max_step:
            return (vec / env.max_step).astype(np.float32)
        return (vec / norm).astype(np.float32)

    def plan(self, env: TimeVaryingPassageMazeEnv, start: Cell, start_t: int, horizon: int | None = None) -> list[Cell]:
        horizon = self.config.replan_horizon if horizon is None else min(int(horizon), self.config.replan_horizon)
        if start == env.goal_cell:
            return [start]

        start_state: State = (start[0], start[1], int(start_t) % env.period)
        queue: deque[tuple[State, int]] = deque([(start_state, 0)])
        parent: dict[State, State | None] = {start_state: None}
        moves = ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))

        while queue:
            (r, c, phase), depth = queue.popleft()
            if (r, c) == env.goal_cell:
                return self._reconstruct(parent, (r, c, phase))
            if depth >= horizon:
                continue

            next_phase = (phase + 1) % env.period
            for dr, dc in moves:
                nr, nc = r + dr, c + dc
                next_cell = (nr, nc)
                if not self._transition_safe(env, (r, c), next_cell, phase, next_phase):
                    continue
                state = (nr, nc, next_phase)
                if state in parent:
                    continue
                parent[state] = (r, c, phase)
                queue.append((state, depth + 1))

        return [start]

    def _transition_safe(
        self,
        env: TimeVaryingPassageMazeEnv,
        current: Cell,
        target: Cell,
        phase: int,
        next_phase: int,
    ) -> bool:
        if not env.passable(current, phase) or not env.passable(target, phase):
            return False
        if not env.passable(target, next_phase):
            return False
        if current == target:
            return True

        return abs(current[0] - target[0]) + abs(current[1] - target[1]) == 1

    @staticmethod
    def _reconstruct(parent: dict[State, State | None], state: State) -> list[Cell]:
        rev: list[Cell] = []
        cur: State | None = state
        while cur is not None:
            rev.append((cur[0], cur[1]))
            cur = parent[cur]
        rev.reverse()
        return rev


def rollout_teacher(
    env: TimeVaryingPassageMazeEnv,
    teacher: TimeExpandedPassageTeacher,
    seed: int,
    phase_offset: int | None = None,
) -> dict[str, Any]:
    options = {} if phase_offset is None else {"phase_offset": int(phase_offset)}
    old_return_graph = getattr(env, "return_graph_obs", True)
    env.return_graph_obs = False
    _, info = env.reset(seed=seed, options=options)
    total_reward = 0.0
    terminated = False
    truncated = False
    while not (terminated or truncated):
        action = teacher.act(env)
        _, reward, terminated, truncated, info = env.step(action)
        total_reward += float(reward)
    env.return_graph_obs = old_return_graph
    return {
        "seed": int(seed),
        "success": bool(info["success"]),
        "collision": bool(info["collision"]),
        "timeout": bool(info["timeout"]),
        "collision_type": str(info["collision_type"]),
        "steps": int(info["step"]),
        "return": float(total_reward),
        "final_distance": float(np.linalg.norm(env.goal - env.pos)),
    }
