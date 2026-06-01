from __future__ import annotations

from collections import deque

import numpy as np

from gap_step.window_maze_env import TimeVaryingWindowMazeEnv


def sample_reference_path(env: TimeVaryingWindowMazeEnv, spacing: float | None = None) -> np.ndarray:
    spacing = env.max_step * 0.85 if spacing is None else float(spacing)
    points = [np.asarray(env.reference_path[0], dtype=np.float32)]
    for a, b in zip(env.reference_path[:-1], env.reference_path[1:]):
        a_arr = np.asarray(a, dtype=np.float32)
        b_arr = np.asarray(b, dtype=np.float32)
        delta = b_arr - a_arr
        dist = float(np.linalg.norm(delta))
        count = max(1, int(np.ceil(dist / max(spacing, 1e-6))))
        for idx in range(1, count + 1):
            points.append(a_arr + delta * (idx / count))
    return np.asarray(points, dtype=np.float32)


def plan_reference_actions(env: TimeVaryingWindowMazeEnv) -> list[np.ndarray] | None:
    samples = sample_reference_path(env)
    goal_idx = len(samples) - 1
    start_state = (0, int(env.t) % env.period)
    queue = deque([start_state])
    parent: dict[tuple[int, int], tuple[tuple[int, int] | None, int]] = {start_state: (None, 0)}
    terminal: tuple[int, int] | None = None

    while queue:
        idx, phase = queue.popleft()
        if idx == goal_idx:
            terminal = (idx, phase)
            break
        for next_idx in (idx, min(goal_idx, idx + 1)):
            next_phase = (phase + 1) % env.period
            if next_idx == idx:
                collision_type = env._point_collision_type(samples[idx], next_phase)
            else:
                collision_type, _ = env._swept_collision(samples[idx], samples[next_idx], phase)
                if not collision_type:
                    collision_type = env._point_collision_type(samples[next_idx], next_phase)
            if collision_type:
                continue
            state = (next_idx, next_phase)
            if state in parent:
                continue
            parent[state] = ((idx, phase), next_idx)
            queue.append(state)

    if terminal is None:
        return None

    transitions: list[tuple[int, int]] = []
    cur = terminal
    while parent[cur][0] is not None:
        prev, next_idx = parent[cur]
        assert prev is not None
        transitions.append((prev[0], next_idx))
        cur = prev
    transitions.reverse()

    actions: list[np.ndarray] = []
    for idx, next_idx in transitions:
        delta = samples[next_idx] - samples[idx]
        actions.append(np.asarray(delta / env.max_step, dtype=np.float32))
    return actions


def planner_action_from_state(env: TimeVaryingWindowMazeEnv) -> np.ndarray | None:
    samples = sample_reference_path(env)
    dists = np.linalg.norm(samples - env.pos[None, :], axis=1)
    start_idx = int(np.argmin(dists))
    if float(dists[start_idx]) > env.max_step * 0.65:
        delta = samples[start_idx] - env.pos
        norm = float(np.linalg.norm(delta))
        return np.asarray(delta / max(env.max_step, norm), dtype=np.float32)

    goal_idx = len(samples) - 1
    start_state = (start_idx, int(env.t) % env.period)
    queue = deque([start_state])
    parent: dict[tuple[int, int], tuple[tuple[int, int] | None, int]] = {start_state: (None, start_idx)}
    terminal: tuple[int, int] | None = None
    while queue:
        idx, phase = queue.popleft()
        if idx == goal_idx:
            terminal = (idx, phase)
            break
        for next_idx in (idx, min(goal_idx, idx + 1)):
            next_phase = (phase + 1) % env.period
            if next_idx == idx:
                collision_type = env._point_collision_type(samples[idx], next_phase)
            else:
                collision_type, _ = env._swept_collision(samples[idx], samples[next_idx], phase)
                if not collision_type:
                    collision_type = env._point_collision_type(samples[next_idx], next_phase)
            if collision_type:
                continue
            state = (next_idx, next_phase)
            if state in parent:
                continue
            parent[state] = ((idx, phase), next_idx)
            queue.append(state)
    if terminal is None:
        return None
    cur = terminal
    first_next = start_idx
    while parent[cur][0] is not None:
        prev, next_idx = parent[cur]
        assert prev is not None
        first_next = next_idx
        if prev == start_state:
            break
        cur = prev
    delta = samples[first_next] - env.pos
    return np.asarray(delta / env.max_step, dtype=np.float32)


def rollout_planner(env: TimeVaryingWindowMazeEnv) -> dict | None:
    actions = plan_reference_actions(env)
    if actions is None:
        return None
    total_reward = 0.0
    final_info = None
    for action in actions:
        _, reward, terminated, truncated, info = env.step(action)
        total_reward += float(reward)
        final_info = info
        if terminated or truncated:
            break
    if final_info is None:
        return None
    return {**final_info, "return": total_reward, "steps": env.step_count, "planned_steps": len(actions)}
