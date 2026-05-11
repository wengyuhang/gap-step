from __future__ import annotations

import numpy as np

from gap_step.env import ContinuousMazeEnv


def _cross_gate_points(gate) -> tuple[np.ndarray, np.ndarray]:
    if gate.orientation == "vertical":
        return (
            np.array([gate.center[0] - 0.02, gate.center[1]], dtype=np.float32),
            np.array([gate.center[0] + 0.02, gate.center[1]], dtype=np.float32),
        )
    return (
        np.array([gate.center[0], gate.center[1] - 0.02], dtype=np.float32),
        np.array([gate.center[0], gate.center[1] + 0.02], dtype=np.float32),
    )


def test_reset_step_render_and_info():
    env = ContinuousMazeEnv({"stage_name": "C1"})
    obs, info = env.reset(seed=0, options={"stage_name": "C1", "split": "train"})
    assert obs.shape == (39,)
    assert np.isclose(info["ray_max_dist"], 0.35 * info["S"])
    next_obs, reward, terminated, truncated, info = env.step(np.zeros(2, dtype=np.float32))
    assert next_obs.shape == (39,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert env.render().shape == (env.render_size, env.render_size, 3)


def test_ray_max_dist_scales_with_episode_maze_size():
    env = ContinuousMazeEnv()
    obs, _ = env.reset(seed=0, options={"stage_name": "C1", "split": "train"})
    assert obs.shape == (39,)
    assert env.S == 15.0
    assert np.isclose(env.ray_max_dist, 5.25)
    assert np.all(obs[-env.num_rays :] >= 0.0)
    assert np.all(obs[-env.num_rays :] <= 1.0)

    env.reset(seed=30000, options={"stage_name": "C5", "split": "ood_dynamics_test"})
    assert env.S in {17.0, 25.0, 31.0}
    assert np.isclose(env.ray_max_dist, 0.35 * env.S)


def test_closed_gate_collision_is_reported():
    env = ContinuousMazeEnv({"stage_name": "C2"})
    env.reset(seed=0, options={"stage_name": "C2", "split": "train"})
    gate = env.maze.gates[0]
    env.t = next(t for t in np.linspace(0.0, 60.0, 601) if not gate.is_safe(float(t), env.robot_radius, env.safe_margin))
    old_pos, new_pos = _cross_gate_points(gate)
    collision_type = env._segment_collision(old_pos, new_pos)
    assert collision_type == "closed_gate"


def test_safe_gate_crossing_is_not_a_collision():
    env = ContinuousMazeEnv({"stage_name": "C1"})
    env.reset(seed=0, options={"stage_name": "C1", "split": "train"})
    gate = env.maze.gates[0]
    env.t = 0.0
    old_pos, new_pos = _cross_gate_points(gate)
    assert env._segment_collision(old_pos, new_pos) == ""


def test_horizontal_wall_crossing_is_checked():
    env = ContinuousMazeEnv({"stage_name": "C5"})
    env.reset(seed=0, options={"stage_name": "C5", "split": "train"})
    gate_wall_ids = {gate.wall_id for gate in env.maze.gates}
    horizontal = next(segment for segment in env.maze.wall_segments if segment.orientation == "horizontal" and segment.id not in gate_wall_ids)
    old_pos = np.array([0.5 * (horizontal.span[0] + horizontal.span[1]), horizontal.coord - 0.02], dtype=np.float32)
    new_pos = np.array([old_pos[0], horizontal.coord + 0.02], dtype=np.float32)
    collision_type = env._segment_collision(old_pos, new_pos)
    assert collision_type == "wall"
