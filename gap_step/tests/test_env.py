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


def test_default_reward_matches_strict_v46_sparse_reward():
    env = ContinuousMazeEnv({"stage_name": "C1"})
    env.reset(seed=0, options={"stage_name": "C1", "split": "train"})
    _, reward, terminated, truncated, info = env.step(np.zeros(2, dtype=np.float32))
    assert np.isclose(reward, -0.01)
    assert not terminated
    assert not truncated
    assert info["progress_reward"] == 0.0


def test_dynamic_geometry_progress_rewards_visible_goal_progress():
    env = ContinuousMazeEnv({"stage_name": "C1", "reward_progress": 2.0, "progress_mode": "dynamic_geometry"})
    env.reset(seed=0, options={"stage_name": "C1", "split": "train"})
    env.pos = env.goal + np.array([-0.8, 0.0], dtype=np.float32)
    env.vel = np.zeros(2, dtype=np.float32)
    env.t = 0.0
    far, _, _ = env._progress_potential(env.pos, env.t)
    near, _, _ = env._progress_potential(env.goal + np.array([-0.4, 0.0], dtype=np.float32), env.t)
    assert near < far


def test_progress_reward_clips_large_potential_jumps():
    env = ContinuousMazeEnv(
        {"stage_name": "C1", "reward_progress": 2.0, "progress_mode": "dynamic_geometry", "progress_delta_clip": 0.25}
    )
    env.reset(seed=0, options={"stage_name": "C1", "split": "train"})
    potentials = [(10.0, 0.0, False), (0.0, 0.0, False)]

    def fake_potential(pos, t):
        return potentials.pop(0)

    env._progress_potential = fake_potential
    _, _, _, _, info = env.step(np.zeros(2, dtype=np.float32))
    assert np.isclose(info["progress_delta"], 0.25)
    assert np.isclose(info["progress_reward"], 0.5)


def test_progress_reward_does_not_reward_time_passing_in_place():
    env = ContinuousMazeEnv(
        {"stage_name": "C2", "reward_progress": 2.0, "progress_mode": "dynamic_geometry", "gate_lookahead_time": 60.0}
    )
    env.reset(seed=0, options={"stage_name": "C2", "split": "train"})
    env.vel = np.zeros(2, dtype=np.float32)
    _, _, terminated, truncated, info = env.step(np.zeros(2, dtype=np.float32))
    assert not terminated
    assert not truncated
    assert np.isclose(info["progress_reward"], 0.0)


def test_dynamic_geometry_gate_waits_for_future_safe_window():
    env = ContinuousMazeEnv(
        {
            "stage_name": "C2",
            "reward_progress": 2.0,
            "progress_mode": "dynamic_geometry",
            "gate_lookahead_time": 60.0,
        }
    )
    env.reset(seed=0, options={"stage_name": "C2", "split": "train"})
    gate = env.maze.gates[0]
    closed_t = next(t for t in np.linspace(0.0, 60.0, 601) if not gate.is_safe(float(t), env.robot_radius, env.safe_margin))
    wait = env._gate_wait_until_safe(gate, float(closed_t))
    assert 0.0 < wait < env.gate_unreachable_cost


def test_dynamic_geometry_potential_can_use_required_future_gate():
    env = ContinuousMazeEnv(
        {
            "stage_name": "C2",
            "reward_progress": 2.0,
            "progress_mode": "dynamic_geometry",
            "gate_lookahead_time": 60.0,
        }
    )
    env.reset(seed=1, options={"stage_name": "C2", "split": "train"})
    potential, wait_time, uses_gate = env._progress_potential(env.pos, env.t)
    assert potential < env.gate_unreachable_cost
    assert wait_time >= 0.0
    assert isinstance(uses_gate, bool)


def test_timeout_reward_is_applied_with_progress_shaping():
    env = ContinuousMazeEnv(
        {
            "stage_name": "C1",
            "max_steps": 1,
            "reward_progress": 2.0,
            "reward_timeout": -5.0,
            "progress_mode": "dynamic_geometry",
        }
    )
    env.reset(seed=0, options={"stage_name": "C1", "split": "train"})
    _, reward, terminated, truncated, _ = env.step(np.zeros(2, dtype=np.float32))
    assert not terminated
    assert truncated
    assert reward <= -5.01


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
