from __future__ import annotations

import numpy as np

from gap_step.graph import GraphObs
from gap_step.window_maze_env import TimeVaryingWindowMazeEnv


def test_start_goal_are_collision_free() -> None:
    env = TimeVaryingWindowMazeEnv()
    env.reset(options={"phase": 0})
    assert env._point_collision_type(env.start, env.t) == ""
    assert env._point_collision_type(env.goal, env.t) == ""


def test_wall_collision_terminates_episode() -> None:
    env = TimeVaryingWindowMazeEnv()
    env.reset(options={"phase": 0})
    env.pos = np.array([1.0, 1.0], dtype=np.float32)
    _, _, terminated, _, info = env.step(np.array([0.0, -1.0], dtype=np.float32))
    assert terminated
    assert info["collision"]
    assert info["wall_collision"]
    assert not info["success"]


def test_dynamic_window_obstacle_is_collision() -> None:
    env = TimeVaryingWindowMazeEnv()
    env.reset(options={"phase": 0})
    found_window_collision = False
    for phase in range(env.period):
        for state in env.window_state(phase):
            for obstacle in state["obstacles"]:
                center = np.array(obstacle, dtype=np.float32).mean(axis=0)
                found_window_collision |= env._point_collision_type(center, phase) == "window"
    assert found_window_collision


def test_windows_vary_across_phases() -> None:
    env = TimeVaryingWindowMazeEnv({"stage_name": "C5", "fixed_layout": True})
    gaps_by_phase = np.array([[state["gap_width"] for state in env.window_state(t)] for t in range(env.period)])
    assert gaps_by_phase.shape[0] == env.period
    assert gaps_by_phase.shape[1] >= 6
    assert np.ptp(gaps_by_phase[:, 0]) > 0.2
    assert np.ptp(gaps_by_phase[:, 1]) > 0.2
    assert np.ptp(gaps_by_phase[:, 2]) > 0.2
    assert np.any(gaps_by_phase <= 0.72)


def test_render_is_nonblank_rgb() -> None:
    env = TimeVaryingWindowMazeEnv({"render_width": 320, "render_height": 180})
    env.reset(options={"phase": 3})
    frame = env.render()
    assert frame.shape == (180, 320, 3)
    assert frame.std() > 10.0


def test_generated_layout_is_reproducible_by_seed() -> None:
    env = TimeVaryingWindowMazeEnv({"stage_name": "C5"})
    env.reset(seed=123, options={"stage_name": "C5", "split": "id_test"})
    grid_a = env.maze_grid.copy()
    windows_a = [(w.start, w.end, w.cell, w.orientation) for w in env.windows]
    env.reset(seed=123, options={"stage_name": "C5", "split": "id_test"})
    grid_b = env.maze_grid.copy()
    windows_b = [(w.start, w.end, w.cell, w.orientation) for w in env.windows]
    assert np.array_equal(grid_a, grid_b)
    assert windows_a == windows_b
    env.reset(seed=124, options={"stage_name": "C5", "split": "id_test"})
    assert not np.array_equal(grid_a, env.maze_grid)


def test_stage_window_counts_and_endpoint_semantics() -> None:
    c1 = TimeVaryingWindowMazeEnv({"stage_name": "C1"})
    c1.reset(seed=1, options={"stage_name": "C1"})
    assert len(c1.windows) == 0
    env = TimeVaryingWindowMazeEnv({"stage_name": "C5"})
    env.reset(seed=2, options={"stage_name": "C5"})
    assert len(env.windows) >= 6
    for window in env.windows:
        r, c = window.cell
        assert env.maze_grid[r, c] == 0
        if window.orientation == "vertical":
            assert env.maze_grid[r - 1, c] == 1
            assert env.maze_grid[r + 1, c] == 1
            assert abs(window.start[0] - (c + 0.5)) < 1e-6
            assert abs(window.end[0] - (c + 0.5)) < 1e-6
        else:
            assert env.maze_grid[r, c - 1] == 1
            assert env.maze_grid[r, c + 1] == 1
            assert abs(window.start[1] - (r + 0.5)) < 1e-6
            assert abs(window.end[1] - (r + 0.5)) < 1e-6


def test_graph_observation_shape_and_no_action_prior() -> None:
    env = TimeVaryingWindowMazeEnv({"stage_name": "C5", "return_graph_obs": True})
    obs, _ = env.reset(seed=5, options={"stage_name": "C5"})
    assert isinstance(obs, GraphObs)
    assert obs.global_features.shape == (26,)
    assert obs.node_features.shape[1] == 32
    assert obs.edge_features.shape[1] == 20
    assert np.allclose(obs.global_features[24:26], 0.0)
