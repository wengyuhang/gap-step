from __future__ import annotations

import numpy as np

from gap_step.passage_env import TimeVaryingPassageMazeEnv
from gap_step.passage_teacher import TimeExpandedPassageTeacher


def test_wall_collision_terminates_with_failure() -> None:
    env = TimeVaryingPassageMazeEnv({"max_steps": 20, "return_graph_obs": False})
    env.reset(seed=1, options={"phase_offset": 0})
    env.pos = np.array([1.25, 16.5], dtype=np.float32)
    _, _, terminated, _, info = env.step(np.array([-1.0, 0.0], dtype=np.float32))
    assert terminated
    assert info["collision"]
    assert info["wall_collision"]
    assert not info["success"]


def test_closed_dynamic_passage_is_collision() -> None:
    env = TimeVaryingPassageMazeEnv({"stage_name": "C5", "return_graph_obs": False})
    env.reset(seed=0)
    passage = env.passages[0]
    closed_phase = passage.closed_phases[0]
    open_phase = next(phase for phase in range(env.period) if passage.active_rows(phase, env.period))
    cell = (passage.rows[len(passage.rows) // 2], passage.col)
    pos = env._cell_center(cell)
    closed_grid = env.maze_at_time(closed_phase - passage.offset)
    open_grid = env.maze_at_time(open_phase)
    assert env._point_collision_type(pos, closed_grid) == "dynamic_passage"
    open_cells = [(r, passage.col) for r in passage.active_rows(open_phase, env.period)]
    assert any(env._point_collision_type(env._cell_center(open_cell), open_grid) == "" for open_cell in open_cells)


def test_teacher_solves_hard_maze_without_collision() -> None:
    env = TimeVaryingPassageMazeEnv({"stage_name": "C5", "max_steps": 260, "return_graph_obs": False})
    teacher = TimeExpandedPassageTeacher()
    env.reset(seed=3, options={"phase_offset": 0})
    terminated = False
    truncated = False
    info = {}
    while not (terminated or truncated):
        _, _, terminated, truncated, info = env.step(teacher.act(env))
    assert info["success"]
    assert not info["collision"]
