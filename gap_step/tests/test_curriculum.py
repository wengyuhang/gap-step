from __future__ import annotations

import numpy as np

from gap_step.curriculum import sample_maze
from gap_step.utils import circle_intersects_rect


def _topology_connected(maze) -> bool:
    seen = {maze.start_cell}
    queue = [maze.start_cell]
    while queue:
        r, c = queue.pop(0)
        for nxt in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            nr, nc = nxt
            if not (0 <= nr < maze.rows and 0 <= nc < maze.cols):
                continue
            edge = ((r, c), nxt) if (r, c) <= nxt else (nxt, (r, c))
            if edge in maze.open_edges and nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    return maze.goal_cell in seen


def test_c5_generates_mixed_orientation_maze():
    maze = sample_maze("C5", "train", seed=0)
    assert maze.S in {15.0, 19.0, 23.0}
    orientations = {segment.orientation for segment in maze.wall_segments}
    assert orientations == {"vertical", "horizontal"}
    assert len(maze.wall_segments) > 20
    assert 6 <= len(maze.gates) <= 10
    assert len(maze.walls) > 0
    assert _topology_connected(maze)


def test_curriculum_gate_counts_and_safe_endpoints():
    for stage, lo, hi in [("C4", 3, 5), ("C5", 6, 10)]:
        maze = sample_maze(stage, "train", seed=4)
        assert lo <= len(maze.gates) <= hi
        assert not any(circle_intersects_rect(maze.start, 0.25, wall) for wall in maze.walls)
        assert not any(circle_intersects_rect(maze.goal, 0.25, wall) for wall in maze.walls)
        assert _topology_connected(maze)


def test_ood_dynamics_uses_out_of_distribution_ranges():
    maze = sample_maze("C5", "ood_dynamics_test", seed=1)
    assert maze.S in {17.0, 25.0, 31.0}
    for gate in maze.gates:
        assert 0.90 <= gate.omega_d <= 1.40
        assert 0.40 <= gate.theta_amp <= 0.70
        assert 0.80 <= gate.omega_theta <= 1.30
        assert 1.00 <= gate.d_max <= 1.35


def test_gate_safety_combines_width_and_angle():
    maze = sample_maze("C1", "train", seed=2)
    gate = maze.gates[0]
    assert gate.is_safe(0.0, robot_radius=0.25, safe_margin=0.1)
    assert np.isclose(gate.width(10.0), 1.4)
