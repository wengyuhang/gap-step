from __future__ import annotations

import numpy as np


class HeuristicTeacher:
    def __init__(self, env, kp: float = 4.0, kd: float = 2.0, staging_dist: float = 0.75, exit_dist: float = 0.8):
        self.env = env
        self.kp = float(kp)
        self.kd = float(kd)
        self.staging_dist = float(staging_dist)
        self.exit_dist = float(exit_dist)

    def act(self) -> tuple[np.ndarray, int]:
        env = self.env
        widths, safe = env.get_gate_labels()
        gate_idx = self._choose_gate(widths, safe)
        gate_y = float(env.gates.centers[gate_idx])

        if env.crossed_wall:
            target = env.goal
        elif safe[gate_idx] > 0.5:
            entry = np.array([env.wall_x - env.entry_dist, gate_y], dtype=np.float32)
            if env.pos[0] < env.wall_x - env.entry_dist - 0.05 and np.linalg.norm(entry - env.pos) > 0.2:
                target = entry
            else:
                target = np.array([env.wall_x + self.exit_dist, gate_y], dtype=np.float32)
        else:
            target = np.array([env.wall_x - self.staging_dist, gate_y], dtype=np.float32)

        acc = self.kp * (target - env.pos) - self.kd * env.vel
        acc = np.clip(acc, -env.max_acc, env.max_acc).astype(np.float32)
        return acc, gate_idx

    def _choose_gate(self, widths: np.ndarray, safe: np.ndarray) -> int:
        env = self.env
        safe_ids = np.flatnonzero(safe > 0.5)
        if len(safe_ids) > 0:
            costs = []
            for idx in safe_ids:
                gate_y = env.gates.centers[idx]
                entry = np.array([env.wall_x - env.entry_dist, gate_y], dtype=np.float32)
                exit_pt = np.array([env.wall_x + self.exit_dist, gate_y], dtype=np.float32)
                costs.append(np.linalg.norm(entry - env.pos) + np.linalg.norm(env.goal - exit_pt))
            return int(safe_ids[int(np.argmin(costs))])
        return int(np.argmax(widths))
