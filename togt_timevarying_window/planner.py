from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from math import ceil, inf

import numpy as np

from .environment import RaceTrack
from .geometry import path_length


@dataclass(frozen=True)
class PlannerConfig:
    dt: float = 0.10
    max_speed: float = 5.0
    max_time: float = 12.0
    gate_samples_per_axis: int = 3
    wait_steps: int = 12
    smoothness_weight: float = 0.03


@dataclass(frozen=True)
class _Node:
    stage: int
    step: int
    candidate: int


@dataclass
class PlannedTrajectory:
    points: list[np.ndarray]
    times: list[float]
    gate_times: list[float]
    gate_points: list[np.ndarray]
    cost: float

    @property
    def lap_time(self) -> float:
        return self.times[-1] if self.times else 0.0

    @property
    def length(self) -> float:
        return path_length(self.points)


class DynamicTOGTPlanner:
    """TOGT-style planner for ordered gates whose position and shape vary with time."""

    def __init__(self, config: PlannerConfig | None = None):
        self.config = PlannerConfig() if config is None else config

    def plan(self, track: RaceTrack) -> PlannedTrajectory | None:
        max_step = int(round(self.config.max_time / self.config.dt))
        start = _Node(stage=-1, step=0, candidate=0)
        goal_stage = len(track.gates)
        parent: dict[_Node, _Node | None] = {start: None}
        point_at: dict[_Node, np.ndarray] = {start: np.asarray(track.start, dtype=np.float64)}
        cost: dict[_Node, float] = {start: 0.0}
        queue: list[tuple[float, int, _Node]] = [(0.0, 0, start)]
        serial = 0
        terminal: _Node | None = None

        while queue:
            _, _, node = heappop(queue)
            node_cost = cost.get(node, inf)
            if node.stage == goal_stage:
                terminal = node
                break
            if node.step > max_step:
                continue
            current_point = point_at[node]
            next_stage = node.stage + 1
            for arrival_step in self._arrival_steps(node.step, current_point, track, next_stage, max_step):
                t = arrival_step * self.config.dt
                candidates = self._stage_candidates(track, next_stage, t)
                for cand_idx, candidate in enumerate(candidates):
                    travel_steps = self._travel_steps(current_point, candidate)
                    if arrival_step - node.step < travel_steps:
                        continue
                    child = _Node(next_stage, arrival_step, cand_idx)
                    edge_cost = (arrival_step - node.step) * self.config.dt + self._turn_cost(parent, point_at, node, candidate)
                    new_cost = node_cost + edge_cost
                    if new_cost >= cost.get(child, inf):
                        continue
                    parent[child] = node
                    point_at[child] = candidate
                    cost[child] = new_cost
                    serial += 1
                    priority = new_cost + self._time_heuristic(track, next_stage, candidate)
                    heappush(queue, (priority, serial, child))

        if terminal is None:
            return None
        nodes = self._reconstruct_nodes(parent, terminal)
        points = [point_at[node] for node in nodes]
        times = [node.step * self.config.dt for node in nodes]
        gate_nodes = [node for node in nodes if 0 <= node.stage < len(track.gates)]
        return PlannedTrajectory(
            points=points,
            times=times,
            gate_times=[node.step * self.config.dt for node in gate_nodes],
            gate_points=[point_at[node] for node in gate_nodes],
            cost=cost[terminal],
        )

    def _stage_candidates(self, track: RaceTrack, stage: int, t: float) -> np.ndarray:
        if stage >= len(track.gates):
            return np.asarray([track.goal], dtype=np.float64)
        return track.gates[stage].candidates(t, self.config.gate_samples_per_axis)

    def _arrival_steps(self, current_step: int, current_point: np.ndarray, track: RaceTrack, stage: int, max_step: int) -> range:
        target = track.goal if stage >= len(track.gates) else track.gates[stage].center_at(current_step * self.config.dt)
        min_steps = self._travel_steps(current_point, target)
        first = current_step + min_steps
        last = min(max_step, first + self.config.wait_steps)
        return range(first, last + 1)

    def _travel_steps(self, a: np.ndarray, b: np.ndarray) -> int:
        distance = float(np.linalg.norm(np.asarray(b) - np.asarray(a)))
        return max(1, int(ceil(distance / max(self.config.max_speed * self.config.dt, 1e-9))))

    def _turn_cost(self, parent: dict[_Node, _Node | None], point_at: dict[_Node, np.ndarray], node: _Node, candidate: np.ndarray) -> float:
        prev = parent.get(node)
        if prev is None:
            return 0.0
        a = point_at[node] - point_at[prev]
        b = candidate - point_at[node]
        an = float(np.linalg.norm(a))
        bn = float(np.linalg.norm(b))
        if an < 1e-9 or bn < 1e-9:
            return 0.0
        cos_angle = float(np.clip(np.dot(a, b) / (an * bn), -1.0, 1.0))
        return self.config.smoothness_weight * (1.0 - cos_angle)

    def _time_heuristic(self, track: RaceTrack, stage: int, point: np.ndarray) -> float:
        anchors = [point]
        anchors.extend(g.center0 for g in track.gates[stage + 1 :])
        anchors.append(track.goal)
        return path_length(anchors) / max(self.config.max_speed, 1e-9)

    def _reconstruct_nodes(self, parent: dict[_Node, _Node | None], terminal: _Node) -> list[_Node]:
        nodes: list[_Node] = []
        node: _Node | None = terminal
        while node is not None:
            nodes.append(node)
            node = parent[node]
        nodes.reverse()
        return nodes
