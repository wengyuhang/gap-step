"""Coarse/fine layered space-time graph construction."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import MDGConfig
from .dynamic_gate import Scenario
from .models import DiscTrack, GraphNode


@dataclass
class LayeredGraph:
    nodes: dict[int, GraphNode]
    layers: list[list[int]]
    v_max: float
    a_max: float
    feasibility_ratio: float
    max_transition_lookback: float | None = None


def _time_grid(start: float, stop: float, dt: float) -> np.ndarray:
    first = np.ceil((start - 1.0e-12) / dt) * dt
    return np.arange(first, stop + 0.5 * dt, dt)


def build_layered_graph(
    scenario: Scenario,
    tracks: dict[int, list[DiscTrack]],
    config: MDGConfig,
    *,
    dt: float,
    v_max: float,
    refinements: dict[int, tuple[float, set[int]]] | None = None,
    terminal_center: float | None = None,
    max_transition_lookback: float | None = None,
) -> LayeredGraph:
    nodes: dict[int, GraphNode] = {}
    layers: list[list[int]] = []
    next_id = 0
    start = GraphNode(
        node_id=next_id,
        gate_index=-1,
        track_id=-1,
        time=0.0,
        center_local=np.zeros(2),
        center_world=scenario.start.position.copy(),
        radius=0.0,
        kind="start",
    )
    nodes[next_id] = start
    layers.append([next_id])
    next_id += 1

    for gate_index in scenario.order:
        gate = scenario.gates[gate_index]
        layer: list[int] = []
        selected_tracks = tracks.get(gate_index, [])
        start_time, stop_time = 0.0, scenario.horizon
        if refinements and gate_index in refinements:
            center, ids = refinements[gate_index]
            start_time = max(0.0, center - config.graph.refine_time_radius)
            stop_time = min(scenario.horizon, center + config.graph.refine_time_radius)
            selected_tracks = [item for item in selected_tracks if item.track_id in ids]
        for track in selected_tracks:
            for interval_start, interval_stop in track.active_intervals:
                low = max(start_time, interval_start, dt)
                high = min(stop_time, interval_stop)
                for time in _time_grid(low, high, dt):
                    try:
                        center, radius, _, _ = track.evaluate(float(time))
                    except ValueError:
                        continue
                    world = gate.local_to_world(center, float(time))
                    nodes[next_id] = GraphNode(
                        next_id,
                        gate_index,
                        track.track_id,
                        float(time),
                        center.copy(),
                        world,
                        float(radius),
                    )
                    layer.append(next_id)
                    next_id += 1
        layer.sort(
            key=lambda node_id: (
                nodes[node_id].time,
                nodes[node_id].track_id,
                node_id,
            )
        )
        layers.append(layer)

    terminal_low, terminal_high = dt, scenario.horizon
    if terminal_center is not None:
        terminal_low = max(dt, terminal_center - config.graph.refine_time_radius)
        terminal_high = min(
            scenario.horizon, terminal_center + config.graph.refine_time_radius
        )
    terminal_layer: list[int] = []
    for time in _time_grid(terminal_low, terminal_high, dt):
        nodes[next_id] = GraphNode(
            next_id,
            len(scenario.gates),
            -1,
            float(time),
            np.zeros(2),
            scenario.start.position.copy(),
            0.0,
            kind="terminal",
        )
        terminal_layer.append(next_id)
        next_id += 1
    layers.append(terminal_layer)
    return LayeredGraph(
        nodes,
        layers,
        float(v_max),
        config.graph.a_max,
        config.graph.feasibility_ratio,
        max_transition_lookback,
    )


def competing_tracks(
    scenario: Scenario,
    tracks: dict[int, list[DiscTrack]],
    selected: list[GraphNode],
    count: int,
) -> dict[int, tuple[float, set[int]]]:
    refinements: dict[int, tuple[float, set[int]]] = {}
    for node in selected:
        if node.kind != "gate":
            continue
        gate_tracks = tracks[node.gate_index]
        distances: list[tuple[float, int]] = []
        for track in gate_tracks:
            if track.track_id == node.track_id or not track.active_at(node.time):
                continue
            center, _, _, _ = track.evaluate(node.time)
            distances.append((float(np.linalg.norm(center - node.center_local)), track.track_id))
        distances.sort()
        ids = {node.track_id, *(item[1] for item in distances[:count])}
        refinements[node.gate_index] = (node.time, ids)
    return refinements


__all__ = ["LayeredGraph", "build_layered_graph", "competing_tracks"]
