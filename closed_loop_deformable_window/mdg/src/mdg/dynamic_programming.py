"""Deterministic Bellman solver for layered MDG graphs."""

from __future__ import annotations

import numpy as np

from .models import GraphEdge, GraphSolution
from .point_mass import lower_bound_time, point_mass_time
from .time_graph import LayeredGraph


def solve_layered_graph(
    graph: LayeredGraph,
    *,
    blocked_edges: set[tuple[int, int]] | None = None,
) -> GraphSolution | None:
    blocked = set() if blocked_edges is None else set(blocked_edges)
    nodes = graph.nodes
    cost: dict[int, float] = {graph.layers[0][0]: 0.0}
    predecessor: dict[int, int] = {}
    winning_edges: dict[int, GraphEdge] = {}
    for layer_index in range(1, len(graph.layers)):
        previous_ids = np.asarray(
            [value for value in graph.layers[layer_index - 1] if value in cost],
            dtype=int,
        )
        if not len(previous_ids):
            return None
        previous_world = np.asarray([nodes[int(value)].center_world for value in previous_ids])
        previous_radius = np.asarray([nodes[int(value)].radius for value in previous_ids])
        previous_time = np.asarray([nodes[int(value)].time for value in previous_ids])
        previous_cost = np.asarray([cost[int(value)] for value in previous_ids])
        previous_tracks = np.asarray(
            [nodes[int(value)].track_id for value in previous_ids]
        )
        for target_id in graph.layers[layer_index]:
            target = nodes[target_id]
            candidate_indices = np.arange(len(previous_ids))
            if graph.max_transition_lookback is not None:
                recent = previous_time >= (
                    target.time - graph.max_transition_lookback
                )
                selected = set(np.flatnonzero(recent).tolist())
                old = np.flatnonzero(~recent)
                for track_id in np.unique(previous_tracks[old]):
                    track_old = old[previous_tracks[old] == track_id]
                    if not len(track_old):
                        continue
                    selected.add(
                        int(track_old[np.argmax(previous_time[track_old])])
                    )
                    selected.add(
                        int(track_old[np.argmin(previous_cost[track_old])])
                    )
                candidate_indices = np.asarray(sorted(selected), dtype=int)
            if not len(candidate_indices):
                continue
            candidate_ids = previous_ids[candidate_indices]
            candidate_world = previous_world[candidate_indices]
            candidate_radius = previous_radius[candidate_indices]
            candidate_time = previous_time[candidate_indices]
            candidate_cost = previous_cost[candidate_indices]
            delta = target.time - candidate_time
            distances = np.maximum(
                0.0,
                np.linalg.norm(candidate_world - target.center_world[None, :], axis=1)
                - candidate_radius
                - target.radius,
            )
            lower = distances / graph.v_max
            switching = graph.v_max * graph.v_max / graph.a_max
            point_times = np.where(
                distances <= switching,
                2.0 * np.sqrt(distances / graph.a_max),
                2.0 * graph.v_max / graph.a_max
                + (distances - switching) / graph.v_max,
            )
            feasible = (
                (delta > 0.0)
                & (delta + 1.0e-12 >= lower)
                & (delta + 1.0e-12 >= graph.feasibility_ratio * point_times)
            )
            if blocked:
                feasible &= np.asarray(
                    [(int(source), target_id) not in blocked for source in candidate_ids]
                )
            indices = np.flatnonzero(feasible)
            if not len(indices):
                continue
            candidates = candidate_cost[indices] + point_times[indices]
            best_local = min(
                range(len(indices)),
                key=lambda offset: (
                    float(candidates[offset]),
                    int(candidate_ids[indices[offset]]),
                ),
            )
            index = int(indices[best_local])
            source = int(candidate_ids[index])
            cost[target_id] = float(candidates[best_local])
            predecessor[target_id] = source
            winning_edges[target_id] = GraphEdge(
                source,
                target_id,
                float(point_times[index]),
                float(distances[index]),
            )
    terminals = [value for value in graph.layers[-1] if value in cost]
    if not terminals:
        return None
    terminal = min(terminals, key=lambda value: (nodes[value].time, cost[value], value))
    path = [terminal]
    while path[-1] in predecessor:
        path.append(predecessor[path[-1]])
    path.reverse()
    if path[0] != graph.layers[0][0] or len(path) != len(graph.layers):
        return None
    edges = [winning_edges[value] for value in sorted(winning_edges)]
    return GraphSolution(
        nodes=nodes,
        edges=edges,
        selected_node_ids=path,
        objective=(nodes[terminal].time, cost[terminal]),
        blocked_edges=blocked,
    )


__all__ = ["solve_layered_graph"]
