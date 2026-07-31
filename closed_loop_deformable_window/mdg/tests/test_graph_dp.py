import itertools

import numpy as np

from mdg.dynamic_programming import solve_layered_graph
from mdg.models import GraphNode
from mdg.point_mass import point_mass_time
from mdg.time_graph import LayeredGraph


def _node(index, layer, time, x, kind="gate"):
    return GraphNode(index, layer, index, time, np.zeros(2), np.array((x, 0.0, 0.0)), 0.0, kind)


def test_layered_dp_matches_brute_force_and_blocked_edge():
    nodes = {
        0: _node(0, -1, 0.0, 0.0, "start"),
        1: _node(1, 0, 1.0, 1.0),
        2: _node(2, 0, 1.5, 0.7),
        3: _node(3, 1, 2.2, 2.0),
        4: _node(4, 1, 2.5, 1.6),
        5: _node(5, 2, 3.5, 0.0, "terminal"),
        6: _node(6, 2, 4.0, 0.0, "terminal"),
    }
    layers = [[0], [1, 2], [3, 4], [5, 6]]
    graph = LayeredGraph(nodes, layers, 8.0, 12.0, 0.8)
    solution = solve_layered_graph(graph)
    assert solution is not None
    candidates = []
    for middle in itertools.product(*layers[1:-1]):
        for terminal in layers[-1]:
            path = (0, *middle, terminal)
            feasible = True
            secondary = 0.0
            for source, target in zip(path, path[1:]):
                distance = np.linalg.norm(nodes[target].center_world - nodes[source].center_world)
                pm = point_mass_time(distance, 8.0, 12.0)
                feasible &= nodes[target].time - nodes[source].time >= 0.8 * pm
                secondary += pm
            if feasible:
                candidates.append(((nodes[terminal].time, secondary), path))
    expected = min(candidates)
    assert tuple(solution.selected_node_ids) == expected[1]
    blocked = {(solution.selected_node_ids[1], solution.selected_node_ids[2])}
    repaired = solve_layered_graph(graph, blocked_edges=blocked)
    assert repaired is not None
    assert tuple(repaired.selected_node_ids) != tuple(solution.selected_node_ids)
