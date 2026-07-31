import numpy as np

from mdg.backend_adapter import optimize_selected_path, selected_constraints
from mdg.models import DiscTrack, GraphNode, GraphSolution


def test_backend_adapter_builds_closed_minco(one_gate_scenario, fast_config):
    track = DiscTrack(
        0,
        0,
        np.array((0.5, 1.5, 2.5)),
        np.zeros((3, 2)),
        np.full(3, 0.25),
        [(0.5, 2.5)],
    )
    nodes = {
        0: GraphNode(0, -1, -1, 0.0, np.zeros(2), one_gate_scenario.start.position, 0.0, "start"),
        1: GraphNode(1, 0, 0, 1.5, np.zeros(2), one_gate_scenario.gates[0].local_to_world(np.zeros(2), 1.5), 0.25),
        2: GraphNode(2, 1, -1, 3.5, np.zeros(2), one_gate_scenario.start.position, 0.0, "terminal"),
    }
    graph = GraphSolution(nodes, [], [0, 1, 2], (3.5, 3.5))
    tracks = {0: [track]}
    result = optimize_selected_path(
        one_gate_scenario, tracks, graph, fast_config, free_points=True
    )
    assert np.all(np.isfinite(result.x))
    assert result.interval_violation <= 1.0e-6
    np.testing.assert_allclose(
        result.trajectory.evaluate(0.0), one_gate_scenario.start.position, atol=1.0e-9
    )
    np.testing.assert_allclose(
        result.trajectory.evaluate(result.total_time),
        one_gate_scenario.start.position,
        atol=1.0e-8,
    )
    constraints = selected_constraints(
        one_gate_scenario, tracks, graph, free_points=True
    )
    assert len(constraints) == 1

