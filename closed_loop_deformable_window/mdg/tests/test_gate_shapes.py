import numpy as np
import pytest

from mdg.dynamic_gate import Scenario
from mdg.gate_shapes import GATE_TYPES

from conftest import static_gate


@pytest.mark.parametrize("kind", tuple(GATE_TYPES))
def test_each_shape_has_100_valid_hole_free_parameter_values(kind):
    gate = static_gate(kind)
    for deformation in np.linspace(-0.4, 0.4, 100):
        gate.deformation_profile.values[:] = deformation
        gate.deformation_profile.__post_init__()
        polygon = gate.local_polygon(1.0)
        assert polygon.is_valid
        assert polygon.area > 0.0
        assert len(polygon.interiors) == 0


def test_gate_pose_round_trip_and_scenario_serialization(tmp_path, one_gate_scenario):
    gate = one_gate_scenario.gates[0]
    local = np.array((0.2, -0.15))
    world = gate.local_to_world(local, 1.0)
    recovered, plane_error = gate.world_to_local(world, 1.0)
    np.testing.assert_allclose(recovered, local, atol=1.0e-12)
    assert plane_error < 1.0e-12
    path = one_gate_scenario.save(tmp_path / "scenario.json")
    loaded = Scenario.load(path)
    assert loaded.to_dict() == one_gate_scenario.to_dict()

