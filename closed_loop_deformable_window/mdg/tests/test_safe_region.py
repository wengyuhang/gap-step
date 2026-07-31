from dataclasses import replace

import numpy as np

from mdg.config import MDGConfig, config_from_mapping
from mdg.dynamic_gate import SplineSeries

from conftest import static_gate


def test_negative_buffer_can_close_without_invalidating_physical_gate():
    gate = static_gate("u_shape")
    gate.scale_profile = SplineSeries(np.array((0.0, 1.0)), np.array((0.12, 0.12)))
    assert gate.local_polygon(0.5).is_valid
    assert gate.local_polygon(0.5).area > 0.0
    assert gate.safe_polygon(0.5, 0.22).is_empty


def test_strict_configuration_rejects_unknown_keys():
    try:
        config_from_mapping({"graph": {"not_a_setting": 1}})
    except ValueError as exc:
        assert "not_a_setting" in str(exc)
    else:
        raise AssertionError("unknown config key was accepted")


def test_safety_radius_is_component_sum():
    config = MDGConfig()
    assert config.safety.safety_radius == (
        config.safety.drone_collision_radius
        + config.safety.tracking_margin
        + config.safety.geometry_discretization_margin
    )

