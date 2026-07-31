import numpy as np

from mdg.geometry import circle_covered
from mdg.safe_disks import generate_safe_disks

from conftest import static_gate


def test_discs_are_nonoverclaiming_and_deterministic(fast_config):
    gate = static_gate("star")
    first = generate_safe_disks(gate, 1.0, fast_config)
    second = generate_safe_disks(gate, 1.0, fast_config)
    assert 1 <= len(first) <= fast_config.disks.max_disks_per_gate
    for left, right in zip(first, second):
        np.testing.assert_allclose(left.center, right.center)
        assert left.radius == right.radius
        assert circle_covered(
            gate.safe_polygon(1.0, fast_config.safety.safety_radius),
            left.center,
            left.radius,
        )


def test_1000_time_checks_keep_static_discs_inside(fast_config):
    gate = static_gate("l_shape")
    discs = generate_safe_disks(gate, 0.0, fast_config)
    for time in np.linspace(0.0, 4.0, 1000):
        safe = gate.safe_polygon(float(time), fast_config.safety.safety_radius)
        assert all(circle_covered(safe, disc.center, disc.radius) for disc in discs)

