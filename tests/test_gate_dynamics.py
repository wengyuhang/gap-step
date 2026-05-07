from __future__ import annotations

import numpy as np

from gap_step.envs.gate_dynamics import DynamicGates, GateSpec


def test_width_range_and_safe_flags():
    gates = DynamicGates([GateSpec(y=4.0, d_min=0.1, d_max=1.1, omega=1.0)], robot_radius=0.18, safe_margin=0.1)
    ts = np.linspace(0.0, 10.0, 200)
    widths = np.array([gates.widths(float(t))[0] for t in ts])
    assert widths.min() >= 0.1 - 1e-6
    assert widths.max() <= 1.1 + 1e-6
    safe = gates.safe_flags(0.0)[0]
    assert safe == float(gates.widths(0.0)[0] > 2 * 0.18 + 0.1)


def test_fixed_width_is_constant():
    gates = DynamicGates([GateSpec(y=4.0, fixed_width=1.2)], robot_radius=0.18, safe_margin=0.1)
    assert gates.widths(0.0)[0] == gates.widths(9.0)[0]
    assert gates.safe_flags(0.0)[0] == 1.0
