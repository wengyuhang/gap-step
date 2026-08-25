from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.comparisons.sc_sip_motion_rate_benchmark.scenario import build_benchmark_scenario


def test_same_seed_levels_only_change_motion_periods() -> None:
    slow = build_benchmark_scenario(2, "slow").value.track
    fast = build_benchmark_scenario(2, "fast").value.track
    assert slow.order == fast.order
    for left, right in zip(slow.windows, fast.windows):
        np.testing.assert_allclose(left.center0, right.center0)
        np.testing.assert_allclose(left.angles0, right.angles0)
        np.testing.assert_allclose(left.motion.translation_amplitude, right.motion.translation_amplitude)
        np.testing.assert_allclose(left.motion.rotation_amplitude, right.motion.rotation_amplitude)
        assert left.motion.scale_amplitude == right.motion.scale_amplitude
        assert right.motion.translation_period == left.motion.translation_period / 3.0
        assert right.motion.rotation_period == left.motion.rotation_period / 3.0
        assert right.motion.scale_period == left.motion.scale_period / 3.0
        assert left.motion.minimum_scale >= 0.40

