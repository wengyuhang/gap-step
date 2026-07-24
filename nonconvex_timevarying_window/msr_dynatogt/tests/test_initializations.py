from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.msr_dynatogt.config import MSRConfig
from nonconvex_timevarying_window.msr_dynatogt.initializations import (
    generate_initial_guesses,
)


def test_multiple_initializations_are_reproducible(static_scenario):
    config = MSRConfig.for_suite("smoke")
    first = generate_initial_guesses(
        static_scenario.track,
        config.optimization,
        config.initialization,
        seed=23,
    )
    second = generate_initial_guesses(
        static_scenario.track,
        config.optimization,
        config.initialization,
        seed=23,
    )
    assert [guess.kind for guess in first] == [
        "sc_center",
        "random_perturbation",
        "turn_aware",
        "dispersed_region",
    ]
    assert len(first) == config.initialization.total_starts
    for left, right in zip(first, second):
        assert left.seed == right.seed
        assert np.array_equal(left.x, right.x)


def test_random_seed_changes_random_start_only_deterministically(static_scenario):
    config = MSRConfig.for_suite("smoke")
    left = generate_initial_guesses(
        static_scenario.track, config.optimization, config.initialization, seed=1
    )
    right = generate_initial_guesses(
        static_scenario.track, config.optimization, config.initialization, seed=2
    )
    assert np.array_equal(left[0].x, right[0].x)
    assert not np.array_equal(left[1].x, right[1].x)
