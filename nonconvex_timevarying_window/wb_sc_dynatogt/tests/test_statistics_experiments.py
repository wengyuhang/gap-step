from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.wb_sc_dynatogt.experiments import FORMAL_METHODS, METHODS, SUITES
from nonconvex_timevarying_window.wb_sc_dynatogt.statistics import (
    paired_bootstrap_interval,
    wilson_interval,
)


def test_formal_protocol_counts_and_methods() -> None:
    formal = SUITES["formal"]
    assert formal.candidate_samples == 100_000
    assert formal.static_seeds == 30
    assert formal.dynamic_seeds == 155
    assert formal.optimizer_iterations == 0
    assert METHODS == ("sc_sphere", "point_model", "wbsc_dynatogt")
    assert METHODS[-1] == "wbsc_dynatogt"
    assert FORMAL_METHODS == ("sc_sphere", "wbsc_dynatogt")


def test_confidence_intervals() -> None:
    low, high = wilson_interval(50, 100)
    assert low < 0.5 < high
    values = np.array([1.0, 2.0, 3.0, 4.0])
    boot_low, boot_high = paired_bootstrap_interval(values, resamples=1_000, seed=4)
    assert boot_low <= np.mean(values) <= boot_high
