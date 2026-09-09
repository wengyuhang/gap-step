import numpy as np

from nonconvex_timevarying_window.comparisons.seven_mixed_sc_fixed_cem.experiment import SHAPES


def test_seven_window_shape_sequence_is_mixed():
    assert len(SHAPES) == 7
    assert len(set(SHAPES)) == 6
    assert {"limacon", "wavy", "line_bezier"}.issubset(SHAPES)
    assert SHAPES[0] == SHAPES[-1] == "balanced_U"
