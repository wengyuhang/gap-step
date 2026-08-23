from __future__ import annotations

import numpy as np
import pytest

from nonconvex_timevarying_window.cwb_sc_dynatogt.body_model import CuboidBody


def test_vertices_and_edges_have_fixed_binary_sign_order() -> None:
    body = CuboidBody(np.array([1.0, 2.0, 3.0]))
    assert np.array_equal(body.vertices_body[0], [-1.0, -2.0, -3.0])
    assert np.array_equal(body.vertices_body[-1], [1.0, 2.0, 3.0])
    assert len(body.edges) == 12
    for left, right in body.edges:
        assert np.count_nonzero(body.vertices_body[left] != body.vertices_body[right]) == 1


@pytest.mark.parametrize("value", [[0, 1, 1], [-1, 1, 1], [1, np.nan, 1], [1, 2]])
def test_invalid_half_extents_raise(value) -> None:
    with pytest.raises(ValueError):
        CuboidBody(np.asarray(value, dtype=float))
