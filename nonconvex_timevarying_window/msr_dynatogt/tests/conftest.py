from __future__ import annotations

import pytest

from nonconvex_timevarying_window.sc_dynatogt.preprocessing import PreprocessingConfig
from nonconvex_timevarying_window.sc_dynatogt.scenarios import build_canonical_scenario


@pytest.fixture(scope="session")
def static_scenario():
    return build_canonical_scenario(
        mode="static",
        preprocessing_config=PreprocessingConfig(
            sc_fit_options={"quadrature_order": 32}
        ),
        gate_count=1,
    )
