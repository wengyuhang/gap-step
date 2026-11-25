import pytest

from convex_timevarying_window.gs_mppi.direct_ctbr.experiment import parser
from convex_timevarying_window.gs_mppi.direct_ctbr.evaluation import condition_label


def test_coupled_variant_is_opt_in_and_cannot_mix_unvalidated_variants():
    default=parser().parse_args(['--outdir','unused'])
    assert not default.coupled_racing and not default.experimental_racing_guidance
    assert parser().parse_args(['--outdir','unused','--coupled-racing']).coupled_racing
    with pytest.raises(SystemExit):
        parser().parse_args(['--outdir','unused','--coupled-racing','--experimental-racing-guidance'])


def test_report_labels_actual_condition_not_hardest():
    ordinary=dict(motion_rate=1,translation_scale=1,rotation_scale=1,phase=0)
    assert condition_label(ordinary)=='条件 r/t/a/phase(rad)：1/1/1/0。'
    assert '最难' not in condition_label(ordinary)
    assert '未知' in condition_label({})
