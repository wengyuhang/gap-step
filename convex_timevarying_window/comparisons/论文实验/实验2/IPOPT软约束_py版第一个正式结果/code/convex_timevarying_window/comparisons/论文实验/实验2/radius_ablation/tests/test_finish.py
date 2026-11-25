from convex_timevarying_window.comparisons.论文实验.实验2.radius_ablation.execution import reference_finished


def test_full_reference_execution_cannot_finish_from_terminal_position_tolerance():
    assert not reference_finished(21., 21., 22.1)
    assert not reference_finished(22.101, 22.09, 22.1)
    assert reference_finished(22.101, 22.1, 22.1)
