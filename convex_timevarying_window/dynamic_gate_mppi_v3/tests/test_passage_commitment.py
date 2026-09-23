from convex_timevarying_window.dynamic_gate_mppi_v3.tests.test_passage import single_gate
from convex_timevarying_window.dynamic_gate_mppi_v3.geometry import GateProgress
from convex_timevarying_window.dynamic_gate_mppi_v3.rollout import hover_state
from convex_timevarying_window.dynamic_gate_mppi_v3.passage_commitment import activation_phase_matches


def test_predicted_exit_cannot_activate_while_tail_still_inside():
    p=GateProgress(single_gate());p.crossing=2
    assert not activation_phase_matches(p,hover_state([.2,0,4]),0.,1,0)
    assert activation_phase_matches(p,hover_state([1.,0,4]),0.,1,0)


def test_predicted_post_center_must_wait_for_actual_center():
    p=GateProgress(single_gate());p.crossing=1
    assert not activation_phase_matches(p,hover_state([-.2,0,4]),0.,0,2)
    assert activation_phase_matches(p,hover_state([.2,0,4]),0.,0,2)
