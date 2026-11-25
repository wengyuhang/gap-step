"""Scene queries only: do not change Checked optimization or admission."""
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import (
    symbolic_pose, symbolic_clearance_sq, frame_clearances)


def install():
    from convex_timevarying_window.receding_gate_racing import planner
    from convex_timevarying_window.continuation_racing import guard
    planner.symbolic_pose = symbolic_pose
    planner.symbolic_clearance_sq = symbolic_clearance_sq
    guard.frame_clearances = frame_clearances
