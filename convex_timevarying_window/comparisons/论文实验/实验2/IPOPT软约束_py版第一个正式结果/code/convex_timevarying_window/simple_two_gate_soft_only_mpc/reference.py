"""Absolute-time reference; the prescribed zero-VA endpoint extends the preview."""
from convex_timevarying_window.simple_two_gate_mpc.reference import Reference as Parent
from convex_timevarying_window.wang_free_state_mpc.reference import Reference as Timeline


class Reference(Parent):
    def state_at(self, t, orders=4):
        # Both the third-center endpoint and final return already have V=A=0.
        # Holding that endpoint is only an extension after its timestamp, not a
        # new brake trajectory or a fictitious crossing/finish event.
        return Timeline.state_at(self, t, orders)

    def candidate_reason(self, candidate, gate, phase):
        return 'stale_gate_stage' if candidate.first_gate != gate or phase != 'approach' else None

