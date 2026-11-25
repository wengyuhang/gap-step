"""Measured-error recovery, in addition to measured whole-body exits."""
from convex_timevarying_window.event_triggered_mpc.scheduler import Scheduler as Parent


class Scheduler(Parent):
    def resolved(self, now, accepted, rejection):
        request = super().resolved(now, accepted, rejection)
        # A correction is not evidence of recovery. Reassess the error against
        # the newly installed reference on the next observation, even if it
        # never dropped through the low hysteresis threshold.
        if accepted:
            self.tracking_latched = False
        return request
