"""Only startup and measured whole-body exits create replanning work."""
from convex_timevarying_window.event_triggered_mpc.scheduler import Scheduler as Events


class Scheduler(Events):
    def observe(self,now,gate,exited,error):
        if not self.started:
            self.started=True
            self.trigger('initial',now,gate)
        elif exited:
            self.trigger('body_exit',now,gate)
        # No periodic trigger or tracking-error replanning mode. A failed
        # pending event retains the common retry logic, rather than vanishing.
