"""Only initial planning and measured whole-body exits create requests."""
from convex_timevarying_window.event_triggered_mpc.scheduler import Scheduler as Parent


class Scheduler(Parent):
    def observe(self,now,gate,exited,error):
        if not self.started:
            self.started=True
            self.trigger('initial',now,gate)
        elif exited:
            self.trigger('body_exit',now,gate)
