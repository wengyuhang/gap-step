"""Risk flags request computation; they never directly trigger a TOGT solve."""
from dataclasses import dataclass
import numpy as np
from .config import Config


@dataclass(frozen=True)
class Request:
    started: float
    anchor: float
    first_gate: int
    reason: str = 'risk'
    generation: int = 0


class Scheduler:
    def __init__(self, config=Config()):
        self.config = config
        self.pending = False
        self.rolling_pending = False
        self.tracking_pending = False
        self.avoiding = False
        self.generation = 0
        self.last_completed_gate = 0
        self.fresh_gate_event = False
        self.streak = 0
        self.running = None
        self.last_request = -np.inf
        self.last_observation = None

    def flag_risk(self):
        self.pending = True
        self.avoiding = True

    def set_risk(self,risk):
        """Flight mode follows current risk, never the planner's busy state."""
        self.avoiding = bool(risk)
        if risk:self.pending = True

    def gate_passed(self,first_gate):
        """Actual FULL exit only. Request next three without enabling avoidance."""
        if first_gate<=self.last_completed_gate:return
        self.last_completed_gate=first_gate
        self.generation+=1
        self.rolling_pending=True
        self.fresh_gate_event=True

    def flag_tracking_error(self):
        # Repeated observations are coalesced, never restart a busy solve.
        self.tracking_pending = True

    def observe(self, now, interval_times, clearances, crossing, first_gate):
        cfg = self.config
        if self.last_observation is not None and now-self.last_observation < cfg.dt-1e-9:
            return None
        if self.last_observation is not None and now-self.last_observation > 1.5*cfg.dt:
            self.streak = 0
        self.last_observation = now
        times, clearances = np.asarray(interval_times), np.asarray(clearances)
        # Require a window from NOW, not a safe island later in the prediction.
        valid = (len(times) == len(clearances)+1 and len(clearances) > 0
                 and times[0] <= now+1e-9 and times[-1] >= now+cfg.computation_duration
                 and np.all(np.diff(times) > 0))
        if valid:
            mask = (times[:-1] < now+cfg.computation_duration) & (times[1:] > now)
            valid = bool(np.all(np.isfinite(clearances[mask])) and
                         np.all(clearances[mask] >= cfg.computation_clearance))
        self.streak = self.streak+1 if valid and not crossing else 0
        # A gate event can start computing while MPPI is still avoiding an
        # unsafe old reference, provided CURRENT controls cover the budget.
        gate_ready=(self.rolling_pending and valid and not crossing)
        tracking_ready=(self.tracking_pending and valid and not crossing)
        risk_ready=(self.pending and self.streak>=cfg.consecutive_windows)
        spacing=(now-self.last_request>=cfg.request_interval or (gate_ready and self.fresh_gate_event))
        if (self.running is None and (gate_ready or tracking_ready or risk_ready) and spacing):
            reason='gate_exit' if gate_ready else ('tracking_error' if tracking_ready else 'risk')
            request = Request(now, now+cfg.anchor_delay, first_gate,reason,self.generation)
            self.running = request
            self.last_request = now
            self.streak = 0
            self.fresh_gate_event=False
            return request
        return None

    def finish(self, request, now, first_gate, position_error, velocity_error, attitude_error):
        if request != self.running:
            return False
        accepted = (request.generation==self.generation
                    and now-request.started <= self.config.planning_budget+1e-9
                    and first_gate == request.first_gate
                    and position_error <= .3 and velocity_error <= 1.
                    and attitude_error <= np.deg2rad(15)
                    and min(position_error, velocity_error, attitude_error) >= 0)
        self.running = None
        if accepted:
            self.pending=False
            self.rolling_pending=False
            self.tracking_pending=False
        elif request.reason=='tracking_error':self.tracking_pending=True
        elif request.reason=='risk':self.pending=True
        else:self.rolling_pending=True
        return accepted

    def failed(self,request):
        if request is None or request!=self.running:return
        self.running=None
        if request.reason=='tracking_error':self.tracking_pending=True
        elif request.reason=='risk':self.pending=True
        else:self.rolling_pending=True
