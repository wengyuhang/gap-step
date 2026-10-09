"""Latched planning events; wall/simulation time alone never creates work."""
from dataclasses import dataclass, field


@dataclass
class Scheduler:
    error_high: float = .35
    error_low: float = .175
    retry_delay: float = .20
    pending: dict | None = None
    inflight: dict | None = None
    next_retry: float = 0.
    tracking_latched: bool = False
    started: bool = False
    serial: int = 0
    log: list = field(default_factory=list)

    def trigger(self, reason, now, gate):
        self.serial += 1
        self.pending = dict(event_id=self.serial, reason=reason, gate=gate,
                            triggered_at=float(now), attempts=0)
        self.next_retry = now
        self.log.append(dict(action='trigger', **self.pending))

    def observe(self, now, gate, exited, error):
        if not self.started:
            self.started = True
            self.trigger('initial', now, gate)
        elif exited:
            self.trigger('body_exit', now, gate)
        if error <= self.error_low:
            self.tracking_latched = False
        elif error > self.error_high and not self.tracking_latched:
            self.tracking_latched = True
            # Existing planning work already covers this state; do not discard it.
            if self.pending is None:
                self.trigger('tracking_error', now, gate)
            else:
                self.log.append(dict(action='coalesced_tracking_error', time=float(now),
                                     gate=gate, event_id=self.pending['event_id']))

    def due(self, now):
        return self.pending is not None and self.inflight is None and now >= self.next_retry

    def submitted(self, now):
        if not self.due(now):
            raise RuntimeError('No pending planning event')
        self.pending['attempts'] += 1
        self.inflight = dict(self.pending)
        self.log.append(dict(action='submit', time=float(now), **self.inflight))

    def resolved(self, now, accepted, rejection):
        if self.inflight is None:
            raise RuntimeError('Response without a planning event')
        request = self.inflight
        self.log.append(dict(action='resolve', time=float(now), accepted=bool(accepted),
                             rejection=rejection, **request))
        # A body-exit event occurring during the solve must survive a stale response.
        if self.pending is not None and self.pending['event_id'] == request['event_id']:
            if accepted:
                self.pending = None
            else:
                self.next_retry = now + self.retry_delay
        self.inflight = None
        return request
