"""Common MPC adapter: use only the available valid reference interval.

The number of shooting nodes and cost/input constraints are unchanged.
When a short N=2 path has <1 s remaining, reduce the horizon's time span
instead of requesting undefined reference or inventing a braking segment.
This is not trajectory extrapolation or a new safety certificate.
"""
from dataclasses import replace
from copy import copy
import time
import numpy as np
from convex_timevarying_window.checked_soft_mpc_isolated.fast.mpc import MPC as BaseMPC


def covered_until(owner, at):
    with owner.lock:
        active = owner.plan_at(at)
        if active is None or active.terminal_goal:
            return np.inf
        end = active.end_time
        pending = owner.pending
        if pending is not None and pending.made_at <= end+1e-8:
            if pending.terminal_goal: return np.inf
            end = max(end, pending.end_time)
        return end


class MPC(BaseMPC):
    def command(self, state, now, reference):
        began = time.perf_counter()
        nominal = self.cfg
        at = now+nominal.state_prediction_s
        owner = getattr(reference, '__self__', None)
        if owner is not None:
            with owner.lock:
                owner = copy(owner)
                reference = owner.sample
        end = np.inf if owner is None else covered_until(owner, at)
        duration = min(nominal.steps*nominal.dt, end-at-1e-6)
        if duration < .20:
            raise RuntimeError('insufficient_valid_reference: less than 200ms remains')
        dt = duration/nominal.steps
        old_dt = getattr(self, '_controls_dt', nominal.dt)
        if self.controls is not None and abs(old_dt-dt)>1e-10:
            # Resample previous optimal inputs in physical time; do not
            # reinterpret an old grid as if it had the new sample interval.
            old_times = np.arange(nominal.steps)*old_dt
            new_times = np.arange(nominal.steps)*dt
            self.controls = np.column_stack([np.interp(new_times, old_times, self.controls[:,j])
                                           for j in range(self.controls.shape[1])])
        self.cfg = replace(nominal, dt=dt)
        try:
            result = super().command(state, now, reference)
            self._controls_dt = dt
            self.logs[-1].update(reference_horizon_s=duration, reference_dt_s=dt)
            self.logs[-1]['cycle_ms'] = 1000*(time.perf_counter()-began)
            if self.logs[-1]['cycle_ms'] >= 100:
                raise RuntimeError('MPC adapter deadline')
            return result
        finally:
            self.cfg = nominal
