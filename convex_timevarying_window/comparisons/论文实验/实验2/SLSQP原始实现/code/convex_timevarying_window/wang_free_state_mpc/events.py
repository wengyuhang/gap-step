"""Actual passage stages and sampled single-passage contracts.

One-sided plane events far outside a finite gate are not passage events.
Body completion requires the entire enclosing sphere to leave the moving slab.
All trajectory checks here are sampled evidence, never continuous certification.
"""
from dataclasses import dataclass, field
import numpy as np


def local_coordinates(window, positions, times):
    t = np.asarray(times); m = window.motion
    tr = m.translation_amplitude if m.translation_enabled else np.zeros(3)
    ar = m.rotation_amplitude if m.rotation_enabled else np.zeros(3)
    center = window.center0+tr*np.sin(2*np.pi*t[:, None]/m.translation_period+m.phase+np.array([0., .7, 1.4]))
    angles = window.angles0_rpy+ar*np.sin(2*np.pi*t[:, None]/m.rotation_period+m.phase+np.array([0., .9, 1.8]))
    roll, pitch, yaw = angles.T
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    d = positions-center; x = cy*d[:, 0]+sy*d[:, 1]; y = -sy*d[:, 0]+cy*d[:, 1]
    qx = cp*x-sp*d[:, 2]; z = sp*x+cp*d[:, 2]
    return np.column_stack((qx, cr*y+sr*z, -sr*y+cr*z))


def plane_events(window, sign, positions, times, *, gate, nearby_margin=0.):
    local = local_coordinates(window, positions, times); z = sign*local[:, 2]
    # Skip roundoff zeros at exact polynomial knots to avoid double counting.
    nz = np.flatnonzero(np.abs(z)>1e-7); events = []
    extent = window.aperture.radius if window.aperture.kind == 'circle' else np.linalg.norm(window.aperture.physical_vertices, axis=1).max()
    changes = np.flatnonzero(z[nz[:-1]]*z[nz[1:]] < 0.)
    for index in changes:
        a, b = nz[index:index+2]
        alpha = abs(z[a])/(abs(z[a])+abs(z[b])); q = (1-alpha)*local[a, :2]+alpha*local[b, :2]
        inside = window.aperture.contains(q)
        if inside or np.linalg.norm(q) <= extent+nearby_margin:
            events.append(dict(gate=int(gate), time=float((1-alpha)*times[a]+alpha*times[b]),
                direction='forward' if z[a]>0 else 'backward', inside=bool(inside), q=q.tolist()))
    return events, z


def plan_event_check(plan, windows, signs, cfg):
    """Exactly one designated first passage per target, none through other gates."""
    tt = np.unique(np.r_[np.linspace(plan.made_at, plan.end_time,
                         max(2, int(np.ceil((plan.end_time-plan.made_at)/.01))+1)), plan.crossing_times])
    positions = plan.sample_many(tt); expected = dict(zip(
        range(plan.first_gate, plan.first_gate+plan.gate_count), plan.crossing_times))
    slab = cfg.radius+cfg.frame_thickness
    for wi, w in enumerate(windows):
        events, z = plane_events(w, signs[wi], positions, tt, gate=wi, nearby_margin=slab)
        inside = [e for e in events if e['inside']]
        if wi not in expected:
            if inside: return False, f'unplanned_gate_event:{wi}'
            continue
        # Only the current approach has an entry-side head contract. A later
        # finite gate can have its infinite plane behind this distant head.
        if wi == plan.first_gate and z[0] <= 0.:
            return False, f'head_already_beyond_target:{wi}'
        if len(inside) != 1 or inside[0]['direction'] != 'forward':
            return False, f'non_single_passage:{wi}'
        if abs(inside[0]['time']-expected[wi]) > .02:
            return False, f'early_or_late_passage:{wi}'
        if any(e['direction']=='backward' and e['time']>expected[wi] for e in events):
            return False, f'return_to_gate:{wi}'
        remaining = np.flatnonzero((tt>expected[wi]) & (z<=-slab))
        if not len(remaining): return False, f'body_not_exited:{wi}'
    return True, 'single_passage'


@dataclass
class Progress:
    windows: list
    signs: list
    radius: float
    thickness: float
    gate: int = 0
    crossed: bool = False
    crossings: list = field(default_factory=list)
    exits: list = field(default_factory=list)
    violation: str | None = None
    sample_time: float | None = None
    sample_position: np.ndarray | None = field(default=None, repr=False)

    @property
    def phase(self): return 'exit' if self.crossed else 'approach'

    def observe(self, position, measurement_time):
        """Query the moving gate at the position's time, once per fresh sample.

        Re-querying a stale position at each simulation-clock tick can create
        a fictitious reversal while the real body is steadily leaving a gate.
        """
        if self.sample_time is not None and measurement_time <= self.sample_time:
            return False
        old_position, old_time = self.sample_position, self.sample_time
        self.sample_position = np.asarray(position).copy(); self.sample_time = float(measurement_time)
        if old_time is None: return False
        return self.update(old_position, self.sample_position, old_time, self.sample_time)

    def update(self, p0, p1, t0, t1):
        if self.gate >= len(self.windows): return False
        w = self.windows[self.gate]; sign = self.signs[self.gate]
        q0, z0 = w.world_to_local(p0, t0); q1, z1 = w.world_to_local(p1, t1)
        a, b = sign*z0, sign*z1
        if not self.crossed and a >= 0. and b < 0.:
            alpha = abs(a)/(abs(a)+abs(b)); q = (1-alpha)*q0+alpha*q1
            if not w.aperture.contains(q) or w.aperture.boundary_distance(q)<self.radius:
                self.violation = 'missed_current_opening'; return False
            self.crossings.append(dict(gate=self.gate, time=float(t0+alpha*(t1-t0)),
                                       q=q.tolist(), margin=float(w.aperture.boundary_distance(q)-self.radius)))
            self.crossed = True
        elif self.crossed and a < 0. and b >= 0.:
            self.violation = 'reversed_before_body_exit'; return False
        if self.crossed and b <= -(self.radius+self.thickness):
            self.exits.append(dict(gate=self.gate, time=float(t1), signed_distance=float(b)))
            self.gate += 1; self.crossed = False
            return True
        return False


def planning_block_reason(progress, reference, target_time, min_leg_seconds):
    """Keep flying the admitted crossing, rather than re-solving an expired leg."""
    if progress.crossed: return 'body_exiting'
    if progress.gate >= len(progress.windows): return None
    plan = reference.plan_at(target_time)
    if plan is None: return None
    k = progress.gate-plan.first_gate
    if 0 <= k < plan.gate_count:
        if plan.crossing_times[k]-target_time <= min_leg_seconds: return 'committed_passage'
    w = progress.windows[progress.gate]
    _, z = w.world_to_local(plan.sample(target_time), target_time)
    if progress.signs[progress.gate]*z <= progress.radius+progress.thickness:
        return 'predicted_body_in_slab'
    return None
