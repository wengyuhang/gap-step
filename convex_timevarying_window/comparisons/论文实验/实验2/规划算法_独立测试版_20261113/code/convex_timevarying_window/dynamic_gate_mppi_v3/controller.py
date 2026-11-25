"""Hard-filtered MPPI. A reference is a cost, never executable authority."""
from dataclasses import dataclass, replace
import time
import numpy as np
from .config import Config
from .rollout import Rollout, MASS, MIXER


@dataclass
class Selection:
    made_at: float
    controls: np.ndarray
    states: np.ndarray
    clearances: np.ndarray
    indices: np.ndarray
    cycle_ms: float
    safe_samples: int
    weighted_accepted: bool
    recovered_window: bool
    phases: np.ndarray | None = None

    @property
    def expires_at(self):
        return self.made_at + .02*len(self.controls)


class Controller:
    """CPU oracle; online entrypoints use make_controller (CUDA by default)."""
    def __init__(self, track, config=replace(Config(),samples=512), seed=0):
        self.config = config
        self.rollout = Rollout(track, config)
        self.nc,self.ns=self.rollout.nc,self.rollout.ns
        self.rng = np.random.default_rng(seed)
        self.cache = None
        self.initial_controls = None
        self.temperature = config.temperature
        if self.temperature <= 0: raise ValueError("MPPI temperature must be positive")
        # Unspecified by the PDF: fixed across all courses, in N/s.
        self.thrust_slew = 80.
        knots, samples = np.arange(self.nc//5+1)*.1, np.arange(self.nc)*.02
        self.interpolation = np.stack([np.interp(samples, knots, np.eye(self.nc//5+1)[i])
                                       for i in range(self.nc//5+1)])
        self.last_diagnostics = {}

    def cached_action(self, state, now):
        c = self.cache
        if c is None or now < c.made_at or now >= c.expires_at-1e-10:
            return None
        # Cache validity is conditional on the assumed tracking tube; no tail extension.
        j = min(int(round((now-c.made_at)/.01)), len(c.states)-1)
        if np.linalg.norm(np.asarray(state)[:3]-c.states[j, :3]) > self.config.error_margin:
            return None
        i = int(np.floor((now-c.made_at)/.02+1e-8))
        return c.controls[i].copy()

    def baseline(self, now):
        if self.cache is None:
            if self.initial_controls is not None:
                return self.initial_controls.copy()
            return np.full((self.nc, 4), MASS*9.8066/4)
        shift = max(0, int(round((now-self.cache.made_at)/.02)))
        ids = np.minimum(np.arange(self.nc)+shift, self.nc-1)
        # Repeated tail is a candidate ONLY. It gets full screening below.
        return self.cache.controls[ids].copy()

    def passage_deadline(self,state,now,first,crossing,prepared):
        """Do not postpone an already feasible passage. Revalidate the entire
        remaining cache from CURRENT state; unsafe old controls never bind.
        """
        if first>=len(self.rollout.track.order) or self.cached_action(state,now) is None:
            return None
        shift=max(0,int(round((now-self.cache.made_at)/self.config.dt)))
        remaining=self.cache.controls[shift:]
        if not len(remaining):return None
        check=self.rollout(state,remaining[None],prepared,first,crossing)
        if not check.valid[0]:return None
        exits=np.flatnonzero(check.indices[0]>first)
        return now+exits[0]*self.config.geometry_dt if len(exits) else None

    def imminent_risk(self,prediction):
        """Mode trigger on the executed near prefix, not an extrapolated tail.
        All replacement candidates still pass the FULL horizon hard filter.
        """
        n=int(round(self.config.computation_duration/self.config.geometry_dt))
        s=prediction.states[0,:n+1]
        acceleration=np.linalg.norm(np.diff(s[:,3:6],axis=0),axis=1)/self.config.geometry_dt
        upright=1.-2.*(s[:,7]**2+s[:,8]**2)
        return bool(np.min(prediction.clearances[0,:n])<self.config.computation_clearance
                    or np.max(np.linalg.norm(s[:,3:6],axis=1))>self.config.speed
                    or np.max(acceleration)>self.config.acceleration
                    or np.min(upright)<np.cos(np.deg2rad(self.config.tilt)))

    def seed_reference(self, reference, dynamics_core):
        """Initial shooting seed only; never an admitted/executable trajectory."""
        times = np.clip(np.arange(self.nc)*.02, 0., reference.trajectory.total_time)
        pvajs = np.stack([reference.trajectory.evaluate(times, d) for d in range(5)], axis=1)
        forces = dynamics_core.sample_dynamics(pvajs)["rotor_thrusts"]
        if forces.shape == (self.nc, 4) and np.all(np.isfinite(forces)):
            self.initial_controls = np.clip(forces, 0., self.config.max_rotor_thrust)

    def reference_risk(self, state, now, first, reference, crossing=0):
        if reference is None: return True
        elapsed = now-reference.made_at
        if elapsed < 0: return False  # A future-anchor reference is not yet active.
        if elapsed >= reference.trajectory.total_time: return True
        # Timing/tracking mismatch alone is not collision danger. Gate-exit
        # rolling updates are independent; actual control feasibility is also
        # checked from the measured state in select().
        # Predict risk on the upcoming reference prefix, not on the full lap.
        # This is a trigger ONLY; reference safety is not execution authority.
        end=min(elapsed+min(1.,self.config.horizon),reference.trajectory.total_time)
        count=max(2,int(np.ceil((end-elapsed)/self.config.geometry_dt))+1)
        ts=np.linspace(elapsed,end,count)
        positions=reference.trajectory.evaluate(ts,0)
        velocities=reference.trajectory.evaluate(ts,1)
        poses=self.rollout.geometry.poses(ts+reference.made_at)
        if self.rollout.geometry.interval_clearances(positions,velocities,ts+reference.made_at,poses=poses).min()<.05:
            return True
        if first<len(self.rollout.track.order):
            wi=self.rollout.track.order[first]
            centers,rotations=poses
            normal=rotations[:,wi,:,2]*self.rollout.signs[first]
            z=np.sum((positions-centers[:,wi])*normal,axis=1)
            slab=self.config.radius+self.config.error_margin+self.config.frame_thickness
            # A point trajectory may touch the gate and reverse before the
            # vehicle has fully exited (especially on a return course).
            # This is an explicit task risk, not a gate-index cost mask.
            entered=bool(crossing)
            for value in z:
                if value>slab:return False
                if value>-slab:entered=True
                elif entered:return True
            if crossing:return True  # no complete exit within the lookahead
        return False

    def tracking_error(self, state, now, reference):
        if reference is None:
            return dict(position_error=None, velocity_error=None, tracking_failed=True)
        elapsed = now-reference.made_at
        t = np.clip(elapsed, 0., reference.trajectory.total_time)
        pe = float(np.linalg.norm(state[:3]-reference.trajectory.evaluate(t, 0)))
        ve = float(np.linalg.norm(state[3:6]-reference.trajectory.evaluate(t, 1)))
        return dict(position_error=pe, velocity_error=ve,
                    tracking_failed=bool(elapsed>=reference.trajectory.total_time or
                        pe>self.config.tracking_position_threshold or
                        ve>self.config.tracking_velocity_threshold))

    def sample(self, baseline, avoid, previous_input):
        cfg = self.config
        n_small = int(cfg.samples*(.5 if avoid else .8))
        std = np.full(cfg.samples, .15*cfg.max_rotor_thrust)
        std[:n_small] = .05*cfg.max_rotor_thrust
        noise = self.rng.normal(size=(cfg.samples, self.nc//5+1, 4))*std[:, None, None]
        perturbation = np.einsum("nka,kt->nta", noise, self.interpolation)
        # Wrench-biased exploration uses the physical rotor mixer.
        biases = np.array([[3., 0., 0., 0.], [-3., 0., 0., 0.],
                           [0., .05, 0., 0.], [0., -.05, 0., 0.],
                           [0., 0., .05, 0.], [0., 0., -.05, 0.]])
        rotor_bias = np.linalg.solve(MIXER, biases.T).T
        fade = np.maximum(0., 1.-np.arange(self.nc)*.02/.2)
        group = np.arange(cfg.samples-n_small) % len(biases)
        perturbation[n_small:] += rotor_bias[group, None, :]*fade[None, :, None]
        controls = np.clip(baseline[None]+perturbation, cfg.min_rotor_thrust, cfg.max_rotor_thrust)
        controls[0] = baseline
        previous = np.broadcast_to(previous_input, (cfg.samples, 4)).copy()
        for i in range(self.nc):
            controls[:, i] = previous + np.clip(controls[:, i]-previous,
                                               -self.thrust_slew*.02, self.thrust_slew*.02)
            previous = controls[:, i]
        return controls

    def select(self, state, now, first=0, crossing=False, avoid=False, reference=None,
               previous_input=None):
        started = time.perf_counter()
        cfg = self.config
        baseline = self.baseline(now)
        if previous_input is None: previous_input = baseline[0]
        geometry = self.rollout.prepare(now)
        baseline_check = self.rollout(state,baseline[None],geometry,first,crossing)
        avoid = bool(avoid or not baseline_check.valid[0] or baseline_check.clearances.min()<.05)
        u = self.sample(baseline, avoid, previous_input)
        # Avoidance must not consult the stale TOGT, including its gate points.
        local_points = None if reference is None or avoid else reference.local_points
        prepared = geometry if local_points is None else self.rollout.prepare(now, local_points)
        proposals = self.rollout.guided(state, prepared, first, crossing, previous_input)
        if len(proposals): u[-len(proposals):] = proposals
        batch = self.rollout(state, u, prepared, first, crossing)
        safe = np.flatnonzero(batch.valid)
        self.last_diagnostics = {"safe_samples": len(safe), "cycle_ms": 0.,
                                 "avoidance":bool(avoid),
                                 "baseline_valid": bool(batch.valid[0]),
                                 "baseline_clearance": float(np.min(batch.clearances[0]))}
        if not len(safe):
            self.last_diagnostics["cycle_ms"] = (time.perf_counter()-started)*1000
            return None
        costs = (batch.race.copy() if avoid or reference is None
                 else np.zeros(cfg.samples))
        changes = np.diff(np.concatenate((np.broadcast_to(previous_input, (cfg.samples, 1, 4)), u), axis=1), axis=1)
        costs += .02*np.mean(np.sum((changes/cfg.max_rotor_thrust)**2, axis=-1), axis=1)
        if reference is not None and not avoid:
            times = now+np.arange(self.ns+1)*.01
            p, v, active = reference.sample(times, batch.indices)
            cost_ref = np.sum(((batch.states[..., :3]-p)/.5)**2, axis=-1)
            cost_ref += np.sum(((batch.states[..., 3:6]-v)/2.)**2, axis=-1)
            costs += .01*np.sum(cost_ref*active, axis=1)
        recovered = np.zeros(cfg.samples, dtype=bool)
        # Future windows rank avoidance candidates, but do not start the worker.
        bad = (batch.clearances < cfg.computation_clearance).astype(np.int32)
        prefix = np.pad(np.cumsum(bad, axis=1), ((0, 0), (1, 0)))
        future_windows = (prefix[:, 10:]-prefix[:, :-10] == 0) & (batch.phases[:, :self.ns-9] == 0)
        future_windows &= self.computation_sides(batch,prepared)[:,:self.ns-9]
        recovered = np.any(future_windows, axis=1)
        # In the slab, finish forward if a safe full-exit candidate exists;
        # otherwise allow a checked complete retreat to the entry side.
        pool = safe
        if crossing:
            forward = safe[batch.indices[safe, -1] > first]
            if len(forward): pool = forward
        if avoid and not crossing and np.any(recovered[pool]):
            pool = pool[recovered[pool]]
        minimum = np.min(costs[pool])
        weights = np.exp(np.clip(-(costs[pool]-minimum)/self.temperature, -700, 0))
        weights /= weights.sum()
        weighted = np.einsum("n,nta->ta", weights, u[pool])
        check = self.rollout(state, weighted[None], prepared, first, crossing)
        check_bad = (check.clearances < cfg.computation_clearance).astype(np.int32)
        check_prefix = np.pad(np.cumsum(check_bad, axis=1), ((0, 0), (1, 0)))
        weighted_recovered = bool(np.any((check_prefix[:, 10:]-check_prefix[:, :-10] == 0)
                                         & (check.phases[:, :self.ns-9] == 0)
                                         & self.computation_sides(check,prepared)[:,:self.ns-9]))
        use_weighted = bool(check.valid[0] and
                            (not crossing or not np.any(batch.indices[pool, -1] > first)
                             or check.indices[0, -1] > first) and
                            (not avoid or crossing or not recovered[pool].any() or weighted_recovered))
        if use_weighted:
            controls, result, index = weighted, check, 0
        else:
            index = pool[np.argmin(costs[pool])]
            controls, result = u[index], batch
        elapsed = (time.perf_counter()-started)*1000
        selection = Selection(now, controls.copy(), result.states[index].copy(),
                              result.clearances[index].copy(), result.indices[index].copy(),
                              elapsed, len(safe), use_weighted,
                              weighted_recovered if use_weighted else bool(recovered[index]),
                              result.phases[index].copy())
        self.last_diagnostics.update(cycle_ms=elapsed, avoidance=bool(avoid),
            crossing_phase=int(crossing), forward_samples=int(np.sum(batch.indices[safe,-1]>first)),
            reference_weight=0. if avoid or reference is None else 1.)
        # Publication/deadline alignment belongs to the execution adapter.
        # Calling select alone does NOT publish a possibly late command.
        return selection

    def computation_sides(self,batch,prepared):
        """No computing on the far side of an uncompleted target, even when
        crossing its infinite plane far away was physically harmless."""
        gates=len(self.rollout.track.order)
        ids=np.minimum(batch.indices,gates-1)
        poses=prepared[0][np.arange(self.ns+1)[None,:],ids]
        normal=poses[..., [5,8,11]]
        z=np.sum((batch.states[...,:3]-poses[...,:3])*normal,axis=-1)*self.rollout.signs[ids]
        slab=self.config.radius+self.config.error_margin+self.config.frame_thickness
        return (batch.indices>=gates)|(z<=-slab)

    def publish(self, selection, actual_state, now):
        if selection is None or selection.cycle_ms >= 100. or now >= selection.expires_at:
            return False
        index = int(round((now-selection.made_at)/.01))
        if index < 0 or index >= len(selection.states): return False
        if np.linalg.norm(actual_state[:3]-selection.states[index, :3]) > self.config.error_margin:
            return False
        self.cache = selection
        return True


def make_controller(track,config=Config(),seed=0,backend='cuda'):
    if backend=='ctbr':
        from .commitment_gpu import CommitmentController
        return CommitmentController(track,config,seed)
    if backend=='cuda':
        from .gpu import GPUController
        return GPUController(track,config,seed)
    if backend=='cpu':return Controller(track,config,seed)
    raise ValueError('backend must be cuda or cpu; no silent fallback')
