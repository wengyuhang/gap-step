"""Center-point MINCO initial curves with independently sampled gate timings."""
import numpy as np
from scipy.stats import qmc

from convex_timevarying_window.single_gate_stop_racing import point_mapping
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from .inset_geometry import Planner as InsetPlanner


class Planner(InsetPlanner):
    def __init__(self, *args, initial_count=8, sampling_seed=0, **kwargs):
        if not isinstance(initial_count, int) or initial_count < 1:
            raise ValueError('Initial count must be a positive integer')
        self.initial_count = initial_count
        self.sampling_seed = sampling_seed
        super().__init__(*args, **kwargs)

    def initial_times(self):
        nodes = np.vstack([self.track.start,
            *[window.center0 for window in self.track.windows], self.track.goal])
        distances = np.linalg.norm(np.diff(nodes, axis=0), axis=1)
        fast = np.maximum(.8, distances/max(self.seed_speed, self.cfg.speed))
        slow = np.maximum(.8, distances/min(self.seed_speed, self.cfg.speed))
        fast[0] += .5
        slow[0] += .5
        tail = max(.8, np.sqrt(6.*distances[-1]/self.cfg.acceleration),
            (60.*distances[-1]/self.cfg.jerk)**(1./3.))
        lower, upper = np.exp(self.lower[:self.n]), np.exp(self.upper[:self.n])
        sampler = qmc.Sobol(2*self.m, scramble=True, seed=self.sampling_seed)
        samples = sampler.random_base2((self.initial_count-1).bit_length())[:self.initial_count]
        results = []
        for sample in samples:
            previous = 0.
            durations = []
            for gate, window in enumerate(self.track.windows):
                estimate = fast[gate]+sample[gate]*(slow[gate]-fast[gate])
                target = previous+np.clip(estimate, lower[gate], upper[gate])
                motion = window.motion
                if motion.amplitude > 0. and motion.frequency > 0.:
                    phase = sample[self.m+gate]-motion.phase/(2.*np.pi)
                    first = int(np.ceil(motion.frequency*(previous+lower[gate])-phase))
                    last = int(np.floor(motion.frequency*(previous+upper[gate])-phase))
                    if first <= last:
                        cycle = int(np.clip(round(motion.frequency*target-phase), first, last))
                        target = (cycle+phase)/motion.frequency
                durations.append(float(target-previous))
                previous = target
            durations.append(float(np.clip(tail, lower[-1], upper[-1])))
            results.append(np.asarray(durations))
        return results

    def seeds(self):
        parameters = (np.concatenate([point_mapping.inverse(aperture, np.zeros(2))
            for aperture in self.point_apertures]) if self.free_points else np.empty(0))
        seeds = []
        for durations in self.initial_times():
            arrivals = np.cumsum(durations)[:-1]
            points = np.array([window.state_at(at)[0]
                for window, at in zip(self.track.windows, arrivals)])
            curve = MincoSnap(BoundaryState(self.track.start), BoundaryState(self.track.goal),
                points, durations)
            derivatives = np.concatenate([curve.evaluate(arrivals, order).ravel()
                for order in (1, 2, 3)])
            seeds.append(np.r_[np.log(durations), parameters, derivatives])
        return seeds

    def solve_all(self, *args, **kwargs):
        plan, result = super().solve_all(*args, **kwargs)
        result.update(temporal_scales=None, all_starts_attempted=True,
            initialization='center points at sampled independent gate times and analytic MINCO PVAJ',
            initial_sampling='scrambled Sobol gate travel estimates and motion phases',
            sampling_seed=self.sampling_seed, global_initial_time_scaling=False,
            point_initialization='moving-window centers', center_solution_used=False,
            external_solution_used=False)
        return plan, result
