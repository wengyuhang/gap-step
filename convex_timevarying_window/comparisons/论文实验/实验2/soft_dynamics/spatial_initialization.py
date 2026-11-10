"""Area-distributed crossing-point initializations at the existing sampled times."""
import numpy as np
from scipy.stats import qmc

from convex_timevarying_window.single_gate_stop_racing import point_mapping
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from .phase_initialization import Planner as PhasePlanner


def area_sample(aperture, first, second):
    if aperture.kind == 'circle':
        angle = 2.*np.pi*second
        return aperture.radius*np.sqrt(first)*np.array([np.cos(angle), np.sin(angle)])
    vertices = aperture.physical_vertices
    center = vertices.mean(axis=0)
    starts = vertices-center
    ends = np.roll(starts, -1, axis=0)
    areas = abs(starts[:, 0]*ends[:, 1]-starts[:, 1]*ends[:, 0])/2.
    cumulative = np.r_[0., np.cumsum(areas)/areas.sum()]
    triangle = int(np.searchsorted(cumulative, first, side='right')-1)
    fraction = (first-cumulative[triangle])/(cumulative[triangle+1]-cumulative[triangle])
    return center+np.sqrt(fraction)*((1.-second)*starts[triangle]+second*ends[triangle])


class Planner(PhasePlanner):
    def __init__(self, *args, point_sampling_seed=1, **kwargs):
        self.point_sampling_seed = point_sampling_seed
        super().__init__(*args, **kwargs)

    def initial_points(self):
        sampler = qmc.Sobol(2*self.m, scramble=True, seed=self.point_sampling_seed)
        samples = sampler.random_base2((self.initial_count-1).bit_length())[:self.initial_count]
        return np.array([[area_sample(aperture, sample[2*gate], sample[2*gate+1])
            for gate, aperture in enumerate(self.point_apertures)] for sample in samples])

    def seeds(self):
        if not self.free_points:
            return super().seeds()
        seeds = []
        for durations, local_points in zip(self.initial_times(), self.initial_points()):
            arrivals = np.cumsum(durations)[:-1]
            parameters = np.concatenate([point_mapping.inverse(aperture, point)
                for aperture, point in zip(self.point_apertures, local_points)])
            points = np.array([window.state_at(at)[0]+window.state_at(at)[1][:, :2]@point
                for window, at, point in zip(self.track.windows, arrivals, local_points)])
            curve = MincoSnap(BoundaryState(self.track.start), BoundaryState(self.track.goal),
                points, durations)
            derivatives = np.concatenate([curve.evaluate(arrivals, order).ravel()
                for order in (1, 2, 3)])
            seeds.append(np.r_[np.log(durations), parameters, derivatives])
        return seeds

    def solve_all(self, *args, **kwargs):
        plan, result = super().solve_all(*args, **kwargs)
        if self.free_points:
            result.update(point_initialization='area-distributed points in the inset openings',
                point_sampling_seed=self.point_sampling_seed,
                initialization='same independent gate times, area-distributed points and recomputed analytic MINCO PVAJ')
        return plan, result
