"""Execute the exact frozen polynomials with their north-chart flatness map."""
import numpy as np
from scipy.spatial.transform import Rotation
from convex_timevarying_window.global_gate_minco_probe.flight import Reference as Timeline
from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC as TrackingMPC
from convex_timevarying_window.tilt_heading_minco import dynamics
from convex_timevarying_window.togt_style_mpc.flatness import maps


class Reference(Timeline):
    def sample(self, times):
        with self.lock:
            return np.array([self.state_at(at, 5)[0] for at in times])


class MPC(TrackingMPC):
    def __init__(self, cfg=None):
        super().__init__(cfg)
        count = self.cfg.steps+1
        self.flatness_map = maps().map(count)
        self.metadata.update(reference_attitude='north tilt/heading pi/2',
            reference_derivatives='exact polynomial PVAJS and analytic angular velocity/acceleration')

    def references(self, reference, now, initial_quaternion):
        times = now+np.arange(self.cfg.steps+1)*self.cfg.dt
        values = reference(times)
        velocity, acceleration, jerk, snap = [values[:, k:k+3].T for k in (3, 6, 9, 12)]
        matrices, force, omega, alpha, forces = self.flatness_map(velocity, acceleration, jerk, snap)
        margin = 1.+np.asarray(matrices)[2, 2::3]
        if not np.isfinite(np.asarray(matrices)).all() or np.min(np.asarray(margin)) <= dynamics.CHART_MIN:
            raise RuntimeError('reference_attitude_outside_north_chart_domain')
        rotations = np.stack(np.hsplit(np.asarray(matrices), len(times)))
        quaternions = Rotation.from_matrix(rotations).as_quat()[:, [3, 0, 1, 2]]
        for index, quaternion in enumerate(quaternions):
            previous = initial_quaternion if index == 0 else quaternions[index-1]
            if quaternion@previous < 0.:
                quaternion *= -1.
        omega = np.asarray(omega).T
        states = np.c_[values[:, :6], quaternions, omega]
        return states, np.asarray(forces).T
