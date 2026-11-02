"""Known rigid spin R(t)=R0 Rz(2*pi*f_spin*t) about the original gate origin."""
from dataclasses import dataclass

import casadi as ca
import numpy as np

from convex_timevarying_window.geometry import PeriodicConvexWindow
from nonconvex_timevarying_window.sc_dynatogt.environment import rotation_and_derivative
from ..scenario import Case, build_case as translation_case


@dataclass(frozen=True)
class SpinMotion:
    translation_amplitude: np.ndarray
    translation_period: float
    phase: float
    spin_hz: float
    spin_phase: float = 0.

    @property
    def rotation_enabled(self):
        return self.spin_hz != 0. or self.spin_phase != 0.

    def translation(self, t):
        omega = 2*np.pi/self.translation_period
        return (self.translation_amplitude*np.sin(omega*t+self.phase),
                self.translation_amplitude*omega*np.cos(omega*t+self.phase))


class SpinningWindow(PeriodicConvexWindow):
    def __post_init__(self):
        super().__post_init__()
        self.base_rotation = rotation_and_derivative(self.angles0_rpy, np.zeros(3))[0]

    def state_at(self, t):
        m = self.motion
        offset, velocity = m.translation(float(t))
        omega = 2*np.pi*m.spin_hz
        theta = omega*t+m.spin_phase
        c, s = np.cos(theta), np.sin(theta)
        rot = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        derivative = omega*np.array([[-s, -c, 0.], [c, -s, 0.], [0., 0., 0.]])
        return self.center0+offset, self.base_rotation@rot, velocity, self.base_rotation@derivative


def build_case(spin_hz=0., frequency=3., amplitude=2., spin_phase=0.):
    if not np.isfinite(spin_hz) or not np.isfinite(spin_phase):
        raise ValueError('Finite spin parameters required')
    track = translation_case(Case(frequency, amplitude))
    old = track.windows[0]
    window = SpinningWindow(old.name, old.aperture, old.center0.copy(), old.angles0_rpy.copy(),
                            SpinMotion(old.motion.translation_amplitude.copy(),
                                       old.motion.translation_period, old.motion.phase,
                                       spin_hz, spin_phase))
    # SCWindowTrack is frozen in some configurations: construct, do not mutate.
    return type(track)('spinning_dumbbell', track.start.copy(), track.goal.copy(), (window,), (0,))


def symbolic_pose(window, t):
    m = window.motion
    center = ca.DM(window.center0)+ca.DM(m.translation_amplitude)*ca.sin(
        2*np.pi*t/m.translation_period+m.phase)
    theta = 2*np.pi*m.spin_hz*t+m.spin_phase
    c, s = ca.cos(theta), ca.sin(theta)
    rotation = ca.vertcat(ca.horzcat(c, -s, 0), ca.horzcat(s, c, 0), ca.horzcat(0, 0, 1))
    return center, ca.DM(window.base_rotation)@rotation


def local_coordinates(window, positions, times):
    """Vectorized exact moving-frame coordinates for an independent audit."""
    t = np.asarray(times)
    m = window.motion
    centers = window.center0+np.sin(2*np.pi*t[:, None]/m.translation_period+m.phase)*m.translation_amplitude
    base = (np.asarray(positions)-centers)@window.base_rotation
    theta = 2*np.pi*m.spin_hz*t+m.spin_phase
    c, s = np.cos(theta), np.sin(theta)
    return np.column_stack((c*base[:, 0]+s*base[:, 1], -s*base[:, 0]+c*base[:, 1], base[:, 2]))
