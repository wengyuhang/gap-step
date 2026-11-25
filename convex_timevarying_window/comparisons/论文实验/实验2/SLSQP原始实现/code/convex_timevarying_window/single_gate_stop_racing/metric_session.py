"""Convert PX4 spherical GPS-map positions to Gazebo WGS84 world metres.

This is a coordinate/unit conversion, not ground-truth feedback, a fitted
tracking bias, or a gate-dependent correction. Velocity is already supplied
by the simulator in physical m/s and must NOT be scaled a second time.
"""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr_probe import Session as BaseSession


def metric_scale(latitude_deg):
    latitude = np.radians(latitude_deg)
    a, eccentricity2, px4_radius = 6378137., 6.6943799901413165e-3, 6371000.
    denominator = 1.-eccentricity2*np.sin(latitude)**2
    east_radius = a/np.sqrt(denominator)
    north_radius = a*(1.-eccentricity2)/denominator**1.5
    return np.array([east_radius/px4_radius, north_radius/px4_radius, 1.])


def convert_position(position, anchor, scale, inverse=False):
    return np.asarray(anchor)+(np.asarray(position)-anchor)*(1./scale if inverse else scale)


class Session(BaseSession):
    def __init__(self, out, track, consistent_velocity=False):
        self.metric_anchor = np.asarray(track.start).copy()
        self.metric_world_path = Path(out)/'world.sdf'
        self.metric_scale_enu = None
        super().__init__(out, track, consistent_velocity=consistent_velocity)

    def convert(self, state):
        if self.metric_scale_enu is None:
            root = ET.parse(self.metric_world_path).getroot()
            spherical = root.find('./world/spherical_coordinates')
            if spherical is None or spherical.findtext('surface_model') != 'EARTH_WGS84':
                raise ValueError('GPS-map conversion requires the actual EARTH_WGS84 datum')
            self.metric_latitude_deg = float(spherical.findtext('latitude_deg'))
            self.metric_scale_enu = metric_scale(self.metric_latitude_deg)
        x = state.copy()
        x[:3] = convert_position(x[:3], self.metric_anchor, self.metric_scale_enu)
        return x

    def read(self):
        t, x, age = super().read()
        return t, self.convert(x), age

    def drain_measurements(self):
        return [(t, self.convert(x)) for t, x in super().drain_measurements()]
