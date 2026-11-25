"""Stronger frame clearance with crossing points mapped into its inset opening."""
from dataclasses import replace

import numpy as np

from convex_timevarying_window.global_gate_minco_probe import planner as core
from convex_timevarying_window.single_gate_stop_racing import point_mapping
from ..safety import frame_aperture
from .planner import Planner as OriginalPlanner


def inset_aperture(aperture, distance):
    """Erode the existing frame-centerline geometry without changing the scene."""
    if not np.isfinite(distance) or distance < 0.:
        raise ValueError('Inset distance must be finite and nonnegative')
    inset = frame_aperture(aperture)
    if inset.kind == 'circle':
        inset.radius -= distance
        if inset.radius <= 0.:
            raise ValueError('Clearance leaves no circular crossing region')
    else:
        vertices = inset.physical_vertices
        edges = np.roll(vertices, -1, axis=0)-vertices
        normals = np.c_[edges[:, 1], -edges[:, 0]]/np.linalg.norm(edges, axis=1)[:, None]
        limits = np.sum(normals*vertices, axis=1)-distance
        pairs = np.stack((np.roll(normals, 1, axis=0), normals), axis=1)
        bounds = np.column_stack((np.roll(limits, 1), limits))
        inset.physical_vertices = np.linalg.solve(pairs, bounds[..., None])[..., 0]
        if np.any(inset.physical_vertices@normals.T > limits+1e-10):
            raise ValueError('Clearance leaves no valid convex crossing region')
    return inset


class Planner(OriginalPlanner):
    def __init__(self, track, free_points=True, *, extra_clearance_m=.05, **kwargs):
        if not np.isfinite(extra_clearance_m) or extra_clearance_m < 0.:
            raise ValueError('Additional clearance must be finite and nonnegative')
        config_factory = core.Config
        configured = replace(config_factory(),
            passage_margin=config_factory().passage_margin+extra_clearance_m)
        radius = (configured.radius+configured.frame_thickness+
            configured.passage_margin+configured.witness_slack)
        self.point_apertures = tuple(inset_aperture(window.aperture, radius)
            for window in track.windows)
        self._point_apertures_by_id = dict(zip(
            (id(window.aperture) for window in track.windows), self.point_apertures))
        symbolic = point_mapping.symbolic
        core.Config = lambda: configured
        point_mapping.symbolic = lambda aperture, value: symbolic(
            self._point_apertures_by_id[id(aperture)], value)
        try:
            super().__init__(track, free_points, **kwargs)
        finally:
            core.Config = config_factory
            point_mapping.symbolic = symbolic
        assert self.cfg.passage_margin == configured.passage_margin

    def seeds(self):
        inverse = point_mapping.inverse
        point_mapping.inverse = lambda aperture, point: inverse(
            self._point_apertures_by_id[id(aperture)], point)
        try:
            return super().seeds()
        finally:
            point_mapping.inverse = inverse
