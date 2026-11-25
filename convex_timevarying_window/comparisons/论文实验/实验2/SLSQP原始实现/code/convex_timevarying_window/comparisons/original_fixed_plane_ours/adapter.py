"""Scene-only replacements for legacy hard-coded XYZ motion phases.

No cost, constraints, solver, initialization or acceptance-policy changes.
Install explicitly inside the isolated planner process, not in shared files.
"""
import multiprocessing as mp
from types import SimpleNamespace
import casadi as ca
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import centers, PlanarMotion
from convex_timevarying_window.wang_free_state_mpc.worker import Worker as BaseWorker


def symbolic_pose(window, t):
    if not isinstance(window.motion, PlanarMotion):
        raise TypeError('This adapter accepts only the frozen original fixed-plane scene')
    m = window.motion
    c = ca.DM(window.center0)+ca.DM(m.translation_amplitude)*ca.sin(2*np.pi*t/m.translation_period+m.phase)
    return c, ca.DM(window.state_at(0.)[1])


def local_coordinates(window, positions, times):
    return (np.asarray(positions)-centers(window, times))@window.state_at(0.)[1]


def symbolic_clearance_sq(window, position, t):
    center, rotation = symbolic_pose(window, t)
    q = rotation.T@(position-center)
    if window.aperture.kind == 'circle':
        d = ca.sqrt(ca.sumsqr(q[:2])+1e-12)-window.aperture.radius
        return d*d+q[2]*q[2]
    vertices = window.aperture.physical_vertices; distance = None
    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
        e = b-a; u = ca.fmin(1., ca.fmax(0., ca.dot(q[:2]-a, e)/float(e@e)))
        value = ca.sumsqr(q[:2]-a-u*e)
        distance = value if distance is None else ca.fmin(distance, value)
    return distance+q[2]*q[2]


def frame_clearances(frames, positions, times, radius, thickness):
    result = np.full(len(times), np.inf)
    for window in frames:
        q = local_coordinates(window, positions, times)
        distance = window.aperture.boundary_distance(q[:,:2])
        result = np.minimum(result, np.hypot(distance, q[:,2])-radius-thickness)
    return result


def install_scene_queries():
    from convex_timevarying_window.wang_paper_aligned_mpc import planner, events
    planner.symbolic_pose = symbolic_pose
    planner.symbolic_clearance_sq = symbolic_clearance_sq
    planner.frame_clearances = frame_clearances
    events.local_coordinates = local_coordinates


def serve(track, cfg, requests, results, ready):
    install_scene_queries()
    from convex_timevarying_window.wang_free_state_mpc import worker as base
    from convex_timevarying_window.wang_paper_aligned_mpc.planner import Planner
    base.Planner = Planner
    base._serve(track, cfg, requests, results, ready)


class Worker(BaseWorker):
    def __init__(self, track, cfg):
        ctx = mp.get_context('spawn')
        self.requests, self.results, self.ready = (ctx.Queue(1) for _ in range(3))
        self.process = ctx.Process(target=serve, args=(track,cfg,self.requests,self.results,self.ready), daemon=True)
        self.admitted = None; self.busy = False; self.metrics = []; self.process.start()
        try:
            self.startup = self.ready.get(timeout=600.)
            if 'failure' in self.startup: raise RuntimeError(self.startup['failure'])
        except Exception:
            self.close(); raise
        self.geometry = SimpleNamespace(cfg=cfg, windows=[track.windows[i] for i in track.order], signs=self.startup['signs'])
