"""Continuous-time numerical separation of a body sphere and moving STL frames.

Bernstein boxes enclose each polynomial and analytic motion ranges enclose the
moving frame. Distance to a fixed mesh is 1-Lipschitz, giving a lower bound.
A negative pointwise clearance is collision evidence; unresolved boxes NEVER
count as safe. Ordinary floating point, not an interval-arithmetic certificate.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy.spatial import cKDTree
import trimesh
from convex_timevarying_window.dynamic_passage_minco.safety import power_to_bernstein, split
from convex_timevarying_window.single_gate_stop_racing.planner import route_signs

BODY_RADIUS = .3859447197013662


@dataclass(frozen=True)
class CheckConfig:
    radius: float = BODY_RADIUS
    numerical_guard_m: float = 1e-8
    minimum_interval_s: float = 1e-7
    max_boxes_per_piece: int = 32768
    seconds_per_trajectory: float = 120.


class MeshDistance:
    def __init__(self, mesh):
        self.triangles = np.asarray(mesh.triangles, float)
        if not len(self.triangles) or not np.isfinite(self.triangles).all():
            raise ValueError('missing or nonfinite frame triangles')
        self.centers = self.triangles.mean(axis=1)
        self.max_radius = np.linalg.norm(self.triangles-self.centers[:, None], axis=2).max()
        self.tree = cKDTree(self.centers)
        self.bounds = np.array([self.triangles.min(axis=(0, 1)), self.triangles.max(axis=(0, 1))])

    def distance(self, point):
        """Point/triangle minimum up to floating point; no rtree dependency."""
        point = np.asarray(point, float)
        _, first = self.tree.query(point)
        nearest = trimesh.triangles.closest_point(self.triangles[[first]], point[None])[0]
        upper = np.linalg.norm(point-nearest)
        # Any improving triangle has centroid distance <= upper + its radius.
        ids = self.tree.query_ball_point(point, upper+self.max_radius+1e-10)
        nearest = trimesh.triangles.closest_point(self.triangles[ids], np.broadcast_to(point, (len(ids), 3)))
        return float(np.linalg.norm(nearest-point, axis=1).min())


def _piece(C, T, start, window, mesh, cfg, deadline):
    local = C@window.R
    local = local.copy(); local[0] -= window.center0@window.R
    stack = [(0., 1., power_to_bernstein(local))]
    minimum = np.inf; witness = None; boxes = 0; lower_bound = np.inf
    while stack:
        if boxes >= cfg.max_boxes_per_piece or time.perf_counter() > deadline:
            return dict(outcome='unresolved', reason='check_budget', boxes=boxes,
                        minimum_sample_clearance_m=None if witness is None else minimum, witness=witness)
        a, b, control = stack.pop(); boxes += 1
        ml, mh = window.motion.local_bounds(start+a*T, start+b*T)
        lo = control.min(axis=0)-mh-cfg.numerical_guard_m
        hi = control.max(axis=0)-ml+cfg.numerical_guard_m
        gap = np.maximum(np.maximum(mesh.bounds[0]-hi, lo-mesh.bounds[1]), 0.)
        bound = float(np.linalg.norm(gap))
        if bound > cfg.radius+cfg.numerical_guard_m:
            lower_bound = min(lower_bound, bound-cfg.radius); continue
        center = (lo+hi)/2; half = (hi-lo)/2
        bound = max(bound, mesh.distance(center)-np.linalg.norm(half))
        if bound > cfg.radius+cfg.numerical_guard_m:
            lower_bound = min(lower_bound, bound-cfg.radius); continue
        for u in (a, (a+b)/2, b):
            at = start+u*T
            q = np.polynomial.polynomial.polyval(u, local)-window.motion.local(at)[0]
            clearance = mesh.distance(q)-cfg.radius
            if clearance < minimum:
                minimum = clearance
                witness = dict(time_s=float(at), u=float(u), local_position=q.tolist(), clearance_m=clearance)
            if clearance < -cfg.numerical_guard_m:
                return dict(outcome='collision', boxes=boxes, minimum_sample_clearance_m=minimum, witness=witness)
        if (b-a)*T <= cfg.minimum_interval_s:
            return dict(outcome='unresolved', reason='near_tangency', boxes=boxes,
                        minimum_sample_clearance_m=minimum, witness=witness)
        left, right = split(control, .5); mid = (a+b)/2
        stack.extend([(mid, b, right), (a, mid, left)])
    return dict(outcome='clear', boxes=boxes, clearance_lower_bound_m=float(lower_bound),
                minimum_sample_clearance_m=None if witness is None else minimum, witness=witness)


def opening_events(C, T, track):
    starts = np.r_[0., np.cumsum(T)[:-1]]; result = []
    for w, sign in zip(track.windows, route_signs(track)):
        events = []; degenerate = False
        for piece, (coeff, duration, start) in enumerate(zip(C, T, starts)):
            local = coeff@w.R; local = local.copy(); local[0] -= w.center0@w.R
            normal = local[:, 2]
            if np.max(abs(normal)) < 1e-10:
                degenerate = True; continue
            for z in np.polynomial.polynomial.polyroots(normal):
                if abs(z.imag) > 1e-7 or not -1e-8 <= z.real <= 1+1e-8: continue
                u = float(np.clip(z.real, 0., 1.)); at = float(start+u*duration)
                q = np.polynomial.polynomial.polyval(u, local)-w.motion.local(at)[0]
                if not w.aperture.contains(q[:2]): continue
                rate = np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(normal))/duration
                if any(abs(e['time_s']-at) < 1e-6 for e in events): continue
                events.append(dict(time_s=at, forward=bool(sign*rate < -1e-8), piece=piece, local_point=q[:2].tolist()))
        result.append(dict(gate=w.name, events=sorted(events, key=lambda e:e['time_s']), degenerate_plane_piece=degenerate))
    exactly_once = all(len(r['events']) == 1 and r['events'][0]['forward'] and not r['degenerate_plane_piece'] for r in result)
    ordered = exactly_once and bool(np.all(np.diff([r['events'][0]['time_s'] for r in result]) > 0))
    return result, ordered


def evaluate(C, T, track, mesh_directory, cfg=CheckConfig()):
    began = time.perf_counter(); deadline = began+cfg.seconds_per_trajectory
    C, T = np.asarray(C, float), np.asarray(T, float)
    if C.shape != (len(T), 8, 3) or np.any(T <= 0) or not np.isfinite(C).all() or not np.isfinite(T).all():
        raise ValueError('invalid normalized polynomial trajectory')
    starts = np.r_[0., np.cumsum(T)[:-1]]; checks = []
    for w in track.windows:
        mesh = MeshDistance(trimesh.load_mesh(mesh_directory/f'gate_{w.name}_sleeve.stl'))
        for piece, (coeff, duration, start) in enumerate(zip(C, T, starts)):
            checks.append(dict(gate=w.name, piece=piece, **_piece(coeff, duration, start, w, mesh, cfg, deadline)))
    ground = []
    for k, coeff in enumerate(C):
        poly = coeff[:, 2]; roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(poly))
        candidates = [0., 1.]+[float(z.real) for z in roots if abs(z.imag) < 1e-7 and 0 < z.real < 1]
        values = np.polynomial.polynomial.polyval(candidates, poly)-cfg.radius
        i = int(np.argmin(values)); value = float(values[i])
        outcome = 'collision' if value < -cfg.numerical_guard_m else 'clear' if value > cfg.numerical_guard_m else 'unresolved'
        ground.append(dict(piece=k, outcome=outcome, minimum_clearance_m=value, time_s=float(starts[k]+candidates[i]*T[k])))
    events, ordered = opening_events(C, T, track)
    def derivative(coeff, u, d, duration):
        return np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(coeff, d))/duration**d
    errors = dict(start_position_m=float(np.linalg.norm(C[0, 0]-track.start)),
                  goal_position_m=float(np.linalg.norm(derivative(C[-1], 1., 0, T[-1])-track.goal)),
                  start_vaj=[float(np.linalg.norm(derivative(C[0], 0., d, T[0]))) for d in (1,2,3)],
                  goal_vaj=[float(np.linalg.norm(derivative(C[-1], 1., d, T[-1]))) for d in (1,2,3)])
    endpoints = max(errors['start_position_m'], errors['goal_position_m'], *errors['start_vaj'], *errors['goal_vaj']) < 1e-5
    collision = any(r['outcome'] == 'collision' for r in checks+ground)
    unresolved = any(r['outcome'] == 'unresolved' for r in checks+ground)
    verdict = ('nominal_collision' if collision else 'nominal_task_invalid' if not (ordered and endpoints)
               else 'nominal_unresolved' if unresolved else 'nominal_safe_complete')
    u, weight = np.polynomial.legendre.leggauss(128); u=(u+1)/2; weight=weight/2
    length = sum(float(weight@np.linalg.norm(np.polynomial.polynomial.polyval(u, np.polynomial.polynomial.polyder(c)).T, axis=1)) for c in C)
    return dict(verdict=verdict, passed=verdict=='nominal_safe_complete', collision_detected=collision,
                full_trajectory_time_s=float(T.sum()), trajectory_distance_m=length, check_s=time.perf_counter()-began,
                checks=checks, ground=ground, opening_events=events, ordered_exactly_once=ordered,
                endpoint_errors=errors, endpoints_passed=endpoints, body_radius_m=cfg.radius,
                evidence='floating-point adaptive Bernstein/analytic-motion boxes + exact STL triangle distance; nominal enclosing-sphere model, not Gazebo contact or tracking evidence',
                formal_interval_certificate=False, gazebo_run=False)
