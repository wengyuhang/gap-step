"""Exact frame geometry and conservative interval motion bounds.

The model/error bound is an assumption, NOT a measured tracking certificate.
In particular, passing these tests does not certify the PX4 execution.
"""
from dataclasses import replace
import numpy as np
from convex_timevarying_window.geometry import ConvexAperture
from .config import Config


def frame_aperture(aperture, offset):
    if aperture.kind == "circle":
        return replace(aperture, radius=aperture.radius + offset)
    vertices = aperture.physical_vertices
    edge = np.roll(vertices, -1, axis=0) - vertices
    signed_area = np.sum(vertices[:, 0]*np.roll(vertices[:, 1], -1)
                         - vertices[:, 1]*np.roll(vertices[:, 0], -1))
    normals = np.column_stack((edge[:, 1], -edge[:, 0]))
    normals *= np.sign(signed_area)/np.linalg.norm(normals, axis=1)[:, None]
    distances = np.sum(normals*vertices, axis=1) + offset
    expanded = np.array([np.linalg.solve(normals[[i-1, i]], distances[[i-1, i]])
                         for i in range(len(vertices))])
    return ConvexAperture("polygon", physical_vertices=expanded, safe_vertices=expanded)


class Geometry:
    def __init__(self, track, config=Config()):
        self.track, self.config = track, config
        self.frames = tuple(frame_aperture(w.aperture, config.frame_offset)
                            for w in track.windows)
        self.frame_speeds = []
        for w, frame in zip(track.windows, self.frames):
            m = w.motion
            translation = (np.linalg.norm(m.translation_amplitude)*2*np.pi/m.translation_period
                           if m.translation_enabled else 0.)
            # Rz Ry Rx: ||omega|| <= |roll_dot|+|pitch_dot|+|yaw_dot|.
            angular = (np.sum(np.abs(m.rotation_amplitude))*2*np.pi/m.rotation_period
                       if m.rotation_enabled else 0.)
            radius = (frame.radius if frame.kind == "circle" else
                      np.max(np.linalg.norm(frame.physical_vertices, axis=1)))
            self.frame_speeds.append(translation + angular*radius)
        self.frame_speeds = np.asarray(self.frame_speeds)

    def poses(self, times):
        """Shared queries, independent of the number of candidate trajectories."""
        times = np.asarray(times)[:, None]
        centers, rotations = [], []
        for w in self.track.windows:
            m = w.motion
            tr = m.translation_amplitude if m.translation_enabled else np.zeros(3)
            ar = m.rotation_amplitude if m.rotation_enabled else np.zeros(3)
            centers.append(w.center0 + tr*np.sin(2*np.pi*times/m.translation_period
                                                + m.phase + np.array([0., .7, 1.4])))
            angles = w.angles0_rpy + ar*np.sin(2*np.pi*times/m.rotation_period
                                              + m.phase + np.array([0., .9, 1.8]))
            roll, pitch, yaw = angles.T
            cr, sr = np.cos(roll), np.sin(roll)
            cp, sp = np.cos(pitch), np.sin(pitch)
            cy, sy = np.cos(yaw), np.sin(yaw)
            r = np.stack((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                          sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr,
                          -sp, cp*sr, cp*cr), axis=-1).reshape(-1, 3, 3)
            rotations.append(r)
        return np.stack(centers, axis=1), np.stack(rotations, axis=1)

    def interval_clearances(self, positions, velocities, times, poses=None,
                            acceleration_bound=None):
        """Return [candidate, interval] clearance AFTER interval/error margins.

        acceleration_bound must bound the entire integration interval. Default
        is the physical rotor-limited force bound, not sampled acceleration.
        """
        p, v, times = map(np.asarray, (positions, velocities, times))
        if p.ndim == 2:
            p, v = p[None], v[None]
        if p.shape != v.shape or p.shape[1:] != (len(times), 3):
            raise ValueError("positions/velocities must be [batch, time, 3]")
        dt = np.diff(times)
        if np.any(dt <= 0):
            raise ValueError("geometry times must be strictly increasing")
        c, r = self.poses(times) if poses is None else poses
        cfg = self.config
        if acceleration_bound is None:
            mass = 2.0643076923076924
            acceleration_bound = (4*cfg.max_rotor_thrust/mass + 9.8066
                                  + 4*8.06428e-5*1000/mass*cfg.speed)
        speed_bound = np.linalg.norm(v[:, :-1], axis=-1) + acceleration_bound*dt
        distances = []
        for i, frame in enumerate(self.frames):
            local = np.einsum("tij,btj->bti", r[:, i].transpose(0, 2, 1), p-c[:, i])
            d = np.hypot(frame.boundary_distance(local[..., :2]), local[..., 2])
            clearance = (np.minimum(d[:, :-1], d[:, 1:]) - cfg.frame_thickness
                         - cfg.radius - cfg.error_margin
                         - .5*(speed_bound+self.frame_speeds[i])*dt)
            distances.append(clearance)
        # Workspace bounds follow the old protocol's centre-position bounds.
        # Tracking uncertainty and intersample motion still consume clearance.
        room = np.minimum(p-np.asarray(cfg.lower), np.asarray(cfg.upper)-p).min(axis=-1)
        remaining = (np.minimum(room[:, :-1], room[:, 1:]) - cfg.error_margin
                     - .5*speed_bound*dt)
        if cfg.axis_workspace_bounds:
            # Each plane distance is 1-Lipschitz in its NORMAL coordinate.
            # Tangential flight cannot close that plane's clearance. The
            # same full physical acceleration bound still applies per axis.
            room_axes=np.minimum(p-np.asarray(cfg.lower),np.asarray(cfg.upper)-p)
            normal_speed=np.abs(v[:,:-1])+acceleration_bound*dt[None,:,None]
            remaining=(np.minimum(room_axes[:,:-1],room_axes[:,1:])-cfg.error_margin
                       -.5*normal_speed*dt[None,:,None]).min(axis=-1)
        for d in distances:
            remaining = np.minimum(remaining, d)
        return remaining


class GateProgress:
    """Actual or candidate-local ordered FSM; predictions never mutate actual state."""
    def __init__(self, track, config=Config(), first=0, crossing=False):
        self.track, self.config = track, config
        # 0: before gate; 1: body overlaps slab, centre still before plane;
        # 2: body overlaps slab, centre has crossed. Truthiness stays compatible
        # with the scheduler, while native prediction receives the full phase.
        self.first, self.crossing = first, int(crossing)
        self.violation = None
        self.signs = []
        previous = track.start
        for index in track.order:
            w = track.windows[index]
            c, r, *_ = w.state_at(0.)
            signed = float(r[:, 2] @ (previous-c))
            if abs(signed) < 1e-8:
                raise ValueError("gate direction is ambiguous")
            self.signs.append(-np.sign(signed))
            previous = c

    def update(self, p0, t0, p1, t1, collision_free):
        if self.first >= len(self.track.order) or not collision_free:
            return False
        w = self.track.windows[self.track.order[self.first]]
        q0, z0 = w.world_to_local(p0, t0)
        q1, z1 = w.world_to_local(p1, t1)
        sign = self.signs[self.first]
        margin = self.config.radius + self.config.error_margin
        slab = margin+self.config.frame_thickness
        a, b = sign*z0, sign*z1
        extent = (w.aperture.radius if w.aperture.kind=='circle' else
                  np.linalg.norm(w.aperture.physical_vertices,axis=1).max())
        reach=extent+self.config.frame_offset+slab
        near=min(np.hypot(np.linalg.norm(q0),z0),np.hypot(np.linalg.norm(q1),z1))<=reach
        if not self.crossing and a > -slab+1e-8:
            self.violation = "unregistered_gate_entry"
            return False
        if not self.crossing and a <= -slab < b:
            self.crossing = 1
        if self.crossing:
            # Evaluate clipped slab endpoints in the moving frame, not a
            # frozen frame. This is an observed-event check, not a certificate.
            if min(a,b) <= slab and max(a,b) >= -slab:
                fractions = [0., 1.]
                if abs(b-a) > 1e-12:
                    bounds = sorted(((-slab-a)/(b-a), (slab-a)/(b-a)))
                    fractions = [max(0.,bounds[0]), min(1.,bounds[1])]
                for f in fractions:
                    q, _ = w.world_to_local((1-f)*p0+f*p1, (1-f)*t0+f*t1)
                    if not w.aperture.contains(q) or w.aperture.boundary_distance(q) <= margin:
                        self.violation = "illegal_gate_slab_crossing"
                        return False
            if a < 0 <= b: self.crossing = 2
            if b < 0 and self.crossing == 2: self.crossing = 1
            if b <= -slab:
                self.crossing = 0
                return False
        if self.crossing == 2 and b > slab:
            self.first += 1
            self.crossing = 0
            return True
        return False

    def computation_allowed(self,position,time):
        if self.crossing:return False
        if self.first>=len(self.track.order):return True
        _,z=self.track.windows[self.track.order[self.first]].world_to_local(position,time)
        return self.signs[self.first]*z <= -(self.config.radius+self.config.error_margin+self.config.frame_thickness)
