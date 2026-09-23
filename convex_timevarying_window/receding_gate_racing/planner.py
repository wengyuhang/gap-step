"""Small, warm-started space/time NLP over the next three moving apertures.

Each knot has a crossing time, two aperture coordinates, velocity and
acceleration. Quintic Hermite segments connect the measured PVA to the knots.
There are no saved trajectories, gate-number cases, waits or centre targets.
Collision constraints are sampled; the committed prefix is checked separately.
"""
from __future__ import annotations

from dataclasses import dataclass
import copy
import math
import time

import casadi as ca
import numpy as np

from convex_timevarying_window.conditional_dual_constraint_cem.safety_penalty import polygon_halfspaces


@dataclass(frozen=True)
class Config:
    horizon: int = 3
    speed: float = 12.0
    acceleration: float = 8.0
    jerk: float = 25.0
    radius: float = 0.385944720
    tracking_margin: float = 0.35
    frame_thickness: float = 0.085
    frame_offset: float = 0.065
    replan_period: float = 0.20
    solver_seconds: float = 0.065
    nodes: int = 12
    jerk_weight: float = 0.0006
    response_time: float = 0.20
    free_points: bool = True
    crossing_constraint: str = 'local_sweep'
    workspace_lower: tuple[float,float,float] | None = None
    workspace_upper: tuple[float,float,float] | None = None


def rotation(a):
    r, p, y = a[0], a[1], a[2]
    cr, sr, cp, sp, cy, sy = ca.cos(r), ca.sin(r), ca.cos(p), ca.sin(p), ca.cos(y), ca.sin(y)
    return ca.vertcat(ca.horzcat(cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                      ca.horzcat(sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                      ca.horzcat(-sp, cp*sr, cp*cr))


def symbolic_pose(w, t):
    m = w.motion
    tr = m.translation_amplitude if m.translation_enabled else np.zeros(3)
    ar = m.rotation_amplitude if m.rotation_enabled else np.zeros(3)
    c = w.center0 + ca.DM(tr)*ca.sin(2*math.pi*t/m.translation_period + m.phase + ca.DM([0, .7, 1.4]))
    a = w.angles0_rpy + ca.DM(ar)*ca.sin(2*math.pi*t/m.rotation_period + m.phase + ca.DM([0, .9, 1.8]))
    return c, rotation(a)


def coefficients(p0, v0, a0, p1, v1, a1, T, symbolic=False):
    b0, b1, b2 = p0, T*v0, .5*T*T*a0
    d, e, f = p1-b0-b1-b2, T*v1-b1-2*b2, T*T*a1-2*b2
    items = (b0, b1, b2, 10*d-4*e+.5*f, -15*d+7*e-f, 6*d-3*e+.5*f)
    return ca.horzcat(*items).T if symbolic else np.stack(items)


def poly(C, s, T, order=0, symbolic=False):
    values = [math.factorial(k)/math.factorial(k-order)*s**(k-order)/T**order
              if k >= order else 0.0 for k in range(6)]
    return C.T @ (ca.vertcat(*values) if symbolic else np.array(values))


def symbolic_clearance_sq(w, p, t):
    c, R = symbolic_pose(w, t)
    q = R.T @ (p-c)
    if w.aperture.kind == 'circle':
        d = ca.sqrt(ca.sumsqr(q[:2])+1e-12)-w.aperture.radius
        return d*d+q[2]*q[2]
    vertices = w.aperture.physical_vertices
    d2 = None
    for a, b in zip(vertices, np.roll(vertices, -1, axis=0)):
        e = b-a
        u = ca.fmin(1., ca.fmax(0., ca.dot(q[:2]-a, e)/float(e@e)))
        value = ca.sumsqr(q[:2]-a-u*e)
        d2 = value if d2 is None else ca.fmin(d2, value)
    return d2+q[2]*q[2]


@dataclass
class Plan:
    made_at: float
    first_gate: int
    durations: np.ndarray
    coefficients: np.ndarray
    x: np.ndarray
    solve_ms: float
    violation: float
    status: str

    @property
    def end_time(self):
        return self.made_at+float(self.durations.sum())

    @property
    def crossing_times(self):
        return self.made_at+np.cumsum(self.durations)

    def sample(self, t, order=0):
        elapsed = max(0., float(t)-self.made_at)
        ends = np.cumsum(self.durations)
        i = min(int(np.searchsorted(ends, elapsed, side='right')), len(ends)-1)
        local = elapsed-(ends[i-1] if i else 0.)
        return poly(self.coefficients[i], np.clip(local/self.durations[i], 0, 1), self.durations[i], order)


class Planner:
    def __init__(self, track, cfg=Config()):
        self.track, self.cfg = track, cfg
        self.windows = [track.windows[i] for i in track.order]
        # Exact sleeve centreline geometry. The capsule model encloses the
        # polygonal tube mesh, including contact away from the aperture plane.
        self.frames = []
        for w in self.windows:
            frame=copy.copy(w)
            aperture=copy.copy(w.aperture)
            if aperture.kind=='circle':
                object.__setattr__(aperture,'radius',aperture.radius+cfg.frame_offset)
            else:
                hs=polygon_halfspaces(aperture.physical_vertices)
                vertices=np.array([np.linalg.solve(np.stack([hs.normals[j-1],hs.normals[j]]),
                    np.array([hs.offsets[j-1],hs.offsets[j]])+cfg.frame_offset) for j in range(len(hs.offsets))])
                object.__setattr__(aperture,'physical_vertices',vertices)
            frame.aperture=aperture; self.frames.append(frame)
        self.signs = []
        for i, w in enumerate(self.windows):
            source = track.start if i == 0 else self.windows[i-1].center0
            _, R, *_ = w.state_at(0.)
            self.signs.append(float(np.sign(R[:, 2]@(source-w.center0)) or 1.))
        self.solvers = [self._build(i) for i in range(len(self.windows)+1)]
        self.previous = None
        self.search = None
        self.dual_cache = {}

    def horizon_ids(self, first):
        return list(range(first, min(first+self.cfg.horizon, len(self.windows)+1)))

    def terminal_position(self, first):
        return np.asarray(self.track.goal)

    def workspace_inset(self):
        return 0.

    def segment_frames(self, first, wi, k):
        return range(max(0, wi-1), min(len(self.windows), wi+1))

    def terminal_constraints(self, con, p):
        pass

    def seed_incumbent(self):
        return self.previous

    def initial_duration(self, distance, k, wi):
        return max(.6, distance/6.5+(.5 if k==0 else 0.))

    def tilt_expression(self, a, demanded, thrust):
        return ca.sumsqr(a[:2])-(math.tan(math.radians(42))*thrust[2])**2

    def collocation_parameters(self, first, ids):
        return np.empty(0)

    def _build(self, first):
        cfg = self.cfg
        # The return-to-start endpoint is just one final knot with PVA fixed.
        ids = self.horizon_ids(first)
        x = ca.SX.sym('x', 9*len(ids))
        extra_nodes=getattr(self,'extra_collocation',0)
        par = ca.SX.sym('p', 10+extra_nodes*len(ids))
        absolute, p0, v0, a0 = par[0], par[1:4], par[4:7], par[7:10]
        constraints, lower, upper, coeffs = [], [], [], []
        lbx, ubx = [], []
        def con(value, lo, hi):
            finite=[abs(float(b)) for b in (lo,hi) if np.isfinite(b)]
            scale=max([1.,*finite])
            constraints.append(ca.vec(value)/scale)
            lower.extend(np.broadcast_to(lo/scale, (value.numel(),)).tolist())
            upper.extend(np.broadcast_to(hi/scale, (value.numel(),)).tolist())
        objective = 0.
        for k, wi in enumerate(ids):
            T, q, v1, a1 = x[9*k], x[9*k+1:9*k+3], x[9*k+3:9*k+6], x[9*k+6:9*k+9]
            lbx.extend([.035, -2, -2, *([-cfg.speed]*3), *([-cfg.acceleration]*3)])
            ubx.extend([15., 2, 2, *([cfg.speed]*3), *([cfg.acceleration]*3)])
            if not cfg.free_points:
                lbx[9*k+1:9*k+3]=[0.,0.]; ubx[9*k+1:9*k+3]=[0.,0.]
            absolute_end = absolute+T
            if wi < len(self.windows):
                w = self.windows[wi]
                center, R = symbolic_pose(w, absolute_end)
                p1 = center+R[:, :2]@q
                inset = cfg.radius+cfg.tracking_margin+cfg.frame_thickness-cfg.frame_offset
                if w.aperture.kind == 'circle':
                    if w.aperture.radius <= inset:
                        raise ValueError('tracking tube leaves no circular opening')
                    con(ca.sumsqr(q), 0., (w.aperture.radius-inset)**2)
                else:
                    hs = polygon_halfspaces(w.aperture.physical_vertices)
                    for n, b in zip(hs.normals, hs.offsets): con(ca.dot(n,q), -np.inf, b-inset)
                # A whole vehicle must clear the moving plane before it can
                # reverse. Relative stopping distance vn^2/(2*Arel) >= rho
                # supplies the same geometric viability rule for every gate.
                tv=ca.SX.sym('gate_time')
                cc,rr=symbolic_pose(w,tv)
                rates=ca.Function(f'rates_{first}_{wi}',[tv],
                    [ca.jacobian(cc,tv),ca.reshape(ca.jacobian(ca.vec(rr),tv),3,3)])
                dc,dR=rates(absolute_end)
                vn=-self.signs[wi]*(ca.dot(R[:,2],v1-dc)+ca.dot(dR[:,2],p1-center))
                m=w.motion
                omega=float(np.sum(np.abs(m.rotation_amplitude)))*2*math.pi/m.rotation_period
                omega_dot=float(np.sum(np.abs(m.rotation_amplitude)))*(2*math.pi/m.rotation_period)**2+omega**2
                cv=float(np.linalg.norm(m.translation_amplitude))*2*math.pi/m.translation_period
                ca_bound=float(np.linalg.norm(m.translation_amplitude))*(2*math.pi/m.translation_period)**2
                extent=float(w.aperture.radius) if w.aperture.kind=='circle' else float(np.linalg.norm(w.aperture.physical_vertices,axis=1).max())
                rho=cfg.radius+cfg.frame_thickness+cfg.tracking_margin
                relative_acc=cfg.acceleration+ca_bound+2*omega*(cfg.speed+cv)+(omega_dot+omega**2)*(extent+rho)
                minimum_normal_speed=math.sqrt(2*relative_acc*rho)
                if cfg.crossing_constraint == 'worst_case':
                    con(vn, minimum_normal_speed, cfg.speed+cv)
                elif cfg.crossing_constraint == 'local_sweep':
                    # Only require the intended crossing direction here. The
                    # actual time-dependent approach/departure is constrained
                    # on the polynomial below, not by a global motion bound.
                    con(vn, .1, np.inf)
                else:
                    raise ValueError('unknown crossing constraint')
            else:
                p1 = ca.DM(self.terminal_position(first))
                for j in range(1, 9): lbx[9*k+j] = ubx[9*k+j] = 0.
                self.terminal_constraints(con, p1)
            C = coefficients(p0, v0, a0, p1, v1, a1, T, True)
            coeffs.append(C)
            for slot in range(extra_nodes):
                u=par[10+k*extra_nodes+slot]
                point=poly(C,u,T,0,True)
                vv,aa,jj=[poly(C,u,T,d,True) for d in (1,2,3)]
                uu=aa+cfg.response_time*jj;ff=uu+ca.DM([0,0,9.8066])
                for value,limit in ((vv,cfg.speed),(aa,cfg.acceleration),(jj,cfg.jerk),(uu,cfg.acceleration)):
                    con(ca.sumsqr(value),0.,limit**2)
                con(ca.sumsqr(ff),4.,(4*8.54858/2.0643076923*.90)**2)
                con(self.tilt_expression(aa,uu,ff),-np.inf,0.)
                if cfg.workspace_lower is not None:
                    for axis in range(3):con(point[axis],cfg.workspace_lower[axis]+self.workspace_inset(),cfg.workspace_upper[axis]-self.workspace_inset())
                for gj in self.segment_frames(first,wi,k):
                    con(symbolic_clearance_sq(self.frames[gj],point,absolute+T*u),
                        (cfg.radius+cfg.tracking_margin+cfg.frame_thickness+.015)**2,np.inf)
            for u in np.linspace(0., 1., cfg.nodes+1):
                p, v, a, jerk = [poly(C, float(u), T, d, True) for d in range(4)]
                con(ca.sumsqr(v), 0., cfg.speed**2)
                con(ca.sumsqr(a), 0., cfg.acceleration**2)
                con(ca.sumsqr(jerk), 0., cfg.jerk**2)
                con(p[2], .65, 12.)
                if cfg.workspace_lower is not None and cfg.workspace_upper is not None:
                    for axis in range(3):
                        con(p[axis],cfg.workspace_lower[axis]+self.workspace_inset(),cfg.workspace_upper[axis]-self.workspace_inset())
                # X500 tilt/thrust reserve, in acceleration units.
                # Inverse of the assumed first-order acceleration response:
                # tau * a_dot + a = u. Optimise the actual demanded input too.
                demanded = a+cfg.response_time*jerk
                con(ca.sumsqr(demanded), 0., cfg.acceleration**2)
                thrust = demanded+ca.DM([0, 0, 9.8066])
                con(ca.sumsqr(thrust), 4., (4*8.54858/2.0643076923*.90)**2)
                con(self.tilt_expression(a,demanded,thrust), -np.inf, 0.)
                objective += cfg.jerk_weight*T/(cfg.nodes+1)*ca.sumsqr(jerk)
                # Only the neighbouring frames constrain a segment in the NLP.
                # Every frame is checked on the short executable prefix below.
                for gj in self.segment_frames(first, wi, k):
                    con(symbolic_clearance_sq(self.frames[gj], p, absolute+T*u), (cfg.radius+cfg.tracking_margin+cfg.frame_thickness)**2, np.inf)
            # Evaluate the actual polynomial near each end (not a local Taylor
            # surrogate). These short crossing intervals need finer resolution.
            for offset in (.04,.08,.16,.28,.42):
                fraction=ca.fmin(.45,offset/T)
                for u in (fraction,1-fraction):
                    point=poly(C,u,T,0,True)
                    for gj in self.segment_frames(first, wi, k):
                        con(symbolic_clearance_sq(self.frames[gj],point,absolute+T*u),
                            (cfg.radius+cfg.tracking_margin+cfg.frame_thickness+.015)**2,np.inf)
                if cfg.crossing_constraint == 'local_sweep':
                    # A short, signed clearance schedule on both sides of a
                    # crossing. Query the real pose at each absolute time.
                    # It is a sampled local constraint, NOT a safety certificate.
                    distance=(cfg.radius+cfg.tracking_margin+cfg.frame_thickness)*ca.fmin(1.,fraction*T/.28)
                    if wi < len(self.windows):
                        at=absolute+T*(1-fraction)
                        cc,rr=symbolic_pose(self.windows[wi],at)
                        pp=poly(C,1-fraction,T,0,True)
                        con(self.signs[wi]*ca.dot(rr[:,2],pp-cc)-distance,0.,np.inf)
                    if k>0 and ids[k-1] < len(self.windows):
                        previous_gate=ids[k-1];at=absolute+T*fraction
                        cc,rr=symbolic_pose(self.windows[previous_gate],at)
                        pp=poly(C,fraction,T,0,True)
                        con(-self.signs[previous_gate]*ca.dot(rr[:,2],pp-cc)-distance,0.,np.inf)
            objective += T
            absolute, p0, v0, a0 = absolute_end, p1, v1, a1
        if ids[-1] < len(self.windows):
            next_p = self.windows[ids[-1]+1].center0 if ids[-1]+1 < len(self.windows) else self.track.goal
            direction = next_p-p0
            objective -= .055*ca.dot(v0, direction)/ca.sqrt(ca.sumsqr(direction)+1e-8)
        g = ca.vertcat(*constraints)
        opts = {'print_time':False, 'ipopt.print_level':0, 'ipopt.sb':'yes',
                'ipopt.max_iter':35, 'ipopt.max_cpu_time':cfg.solver_seconds,
                'ipopt.tol':2e-3, 'ipopt.acceptable_tol':1e-2,
                'ipopt.hessian_approximation':'exact'}
        if getattr(self,'warm_start_duals',False):
            opts.update({'ipopt.warm_start_init_point':'yes','ipopt.warm_start_bound_push':1e-6,
                         'ipopt.warm_start_mult_bound_push':1e-6,'ipopt.mu_init':1e-3})
        solver = ca.nlpsol(f'racing_{first}', 'ipopt', {'x':x,'p':par,'f':objective,'g':g}, opts)
        evaluate = ca.Function(f'eval_{first}', [x,par], [g, ca.vertcat(*coeffs)])
        return ids, solver, evaluate, np.array(lbx), np.array(ubx), np.array(lower), np.array(upper)

    def solve(self, state, now, first):
        started = time.perf_counter()
        ids, solver, evaluate, lbx, ubx, lbg, ubg = self.solvers[first]
        p = np.r_[now, np.asarray(state).ravel(), self.collocation_parameters(first,ids)]
        guess = []
        pos = np.asarray(state)[:3].copy()
        instant = float(now)
        good = self.seed_incumbent()
        old = good
        search=getattr(self,'search',None)
        if search is not None and search.first_gate==first and (good is None or good.first_gate!=first):
            old=search
        unexecuted=(getattr(self,'preserve_unexecuted_durations',False) and old is not None
                    and old is not getattr(self,'admitted',None))
        if getattr(self,'discard_stale_seed',False) and old is not None and not unexecuted:
            old_ids=self.horizon_ids(old.first_gate)
            if first in old_ids and old.crossing_times[old_ids.index(first)] < now+.10:
                old=None
        for k, wi in enumerate(ids):
            old_k = wi-old.first_gate if old is not None else -1
            if getattr(self,'discard_stale_seed',False) and old is not None:
                old_ids=self.horizon_ids(old.first_gate)
                old_k=old_ids.index(wi) if wi in old_ids else -1
                if wi==len(self.windows) and not np.allclose(self.terminal_position(first),self.terminal_position(old.first_gate)):
                    old_k=-1
            if old is not None and 0 <= old_k < len(old.durations):
                row = old.x.reshape(-1,9)[old_k].copy()
                row[0] = old.durations[old_k] if unexecuted else max(.04, old.crossing_times[old_k]-instant)
                instant += row[0]
            else:
                target = self.windows[wi].state_at(instant)[0] if wi < len(self.windows) else self.terminal_position(first)
                duration = self.initial_duration(np.linalg.norm(target-pos),k,wi)
                instant += duration
                if wi < len(self.windows):
                    target, R, *_ = self.windows[wi].state_at(instant)
                    after = self.windows[wi+1].center0 if wi+1<len(self.windows) else self.track.goal
                    direction = (target-pos)/max(np.linalg.norm(target-pos),1e-9)+(after-target)/max(np.linalg.norm(after-target),1e-9)
                    v = 6.*direction/max(np.linalg.norm(direction),1e-9)
                    if -self.signs[wi]*R[:,2]@v < 1.: v = -4.*self.signs[wi]*R[:,2]
                else: v = np.zeros(3)
                row = np.r_[duration, 0., 0., v, np.zeros(3)]
            guess.extend(row)
            pos = self.windows[wi].state_at(instant)[0] if wi < len(self.windows) else self.terminal_position(first)
        dual={}
        if getattr(self,'warm_start_duals',False) and old is not None:
            dual=self.dual_cache.get(first,{})
        result = solver(x0=np.clip(guess,lbx,ubx), p=p, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, **dual)
        if getattr(self,'warm_start_duals',False):
            self.dual_cache[first]={'lam_x0':result['lam_x'],'lam_g0':result['lam_g']}
        values = np.asarray(result['x']).ravel()
        g,C = evaluate(values,p)
        g = np.asarray(g).ravel()
        violation = float(max(np.max(lbg-g),np.max(g-ubg),0.))
        status = solver.stats()['return_status']
        plan = Plan(now,first,values[::9],np.asarray(C).reshape(len(ids),6,3),values,
                    1000*(time.perf_counter()-started),violation,status)
        # A failed near-crossing solve must not poison the next horizon's seed.
        # Keep the last feasible trajectory; use infeasible iterates only during
        # cold start, before any feasible seed exists.
        self.search=plan
        if violation < .003 or good is None or good.violation >= .003:
            self.previous = plan
        return plan

    def prefix_clearance(self, plan, now, duration=.30):
        """All-frame 10 ms sampled prefix audit; never an entire-lap scan."""
        minimum = math.inf
        for t in np.arange(now, min(now+duration,plan.end_time)+.00001, .01):
            pos = plan.sample(t)
            if self.cfg.workspace_lower is not None:
                if np.any(pos < np.asarray(self.cfg.workspace_lower)-.01) or np.any(pos > np.asarray(self.cfg.workspace_upper)+.01):
                    return -math.inf
            for w in self.frames:
                q,z = w.world_to_local(pos,t)
                minimum = min(minimum, math.hypot(float(w.aperture.boundary_distance(q)),z)-self.cfg.radius-self.cfg.frame_thickness)
        return minimum
