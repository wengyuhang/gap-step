"""Current PV-MINCO/hard passage NLP, with SC instead of a convex point map.

The inner CasADi graph differentiates with respect to the physical 2-D knot.
The SC analytic Jacobian is then applied by the chain rule. No finite
differences, convex hull, discrete room switch, or new trajectory knots.
"""
import time
import casadi as ca
import numpy as np
from scipy.optimize import least_squares

from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import B, SCDiskMap
from convex_timevarying_window.single_gate_stop_racing.planner import Single
from convex_timevarying_window.single_gate_stop_racing.geometry import symbolic_constraints, passage
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import minco_coefficients
from convex_timevarying_window.togt_minco_mpc.trajectory import basis
from convex_timevarying_window.comparisons.original_fixed_plane_ours.adapter import symbolic_pose
from .scenario import LEFT_INITIAL_POINT


def inverse_along_corridor(mapping, target):
    """Offline inverse continuation to the requested left-room target.

    The existing two-start inverse fails in the small room for this polygon.
    Warm-starting the inverse along its interior centreline avoids that
    numerical problem. This does not initialize trajectory optimization in
    the right room: only the final left-room inverse is passed to the planner.
    """
    d = np.zeros(2)
    for point in np.linspace(mapping.q0, target, 41)[1:]:
        result = least_squares(lambda z: mapping.map_unconstrained(z) - point, d,
                               jac=mapping.jacobian_unconstrained,
                               max_nfev=100, ftol=1e-12, xtol=1e-12, gtol=1e-12)
        if np.linalg.norm(result.fun) > 1e-7:
            raise RuntimeError('Offline SC inverse continuation failed')
        d = result.x
    return d


def recenter_at_left(mapping):
    """Equivalent SC disk automorphism; change coordinates, not physical set."""
    d = inverse_along_corridor(mapping, LEFT_INITIAL_POINT)
    z = B(d)
    raw = mapping._automorphism(complex(*z))
    centered = SCDiskMap(vertices=mapping.vertices, alpha=mapping.alpha,
                         prevertices=mapping.prevertices, A=mapping.A, C=mapping.C,
                         normalization=raw, q0=LEFT_INITIAL_POINT.copy(),
                         quadrature_order=mapping.quadrature_order,
                         diagnostics=mapping.diagnostics)
    np.testing.assert_allclose(centered.map_unconstrained(np.zeros(2)), LEFT_INITIAL_POINT, atol=1e-7)
    return centered


class Planner(Single):
    # Scene-specific hooks; defaults reproduce the original fixed-plane case.
    symbolic_pose = staticmethod(symbolic_pose)

    def check_passage(self, plan, window, radius, piece, deadline):
        return passage(plan, window, radius, piece, deadline)

    def __init__(self, track, cfg, mapping):
        self.mapping = mapping
        self.initial_d = (np.zeros(2) if np.linalg.norm(mapping.q0 - LEFT_INITIAL_POINT) < 1e-7
                          else inverse_along_corridor(mapping, LEFT_INITIAL_POINT))
        self.evaluations = []
        self.map_ms = 0.
        self._map_cached_d = None
        self._map_cached_value = None
        sign = float(np.sign((track.start - track.windows[0].center0)
                             @ track.windows[0].state_at(0.)[1][:, 2]))
        super().__init__(track, cfg, sign)

    def _build(self):
        cfg, w = self.cfg, self.windows[0]
        self.point_dim, self.nvar = 2, 7
        x = ca.MX.sym('physical_knot_x', self.nvar)
        par = ca.MX.sym('head', 13 + 2 * cfg.witness_slots)
        times = ca.exp(x[:2])
        cross = par[0] + times[0]
        center, R = self.symbolic_pose(w, cross)
        point = center + R[:, :2] @ x[2:4]
        C = minco_coefficients(ca.reshape(par[1:13], 3, 4),
                              ca.horzcat(ca.DM(self.track.goal), ca.DM.zeros(3, 3)),
                              point, x[-3:], times)

        def state(piece, u, d=0):
            return C[piece * 8:piece * 8 + 8, :].T @ basis(u, d) / times[piece] ** d

        constraints = self.thrust_constraints(state, times)
        radius = cfg.radius + cfg.frame_thickness + cfg.passage_margin + cfg.witness_slack
        for piece in range(2):
            for j in range(cfg.witness_slots):
                offset = ca.fmin(par[13 + piece * cfg.witness_slots + j], times[piece])
                u = 1 - offset / times[0] if piece == 0 else offset / times[1]
                t = cross - offset if piece == 0 else cross + offset
                cc, rr = self.symbolic_pose(w, t)
                q = rr.T @ (state(piece, u) - cc)
                constraints.extend(symbolic_constraints(w.aperture, q, radius))
        constraints.append(-self.signs[0] * ca.dot(R[:, 2], state(0, 1., 1)) - .1)
        g = ca.vertcat(*constraints)
        cost = ca.sum1(times) / 1000.
        options = {'cse': True}
        if cfg.compile_objective:
            options.update(jit=True, compiler='shell', jit_options={
                'flags': [cfg.compiler_optimization], 'directory': self._jit_dir.name + '/'})
        self.inner = ca.Function('sc_physical_eval', [x, par],
                                 [cost, ca.gradient(cost, x), g, ca.jacobian(g, x), C,
                                  ca.MX(0.)], options)
        self.lower = np.r_[np.log([.3, .3]), np.full(5, -np.inf)]
        self.upper = np.r_[np.log([15., 15.]), np.full(5, np.inf)]

    def function(self, x, par):
        x = np.asarray(x, float).ravel()
        if self._map_cached_d is None or not np.array_equal(x[2:4], self._map_cached_d):
            began = time.perf_counter()
            self._map_cached_value = self.mapping.map_unconstrained_with_jacobian(x[2:4])
            self._map_cached_d = x[2:4].copy()
            self.map_ms += 1000 * (time.perf_counter() - began)
        q, jac = self._map_cached_value
        physical = x.copy()
        physical[2:4] = q
        raw = list(self.inner(physical, par))
        chain = np.eye(self.nvar)
        chain[2:4, 2:4] = jac
        raw[1] = chain.T @ np.asarray(raw[1]).ravel()
        raw[3] = np.asarray(raw[3]) @ chain
        self.evaluations.append(dict(x=x.tolist(), q=q.tolist(),
                                     minimum_residual=float(np.min(raw[2]))))
        return tuple(raw)

    def solve(self, state, now=0., first=0):
        from .solver import solve
        return solve(self, state, now, first)
