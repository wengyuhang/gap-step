"""Identical soft objective; return the final iterate without admission scans."""
import time
import numpy as np
from convex_timevarying_window.simple_two_gate_mpc.planner import Planner as Parent, Config
from convex_timevarying_window.event_free_terminal_mpc.core import Plan


class Planner(Parent):
    def _build(self, first):
        from .solver import Solver
        old, *rest = super()._build(first)
        return (Solver(old.fg, self.cfg), *rest)

    def solve(self, state, now, first):
        began = time.perf_counter()
        state = np.asarray(state)
        if len(state) == 9: state = np.r_[state, np.zeros(3)]
        solver, evaluate, lbx, ubx, lbg, ubg = self.solvers[first]
        seed = self.initial(state, now, first)
        gates = len(self.ids(first)); legs = gates+1; n = 2*legs
        old = self.search.get(first)
        nodes = self.refine(old) if old is not None else np.tile([.15, .35, .65, .85], n)
        par = np.r_[now, state, nodes]
        result = solver(x0=np.clip(seed, lbx, ubx), p=par, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        x = np.asarray(result['x']).ravel()
        g, C, _ = evaluate(x, par)
        # Existing objective-node residual is recorded, never used for admission.
        violation = float(np.maximum(np.maximum(lbg-np.asarray(g).ravel(), np.asarray(g).ravel()-ubg), 0.).max())
        plan = Plan(now, first, np.repeat(np.exp(x[:legs])/2, 2), np.asarray(C).reshape(n, 6, 3),
                    x, 0., violation, solver.stats()['return_status'], gates)
        self.search[first] = plan
        plan.event_valid = None
        plan.event_reason = 'not_checked_soft_only'
        plan.optimizer_stats = solver.stats().copy()
        plan.terminal_gate = first+gates if self.free_terminal(first) else None
        plan.solve_ms = 1000*(time.perf_counter()-began)
        return plan

    def prefix_clearance(self, plan, at):
        return None
