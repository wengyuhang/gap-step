"""Shared global MINCO initialization, not a safe executable trajectory.

Paper IV-E says to initialize local segments from a global one-segment
MINCO reference, but omits its generation parameters. This explicit
implementation uses equal-time nodes at predicted centers and zero PVA
derivatives at the task start/end. The duration heuristic is an adaptation.
"""
import numpy as np
from 复现.polynomial_dynamic_racing_2023.minco import coefficients
from .trajectory import Plan


class GlobalSeed:
    def __init__(self, track, windows, cfg):
        anchors = np.vstack([track.start, *[w.center0 for w in windows], track.goal])
        leg_lengths = np.linalg.norm(np.diff(anchors, axis=0), axis=1)
        self.step = max(1., float(leg_lengths.sum()/6./len(leg_lengths)))
        self.step = max(self.step, float(np.sqrt(3*leg_lengths.max()/cfg.acceleration)))
        times = self.step*np.arange(1, len(windows)+1)
        points = [w.state_at(float(t))[0] for w, t in zip(windows, times)]
        head, tail = np.zeros((3, 3)), np.zeros((3, 3))
        head[0], tail[0] = track.start, track.goal
        C = coefficients(head, tail, points, self.step*(len(windows)+1))
        self.reference = Plan(0., 0, np.full(len(C), self.step), C,
                              np.array([]), 0., 0., 'global_seed_only', len(windows),
                              terminal_goal=True, pieces_per_leg=1)

    def leg(self, gate, source, target, duration, pieces, half=False):
        span = self.step*(.5 if half else 1.)
        begin, end = gate*self.step, gate*self.step+span
        fractions = np.arange(1, pieces)/pieces
        p0, p1 = self.reference.sample(begin), self.reference.sample(end)
        middle = self.reference.sample_many(begin+fractions*span)
        # Smooth endpoint correction has zero first/second derivatives at both
        # ends, preserving the reference PVA derivatives after time scaling.
        blend = 10*fractions**3-15*fractions**4+6*fractions**5
        middle += (1-blend[:, None])*(source-p0)+blend[:, None]*(target-p1)
        scale = span/duration
        return middle, self.reference.sample(end, 1)*scale, self.reference.sample(end, 2)*scale**2
