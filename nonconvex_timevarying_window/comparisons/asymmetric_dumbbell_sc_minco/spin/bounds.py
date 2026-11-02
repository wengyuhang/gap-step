"""Original local Bernstein enclosure, extended by interval sin/cos products.

The plane is fixed. Only polynomial plane-overlap intervals are subdivided.
No dense-grid acceptance, no changed radius, and no Arb certificate claim.
"""
import math
import time

import numpy as np
from numba import njit

from convex_timevarying_window.dynamic_passage_minco.safety import (
    overlap_intervals, power_to_bernstein, restrict)
from convex_timevarying_window.single_gate_stop_racing.geometry import passage as translation_passage
from convex_timevarying_window.single_gate_stop_racing.local_bounds import _distance


@njit(cache=True)
def sine_bounds(a, b):
    a, b = min(a, b), max(a, b)
    if b-a >= 2*math.pi:
        return -1., 1.
    lo, hi = min(math.sin(a), math.sin(b)), max(math.sin(a), math.sin(b))
    for k in range(math.ceil((a-math.pi/2)/math.pi), math.floor((b-math.pi/2)/math.pi)+1):
        if k % 2:
            lo = -1.
        else:
            hi = 1.
    return lo, hi


@njit(cache=True)
def product_bounds(a, b, c, d):
    return min(a*c, a*d, b*c, b*d), max(a*c, a*d, b*c, b*d)


@njit(cache=True)
def rotate_box(lo, hi, a, b):
    sl, sh = sine_bounds(a, b)
    cl, ch = sine_bounds(a+math.pi/2, b+math.pi/2)
    cx = product_bounds(cl, ch, lo[0], hi[0])
    sy = product_bounds(sl, sh, lo[1], hi[1])
    sx = product_bounds(-sh, -sl, lo[0], hi[0])
    cy = product_bounds(cl, ch, lo[1], hi[1])
    lower = np.array([cx[0]+sy[0], sx[0]+cy[0], lo[2]])-1e-10
    upper = np.array([cx[1]+sy[1], sx[1]+cy[1], hi[2]])+1e-10
    return lower, upper


@njit(cache=True)
def value(u, coeff, start, T, amp, omega, phase, spin, spin_phase, vertices):
    q = np.zeros(3)
    for k in range(7, -1, -1):
        q = q*u+coeff[k]
    t = start+u*T
    q -= amp*math.sin(omega*t+phase)
    c, s = math.cos(spin*t+spin_phase), math.sin(spin*t+spin_phase)
    q = np.array([c*q[0]+s*q[1], -s*q[0]+c*q[1], q[2]])
    return _distance(q, vertices, False, 0.)


@njit(cache=True)
def check(controls, cuts, coeff, start, T, amp, omega, phase, spin, spin_phase, vertices, radius):
    stack_c = np.empty((1030, 8, 3))
    stack_a, stack_b = np.empty(1030), np.empty(1030)
    size, best, witness, boxes = len(cuts), np.inf, 0., 0
    for i in range(size):
        stack_c[i], stack_a[i], stack_b[i] = controls[i], cuts[i, 0], cuts[i, 1]
    while size:
        size -= 1
        a, b, control = stack_a[size], stack_b[size], stack_c[size].copy()
        sl, sh = sine_bounds(omega*(start+a*T)+phase, omega*(start+b*T)+phase)
        lo, hi = np.empty(3), np.empty(3)
        for axis in range(3):
            low, high = control[0, axis], control[0, axis]
            for k in range(1, 8):
                low, high = min(low, control[k, axis]), max(high, control[k, axis])
            lo[axis] = low-max(amp[axis]*sl, amp[axis]*sh)-1e-10
            hi[axis] = high-min(amp[axis]*sl, amp[axis]*sh)+1e-10
        lo, hi = rotate_box(lo, hi, spin*(start+a*T)+spin_phase, spin*(start+b*T)+spin_phase)
        normal = max(lo[2], -hi[2], 0.)
        planar = np.inf
        for i in range(len(vertices)):
            v = vertices[i]
            e = vertices[(i+1) % len(vertices)]-v
            length = math.sqrt(e@e)
            tx, ty = e[0]/length, e[1]/length
            nx, ny = ty, -tx
            cx, cy = (lo[0]+hi[0])/2-v[0], (lo[1]+hi[1])/2-v[1]
            hx, hy = (hi[0]-lo[0])/2, (hi[1]-lo[1])/2
            ac, ar = tx*cx+ty*cy, abs(tx)*hx+abs(ty)*hy
            nc, nr = nx*cx+ny*cy, abs(nx)*hx+abs(ny)*hy
            ag, ng = max(-ac-ar, ac-ar-length, 0.), max(abs(nc)-nr, 0.)
            planar = min(planar, ag*ag+ng*ng)
        for u in (a, (a+b)/2, b):
            val = value(u, coeff, start, T, amp, omega, phase, spin, spin_phase, vertices)
            if val < best:
                best, witness = val, u
        if best < (radius-2e-4)**2:
            left_u, right_u, ratio = a, b, (math.sqrt(5.)-1.)/2
            u1, u2 = right_u-ratio*(right_u-left_u), left_u+ratio*(right_u-left_u)
            f1 = value(u1, coeff, start, T, amp, omega, phase, spin, spin_phase, vertices)
            f2 = value(u2, coeff, start, T, amp, omega, phase, spin, spin_phase, vertices)
            for iteration in range(38):
                if f1 < f2:
                    right_u, u2, f2 = u2, u1, f1
                    u1 = right_u-ratio*(right_u-left_u)
                    f1 = value(u1, coeff, start, T, amp, omega, phase, spin, spin_phase, vertices)
                else:
                    left_u, u1, f1 = u1, u2, f2
                    u2 = left_u+ratio*(right_u-left_u)
                    f2 = value(u2, coeff, start, T, amp, omega, phase, spin, spin_phase, vertices)
            if f1 < best:
                best, witness = f1, u1
            if f2 < best:
                best, witness = f2, u2
            return 1, best, witness, boxes
        if planar+normal*normal >= radius*radius:
            continue
        if (b-a)*T < 1e-6 or boxes >= 512:
            return 2, best, witness, boxes
        work = control.copy()
        left, right = np.empty((8, 3)), np.empty((8, 3))
        left[0], right[7] = work[0], work[7]
        for level in range(1, 8):
            for k in range(8-level):
                work[k] = .5*(work[k]+work[k+1])
            left[level], right[7-level] = work[0], work[7-level]
        mid = (a+b)/2
        stack_c[size], stack_a[size], stack_b[size] = right, mid, b
        stack_c[size+1], stack_a[size+1], stack_b[size+1] = left, a, mid
        size += 2
        boxes += 1
    return 0, best, witness, boxes


def warmup():
    check(np.zeros((0, 8, 3)), np.zeros((0, 2)), np.zeros((8, 3)), 0., 1.,
          np.zeros(3), 1., 0., 1., 0., np.zeros((4, 2)), .57)


def passage(plan, window, radius, piece, deadline=np.inf):
    if not window.motion.rotation_enabled:
        return translation_passage(plan, window, radius, piece, deadline)
    if time.perf_counter() >= deadline:
        return dict(passed=False, outcome='check_budget', minimum_sample_distance_m=np.inf,
                    witness_u=None, boxes=0, overlap_intervals=[])
    R = window.base_rotation
    amp = R.T@window.motion.translation_amplitude
    if abs(amp[2]) > 1e-9:
        raise ValueError('This spin adapter requires in-plane translation')
    coeff = plan.coefficients[piece]@R
    coeff = coeff.copy()
    coeff[0] -= window.center0@R
    start = float(plan.made_at+sum(plan.durations[:piece]))
    T = float(plan.durations[piece])
    for z in np.polynomial.polynomial.polyroots(coeff[:, 2]):
        if abs(z.imag) < 1e-7 and 1e-6 < z.real < 1-1e-6:
            t = start+float(z.real)*T
            c, rr, *_ = window.state_at(t)
            q = rr.T@(plan.sample(t)-c)
            if window.aperture.contains(q[:2]):
                return dict(passed=False, outcome='repeat_aperture_crossing',
                            minimum_sample_distance_m=0., witness_u=None, boxes=0, overlap_intervals=[])
    cuts = np.asarray(overlap_intervals(coeff[:, 2], radius)).reshape(-1, 2)
    base = power_to_bernstein(coeff)
    controls = np.asarray([restrict(base, a, b) for a, b in cuts]).reshape(-1, 8, 3)
    m = window.motion
    outcome, best, witness, boxes = check(controls, cuts, coeff, start, T, amp,
        2*np.pi/m.translation_period, m.phase, 2*np.pi*m.spin_hz, m.spin_phase,
        np.asarray(window.aperture.physical_vertices), float(radius))
    expired = time.perf_counter() >= deadline
    return dict(passed=bool(outcome == 0 and not expired),
                outcome='check_budget' if expired else ('floating_local_bound', 'violated', 'unresolved')[outcome],
                minimum_sample_distance_m=math.sqrt(best), witness_u=float(witness) if len(cuts) else None,
                boxes=int(boxes), overlap_intervals=[dict(piece=piece, start=float(start+a*T),
                                                        end=float(start+b*T)) for a, b in cuts],
                evidence='floating Bernstein/translation sine/spin sin-cos product bounds; not Arb')
