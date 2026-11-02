"""Local adaptive sphere separation from a moving aperture's outside half-planes.

Polynomial roots locate every interval where the sphere can touch the frame
plane. Bernstein hulls and exact sine ranges bound only those intervals. The
bounds use floating-point arithmetic: numerical enclosure evidence, NOT an
interval-arithmetic certificate. No full-flight uniform safety grid is used.
"""
import heapq
import math
import numpy as np
from scipy.optimize import minimize_scalar


def power_to_bernstein(power):
    n = len(power)-1
    transform = np.zeros((n+1, n+1))
    for i in range(n+1):
        for k in range(i+1):
            transform[i, k] = math.comb(i, k)/math.comb(n, k)
    return transform@power


def split(control, fraction):
    work = np.array(control, copy=True)
    left, right = [work[0].copy()], [work[-1].copy()]
    for _ in range(len(control)-1):
        work = (1-fraction)*work[:-1]+fraction*work[1:]
        left.append(work[0].copy()); right.append(work[-1].copy())
    return np.array(left), np.array(right[::-1])


def restrict(control, lower, upper):
    if lower > 0: control = split(control, lower)[1]
    if upper < 1: control = split(control, (upper-lower)/(1-lower))[0]
    return control


def sine_range(lower, upper):
    lo, hi = sorted((lower, upper))
    values = [math.sin(lo), math.sin(hi)]
    for k in range(math.ceil((lo-math.pi/2)/math.pi),
                   math.floor((hi-math.pi/2)/math.pi)+1):
        values.append(1. if k % 2 == 0 else -1.)
    return min(values), max(values)


def overlap_intervals(normal_power, radius):
    cuts = [0., 1.]
    for level in (-radius, radius):
        polynomial = normal_power.copy(); polynomial[0] -= level
        roots = np.polynomial.polynomial.polyroots(polynomial)
        cuts += [float(z.real) for z in roots
                 if abs(z.imag) < 1e-7 and 0 < z.real < 1]
    cuts = sorted(set(cuts))
    return [(a, b) for a, b in zip(cuts, cuts[1:])
            if abs(np.polynomial.polynomial.polyval((a+b)/2, normal_power)) <= radius+1e-9]


def rectangle_distance_squared(q, half_sizes):
    q = np.asarray(q)
    distances = []
    for axis in (0, 1):
        for sign in (-1, 1):
            # Distance to the OUTSIDE half-plane, not a line with two sides.
            # This prevents switching to the exterior branch between samples.
            inside = np.maximum(half_sizes[axis]-sign*q[..., axis], 0.)
            distances.append(inside**2+q[..., 2]**2)
    return np.min(distances, axis=0)


def box_distance_squared(lower, upper, half_sizes):
    values = []
    normal = max(lower[2], -upper[2], 0.)**2
    for axis in (0, 1):
        for sign in (-1, 1):
            outward_maximum = upper[axis] if sign > 0 else -lower[axis]
            inside = max(half_sizes[axis]-outward_maximum, 0.)
            values.append(inside*inside+normal)
    return min(values)


def passage(plan, window, radius, frame_offset=.065, tolerance=2e-4, max_boxes=512,
            piece_indices=None, translation_bounds=None):
    """Return all possible plane-overlap intervals, worst witness, and bounds."""
    rotation = window.state_at(0.)[1]
    amplitude = rotation.T@window.motion.translation_amplitude
    if abs(amplitude[2]) > 1e-9 or window.motion.rotation_enabled:
        raise ValueError('Only fixed-plane in-plane translation is implemented')
    vertices = window.aperture.physical_vertices
    half_sizes = np.max(np.abs(vertices), axis=0)+frame_offset
    omega = 2*np.pi/window.motion.translation_period
    phase = window.motion.phase
    queue = []; serial = 0; intervals = []; best = np.inf; witness = None
    starts = np.r_[plan.made_at, plan.made_at+np.cumsum(plan.durations)[:-1]]

    def value(piece, u):
        t = starts[piece]+plan.durations[piece]*u
        q = rotation.T@(plan.sample(float(t))-window.state_at(float(t))[0])
        return float(rectangle_distance_squared(q, half_sizes))

    def enqueue(piece, a, b, control):
        nonlocal serial, best, witness
        ta, tb = starts[piece]+plan.durations[piece]*np.array([a, b])
        if translation_bounds is None:
            sl, su = sine_range(omega*ta+phase, omega*tb+phase)
            v0, v1 = amplitude*sl, amplitude*su
        else:
            v0, v1 = translation_bounds(ta, tb)
        lower = control.min(axis=0)-np.maximum(v0, v1)-1e-10
        upper = control.max(axis=0)-np.minimum(v0, v1)+1e-10
        bound = box_distance_squared(lower, upper, half_sizes)
        for u in (a, (a+b)/2, b):
            distance = value(piece, u)
            if distance < best: best, witness = distance, (piece, u)
        serial += 1
        heapq.heappush(queue, (bound, serial, piece, a, b, control))

    for piece, coefficients in enumerate(plan.coefficients):
        if piece_indices is not None and piece not in piece_indices: continue
        local = coefficients@rotation
        local = local.copy(); local[0] -= window.center0@rotation
        control = power_to_bernstein(local)
        for a, b in overlap_intervals(local[:, 2], radius):
            intervals.append(dict(piece=piece, start=float(starts[piece]+a*plan.durations[piece]),
                                  end=float(starts[piece]+b*plan.durations[piece])))
            enqueue(piece, a, b, restrict(control, a, b))

    boxes = 0; safe_bound = np.inf
    while queue and boxes < max_boxes:
        bound, _, piece, a, b, control = heapq.heappop(queue)
        if bound >= radius**2:
            safe_bound = min(safe_bound, bound)
            continue
        if best < (radius-tolerance)**2:
            # Refine a detected violating basin; the next outer iteration will
            # bound ALL overlap intervals again, not trust this local minimum.
            result = minimize_scalar(lambda u: value(piece, u), bounds=(a, b),
                                     method='bounded', options={'xatol': 1e-10})
            if result.fun < best: best, witness = float(result.fun), (piece, float(result.x))
            return dict(passed=False, outcome='violated', minimum_sample_distance_m=math.sqrt(best),
                        witness_piece=witness[0], witness_u=witness[1], boxes=boxes,
                        overlap_intervals=intervals)
        if (b-a)*plan.durations[piece] < 1e-6:
            return dict(passed=False, outcome='unresolved', minimum_sample_distance_m=math.sqrt(best),
                        witness_piece=witness[0], witness_u=witness[1], boxes=boxes,
                        overlap_intervals=intervals)
        left, right = split(control, .5); middle = (a+b)/2
        enqueue(piece, a, middle, left); enqueue(piece, middle, b, right); boxes += 1
    passed = not queue
    return dict(passed=passed, outcome='numerically_bounded' if passed else 'budget_unresolved',
                minimum_sample_distance_m=math.sqrt(best),
                distance_lower_bound_m=math.sqrt(safe_bound) if passed else None,
                witness_piece=None if witness is None else witness[0],
                witness_u=None if witness is None else witness[1], boxes=boxes,
                overlap_intervals=intervals,
                evidence='floating-point Bernstein/sine enclosures, not a formal certificate')
