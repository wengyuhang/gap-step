"""Ideal-reference execution of the independent Wang N=2/M=4 planner."""
from dataclasses import asdict, dataclass, replace
import math
import time
import numpy as np
import trimesh
from 复现.wang2023_verified import Config, Planner
from 复现.wang2023_verified import x500
from 复现.wang2023_verified.minco import Trajectory, nonuniform_coefficients
from ..basic_group.whole_course_difficulty import scenario
from ..basic_group.trajectory_only.collision import BODY_RADIUS


@dataclass(frozen=True)
class Settings:
    order: int = 4
    pieces: int = 4
    intervals: int = 32
    rho: float = 1e4
    body_rate_max: float = 3.8
    budget_ms: float = 2000.
    iterations: int = 5000
    seed_speed: float = 6.5
    temporal_scales: tuple = (1., .8, 1.2, 1.5)
    compile_functions: bool = True


def model_config(settings):
    model = x500.Model()
    return replace(Config(), order=settings.order, pieces=settings.pieces, intervals=settings.intervals,
        rho=settings.rho, drag=model, body_rate_max=settings.body_rate_max,
        thrust_min=4*model.thrust_coefficient*150**2,
        thrust_max=.9*4*model.thrust_coefficient*1000**2,
        solve_budget_ms=settings.budget_ms, max_iterations=settings.iterations,
        compile_functions=settings.compile_functions)


def outward_normals(track):
    previous, result = track.start, []
    for window in track.windows:
        sign = float(np.sign((previous-window.center0)@window.R[:, 2]) or 1.)
        result.append(-sign*window.R[:, 2])
        previous = window.center0
    return result


class Reference:
    """One global MINCO initializer and bounded forward cruise terminals."""
    def __init__(self, track, settings, mesh_directory):
        self.track, self.settings = track, settings
        self.normals = outward_normals(track)
        self.body_depths = [BODY_RADIUS+max(abs(trimesh.load_mesh(
            mesh_directory/f'gate_{w.name}_sleeve.stl').bounds[:, 2]))+1e-6 for w in track.windows]
        anchors = np.vstack([track.start, *[w.center0 for w in track.windows], track.goal])
        distances = np.linalg.norm(np.diff(anchors, axis=0), axis=1)
        step = max(1., float(distances.sum()/settings.seed_speed/len(distances)),
            float(np.sqrt(3*distances.max()/8.)))
        times = np.full(len(distances), step)
        self.node_times = np.r_[0., np.cumsum(times)]
        points = [track.start, *[w.state_at(float(self.node_times[k+1]))[0]
            for k, w in enumerate(track.windows)], track.goal]
        head, tail = np.zeros((3, settings.order)), np.zeros((3, settings.order))
        head[:, 0], tail[:, 0] = track.start, track.goal
        c = nonuniform_coefficients(head, tail, np.asarray(points[1:-1]), times, settings.order)
        self.curve = Trajectory(0., times, c, pieces=1, order=settings.order)
        self.tails, self.tail_times, self.tail_choices = [], [], []
        for k, window in enumerate(track.windows):
            if k == len(track.windows)-1:
                state, at, choice = tail.copy(), self.curve.end, 'actual_goal_zero_vaj'
            else:
                at = .5*(self.node_times[k+1]+self.node_times[k+2])
                position = .5*(points[k+1]+points[k+2])
                depth = (position-window.center0)@self.normals[k]
                position += self.normals[k]*max(0., 2*self.body_depths[k]-depth)
                direction = np.asarray(points[k+2])-position
                direction /= np.linalg.norm(direction)
                state = np.zeros((3, settings.order))
                state[:, 0], state[:, 1] = position, settings.seed_speed*direction
                choice = 'forward_geometric_cruise_state'
            self.tails.append(state)
            self.tail_times.append(float(at))
            self.tail_choices.append(choice)

    def seed(self, planner, head, now, gate, scale):
        settings = self.settings
        begin = self.node_times[gate]
        middle, end = self.node_times[gate+1], self.tail_times[gate]
        spans = np.array([middle-begin, end-middle])
        durations = spans*scale
        tail = self.tails[gate]
        center = self.track.windows[gate].state_at(float(now+durations[0]))[0]
        x = np.zeros(planner.dimension)
        x[:2] = np.log(durations)
        for d in range(1, settings.order):
            x[2+3*(d-1):2+3*d] = self.curve.sample(middle, d)*(spans[0]/durations[0])**d/planner.cfg.derivative_scale
        fractions = np.arange(1, settings.pieces)/settings.pieces
        blend = 35*fractions**4-84*fractions**5+70*fractions**6-20*fractions**7
        for leg, (a, b, start, finish) in enumerate(((begin, middle, head[:, 0], center),
                                                    (middle, end, center, tail[:, 0]))):
            points = self.curve.sample(a+fractions*(b-a))
            points += (1-blend[:, None])*(start-self.curve.sample(a))+blend[:, None]*(finish-self.curve.sample(b))
            offset = planner.state_end+3*(settings.pieces-1)*leg
            x[offset:offset+3*(settings.pieces-1)] = points.ravel()/planner.cfg.position_scale
        return x


def crop(coefficients, first, last):
    width = len(coefficients)
    result = np.zeros((width, 3))
    for degree in range(width):
        for source in range(degree, width):
            result[degree] += coefficients[source]*math.comb(source, degree)*first**(source-degree)*(last-first)**degree
    return result


def prefix(trajectory, end):
    blocks, durations = [], []
    begin = trajectory.start
    for c, duration in zip(trajectory.coefficients, trajectory.durations):
        stop = min(float(end), begin+duration)
        if stop > begin+1e-10:
            fraction = np.clip((stop-begin)/duration, 0., 1.)
            blocks.append(crop(c, 0., fraction))
            durations.append(float(stop-begin))
        begin += duration
        if begin >= end-1e-10:
            break
    return blocks, durations


def level_roots(trajectory, window, normal, level):
    roots, start = [], trajectory.start
    for c, duration in zip(trajectory.coefficients, trajectory.durations):
        polynomial = c@normal
        polynomial[0] -= window.center0@normal+level
        candidates = [float(z.real) for z in np.polynomial.polynomial.polyroots(polynomial)
            if abs(z.imag) < 1e-7 and -1e-8 <= z.real <= 1+1e-8]
        if abs(polynomial[0]) < 1e-7:
            candidates.append(0.)
        if abs(np.polynomial.polynomial.polyval(1., polynomial)) < 1e-7:
            candidates.append(1.)
        roots.extend(start+np.clip(u, 0., 1.)*duration for u in candidates)
        start += duration
    result = []
    for at in sorted(roots):
        if not result or abs(at-result[-1]) > 1e-6:
            result.append(float(at))
    return result


def passage(trajectory, window, normal, depth):
    center_crossing = None
    for at in level_roots(trajectory, window, normal, 0.):
        if at <= trajectory.start+1e-7:
            continue
        center, rotation, _, _ = window.state_at(at)
        local = rotation.T@(trajectory.sample(at)-center)
        if window.aperture.contains(local[:2]):
            if trajectory.sample(at, 1)@normal <= 0.:
                return dict(passed=False, failure='reverse_current_aperture_crossing', time_s=at)
            center_crossing = at
            break
    if center_crossing is None:
        return dict(passed=False, failure='no_forward_current_aperture_crossing')
    for at in level_roots(trajectory, window, normal, depth):
        if at > center_crossing and trajectory.sample(at, 1)@normal > 0.:
            return dict(passed=True, center_crossing_s=center_crossing, body_exit_s=at)
    return dict(passed=False, failure='no_complete_body_exit', center_crossing_s=center_crossing)


def solve(level, settings=Settings(), progress=lambda record: None):
    began = time.perf_counter()
    track, _ = scenario.build_case(scenario.Case(level=level))
    reference = Reference(track, settings, scenario.ASSETS/'meshes')
    cfg = model_config(settings)
    planners = [Planner([lambda at, w=w: scenario.pose(w, at)[0]], cfg,
        flatness_function=x500.flatness, yaw=lambda at: 0*at+np.pi/2) for w in track.windows]
    build_s = time.perf_counter()-began
    head = np.zeros((3, settings.order)); head[:, 0] = track.start
    blocks, durations, requests = [], [], []
    now, failure = 0., None
    solve_began = time.perf_counter()
    for gate, planner in enumerate(planners):
        request_began = time.perf_counter()
        candidates = []
        for index, scale in enumerate(settings.temporal_scales):
            seed = reference.seed(planner, head, now, gate, scale)
            trajectory, stats = planner.solve(head, reference.tails[gate], now, seed)
            valid = bool(stats['converged'] and stats['finite'] and np.isfinite(trajectory.coefficients).all()
                and np.isfinite(trajectory.durations).all() and np.all(trajectory.durations > 0.))
            candidates.append((trajectory, dict(seed=index, scale=scale, eligible=valid, **stats)))
        eligible = [i for i, (_, stats) in enumerate(candidates) if stats['eligible']]
        selected = min(eligible, key=lambda i: candidates[i][1]['objective_scaled']) if eligible else None
        request = dict(gate=gate+1, race_time_s=now, head_state=head.tolist(), tail_state=reference.tails[gate].tolist(),
            terminal_choice=reference.tail_choices[gate], selected_seed=selected,
            cycle_ms=1000*(time.perf_counter()-request_began),
            candidates=[dict(stats=s, durations=t.segment_durations.tolist(), coefficients=t.coefficients.tolist()) for t, s in candidates])
        requests.append(request)
        if selected is None:
            failure = 'no_converged_local_candidate'
            progress(dict(gate=gate+1, found=False, failure=failure, cycle_ms=request['cycle_ms']))
            break
        trajectory, _ = candidates[selected]
        event = passage(trajectory, track.windows[gate], reference.normals[gate], reference.body_depths[gate])
        request['event'] = event
        if not event['passed']:
            failure = event['failure']
            request['cycle_ms'] = 1000*(time.perf_counter()-request_began)
            progress(dict(gate=gate+1, found=False, failure=failure, cycle_ms=request['cycle_ms']))
            break
        end = trajectory.end if gate == len(planners)-1 else event['body_exit_s']
        c, t = prefix(trajectory, end)
        blocks.extend(c); durations.extend(t)
        now = float(end)
        head = np.column_stack([trajectory.sample(now, d) for d in range(settings.order)])
        request['cycle_ms'] = 1000*(time.perf_counter()-request_began)
        progress(dict(gate=gate+1, found=True, race_time_s=now, cycle_ms=request['cycle_ms'],
            crossing_s=event['center_crossing_s'], body_exit_s=event['body_exit_s']))
    return dict(found=failure is None, failure=failure, case=dict(layout='togt_whole', level=level),
        method='wang2023_verified_s4', build_s=build_s, solve_s=time.perf_counter()-solve_began,
        nominal_time_s=float(sum(durations)), coefficients=[c.tolist() for c in blocks], durations=durations,
        requests=requests, gate_count=sum(r.get('event', {}).get('passed', False) for r in requests),
        config=asdict(cfg), reference_seed=dict(durations=reference.curve.durations.tolist(), coefficients=reference.curve.coefficients.tolist(),
            tail_states=[s.tolist() for s in reference.tails], tail_choices=reference.tail_choices,
            normals=[n.tolist() for n in reference.normals], body_exit_depths=reference.body_depths),
        control_model='ideal PVAJ reference tracking; planning wall time recorded separately',
        replanning='complete body exit only; keep final return', optimizer_selection='minimum Eq31 objective among converged finite starts',
        free_crossing_points=False, safety_filter=False, additional_frame_penalty=False, made_at=0.)
