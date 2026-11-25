#!/usr/bin/env python3
"""Static ten-gate TOGT decomposition diagnostic.

This is deliberately a diagnostic, not a dynamic-window benchmark.  It
compares a full free-crossing solve with (i) receding-horizon re-planning and
(ii) state-continuous block solves.  Every reported execution includes the
last gate-to-goal segment.  All boundary states come from one *free-crossing*
global TOGT solution.  The script separately records an exact shared-variable
partition: it is an identity check, not a collection of independently solved
subproblems.
"""
from __future__ import annotations

import json
from pathlib import Path
import time
import argparse

import numpy as np

from convex_timevarying_window.comparisons.conditional_cem_vs_togt.planar_ablation import _aperture
from convex_timevarying_window.geometry import ConvexAperture
from convex_timevarying_window.togt.experiment import WINDOW_MARGIN
from convex_timevarying_window.geometry import PeriodicConvexWindow
from convex_timevarying_window.togt.native_backend import AnalyticTOGTCore
from convex_timevarying_window.togt.native_objective import NativeJointTOGTObjective
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState, MincoSnap
from nonconvex_timevarying_window.sc_dynatogt.time_mapping import (
    backpropagate_to_k, durations_from_k, k_from_durations,
)


ROOT = Path(__file__).resolve().parent / "results" / "static_10gate_rolling_horizon_togt_20260912"
OUT = ROOT / "penalty_scale_100_rolling_and_chunks_1to10.json"
BLOCK_OUT = ROOT / "self_pvaj_fixed_block_vs_rolling_1to11.json"
SCALE = 100.0
CENTERS_XY = np.array((
    (-40., 5.), (-31., -10.), (-22., 10.), (-13., -6.), (-4., 12.),
    (5., -12.), (14., 7.), (23., -9.), (32., 9.), (41., -3.),
))
CENTER_Z = np.full(10, 3.2)
GATE_YAW = np.zeros(10)
PAPER_LAYOUT = False
START = np.array((-50., -20., 3.2))
GOAL = np.array((50., 20., 3.2))


def pvaj(traj: MincoSnap, instant: float) -> np.ndarray:
    """C++ layout: coordinate by PVAJ derivative."""
    return np.stack([traj.evaluate(instant, derivative=d) for d in range(4)], axis=1)


def paper_aperture() -> ConvexAperture:
    """2.4 m rectangle matching the rectangle gates in UZH 19g."""
    half = 1.2
    signs = np.array(([-1., -1.], [1., -1.], [1., 1.], [-1., 1.]))
    return ConvexAperture("polygon", margin=WINDOW_MARGIN,
                          physical_vertices=signs * half,
                          safe_vertices=signs * (half - WINDOW_MARGIN / 2.))


def static_track(start: np.ndarray, goal: np.ndarray, gate_indices: list[int], *, dynamic=False, time_offset=0.):
    windows = []
    for index in gate_indices:
        center = np.array((*CENTERS_XY[index], CENTER_Z[index]))
        phase = 0.73 * index - 1.1
        if dynamic:
            # Strong but still wide-aperture planar-motion stressor.  Local
            # planners receive the absolute-time phase shift below.
            motion = MotionProfile(
                translation_amplitude=np.array((0., .65, 0.)),
                rotation_amplitude=np.array((0., 0., np.deg2rad(20.))),
                scale_amplitude=0., translation_period=5., rotation_period=5., scale_period=5.,
                phase=phase + 2. * np.pi * time_offset / 5., scale_enabled=False,
            )
        else:
            motion = MotionProfile(translation_amplitude=np.zeros(3), rotation_amplitude=np.zeros(3),
                                   scale_amplitude=0., translation_period=10., rotation_period=10.,
                                   scale_period=10., phase=0., scale_enabled=False)
        windows.append(PeriodicConvexWindow(
            name=f"G{index + 1}", aperture=paper_aperture() if PAPER_LAYOUT else _aperture(), center0=center,
            angles0_rpy=np.array((0., -np.pi / 2., GATE_YAW[index])),
            motion=motion,
        ))
    return SCWindowTrack("static_10_decomposition", start, goal, tuple(windows), tuple(range(len(windows))))


def trajectory_from(base: NativeJointTOGTObjective, x: np.ndarray) -> MincoSnap:
    _, _, durations, _, points, *_ = base._geometry(x)  # the exact native objective geometry
    return MincoSnap(BoundaryState.from_array(base.head), BoundaryState.from_array(base.tail), points, durations)


def shifted_warm_start(base: NativeJointTOGTObjective, gate_indices: list[int], warm):
    """Shift overlapping durations/crossing variables from the prior horizon."""
    initial = base.initial_guess()
    if warm is None:
        return initial
    k, d = base.split(initial)
    target = durations_from_k(k)
    old_times = warm["durations"]
    target[:min(len(target), len(old_times))] = old_times[:min(len(target), len(old_times))]
    old_d = {index: value for index, value in zip(warm["gate_indices"], warm["d"])}
    for index, value in zip(gate_indices, d):
        if index in old_d:
            value[:] = old_d[index]
    return np.concatenate((k_from_durations(target), *d))


def shift_warm(warm, executed_segments: int):
    return {"gate_indices": warm["gate_indices"][executed_segments:],
            "d": warm["d"][executed_segments:],
            "durations": warm["durations"][executed_segments:]}


def solve_free(core: AnalyticTOGTCore, head: np.ndarray, tail: np.ndarray, gate_indices: list[int], warm=None,
               *, dynamic=False, time_offset=0.):
    if not gate_indices:
        durations0 = np.array((max(np.linalg.norm(tail[:, 0] - head[:, 0]) / 4.0, .15),))
        if warm is not None and len(warm["durations"]): durations0[0] = warm["durations"][0]
        initial = k_from_durations(durations0)
        points = np.empty((0, 3))
        def objective(k):
            durations = durations_from_k(k)
            cost, _, grad_t = core.value_and_gradient(head, tail, points, durations)
            return cost, backpropagate_to_k(k, grad_t)
        started = time.perf_counter()
        result = core.released_lbfgs(initial, objective, function_tolerance=1e-5, gradient_tolerance=1e-5)
        elapsed = time.perf_counter() - started
        final_cost, final_gradient = objective(result["x"])
        result["final_cost_check"] = float(final_cost)
        result["final_gradient_inf_norm"] = float(np.max(np.abs(final_gradient)))
        result["_warm"] = {"gate_indices": [], "d": [], "durations": durations_from_k(result["x"])}
        return result, MincoSnap(BoundaryState.from_array(head), BoundaryState.from_array(tail), points,
                                 durations_from_k(result["x"])), elapsed
    track = static_track(head[:, 0], tail[:, 0], gate_indices, dynamic=dynamic, time_offset=time_offset)
    # The config is only used by initial_guess/track bookkeeping here.
    class Config:
        initial_speed = 4.0
        minimum_initial_duration = .15
        invalid_trial_cost = 1e30
    base = NativeJointTOGTObjective(track, Config())
    base.core = core
    base.head = head.copy(); base.tail = tail.copy()
    initial = shifted_warm_start(base, gate_indices, warm)
    started = time.perf_counter()
    result = core.released_lbfgs(initial, base.value_and_gradient,
                                 function_tolerance=1e-5, gradient_tolerance=1e-5)
    elapsed = time.perf_counter() - started
    final_cost, final_gradient = base.value_and_gradient(result["x"])
    result["final_cost_check"] = float(final_cost)
    result["final_gradient_inf_norm"] = float(np.max(np.abs(final_gradient)))
    _, solved_d = base.split(result["x"])
    result["_warm"] = {"gate_indices": list(gate_indices), "d": [value.copy() for value in solved_d],
                       "durations": base._geometry(result["x"])[2].copy()}
    return result, trajectory_from(base, result["x"]), elapsed


def solve_centres(core: AnalyticTOGTCore):
    points = np.column_stack((CENTERS_XY, np.full(10, 3.2)))
    head = np.zeros((3, 4)); head[:, 0] = START
    tail = np.zeros((3, 4)); tail[:, 0] = GOAL
    lengths = np.linalg.norm(np.diff(np.vstack((START, points, GOAL)), axis=0), axis=1)
    initial = k_from_durations(np.maximum(lengths / 4.0, .15))
    def objective(k):
        durations = durations_from_k(k)
        cost, _, grad_t = core.value_and_gradient(head, tail, points, durations)
        return cost, backpropagate_to_k(k, grad_t)
    result = core.released_lbfgs(initial, objective, function_tolerance=1e-5, gradient_tolerance=1e-5)
    durations = durations_from_k(result["x"])
    return result, MincoSnap(BoundaryState.from_array(head), BoundaryState.from_array(tail), points, durations)


def penalty(core: AnalyticTOGTCore, pieces: list[MincoSnap]) -> float:
    """One-millisecond sampled integral of the scaled released soft penalty."""
    total = 0.0
    for traj in pieces:
        # Segmentwise avoids sampling across a discontinuous optimiser output.
        for segment, duration in enumerate(traj.durations):
            grid = np.linspace(0., float(duration), max(2, int(np.ceil(float(duration) / .001)) + 1))
            states = np.stack([traj.evaluate_segment(segment, grid, derivative=d) for d in range(5)], axis=1)
            values = SCALE * core.sample_dynamics(states)["instantaneous_penalty"]
            total += float(np.trapezoid(values, grid))
    return total


def dynamics_audit(core: AnalyticTOGTCore, pieces: list[MincoSnap]):
    """Dense dynamics extrema and released-TOGT limit residuals (1 ms grid)."""
    values = {"speed": [], "tilt": [], "body_rate": [], "collective": [], "rotor": []}
    for traj in pieces:
        for segment, duration in enumerate(traj.durations):
            grid = np.linspace(0., float(duration), max(2, int(np.ceil(float(duration) / .001)) + 1))
            states = np.stack([traj.evaluate_segment(segment, grid, derivative=d) for d in range(5)], axis=1)
            sampled = core.sample_dynamics(states)
            values["speed"].append(sampled["speed"]); values["tilt"].append(sampled["tilt"])
            values["body_rate"].append(sampled["body_rate"]); values["collective"].append(sampled["collective_thrust"])
            values["rotor"].append(sampled["rotor_thrusts"])
    speed = np.concatenate(values["speed"]); tilt = np.concatenate(values["tilt"])
    rate = np.concatenate(values["body_rate"]); collective = np.concatenate(values["collective"])
    rotor = np.concatenate(values["rotor"])
    max_rotor, min_rotor = float(np.max(rotor)), float(np.min(rotor))
    max_xy_rate = float(np.max(np.linalg.norm(rate[:, :2], axis=1)))
    max_z_rate = float(np.max(np.abs(rate[:, 2])))
    return {"max_speed_mps": float(np.max(speed)), "max_tilt_rad": float(np.max(tilt)),
            "max_body_rate_xy_radps": max_xy_rate, "max_abs_body_rate_z_radps": max_z_rate,
            "max_collective_thrust_n": float(np.max(collective)),
            "min_rotor_thrust_n": min_rotor, "max_rotor_thrust_n": max_rotor,
            "rotor_upper_excess_n": max(0., max_rotor - 5.),
            "rotor_lower_deficit_n": max(0., .25 - min_rotor),
            "tilt_excess_rad": max(0., float(np.max(tilt)) - 6.28),
            "body_rate_xy_excess_radps": max(0., max_xy_rate - 10.),
            "body_rate_z_excess_radps": max(0., max_z_rate - 10.)}


def reference_states(reference: MincoSnap) -> list[np.ndarray]:
    ends = np.cumsum(reference.durations)
    return [pvaj(reference, float(t)) for t in ends]


def rolling(core: AnalyticTOGTCore, horizon: int):
    """Self-contained receding horizon.

    At every stage we constrain exactly the next ``horizon`` gates and the
    real goal.  Hence the first gate is an interior MINCO point: its PVAJ is
    generated by *this* solve, and is the only state passed to the next
    solve.  No reference PVAJ is imposed at a horizon edge.  With horizon=10
    the first solve is the original complete 10-gate problem.
    """
    head = np.zeros((3, 4)); head[:, 0] = START
    terminal = np.zeros((3, 4)); terminal[:, 0] = GOAL
    executed: list[MincoSnap] = []; rows = []; plan_seconds = 0.
    for stage in range(10):
        gates = list(range(stage, min(10, stage + horizon)))
        result, local, elapsed = solve_free(core, head, terminal, gates)
        plan_seconds += elapsed
        if stage < 9:
            # Execute only the first local segment and transfer its full PVAJ.
            cut = float(local.durations[0])
            executed.append(MincoSnap(BoundaryState.from_array(head), BoundaryState.from_array(pvaj(local, cut)),
                                      np.empty((0, 3)), np.array((cut,))))
            head = pvaj(local, cut)
            count = 1
        else:
            executed.append(local); count = local.num_segments
        rows.append({"stage": stage + 1, "horizon": horizon, "status": result["status"],
                     "evaluations": result["evaluations"], "planning_seconds": elapsed,
                     "executed_piece_count": count})
    flight = float(sum(piece.total_time for piece in executed))
    pen = penalty(core, executed)
    return {"flight_time_s": flight, "sampled_penalty": pen, "sampled_total": flight + pen,
            "planning_seconds": plan_seconds, "statuses": [row["status"] for row in rows], "rows": rows}


def partition(count: int) -> list[int]:
    # Number of passed gates after every non-final block, as evenly as possible.
    return [int(np.floor(10 * i / count)) for i in range(1, count)]


def chunks(core: AnalyticTOGTCore, references: list[np.ndarray], count: int):
    cuts = partition(count)
    previous = 0; head = np.zeros((3, 4)); head[:, 0] = START
    pieces: list[MincoSnap] = []; rows = []; plan_seconds = 0.
    for block, cut in enumerate(cuts + [10], start=1):
        if cut < 10:
            tail = references[cut - 1]
            gates = list(range(previous, cut - 1))
        else:
            tail = references[-1]
            gates = list(range(previous, 10))
        result, local, elapsed = solve_free(core, head, tail, gates)
        pieces.append(local); plan_seconds += elapsed
        rows.append({"block": block, "gate_range": [previous + 1, cut], "status": result["status"],
                     "evaluations": result["evaluations"], "planning_seconds": elapsed,
                     "free_gate_count": len(gates)})
        head = tail; previous = cut
    flight = float(sum(piece.total_time for piece in pieces)); pen = penalty(core, pieces)
    return {"flight_time_s": flight, "sampled_penalty": pen, "sampled_total": flight + pen,
            "planning_seconds": plan_seconds, "cuts_after_gates": cuts,
            "statuses": [row["status"] for row in rows], "rows": rows}


def exact_shared_partition(core: AnalyticTOGTCore, global_trajectory: MincoSnap, count: int,
                           global_planning_seconds: float):
    """Identity check for the truly state-shared formulation.

    All segment durations and PVAJ values are shared global variables.  Thus
    changing the number of bookkeeping blocks cannot change the trajectory;
    a joint optimisation over these variables is precisely the original TOGT
    problem, not ``count`` independent optimisations.
    """
    pieces = []
    starts = np.concatenate(([0.], np.cumsum(global_trajectory.durations)[:-1]))
    for index, (begin, duration) in enumerate(zip(starts, global_trajectory.durations)):
        head = pvaj(global_trajectory, float(begin))
        tail = pvaj(global_trajectory, float(begin + duration))
        pieces.append(MincoSnap(BoundaryState.from_array(head), BoundaryState.from_array(tail),
                                np.empty((0, 3)), np.array((duration,))))
    flight = float(sum(piece.total_time for piece in pieces)); pen = penalty(core, pieces)
    return {"flight_time_s": flight, "sampled_penalty": pen, "sampled_total": flight + pen,
            "shared_global_planning_seconds": global_planning_seconds,
            "block_count": count, "note": "exact global-variable partition; no independent reoptimisation"}


def prefix_pieces(trajectory: MincoSnap, piece_count: int) -> list[MincoSnap]:
    """Extract executed prefix pieces without changing their PVAJ or duration."""
    starts = np.concatenate(([0.], np.cumsum(trajectory.durations)[:-1]))
    result = []
    for segment in range(piece_count):
        begin = float(starts[segment]); duration = float(trajectory.durations[segment])
        result.append(MincoSnap(BoundaryState.from_array(pvaj(trajectory, begin)),
                                BoundaryState.from_array(pvaj(trajectory, begin + duration)),
                                np.empty((0, 3)), np.array((duration,))))
    return result


def two_stage_split(core: AnalyticTOGTCore, split_segment: int):
    """Replan after a trajectory segment (1..10 at a gate; 11 at the goal)."""
    start = np.zeros((3, 4)); start[:, 0] = START
    goal = np.zeros((3, 4)); goal[:, 0] = GOAL
    if split_segment == 11:
        result, full, elapsed = solve_free(core, start, goal, list(range(10)))
        pen = penalty(core, [full])
        return {"split_after_segment": 11, "split_after": "goal (no replan)",
                "flight_time_s": float(full.total_time), "sampled_penalty": pen,
                "sampled_total": float(full.total_time) + pen, "planning_seconds": elapsed,
                "first_stage": {"planning_seconds": elapsed, "status": result["status"],
                                "evaluations": result["evaluations"]}, "second_stage": None}
    split_gate = split_segment
    first_result, first, first_seconds = solve_free(core, start, goal, list(range(split_gate)))
    crossing = float(np.sum(first.durations[:split_gate]))
    handoff = pvaj(first, crossing)
    second_result, second, second_seconds = solve_free(core, handoff, goal, list(range(split_gate, 10)))
    executed = prefix_pieces(first, split_gate) + [second]
    flight = float(sum(piece.total_time for piece in executed)); pen = penalty(core, executed)
    return {"split_after_segment": split_segment, "split_after": f"G{split_gate}",
            "flight_time_s": flight, "sampled_penalty": pen,
            "sampled_total": flight + pen, "planning_seconds": first_seconds + second_seconds,
            "first_stage": {"planning_seconds": first_seconds, "status": first_result["status"],
                            "evaluations": first_result["evaluations"]},
            "second_stage": {"planning_seconds": second_seconds, "status": second_result["status"],
                             "evaluations": second_result["evaluations"]}}


def node_state(node: int) -> np.ndarray:
    """Fixed PVAJ terminal anchor at a gate, or at the real goal."""
    state = np.zeros((3, 4))
    state[:, 0] = GOAL if node == len(CENTERS_XY) + 1 else np.array((*CENTERS_XY[node - 1], CENTER_Z[node - 1]))
    return state


def local_self_pvaj_plan(core: AnalyticTOGTCore, head: np.ndarray, node: int, length: int, warm=None,
                         *, dynamic=False, time_offset=0.):
    """Plan up to ``length`` executable segments plus one terminal anchor."""
    final_node = len(CENTERS_XY) + 1
    execute = min(length, final_node - node)
    anchor = min(final_node, node + execute + 1)
    # Gate node q maps to zero-based index q-1.  The anchor itself is a fixed
    # endpoint, not an optimised crossing point; all prior gates are free.
    gates = list(range(node, min(len(CENTERS_XY), anchor - 1)))
    result, trajectory, seconds = solve_free(core, head, node_state(anchor), gates, warm,
                                             dynamic=dynamic, time_offset=time_offset)
    crossing = float(np.sum(trajectory.durations[:execute]))
    return result, trajectory, seconds, execute, pvaj(trajectory, crossing), anchor, shift_warm(result["_warm"], execute)


def fixed_blocks(core: AnalyticTOGTCore, length: int, use_warm_start=False, *, dynamic=False):
    """Execute ``length`` self-planned segments before each replan."""
    head = np.zeros((3, 4)); head[:, 0] = START
    node = 0; pieces = []; rows = []; elapsed = 0.; warm = None; absolute_time = 0.
    while node < len(CENTERS_XY) + 1:
        result, local, seconds, execute, next_head, anchor, next_warm = local_self_pvaj_plan(
            core, head, node, length, warm, dynamic=dynamic, time_offset=absolute_time)
        pieces += prefix_pieces(local, execute); elapsed += seconds
        rows.append({"start_node": node, "executed_segments": execute, "anchor_node": anchor,
                     "status": result["status"], "evaluations": result["evaluations"],
                     "planning_seconds": seconds, "final_cost": result["final_cost_check"],
                     "final_gradient_inf_norm": result["final_gradient_inf_norm"]})
        absolute_time += float(np.sum(local.durations[:execute]))
        node += execute; head = next_head; warm = next_warm if use_warm_start else None
    flight = float(sum(piece.total_time for piece in pieces)); pen = penalty(core, pieces)
    return {"block_length_segments": length, "flight_time_s": flight, "sampled_penalty": pen,
            "sampled_total": flight + pen, "planning_seconds": elapsed,
            "statuses": [row["status"] for row in rows], "rows": rows,
            "dynamics": dynamics_audit(core, pieces)}


def self_pvaj_rolling(core: AnalyticTOGTCore, length: int, use_warm_start=False, *, dynamic=False):
    """Same local window as fixed_blocks, but execute exactly one segment."""
    head = np.zeros((3, 4)); head[:, 0] = START
    node = 0; pieces = []; rows = []; elapsed = 0.; warm = None; absolute_time = 0.
    while node < len(CENTERS_XY) + 1:
        result, local, seconds, _, _, anchor, next_warm = local_self_pvaj_plan(
            core, head, node, length, warm, dynamic=dynamic, time_offset=absolute_time)
        next_head = pvaj(local, float(local.durations[0]))
        pieces += prefix_pieces(local, 1); elapsed += seconds
        rows.append({"start_node": node, "executed_segments": 1, "anchor_node": anchor,
                     "status": result["status"], "evaluations": result["evaluations"],
                     "planning_seconds": seconds, "final_cost": result["final_cost_check"],
                     "final_gradient_inf_norm": result["final_gradient_inf_norm"]})
        absolute_time += float(local.durations[0])
        node += 1; head = next_head; warm = next_warm if use_warm_start else None
    flight = float(sum(piece.total_time for piece in pieces)); pen = penalty(core, pieces)
    return {"rolling_length_segments": length, "flight_time_s": flight, "sampled_penalty": pen,
            "sampled_total": flight + pen, "planning_seconds": elapsed,
            "statuses": [row["status"] for row in rows], "rows": rows,
            "dynamics": dynamics_audit(core, pieces)}


def main():
    global SCALE, START, GOAL, CENTERS_XY, CENTER_Z, GATE_YAW, PAPER_LAYOUT
    parser = argparse.ArgumentParser()
    parser.add_argument("--rolling-only", action="store_true")
    parser.add_argument("--two-stage-only", action="store_true")
    parser.add_argument("--block-rolling-only", action="store_true")
    parser.add_argument("--blocks-only", action="store_true")
    parser.add_argument("--rolling-start", type=int, default=1)
    parser.add_argument("--rolling-end", type=int, default=11)
    parser.add_argument("--warm-block-rolling", action="store_true")
    parser.add_argument("--warm-blocks-only", action="store_true")
    parser.add_argument("--warm-rolling-start", type=int)
    parser.add_argument("--warm-rolling-end", type=int)
    parser.add_argument("--penalty-scale", type=float, default=SCALE)
    parser.add_argument("--dynamic", action="store_true")
    parser.add_argument("--paper-layout", action="store_true")
    parser.add_argument("--paper50", action="store_true",
                        help="50 ordered static traversals of the UZH seven-gate circuit; start equals goal")
    args = parser.parse_args()
    SCALE = float(args.penalty_scale)
    if not np.isfinite(SCALE) or SCALE <= 0.:
        parser.error("--penalty-scale must be positive and finite")
    if args.paper_layout or args.paper50:
        # First ten ordered rectangle gates from the official UZH 19-gate
        # TOGT resource.  This retains its 3-D placement and yaw sequence.
        cycle_xy = np.array(((-1.1,-1.6),(9.2,6.6),(9.2,-4.0),(-4.5,-6.0),(-4.5,-6.0),
                             (4.75,-.9),(-2.8,6.8)))
        cycle_z = np.array((3.6,1.0,1.2,3.5,.8,1.2,1.2))
        cycle_yaw = np.deg2rad((0.,-20.,-130.,180.,0.,70.,200.))
        if args.paper50:
            # The UZH 19-gate resource itself repeats this seven-gate circuit.
            # Seven laps plus Gate 1 gives 50 ordered traversals and a closed task.
            repeat = np.arange(50) % 7
            CENTERS_XY = cycle_xy[repeat]; CENTER_Z = cycle_z[repeat]; GATE_YAW = cycle_yaw[repeat]
            START = np.array((-5.,4.5,1.2)); GOAL = START.copy()
        else:
            take = np.arange(10) % 7
            CENTERS_XY = cycle_xy[take]; CENTER_Z = cycle_z[take]; GATE_YAW = cycle_yaw[take]
            START = np.array((-5.,4.5,1.2)); GOAL = np.array((4.75,-.9,1.2))
        PAPER_LAYOUT = True
    scale_tag = ("" if SCALE == 100. else f"_scale{SCALE:g}") + ("_dynamic" if args.dynamic else "") + ("_paper19g50" if args.paper50 else "_paper19g10" if args.paper_layout else "")
    def result_path(stem: str) -> Path:
        return ROOT / f"{stem}{scale_tag}.json"
    ROOT.mkdir(parents=True, exist_ok=True)
    core = AnalyticTOGTCore(penalty_scale=SCALE)
    head = np.zeros((3, 4)); head[:, 0] = START
    tail = np.zeros((3, 4)); tail[:, 0] = GOAL
    global_result, global_free, global_seconds = solve_free(core, head, tail, list(range(len(CENTERS_XY))), dynamic=args.dynamic)
    refs = reference_states(global_free)
    output = {"protocol": {"penalty_scale": SCALE, "rel_cost_tolerance": 1e-5,
        "rel_grad_tolerance": 1e-5, "audit": "1 ms released-penalty samples × scale",
        "state_reference": "free-crossing global TOGT PVAJ at horizon/block endpoints",
        "final_leg": "included", "dynamic_windows": args.dynamic,
        "paper_layout": args.paper_layout,
        "motion": None if not args.dynamic else {"translation_y_amplitude_m": .65,
            "yaw_amplitude_deg": 20., "period_s": 5., "per_gate_phase": "0.73*i-1.1"}},
        "rolling": {}, "state_continuous_chunks": {}}
    output["global_free"] = {"flight_time_s": float(global_free.total_time),
        "objective": global_result["cost"], "planning_seconds": global_seconds,
        "status": global_result["status"], "dynamics": dynamics_audit(core, [global_free])}
    if args.blocks_only:
        output["fixed_blocks"] = {}
        for length in range(1, 12):
            output["fixed_blocks"][str(length)] = fixed_blocks(core, length, dynamic=args.dynamic)
            print(f"blocks length={length} done", flush=True)
        result_path("self_pvaj_fixed_blocks_1to11").write_text(
            json.dumps(output, indent=2) + "\n", encoding="utf-8")
        return
    if args.warm_blocks_only:
        output["fixed_blocks"] = {str(length): fixed_blocks(core, length, True, dynamic=args.dynamic) for length in range(1, 12)}
        target = result_path("self_pvaj_warm_fixed_blocks_1to11")
        target.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(target)
        return
    if args.warm_rolling_start is not None:
        begin = args.warm_rolling_start; end = args.warm_rolling_end or begin
        output["rolling"] = {str(length): self_pvaj_rolling(core, length, True, dynamic=args.dynamic) for length in range(begin, end + 1)}
        target = result_path(f"self_pvaj_warm_rolling_{begin}to{end}")
        target.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(target)
        return
    if args.warm_block_rolling:
        output["fixed_blocks"] = {}; output["rolling"] = {}
        for length in range(1, 12):
            output["fixed_blocks"][str(length)] = fixed_blocks(core, length, True, dynamic=args.dynamic)
            print(f"warm blocks length={length} done", flush=True)
        for length in range(1, 12):
            output["rolling"][str(length)] = self_pvaj_rolling(core, length, True, dynamic=args.dynamic)
            print(f"warm rolling length={length} done", flush=True)
        target = result_path("self_pvaj_warm_fixed_block_vs_rolling_1to11")
        target.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(target)
        return
    if args.block_rolling_only:
        output["fixed_blocks"] = {}
        output["rolling"] = {}
        for length in range(1, 12):
            output["fixed_blocks"][str(length)] = fixed_blocks(core, length, dynamic=args.dynamic)
            print(f"blocks length={length} done", flush=True)
        for length in range(1, 12):
            output["rolling"][str(length)] = self_pvaj_rolling(core, length, dynamic=args.dynamic)
            print(f"rolling length={length} done", flush=True)
        target = result_path("self_pvaj_fixed_block_vs_rolling_1to11")
        target.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(target)
        return
    if args.rolling_start != 1 or args.rolling_end != 11 or SCALE != 100.:
        output["rolling"] = {}
        for length in range(args.rolling_start, args.rolling_end + 1):
            output["rolling"][str(length)] = self_pvaj_rolling(core, length, dynamic=args.dynamic)
            print(f"rolling length={length} done", flush=True)
        target = result_path(f"self_pvaj_rolling_{args.rolling_start}to{args.rolling_end}")
        target.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(target)
        return
    if args.two_stage_only:
        output["two_stage_self_pvaj"] = {}
        for m in range(1, 12):
            output["two_stage_self_pvaj"][str(m)] = two_stage_split(core, m)
            print(f"two-stage split m={m} done", flush=True)
        OUT.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(OUT)
        return
    for h in range(1, 11):
        output["rolling"][str(h)] = rolling(core, h)
        print(f"rolling h={h} done", flush=True)
    if args.rolling_only:
        OUT.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
        print(OUT)
        return
    for n in range(1, 11):
        output["state_continuous_chunks"][str(n)] = chunks(core, refs, n)
        print(f"chunks n={n} done", flush=True)
    output["exact_shared_partitions"] = {
        str(n): exact_shared_partition(core, global_free, n, global_seconds) for n in range(1, 11)
    }
    OUT.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(OUT)


if __name__ == "__main__":
    main()
