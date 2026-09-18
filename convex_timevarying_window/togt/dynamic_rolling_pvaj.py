#!/usr/bin/env python3
"""True time-varying rolling-horizon TOGT with continuous executed PVAJ.

The local horizon evaluates every periodic gate at the predicted absolute
crossing time.  It is intentionally separate from the static native race
runner: treating a dynamic gate as frozen would invalidate this experiment.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import numpy as np

from convex_timevarying_window.geometry import PeriodicConvexWindow
from convex_timevarying_window.togt.experiment import build_track
from convex_timevarying_window.togt.native_objective import (
    DynamicRightTailObjective, NativeJointTOGTObjective,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from nonconvex_timevarying_window.sc_dynatogt.optimizer import _minimize_togt_lbfgs


def _state_at(trajectory, t: float) -> BoundaryState:
    return BoundaryState(*[np.asarray(trajectory.evaluate(t, derivative=d), dtype=float)
                           for d in range(4)])


class _ClockShiftWindow:
    """Adapter allowing the joint terminal solver to use an absolute clock."""
    def __init__(self, window, offset): self.window, self.offset = window, float(offset)
    @property
    def aperture(self): return self.window.aperture
    def point_and_jacobians(self, d, time):
        return self.window.point_and_jacobians(d, float(time) + self.offset)
    def to_point(self, d, time): return self.window.to_point(d, float(time) + self.offset)


def build_heterogeneous_50():
    """50 traversals of the UZH layout, each with distinct strong motion."""
    base, config = build_track()
    windows = []
    for i in range(50):
        source = base.windows[i % len(base.windows)]
        # 0.25--0.55 m translations and 15--45 degree RPY oscillations;
        # frequencies/phases are deliberately not shared between windows.
        a = 0.25 + 0.06 * (i % 6)
        motion = MotionProfile(
            translation_amplitude=np.array([a, 0.22 + .05*((3*i) % 7), .16 + .04*((5*i) % 5)]),
            rotation_amplitude=np.deg2rad(np.array([15 + 5*(i % 5), 18 + 4*((2*i) % 6), 20 + 5*((4*i) % 6)])),
            scale_amplitude=0.0,
            # Fast enough to make timing matter, but not so fast that the
            # optimizer can exploit repeated opening cycles by waiting.
            translation_period=9.5 + .47*(i % 11),
            rotation_period=8.0 + .43*((3*i) % 13), scale_period=1.0,
            phase=-1.3 + .37*i, scale_enabled=False,
        )
        windows.append(PeriodicConvexWindow(
            name=f"{source.name}_lap{i//7 + 1}", aperture=source.aperture,
            center0=source.center0, angles0_rpy=source.angles0_rpy, motion=motion))
    return SCWindowTrack("heterogeneous_dynamic_uzh_50g_closed", base.start, base.goal,
                        tuple(windows), tuple(range(50))), config


def _terminal_problem(track, head, absolute_time, config):
    shifted = tuple(_ClockShiftWindow(w, absolute_time) for w in track.windows)
    local = SCWindowTrack("terminal", head.position, track.goal, shifted,
                          tuple(range(len(shifted))))
    objective = NativeJointTOGTObjective(local, config)
    # NativeJoint has zero PVAJ hard-coded at endpoints.  Preserve the actual
    # left state by overriding only its C++ head boundary.
    objective.head = head.matrix.T.copy()
    return objective


def solve(track, config, horizon: int):
    head = BoundaryState(track.start)
    absolute_time = 0.0
    index = 0
    executed_time = 0.0
    blocks = []
    tail_seed = np.zeros((3, 4))
    while len(track.windows) - index > horizon:
        objective = DynamicRightTailObjective(track.windows[index:index+horizon], head,
                                              absolute_time, config, tail_seed)
        x0 = objective.initial_guess()
        started = time.perf_counter()
        result = _minimize_togt_lbfgs(objective.scipy_value_and_gradient, x0, config)
        cpu = time.perf_counter() - started
        if not result.success:
            raise RuntimeError(f"block {index + 1} did not converge: {result.message}")
        forward = objective.forward(result.x)
        duration = float(forward.durations[0])
        head = _state_at(forward.trajectory, duration)
        absolute_time += duration; executed_time += duration
        tail_seed = np.asarray(forward.trajectory.evaluate(forward.trajectory.total_time, 0), dtype=float)
        # Retain all four predicted terminal derivatives as the next soft seed.
        tail_seed = BoundaryState(*[forward.trajectory.evaluate(forward.trajectory.total_time, d)
                                    for d in range(4)]).matrix.T
        blocks.append({"first_gate": index + 1, "kind": "rolling", "cpu_seconds": cpu,
                       "executed_duration": duration, "horizon_cost": float(result.fun),
                       "iterations": int(result.nit), "evaluations": int(result.nfev)})
        index += 1

    # The terminal block is joint TOGT over all remaining gates plus the true
    # return-to-start segment, with the executed left PVAJ fixed exactly.
    tail_track = SCWindowTrack("tail", head.position, track.goal,
                               track.windows[index:], tuple(range(len(track.windows)-index)))
    objective = _terminal_problem(tail_track, head, absolute_time, config)
    started = time.perf_counter(); result = _minimize_togt_lbfgs(
        objective.scipy_value_and_gradient, objective.initial_guess(), config)
    cpu = time.perf_counter() - started
    if not result.success:
        raise RuntimeError(f"terminal block {index + 1} did not converge: {result.message}")
    forward = objective.forward(result.x)
    executed_time += float(forward.trajectory.total_time)
    blocks.append({"first_gate": index + 1, "kind": "terminal", "cpu_seconds": cpu,
                   "executed_duration": float(forward.trajectory.total_time), "horizon_cost": float(result.fun),
                   "iterations": int(result.nit), "evaluations": int(result.nfev)})
    return {"flight_time_seconds": executed_time, "blocks": blocks,
            "planning_seconds": float(sum(b["cpu_seconds"] for b in blocks)),
            "block_count": len(blocks), "horizon": horizon}


def main(argv=None):
    p = argparse.ArgumentParser(); p.add_argument("--horizon", type=int, default=3)
    p.add_argument("--out", type=Path, required=True); args = p.parse_args(argv)
    track, config = build_heterogeneous_50()
    output = solve(track, config, args.horizon)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(output, indent=2))

if __name__ == "__main__": main()
