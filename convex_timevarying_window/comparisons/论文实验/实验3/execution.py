"""Frozen-reference CTBR execution through the full terminal reference clock."""
import argparse
import csv
from dataclasses import asdict, replace
import json
from pathlib import Path
import shutil
import subprocess
import threading
import time

import numpy as np
from pyulog import ULog

from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state
from convex_timevarying_window.dynamic_gate_mppi_v3 import ctbr_probe
from convex_timevarying_window.global_gate_minco_probe.flight import Progress
from convex_timevarying_window.simple_two_gate_soft_only_mpc.px4 import PARAMETERS
from convex_timevarying_window.single_gate_stop_racing.planner import Config, route_signs
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import set_parameter
from . import scenario
from . import scenario as world
from .session import RecordedSession, CONTAINER, PARTITION, launch
from ..实验2.gazebo_matrix.reference import MPC, Reference
from .audit import audit
from .planning import frozen_plan


def reference_finished(clock_time, measurement_time, reference_duration):
    return bool(clock_time >= reference_duration and measurement_time >= reference_duration)


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def execute(source, variant, out):
    raw_plan = json.loads(source.read_text())
    if not raw_plan['found']:
        raise ValueError('A selected trajectory is required')
    plan = frozen_plan(raw_plan)
    duration = float(plan.durations.sum())
    case = scenario.Case(**raw_plan['case'])
    track, _ = scenario.build_case(case)
    cfg = replace(Config(), horizon=1, radius=.3859447197013662,
        workspace_lower=(-100., -120., .65), workspace_upper=(150., 120., 20.))
    progress = Progress(list(track.windows), route_signs(track), cfg.radius, cfg.frame_thickness)
    reference = Reference(track.start, plan)
    mpc = MPC()
    mpc.command(hover_state(track.start), -1., reference.sample)
    warmup_ms = mpc.logs[-1]['cycle_ms']
    mpc.logs.clear()
    mpc.controls, mpc.last_time = None, None
    ctbr_probe.launch_variant = launch
    session = RecordedSession(out, track, consistent_velocity=False)
    mpc.bind_session(session)
    recorders = []
    for i, topic in enumerate(world.contact_topics(track), 1):
        stream = (out/f'gate_{i:02d}_contacts.pbtxt').open('w')
        process = subprocess.Popen(['docker', 'exec', '-e', 'GZ_PARTITION='+PARTITION,
            CONTAINER, 'gz', 'topic', '-e', '-t', topic], stdout=stream, stderr=subprocess.DEVNULL)
        recorders.append((process, stream))
    for key, value in PARAMETERS.items():
        set_parameter(key, value)
    header = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'rx', 'ry', 'rz', 'gate', 'error', 'state_time']
    rows = []
    control_log = (out/'mpc_live.jsonl').open('w')
    telemetry = (out/'telemetry_live.csv').open('w')
    writer = csv.writer(telemetry)
    writer.writerow(header)
    now, stamp, last_record = 0., 0., -1.
    completed, failure = None, None
    state = hover_state(track.start)

    def feedback(x, at):
        if mpc.last_time is None or at-mpc.last_time >= mpc.cfg.update_period-1e-6:
            command = mpc.command(x, at, reference.sample)
            control_log.write(json.dumps(mpc.logs[-1])+'\n')
            control_log.flush()
            return command
        return mpc.previous.copy()

    def handover():
        while not session.stop and session.read()[0] < -2.:
            time.sleep(.01)
        if not session.stop and session.armed:
            session.feedback = feedback

    def record(row):
        rows.append(row)
        writer.writerow(row)
        telemetry.flush()

    began = time.perf_counter()
    try:
        threading.Thread(target=handover, daemon=True).start()
        session.prepare()
        session.feedback = feedback
        while session.clock.value < 210.:
            now = session.clock.value-30.
            stamp, state, age = session.read()
            if session.failure:
                raise RuntimeError(session.failure)
            if age > .1 or not np.isfinite(state).all():
                raise RuntimeError('stale_or_nonfinite_state')
            exited = progress.observe(state[:3], stamp)
            if progress.violation == 'missed_current_opening':
                raise RuntimeError('missed_opening_no_reverse_reentry')
            reference.advance(now, progress.gate, progress.phase)
            desired = plan.sample(now)
            row = [now, *state[:6], 0., 0., 0., *desired, progress.gate,
                float(np.linalg.norm(state[:3]-desired)), stamp]
            if now-last_record >= .018:
                record(row)
                last_record = now
            if exited:
                print('BODY_EXIT', progress.gate, now, flush=True)
            if reference_finished(now, stamp, duration):
                record(row)
                completed = now
                break
            time.sleep(.001)
        if completed is None:
            failure = 'timeout'
    except Exception as exc:
        failure = repr(exc)
    finally:
        session.feedback = None
        session.hold = state[:3].copy()
        session.close()
        for process, stream in recorders:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            stream.close()
        telemetry.close()
        control_log.close()
        subprocess.run(['docker', 'exec', '-e', 'GZ_PARTITION='+PARTITION, CONTAINER, 'gz', 'service',
            '-s', '/world/convex_seven_dynamic_px4_togt/control', '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', 'pause: true'], capture_output=True, timeout=5)
    with (out/'telemetry.csv').open('w') as stream:
        csv_writer = csv.writer(stream)
        csv_writer.writerow(header)
        csv_writer.writerows(rows)
    shutil.copyfile(source, out/'source_result.json')
    np.savez_compressed(out/'reference_trajectory.npz', durations=plan.durations, coefficients=plan.coefficients)
    save(out/'scene.json', scenario.manifest(case))
    contacts = world.parse_all_contacts(out, track, 30.+now)
    result = dict(experiment_case=asdict(case), experiment_variant=variant,
        config=asdict(cfg), planning_proxy_radius_m=raw_plan['config']['radius'],
        completed=completed is not None, failure=failure, flight_time_s=completed,
        last_time_s=now, finish_measurement_time_s=stamp, nominal_time_s=duration,
        finish_definition='actual_clock_and_state_timestamp_reach_full_reference_T',
        old_terminal_threshold_used=False, goal_error_m=float(np.linalg.norm(state[:3]-track.goal)),
        goal_speed_mps=float(np.linalg.norm(state[3:6])), contacts=contacts,
        parameters=PARAMETERS, mpc_cycles=mpc.logs, mpc_config=asdict(mpc.cfg), mpc_warmup_ms=warmup_ms,
        solves=[], planning_scheduler='none_frozen_reference', tracker_metadata=mpc.metadata,
        wall_execution_s=time.perf_counter()-began)
    save(out/'result.json', result)
    if (out/'flight.ulg').exists() and now > .01:
        print('AUDIT', json.dumps(audit(out), ensure_ascii=False), flush=True)
    else:
        raise RuntimeError('missing_flight_evidence')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--variant', choices=('ours', 'instant', 'center', 'togt_center'), required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    execute(args.source.resolve(), args.variant, args.outdir.resolve())


if __name__ == '__main__':
    main()
