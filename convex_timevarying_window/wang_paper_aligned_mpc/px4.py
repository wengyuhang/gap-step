"""Independent MINCO adapter; unchanged condensed MPC and PX4 session."""
import argparse
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import subprocess
import threading
import time

import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import (
    CONTAINER, CONTACT_TOPICS, PARTITION, set_parameter)
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import parse_contacts
from .planner import Config as PlannerConfig
from convex_timevarying_window.dynamic_gate_mppi_v3.ctbr_probe import Session
from .events import Progress, planning_block_reason
from convex_timevarying_window.wang_center_seed_mpc.reference import Reference
from .worker import Worker
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from convex_timevarying_window.checked_soft_mpc_racing.mpc import MPC, Config

ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = {'MPC_XY_VEL_MAX':14., 'MPC_XY_CRUISE':12., 'MPC_Z_VEL_MAX_UP':7.,
              'MPC_Z_VEL_MAX_DN':7., 'MPC_ACC_HOR':10., 'MPC_ACC_HOR_MAX':12.,
              'MPC_ACC_UP_MAX':9., 'MPC_ACC_DOWN_MAX':9., 'MPC_JERK_AUTO':35.,
              'MPC_JERK_MAX':45., 'MPC_TILTMAX_AIR':45., 'MPC_XY_P':2.,
              'MPC_XY_VEL_P_ACC':3.5, 'MPC_Z_P':2., 'SENS_GPS0_DELAY':0., 'SENS_GPS1_DELAY':0.}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--outdir', type=Path, required=True)
    p.add_argument('--motion-rate', type=float, default=10.)
    p.add_argument('--translation-scale', type=float, default=4.)
    p.add_argument('--rotation-scale', type=float, default=1.)
    p.add_argument('--phase', type=float, default=2*np.pi/3)
    p.add_argument('--layout', type=float, default=0.)
    p.add_argument('--max-flight-time', type=float, default=60.)
    p.add_argument('--center-only', action='store_true', help='Matched fixed-center ablation')
    p.add_argument('--cold-state', action='store_true', help='Free-point global-reference initialization ablation')
    p.add_argument('--warm-center', action='store_true', help='Center-only matched warm-start baseline')
    p.add_argument('--condensed', action='store_true', help='Exactly eliminate linearized MPC states')
    a = p.parse_args(); out = a.outdir.resolve()
    track, _ = build_track()
    vary_track(track, a.phase, a.layout, a.motion_rate, a.translation_scale, a.rotation_scale)
    cfg = replace(PlannerConfig(), free_points=not a.center_only, state_warm_start=a.warm_center or not (a.center_only or a.cold_state), horizon=1, workspace_lower=(-28.,-30.,.65), workspace_upper=(28.,22.,12.))
    began = time.perf_counter(); worker = Worker(track, cfg); build_s = time.perf_counter()-began
    planner = worker.geometry
    progress = Progress(planner.windows, planner.signs, cfg.radius, cfg.frame_thickness)
    controller_type = MPC
    if a.condensed:
        from .mpc import MPC as controller_type
    began = time.perf_counter(); mpc = controller_type(); mpc_build_s = time.perf_counter()-began
    # No first-use allocation/factorization during takeoff/racing; construction
    # and one hover warmup are recorded separately, not hidden as online time.
    from convex_timevarying_window.checked_soft_mpc_racing.mpc import hover_state
    ref = Reference(track.start)
    mpc.command(hover_state(track.start), -1., ref.sample)
    warmup_ms = mpc.logs[-1]['cycle_ms']; mpc.logs.clear(); mpc.controls = None; mpc.last_time = None
    try: session = Session(out, track, consistent_velocity=False)
    except Exception:
        worker.close()
        raise
    processes, contact_paths = [], []
    for name, topic in [('barometer', '/world/convex_seven_dynamic_px4_togt/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure'),
                        ('truth', '/model/x500_0/odometry_with_covariance')]+[(f'gate_{i+1:02d}_contacts', t) for i, t in enumerate(CONTACT_TOPICS)]:
        f = (out/f'{name}.pbtxt').open('w') if name != 'truth' else None
        proc = subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,
                                 'gz','topic','-e','-t',topic], stdout=f or subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        processes.append((proc, f))
        if 'contacts' in name: contact_paths.append(out/f'{name}.pbtxt')
    for key, value in PARAMETERS.items(): set_parameter(key, value)
    paths = []
    for directory in ('wang_paper_aligned_mpc', 'wang_center_seed_mpc', 'wang_free_state_mpc', 'togt_minco_mpc', 'checked_soft_mpc_isolated', 'checked_soft_mpc_racing', 'checked_soft_racing', 'continuation_racing', 'receding_gate_racing'):
        paths.extend((ROOT/'convex_timevarying_window'/directory).glob('*.py'))
    paths.extend((ROOT/'convex_timevarying_window/checked_soft_mpc_isolated/fast').glob('*.py'))
    paths.extend((ROOT/'复现/polynomial_dynamic_racing_2023').glob('*.py'))
    paths += [ROOT/'convex_timevarying_window/dynamic_gate_mppi_v3/ctbr_probe.py',
              ROOT/'convex_timevarying_window/dynamic_gate_mppi_v3/ctbr.py',
              ROOT/'复现/waypoint_mppi/gp_px4.py']
    sources = {str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest() for f in paths}
    import shutil
    for f in paths:
        dst = out/'source_snapshot'/f.relative_to(ROOT); dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(f, dst)
    rows, solves = [], []
    crossings = progress.crossings
    planning_skips = {}
    trace = (out/'telemetry_live.csv').open('w'); writer = csv.writer(trace)
    header = ['t','x','y','z','vx','vy','vz','ax','ay','az','rx','ry','rz','gate','error','state_time']
    writer.writerow(header)
    solve_log = (out/'solves_live.jsonl').open('w'); control_log = (out/'mpc_live.jsonl').open('w')
    gate = 0; next_request = 0.; last_row = -1.; now = 0.; finished = False; failure = None; completed = None
    acceleration = np.zeros(3); last_v = np.zeros(3); velocity_t = None
    state = np.r_[track.start, np.zeros(6)]

    def feedback(x, stamp):
        if mpc.last_time is None or stamp-mpc.last_time >= mpc.cfg.update_period-1e-6:
            started = time.perf_counter()
            try: command = mpc.command(x, stamp, ref.sample)
            except Exception as exc:
                failed = dict(time=stamp, cycle_ms=1000*(time.perf_counter()-started), failure=repr(exc))
                mpc.logs.append(failed); control_log.write(json.dumps(failed)+'\n'); control_log.flush()
                raise
            control_log.write(json.dumps(mpc.logs[-1])+'\n'); control_log.flush()
            return command
        return mpc.previous.copy()

    def record(row):
        rows.append(row); writer.writerow(row); trace.flush()

    def handover_before_start():
        # PX4 position -> body-rate control has a transient: old attitude-loop
        # setpoints can still be published during the mode update. Complete
        # this handover in the EXISTING 30 s preparation, while hovering, not
        # simultaneously with the first accelerating racing reference.
        while not session.stop and session.read()[0] < -2.:
            time.sleep(.01)
        if not session.stop and session.armed:
            session.feedback = feedback

    handover = threading.Thread(target=handover_before_start, daemon=True)
    try:
        handover.start()
        session.prepare(); session.feedback = feedback
        while session.clock.value < 30.+a.max_flight_time:
            now = session.clock.value-30.
            stamp, x, age = session.read()
            if session.failure: raise RuntimeError(session.failure)
            if age > .1: raise RuntimeError('stale state in planner')
            pos, vel = x[:3], x[3:6]
            if velocity_t is not None and now-velocity_t >= .018:
                raw = (vel-last_v)/(now-velocity_t)
                raw *= min(1., cfg.acceleration/max(np.linalg.norm(raw), 1e-9))
                acceleration = .8*acceleration+.2*raw; last_v = vel.copy(); velocity_t = now
            elif velocity_t is None: last_v = vel.copy(); velocity_t = now
            state = np.r_[pos, vel, acceleration]
            if not np.isfinite(state).all() or np.linalg.norm(vel) > 4*cfg.speed:
                raise RuntimeError('simulation_state_diverged')
            exited = progress.observe(pos, stamp)
            if progress.violation: raise RuntimeError(progress.violation)
            gate = progress.gate
            active = ref.advance(now, gate, progress.phase)
            if exited:
                next_request = now
                print('MPC body exit', gate, round(now, 3), flush=True)
            response = worker.poll()
            if response is not None:
                candidate, clearance, elapsed, error = response
                decision_started = time.perf_counter()
                reason = error if candidate is None else None
                if candidate is not None:
                    if candidate.first_gate != gate: reason = 'stale_gate'
                    elif progress.crossed: reason = 'body_exiting'
                    elif candidate.violation >= .003: reason = 'optimizer_residual'
                    elif not candidate.event_valid: reason = candidate.event_reason
                    elif clearance < cfg.tracking_margin: reason = 'geometry_or_dynamics'
                    elif now > candidate.made_at-.05: reason = 'late_result'
                    else:
                        reason = planning_block_reason(progress, ref, candidate.made_at, cfg.min_leg_seconds)
                        if reason is None: reason = ref.candidate_reason(candidate, gate, progress.phase)
                elapsed += 1000*(time.perf_counter()-decision_started)
                if elapsed >= 100.: reason = 'deadline'
                accepted = candidate is not None and reason is None
                if accepted: accepted = ref.install(candidate)
                entry = dict(time=now, cycle_ms=elapsed, accepted=accepted, gate=gate, observed_gate=gate,
                             phase=progress.phase, rejection_reason=reason,
                             violation=candidate.violation if candidate else None,
                             status=candidate.status if candidate else error, prefix_clearance=clearance)
                if candidate is not None:
                    entry.update(candidate_first_gate=candidate.first_gate, event_valid=candidate.event_valid,
                                 event_reason=candidate.event_reason)
                    entry.update(made_at=candidate.made_at, durations=candidate.durations.tolist(), variables=candidate.x.tolist(),
                                 gate_count=candidate.gate_count, terminal_goal=candidate.terminal_goal,
                                 pieces_per_leg=candidate.pieces_per_leg, seed_provenance=candidate.seed_provenance,
                                 initial_variables=candidate.initial_variables.tolist(), objective=candidate.objective)
                    if candidate.violation < .003:
                        entry['coefficients'] = candidate.coefficients.tolist()
                        if hasattr(candidate, 'brake'):
                            entry['brake'] = dict(made_at=candidate.brake.made_at, durations=candidate.brake.durations.tolist(),
                                                  coefficients=candidate.brake.coefficients.tolist(), clearance=candidate.clearance)
                entry.update(worker.metrics[-1]); entry['cycle_ms'] = elapsed
                solves.append(entry); solve_log.write(json.dumps(entry)+'\n'); solve_log.flush()
            if now >= next_request and not worker.busy and gate < len(track.windows):
                target_time = now+.15
                blocked = planning_block_reason(progress, ref, target_time, cfg.min_leg_seconds)
                if blocked:
                    planning_skips[blocked] = planning_skips.get(blocked, 0)+1
                    next_request = now+.02
                else:
                    future, incumbent = ref.state_at(target_time)
                    if incumbent is None:
                        future = state.copy(); future[:3] += vel*.15+.5*acceleration*.15**2
                        future[3:6] += acceleration*.15; future = np.r_[future, np.zeros(3)]
                    worker.commit(incumbent)
                    worker.submit(future, target_time, gate); next_request = now+cfg.replan_period
            desired = active.sample(now) if active is not None else track.start
            row = [now, *pos, *vel, *acceleration, *desired, gate, float(np.linalg.norm(pos-desired)), stamp]
            if now-last_row >= .018: record(row); last_row = now
            if gate == len(track.windows) and np.linalg.norm(pos-track.goal) < .55 and np.linalg.norm(vel) < 1.:
                record(row); finished = True; completed = now; break
            time.sleep(.001)
        if not finished: failure = 'timeout'
    except Exception as exc: failure = repr(exc)
    finally:
        session.feedback = None; session.hold = state[:3].copy()
        worker.close()
        session.close()
        for proc, f in processes:
            proc.terminate()
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired: proc.kill()
            if f: f.close()
        trace.close(); solve_log.close(); control_log.close()
    contacts = [parse_contacts(path) for path in contact_paths]
    with (out/'telemetry.csv').open('w') as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)
    result = dict(method=('paper_aligned_free_warm_mpc' if cfg.state_warm_start else 'paper_aligned_free_global_mpc') if cfg.free_points else ('paper_aligned_wang_center_warm_mpc' if cfg.state_warm_start else 'paper_aligned_wang_center_mpc'), condensed=a.condensed, planner_process=worker.startup, planner_timing=worker.metrics, completed=finished, failure=failure,
        flight_time_s=completed, last_time_s=now, collision_detected=any(c['contact_pair_count'] > 0 for c in contacts),
        contacts=contacts, crossings=crossings, body_exits=progress.exits, progress_violation=progress.violation,
        pending_discarded=ref.discarded, planning_skips=planning_skips,
        progress_clock='Fresh LOCAL_POSITION_NED measurement timestamp; gate queried at the same timestamp',
        solves=solves, config=asdict(cfg), parameters=PARAMETERS,
        build_s=build_s, mpc_build_s=mpc_build_s, mpc_warmup_ms=warmup_ms, mpc_config=asdict(mpc.cfg),
        mpc_cycles=mpc.logs, sender_failure=session.failure, source_sha256=sources,
        phase=a.phase, layout=a.layout, motion_rate=a.motion_rate, translation_scale=a.translation_scale,
        rotation_scale=a.rotation_scale, numpy_version=np.__version__, origin_enu_m=session.origin.tolist(),
        checked=True, checked_soft=False, continuation=False, local_stop_terminal=False,
        center_initialized=True, state_warm_start=cfg.state_warm_start, N=2, M=4, passage_deadlines=False, tracking_preview_s=0., planning_lead_s=.15,
        goal_error_m=float(np.linalg.norm(state[:3]-track.goal)), goal_speed_mps=float(np.linalg.norm(state[3:6])),
        evidence='PX4 CTBR NMPC tracking plus Gazebo contacts and independent truth audit, not a safety certificate')
    (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('solves','mpc_cycles','source_sha256')}, indent=2), flush=True)


if __name__ == '__main__': main()
