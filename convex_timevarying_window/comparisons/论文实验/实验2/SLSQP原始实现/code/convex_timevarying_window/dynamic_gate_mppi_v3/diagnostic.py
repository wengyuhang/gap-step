"""Nominal-model diagnostic, explicitly NOT a PX4/Gazebo comparison result.

Model time is stepped after each computation; this does not demonstrate
real-time execution. Every wall-clock overrun is retained in the output.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
from scipy.spatial.transform import Rotation
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from .config import Config
from .controller import make_controller
from .geometry import GateProgress
from .reference import Planner, AsyncPlanner
from .rollout import hover_state
from .scheduler import Scheduler


def pvaj_from_cache(selection, offset):
    i = int(round(offset/.01))
    s = selection.states
    i = max(1, min(i, len(s)-2))
    acceleration = (s[i+1, 3:6]-s[i-1, 3:6])/.02
    jerk = (s[i+1, 3:6]-2*s[i, 3:6]+s[i-1, 3:6])/.0001
    return BoundaryState(s[i, :3], s[i, 3:6], acceleration, jerk)


def pvaj_from_state(rollout, state, action):
    """Measured P,V; model-estimated current A,J under the current motor input.

    Forward differences estimate derivatives only; the head is NOT advanced.
    """
    h = .001
    s1 = rollout.integrate(state, action, h)
    s2 = rollout.integrate(s1, action, h)
    a = (-3*state[3:6]+4*s1[3:6]-s2[3:6])/(2*h)
    j = (s2[3:6]-2*s1[3:6]+state[3:6])/(h*h)
    return BoundaryState(state[:3].copy(), state[3:6].copy(), a, j)


def run(args):
    out = args.outdir
    out.mkdir(parents=True, exist_ok=False)
    source_sha256 = {}
    for path in Path(__file__).parent.rglob("*"):
        if path.suffix not in (".py", ".cpp"): continue
        relative = path.relative_to(Path(__file__).parent)
        if any(x in relative.parts for x in ("results","build","install","log")): continue
        target = out / "source_snapshot" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        source_sha256[str(relative)] = hashlib.sha256(path.read_bytes()).hexdigest()
    track, opt = build_track()
    vary_track(track, args.phase, 0, args.motion_rate, args.translation_scale, args.rotation_scale)
    cfg = replace(Config(), temperature=args.temperature,samples=args.samples,horizon=args.horizon)
    controller = make_controller(track, cfg, args.seed,args.backend)
    planner = Planner(track, opt)
    state = hover_state(track.start)
    ref = planner.solve(BoundaryState(track.start), 0., 0)
    if ref is not None: controller.seed_reference(ref, planner.core)
    asynchronous = AsyncPlanner(track, opt)
    progress, scheduler = GateProgress(track, cfg), Scheduler(cfg)
    requests = []; rows = []; actions = []; failure = None; tracking_log = []
    now = 0.; last = (np.array([2.0643076923076924*9.8066,0,0,0]) if args.backend=='ctbr'
                     else np.full(4, 2.0643076923076924*9.8066/4))
    status = out / "status.json"
    started = time.perf_counter()
    next_tick = started
    try:
        while now < args.duration:
            if args.paced:
                remaining = next_tick-time.perf_counter()
                if remaining > 0: time.sleep(remaining)
                # Never accelerate the model clock to catch up after a slow
                # cycle: that falsely turns a 75 ms solve into 120 ms of
                # model time. Overruns remain in cycle_ms and wall_seconds.
                next_tick = time.perf_counter()+cfg.dt
            result = asynchronous.poll()
            if result is not None:
                identifier, candidate = result
                request = scheduler.running
                accepted = False; validation = {}
                if candidate is not None and not isinstance(candidate, dict) and request is not None:
                    errors = controller.tracking_error(state, now, candidate)
                    validation = dict(elapsed_sim=now-request.started, solve_ms=candidate.solve_ms,
                                      actual_gate=progress.first, **errors)
                    # PVAJ reference is not an attitude command; all controls
                    # still undergo full-state prediction and hard admission.
                    accepted = scheduler.finish(request, now, progress.first,
                        errors['position_error'], errors['velocity_error'], 0.)
                    if accepted: ref = candidate
                else:
                    scheduler.failed(request)
                requests.append({"time": now, "accepted": accepted,
                                 "reason":request.reason if request else None,
                                 "validation":validation,
                                 "error": candidate if isinstance(candidate, dict) else None})
            risk=controller.reference_risk(state, now, progress.first, ref,progress.crossing)
            if args.backend=='ctbr' and controller.reference_warning(state,now,progress.first,ref,progress.crossing):
                scheduler.flag_prediction_warning()
            errors=controller.tracking_error(state,now,ref)
            if errors['tracking_failed']:scheduler.flag_tracking_error()
            selection = controller.select(state, now, progress.first, progress.crossing,
                                           risk, ref, last)
            scheduler.set_risk(controller.last_diagnostics.get('avoidance',risk))
            if controller.last_diagnostics.get('replan_requested'):scheduler.flag_prediction_warning()
            tracking_log.append(dict(time=now, gate=progress.first, **errors,
                reference_risk=risk, reference_age=None if ref is None else now-ref.made_at,
                baseline_valid=controller.last_diagnostics.get('baseline_valid'),
                baseline_clearance=controller.last_diagnostics.get('baseline_clearance'),
                avoidance=scheduler.avoiding, reference_weight=0. if scheduler.avoiding or ref is None else 1.))
            if selection is not None:
                if args.backend=='ctbr':controller.publish(selection,state,now,progress.first)
                else:controller.publish(selection, state, now)
            action = controller.cached_action(state, now)
            if action is None:
                failure = "validated_cache_exhausted"; break
            if selection is not None and asynchronous.ready:
                request = scheduler.observe(now, now+np.arange(len(selection.clearances)+1)*.01,
                                            selection.clearances, not progress.computation_allowed(state[:3],now),
                                            progress.first)
                if request is not None:
                    asynchronous.submit(len(requests), pvaj_from_state(controller.rollout,state,action),
                                        request.anchor, request.first_gate)
            middle = controller.rollout.integrate(state, action, .01)
            new = controller.rollout.integrate(middle, action, .01)
            positions = np.array([state[:3], middle[:3], new[:3]])
            velocities = np.array([state[3:6], middle[3:6], new[3:6]])
            clear = controller.rollout.geometry.interval_clearances(positions, velocities,
                                                       np.array([now, now+.01, now+.02])).min()
            if progress.update(state[:3], now, middle[:3], now+.01, clear > 0):
                scheduler.gate_passed(progress.first)
            if progress.update(middle[:3], now+.01, new[:3], now+.02, clear > 0):
                scheduler.gate_passed(progress.first)
            rows.append([now, *state, progress.first, clear,
                         controller.last_diagnostics["cycle_ms"],
                         controller.last_diagnostics["safe_samples"], int(scheduler.avoiding)])
            actions.append(action.copy())
            state = new; last = action; now += .02
            if clear <= 0:
                failure = "executed_interval_check_failed"; break
            if progress.violation:
                failure = progress.violation; break
            if progress.first == 7 and np.linalg.norm(state[:3]-track.goal) < .55 and np.linalg.norm(state[3:6]) < 1:
                break
            if len(rows) % 50 == 0:
                update = {"simulation_time": now, "gates": progress.first,
                          "speed": float(np.linalg.norm(state[3:6])), "wall_seconds": time.perf_counter()-started}
                status.write_text(json.dumps(update, indent=2)+"\n")
                print(json.dumps(update), flush=True)
    finally:
        asynchronous.close()
    rows = np.asarray(rows)
    np.savez_compressed(out / "trace.npz", rows=rows, actions=np.asarray(actions),final_state=state)
    (out / 'tracking.json').write_text(json.dumps(tracking_log,indent=2)+'\n')
    ms = rows[:, 20] if len(rows) else np.array([])
    completed=bool(progress.first == 7 and np.linalg.norm(state[:3]-track.goal)<.55
                   and np.linalg.norm(state[3:6])<1.)
    if failure is None and now >= args.duration and not completed:
        failure = "task_timeout" if progress.first<7 else "return_goal_timeout"
    summary = {"evidence":"nominal same-model diagnostic; NOT PX4, NOT real-time closed loop",
               "config":asdict(cfg), "condition":{k:v for k,v in vars(args).items() if k != "outdir"},
               "gates":progress.first, "simulation_time":now, "failure":failure,
               "completed": bool(progress.first == 7 and np.linalg.norm(state[:3]-track.goal) < .55
                                  and np.linalg.norm(state[3:6]) < 1),
               "cycle_ms":{"median":float(np.median(ms)) if len(ms) else None,
                           "max":float(ms.max()) if len(ms) else None,
                           "over_20ms":int(np.sum(ms>20)), "over_100ms":int(np.sum(ms>100))},
               "reference_updates":requests,
               "source_sha256":source_sha256,
               "wall_seconds":time.perf_counter()-started}
    (out / "summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps(summary, indent=2), flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--outdir", type=Path, required=True)
    p.add_argument("--duration", type=float, default=60.)
    p.add_argument("--horizon",type=float,default=2.5)
    p.add_argument("--phase", type=float, default=.4)
    p.add_argument("--motion-rate", type=float, default=1.)
    p.add_argument("--translation-scale", type=float, default=1.)
    p.add_argument("--rotation-scale", type=float, default=1.)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--temperature", type=float, default=1.)
    p.add_argument('--backend',choices=('cuda','cpu','ctbr'),default='cuda')
    p.add_argument('--samples',type=int,default=Config().samples)
    p.add_argument("--paced", action="store_true", help="do not let model clock outrun async worker wall clock; still NOT PX4")
    run(p.parse_args())


if __name__ == "__main__": main()
