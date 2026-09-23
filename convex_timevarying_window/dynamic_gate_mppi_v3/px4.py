"""PX4/Gazebo V3 experiment, direct motors via ROS 2 and PX4 mixing output.

Position control is used ONLY for the common 30 s preparation and explicit
post-failure simulation recovery. Neither is counted as racing safety control.
"""
import argparse
import copy
from collections import deque
import csv
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import shutil
import socket
import subprocess
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import (
    CONTAINER, CONTACT_TOPICS, PARTITION, ClockReader, StallWatchdog, set_parameter, mavutil)
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    enu_to_ned, parse_contacts, send_gcs_heartbeat, send_setpoint)
from convex_timevarying_window.receding_gate_racing.scenario import vary_track, launch_variant
from nonconvex_timevarying_window.sc_dynatogt.minco import BoundaryState
from 复现.waypoint_mppi.gp_px4 import MotorEstimate
from .config import Config
from .controller import make_controller, Selection
from .geometry import GateProgress
from .reference import Planner, AsyncPlanner
from .scheduler import Scheduler
from .diagnostic import pvaj_from_state
from .rollout import KF


def motor_normalized(thrust):
    """Inverse of installed THR_MDL_FAC=0, SIM_GZ_EC_MIN/MAX=150/1000."""
    speed = np.sqrt(np.clip(thrust, KF*150**2, KF*1000**2)/KF)
    return (speed-150.)/850.


def run(args):
    out = args.outdir; out.mkdir(parents=True, exist_ok=False)
    track, opt = build_track()
    vary_track(track, args.phase, 0., args.motion_rate, args.translation_scale, args.rotation_scale)
    cfg = replace(Config(),samples=args.samples,horizon=args.horizon); ctrl = make_controller(track,cfg,args.seed,args.backend)
    planner = Planner(track, opt); worker = AsyncPlanner(track, opt)
    hashes = {}
    for source in Path(__file__).parent.rglob("*"):
        rel = source.relative_to(Path(__file__).parent)
        if "results" in rel.parts or any(x in rel.parts for x in ("build", "install", "log")): continue
        if source.suffix not in (".py", ".cpp", ".msg", ".xml", ".txt"): continue
        target = out/"source_snapshot"/rel; target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target); hashes[str(rel)] = hashlib.sha256(source.read_bytes()).hexdigest()
    launch_variant(track, Path("convex_dynamic_seven_window_gazebo").resolve(), out)
    logs = []; contact_paths = []
    topics = list(CONTACT_TOPICS)+[
        "/world/convex_seven_dynamic_px4_togt/model/x500_0/link/base_link/sensor/air_pressure_sensor/air_pressure",
        "/model/x500_0/odometry_with_covariance"]
    for i, topic in enumerate(topics):
        path = out/(f"gate_{i+1:02d}_contacts.pbtxt" if i<7 else f"sensor_keepalive_{i}.pbtxt")
        stream = path.open("w")
        proc = subprocess.Popen(["docker", "exec", "-e", f"GZ_PARTITION={PARTITION}", CONTAINER,
                                 "gz", "topic", "-e", "-t", topic], stdout=stream, stderr=subprocess.DEVNULL)
        logs.append((proc, stream))
        if i<7: contact_paths.append(path)
    parameters = dict(MPC_XY_VEL_MAX=14., MPC_XY_CRUISE=12., MPC_Z_VEL_MAX_UP=7., MPC_Z_VEL_MAX_DN=7.,
        MPC_ACC_HOR=10., MPC_ACC_HOR_MAX=12., MPC_ACC_UP_MAX=9., MPC_ACC_DOWN_MAX=9.,
        MPC_JERK_AUTO=35., MPC_JERK_MAX=45., MPC_TILTMAX_AIR=45., MPC_XY_P=2.,
        MPC_XY_VEL_P_ACC=3.5, MPC_Z_P=2., SENS_GPS0_DELAY=0., SENS_GPS1_DELAY=0., MAV_PROTO_VER=2.)
    for key, value in parameters.items(): set_parameter(key, value)
    import os
    os.environ["MAVLINK20"] = "1"; mavutil.set_dialect("common")
    link = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=254, source_component=190)
    send_gcs_heartbeat(link)
    if link.wait_heartbeat(timeout=10) is None: raise RuntimeError("missing heartbeat")
    for message in (31, 32, 375):
        link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message, 10000, 0, 0, 0, 0, 0)
    latest = attitude = heartbeat = None; motors = MotorEstimate(); outputs = None
    motor_wall = state_wall = 0.
    def receive():
        nonlocal latest, attitude, heartbeat, outputs, motor_wall, state_wall
        while True:
            message = link.recv_match(blocking=False)
            if message is None: break
            kind = message.get_type()
            if kind == "LOCAL_POSITION_NED": latest = message; state_wall = time.monotonic()
            elif kind == "ATTITUDE_QUATERNION": attitude = message
            elif kind == "HEARTBEAT": heartbeat = message
            elif kind == "ACTUATOR_OUTPUT_STATUS":
                outputs = np.array(message.actuator[:4]); motor_wall = time.monotonic()
                motors.update(message.time_usec*1e-6, outputs)
    clock = ClockReader(); deadline = time.monotonic()+8
    while time.monotonic() < deadline:
        receive()
        if latest is not None and attitude is not None and clock.value is not None: break
        time.sleep(.01)
    if latest is None or attitude is None or clock.value is None: raise RuntimeError("missing telemetry")
    origin = np.array([-16.-latest.y, 4.-latest.x, -5.76+latest.z])
    convert = np.array([[0,1,0], [1,0,0], [0,0,-1.]])
    body = np.diag([1.,-1.,-1.])
    def state():
        pos = np.array([latest.y, latest.x, -latest.z])+origin
        vel = np.array([latest.vy, latest.vx, -latest.vz])
        R = convert@Rotation.from_quat([attitude.q2,attitude.q3,attitude.q4,attitude.q1]).as_matrix()@body
        q = Rotation.from_matrix(R).as_quat()
        omega = body@np.array([attitude.rollspeed,attitude.pitchspeed,attitude.yawspeed])
        speed = motors.speed if motors.speed is not None else np.full(4, 770.)
        return np.r_[pos, vel, q[3], q[:3], omega, speed]
    shared = {"stop":False, "race":False, "recovery":False, "active":None,
              "failure":None, "hold":track.start.copy(), "last_sent":None, "probe":None}
    pending=deque();activations=[]
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    def sender():
        beat = 0.
        while not shared["stop"]:
            tick = time.monotonic(); now = (clock.value or 0.)-30.
            # Activation belongs to the independent actuator clock, not to
            # the end of a potentially long MPPI solve. deque appends/pops
            # are atomic; only this thread consumes pending selections.
            while pending and now>=pending[0].made_at:
                candidate=pending.popleft()
                accepted=ctrl.publish(candidate,state(),now)
                activations.append(dict(anchor=candidate.made_at,time=now,
                    lateness_ms=1000*(now-candidate.made_at),accepted=bool(accepted)))
                if accepted:shared['active']=candidate
            active = shared["active"]
            if shared["recovery"] or not shared["race"]:
                send_setpoint(link, enu_to_ned(shared["hold"], position=True, origin_enu=origin))
            else:
                action = shared["probe"]
                if action is None and active is not None:
                    index = int(np.floor((now-active.made_at)/.02+1e-8))
                    if 0 <= index < len(active.controls) and now < active.expires_at:
                        action = active.controls[index]
                if action is None:
                    shared["failure"] = "validated_cache_exhausted"
                else:
                    values = motor_normalized(action)
                    payload = {"timestamp_us":int(max(0., now+30)*1e6),
                               "expires_monotonic":tick+.025, "controls":values.tolist()}
                    udp.sendto(json.dumps(payload).encode(), ("127.0.0.1", 17884))
                    shared["last_sent"] = np.asarray(action).copy()
            if tick-beat > .4: send_gcs_heartbeat(link); beat = tick
            time.sleep(max(.001, .005-(time.monotonic()-tick)))
    thread = threading.Thread(target=sender, daemon=True); thread.start()
    progress = GateProgress(track, cfg); scheduler = Scheduler(cfg)
    rows = []; solves = []; crossings = []; updates = []; probe_rows = []
    completed = False; failure = None; now = 0.; ref = None
    last_state = state(); last_t = 0.; next_control = 0.; request_anchor = None; ref_id = 0
    header = ["t","x","y","z","vx","vy","vz","ax","ay","az","rx","ry","rz","gate","error"]
    trace = (out/"telemetry.csv").open("w", buffering=1); writer = csv.writer(trace); writer.writerow(header)
    live = (out/"solves_live.jsonl").open("w", buffering=1)
    watchdog = StallWatchdog()
    try:
        time.sleep(1.25)
        link.mav.set_mode_send(1, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 6<<16)
        deadline = time.monotonic()+12; last_arm = 0.; armed = False
        while time.monotonic() < deadline:
            receive()
            if heartbeat is not None and heartbeat.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed = True; break
            if time.monotonic()-last_arm > 1:
                link.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
                last_arm = time.monotonic()
            time.sleep(.002)
        if not armed: raise RuntimeError("arm failed")
        while clock.value < 30.:
            receive(); worker.poll()
            if not args.interface_probe and clock.value >= 29.:
                if motors.speed is None: raise RuntimeError("motor telemetry unavailable for handover")
                # Switch during the existing preparation interval; mirror the
                # current motor output until Commander disables allocation.
                shared["probe"] = KF*np.asarray(outputs)**2
                shared["race"] = True
            time.sleep(.002)
        receive(); initial = state(); now = clock.value-30.
        if motors.speed is None: raise RuntimeError("motor telemetry unavailable")
        if args.interface_probe:
            shared["probe"] = KF*initial[13:]**2 + np.array([.03,-.03,.03,-.03])
            shared["race"] = True
        else:
            held = np.tile(KF*initial[13:]**2,(ctrl.nc,1))
            checked = ctrl.rollout(initial,held[None],ctrl.rollout.prepare(now),0,False)
            if not checked.valid[0]: raise RuntimeError("initial_cached_control_invalid")
            starter = Selection(now,held,checked.states[0],checked.clearances[0],checked.indices[0],0.,1,False,False)
            ctrl.publish(starter,initial,now); shared["active"] = starter; shared["probe"] = None
            # Initial computation is inside the common race clock, never hidden in preparation.
            tick = time.perf_counter()
            ref = planner.solve(pvaj_from_state(ctrl.rollout,initial,held[0]), now, 0)
            solves.append({"time":now,"cycle_ms":1000*(time.perf_counter()-tick),"accepted":ref is not None,"status":"initial_togt"})
            if ref is None: raise RuntimeError("initial_reference_budget_failure")
            ctrl.seed_reference(ref, planner.core)
        while clock.value < 30.+args.max_flight_time:
            receive(); now = clock.value-30.; x = state()
            if watchdog.stalled(clock.value): failure = "clock_stalled"; break
            if time.monotonic()-state_wall > .10 or time.monotonic()-motor_wall > .10:
                failure = "stale_state_or_motor_feedback"; break
            if shared["failure"]: failure = shared["failure"]; break
            if not np.isfinite(x).all(): failure = "invalid_state"; break
            if np.any(x[:3] < cfg.lower) or np.any(x[:3] > cfg.upper): failure = "workspace_abort"; break
            if args.interface_probe:
                if outputs is not None:
                    probe_rows.append({"t":now,"expected_speed":np.sqrt(shared["probe"]/KF).tolist(),
                                       "reported_output":outputs.tolist(),"position":x[:3].tolist()})
                if now > 1.: break
            else:
                if shared["race"] and ctrl.cached_action(x, now) is None:
                    failure = "cache_expired_or_tracking_error"; break
                if now > last_t:
                    clearance = ctrl.rollout.geometry.interval_clearances(
                        np.array([last_state[:3],x[:3]]), np.array([last_state[3:6],x[3:6]]),np.array([last_t,now])).min()
                    if progress.update(last_state[:3], last_t, x[:3], now, clearance > 0):
                        scheduler.gate_passed(progress.first)
                        crossings.append({"window_index":progress.first-1,"time":now,"position":x[:3].tolist()})
                        print("V3 PASS",progress.first,now,flush=True)
                    last_state = x.copy(); last_t = now
                    if progress.violation: failure = progress.violation; break
                returned = worker.poll()
                if returned is not None:
                    _, candidate = returned; accepted = False; request = scheduler.running
                    validation={}
                    if request is not None and candidate is not None and not isinstance(candidate,dict):
                        errors=ctrl.tracking_error(x,now,candidate)
                        validation=dict(**errors,elapsed_sim=now-request.started,solve_ms=candidate.solve_ms)
                        accepted = scheduler.finish(request,now,progress.first,
                            errors['position_error'],errors['velocity_error'],0.)
                    if accepted: ref = candidate
                    else: scheduler.failed(request)
                    updates.append({"time":now,"accepted":bool(accepted),"reason":request.reason if request else None,
                                    "validation":validation,
                                    "error":candidate if isinstance(candidate,dict) else None})
                if now >= next_control:
                    # A tracking solve plus autonomous fallback can take
                    # 40--46 ms. Keep one fixed 60 ms actuation lead (<100 ms)
                    # and predict the actual queued inputs over all of it.
                    tick = time.perf_counter(); lead = .06; anchor = now+lead
                    prediction = x.copy(); current = shared["active"]
                    predicted_progress = copy.copy(progress)
                    for k in range(int(round(lead/.005))):
                        at = now+k*.005
                        for queued in tuple(pending):
                            if queued.made_at <= at: current = queued
                        if current is None: action = KF*x[13:]**2
                        else:
                            i = int(np.floor((at-current.made_at)/.02+1e-8))
                            if not 0 <= i < len(current.controls): break
                            action = current.controls[i]
                        prior = prediction
                        prediction = ctrl.rollout.integrate(prediction,action,.005)
                        predicted_progress.update(prior[:3],at,prediction[:3],at+.005,True)
                    risk=ctrl.reference_risk(x, now, progress.first, ref,progress.crossing)
                    errors=ctrl.tracking_error(x,now,ref)
                    if errors['tracking_failed']:scheduler.flag_tracking_error()
                    if predicted_progress.violation:
                        failure = "queued_control_"+predicted_progress.violation; break
                    selection = ctrl.select(prediction, anchor, predicted_progress.first, predicted_progress.crossing,
                        risk, ref,
                        shared["last_sent"] if shared["last_sent"] is not None else KF*x[13:]**2)
                    scheduler.set_risk(ctrl.last_diagnostics.get('avoidance',risk))
                    receive(); returned_time = clock.value-30.
                    elapsed = 1000*(time.perf_counter()-tick)
                    accepted = selection is not None and returned_time < anchor and elapsed < 100
                    row = {"time":now,"cycle_ms":elapsed,"accepted":bool(accepted),"status":"direct_motor_mppi",
                           "safe_samples":ctrl.last_diagnostics["safe_samples"],"late_for_anchor":returned_time>=anchor,
                           "gate":progress.first,"avoidance":scheduler.avoiding,
                           "reference_risk":risk,"imminent_risk":ctrl.last_diagnostics.get('imminent_risk'),
                           "baseline_valid":ctrl.last_diagnostics.get('baseline_valid'),
                           "tracking_infeasible_fallback":ctrl.last_diagnostics.get('tracking_infeasible_fallback'),
                           "rolling_pending":scheduler.rolling_pending,"backend":args.backend,"samples":cfg.samples}
                    row.update(errors)
                    row['reference_weight']=ctrl.last_diagnostics.get('reference_weight',0.)
                    solves.append(row); live.write(json.dumps(row)+"\n")
                    if accepted: pending.append(selection)
                    elif not shared["race"]: failure = "initial_control_admission_failed"; break
                    next_control = now+.02
                # A solve has elapsed since x/now were acquired. Request
                # TOGT from the CURRENT measured head, not the pre-MPPI head
                # with part of its 100 ms budget already spent.
                receive();now=clock.value-30.;x=state()
                cache = shared["active"]
                if cache is not None and worker.ready and scheduler.running is None:
                    i = max(0,int(np.floor((now-cache.made_at)/.01)))
                    if i+10 < len(cache.states):
                        times = cache.made_at+np.arange(i,len(cache.states))*.01
                        request = scheduler.observe(now,times,cache.clearances[i:],
                            not progress.computation_allowed(x[:3],now),progress.first)
                        if request is not None:
                            action=shared['last_sent'] if shared['last_sent'] is not None else KF*x[13:]**2
                            worker.submit(ref_id,pvaj_from_state(ctrl.rollout,x,action),request.anchor,request.first_gate)
                            ref_id += 1
                if progress.first == 7 and np.linalg.norm(x[:3]-track.goal)<.55 and np.linalg.norm(x[3:6])<1:
                    completed = True; break
            row = [now,*x[:6],0.,0.,0.,*x[:3],progress.first,0.]
            rows.append(row); writer.writerow(row)
            time.sleep(.001)
        if not completed and failure is None and not args.interface_probe: failure = "timeout"
    except Exception as exc: failure = repr(exc)
    finally:
        terminal_state = state().copy()
        # Simulation recovery after declared termination, not algorithm safety.
        shared["recovery"] = True; shared["hold"] = state()[:3].copy()
        time.sleep(.5); shared["stop"] = True; thread.join(timeout=1)
        udp.close(); trace.close(); live.close(); worker.close(); clock.close()
        for proc, stream in logs:
            proc.terminate()
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired: proc.kill()
            stream.close()
    contacts = [parse_contacts(p) for p in contact_paths]
    subprocess.run(["docker","exec",CONTAINER,"/opt/px4-gazebo/bin/px4-logger","stop"],capture_output=True)
    listing = subprocess.run(["docker","exec",CONTAINER,"bash","-lc","ls -t /opt/px4-gazebo/log/*/*.ulg"],capture_output=True,text=True)
    if listing.returncode == 0 and listing.stdout.strip():
        subprocess.run(["docker","cp",f"{CONTAINER}:{listing.stdout.splitlines()[0]}",str(out/"flight.ulg")],check=True)
    x = terminal_state
    result = {"method":"dynamic_gate_mppi_v3_direct_motor", "completed":completed,"failure":failure,
        "flight_time_s":now if completed else None,"last_time_s":now,"crossings":crossings,
        "collision_detected":any(c["contact_pair_count"]>0 for c in contacts),"contacts":contacts,
        "config":{**asdict(cfg),"backend":args.backend,"workspace_lower":list(cfg.lower),"workspace_upper":list(cfg.upper)},
        "parameters":parameters,"solves":solves,"reference_updates":updates,"activations":activations,"phase":args.phase,"layout":0.,
        "motion_rate":args.motion_rate,"translation_scale":args.translation_scale,"rotation_scale":args.rotation_scale,
        "origin_enu_m":origin.tolist(),"goal_error_m":float(np.linalg.norm(x[:3]-track.goal)),
        "goal_speed_mps":float(np.linalg.norm(x[3:6])),"source_sha256":hashes,"interface_probe":probe_rows,
        "evidence":"PX4 direct actuator experiment; no proven tracking-error tube or recursive safety"}
    (out/"result.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps({k:result[k] for k in ("completed","failure","last_time_s","crossings","collision_detected")}),flush=True)


def main():
    p = argparse.ArgumentParser(); p.add_argument("--outdir",type=Path,required=True)
    for name, default in [("phase",.4),("motion-rate",1.),("translation-scale",1.),("rotation-scale",1.),("max-flight-time",60.)]:
        p.add_argument("--"+name,type=float,default=default)
    p.add_argument("--seed",type=int,default=0)
    p.add_argument("--horizon",type=float,default=2.5)
    p.add_argument("--interface-probe",action="store_true")
    p.add_argument('--backend',choices=('cuda','cpu'),default='cuda')
    p.add_argument('--samples',type=int,default=Config().samples)
    run(p.parse_args())


if __name__ == "__main__": main()
