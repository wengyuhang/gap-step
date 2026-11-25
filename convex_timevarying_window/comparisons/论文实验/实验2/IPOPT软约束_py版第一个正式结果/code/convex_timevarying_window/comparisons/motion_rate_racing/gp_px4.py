"""Independent GP-MPPI -> PX4 collective thrust/body-rate adaptation.

This is NOT the author's controller. Full rotor-rate rollouts are projected
onto PX4 CTBR; that projection and motor-state estimation are adaptations.
"""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import (
    CONTAINER,CONTACT_TOPICS,PARTITION,ClockReader,StallWatchdog,set_parameter,mavutil)
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    enu_to_ned,parse_contacts,send_gcs_heartbeat,send_setpoint,send_bodyrate_thrust)
from convex_dynamic_seven_window_gazebo.togt_nmpc import collective_thrust_to_px4
from convex_timevarying_window.receding_gate_racing.scenario import vary_track,launch_variant
from convex_timevarying_window.receding_gate_racing.experiment import advance_gate
from .gp_mppi import GPMppi,GPConfig


class MotorEstimate:
    """PX4 output telemetry + known motor lag; never Gazebo ground truth."""
    def __init__(self):
        self.thrust=None;self.last_time=None;self.speed=None

    def update(self,stamp,values):
        values=np.asarray(values,dtype=float)
        if not np.all(np.isfinite(values)):return
        # In this PX4 Gazebo backend ACTUATOR_OUTPUT_STATUS relays
        # actuator_outputs in rad/s, NOT normalized actuator_motors controls.
        target=np.clip(values,0,1000)
        if self.speed is None:self.speed=target
        else:
            dt=max(0.,stamp-self.last_time);tau=np.where(target>self.speed,.0125,.025)
            self.speed=target+(self.speed-target)*np.exp(-dt/tau)
        self.thrust=8.54858e-6*self.speed**2;self.last_time=stamp

    def close(self):
        pass


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--outdir',type=Path,required=True)
    parser.add_argument('--motion-rate',type=float,default=1);parser.add_argument('--phase',type=float,default=0)
    parser.add_argument('--seed',type=int,default=0);parser.add_argument('--max-flight-time',type=float,default=60)
    parser.add_argument('--noise-std',type=float,default=3.)
    parser.add_argument('--motor-telemetry',action='store_true')
    parser.add_argument('--input-mode',choices=['rotor_rate','ctbr'],default='rotor_rate')
    parser.add_argument('--horizon',type=int,default=20)
    args=parser.parse_args();args.outdir.mkdir(parents=True,exist_ok=False)
    source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.iterdir() if p.suffix in ['.py','.cpp']}
    track,_=build_track();vary_track(track,args.phase,0,args.motion_rate)
    start=time.perf_counter();model=GPMppi(track,args.seed,GPConfig(noise_std=args.noise_std,input_mode=args.input_mode,horizon=args.horizon));build=time.perf_counter()-start
    assets=Path('convex_dynamic_seven_window_gazebo').resolve();launch_variant(track,assets,args.outdir)
    parameters={'MPC_XY_VEL_MAX':14.,'MPC_XY_CRUISE':12.,'MPC_Z_VEL_MAX_UP':7.,
                'MPC_Z_VEL_MAX_DN':7.,'MPC_ACC_HOR':10.,'MPC_ACC_HOR_MAX':12.,
                'MPC_ACC_UP_MAX':9.,'MPC_ACC_DOWN_MAX':9.,'MPC_JERK_AUTO':35.,
                'MPC_JERK_MAX':45.,'MPC_TILTMAX_AIR':45.,'MPC_XY_P':2.,'MPC_XY_VEL_P_ACC':3.5,
                'MPC_Z_P':2.,'SENS_GPS0_DELAY':0.,'SENS_GPS1_DELAY':0.}
    for key,value in parameters.items():set_parameter(key,value)
    if args.motor_telemetry:
        os.environ['MAVLINK20']='1';mavutil.set_dialect('common')
        set_parameter('MAV_PROTO_VER',2);parameters['MAV_PROTO_VER']=2
    link=mavutil.mavlink_connection('udpin:0.0.0.0:14550',source_system=254,source_component=190)
    send_gcs_heartbeat(link)
    if link.wait_heartbeat(timeout=10) is None:raise RuntimeError('missing heartbeat')
    for message_id in ((31,32,375) if args.motor_telemetry else (31,32)):
        link.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,message_id,10000,0,0,0,0,0)
    latest=None;attitude=None;heartbeat=None
    motors=MotorEstimate() if args.motor_telemetry else None
    def receive():
        nonlocal latest,attitude,heartbeat
        while True:
            msg=link.recv_match(blocking=False)
            if msg is None:break
            if msg.get_type()=='LOCAL_POSITION_NED':latest=msg
            if msg.get_type()=='ATTITUDE_QUATERNION':attitude=msg
            if msg.get_type()=='HEARTBEAT':heartbeat=msg
            if msg.get_type()=='ACTUATOR_OUTPUT_STATUS' and motors is not None:motors.update(msg.time_usec*1e-6,msg.actuator[:4])
    clock=ClockReader();deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        receive()
        if latest is not None and attitude is not None and clock.value is not None:break
        time.sleep(.01)
    if latest is None or attitude is None:raise RuntimeError('missing local position/attitude')
    if clock.value is None or clock.value>18:raise RuntimeError(f'preparation clock {clock.value}')
    origin=np.array([-16.-latest.y,4.-latest.x,-5.76+latest.z])
    shared={'command':None,'hold':np.asarray(track.start),'stop':False};sending=[]
    def sender():
        beat=0.
        while not shared['stop']:
            tick=time.perf_counter();cmd=shared['command']
            if cmd is None:send_setpoint(link,enu_to_ned(shared['hold'],position=True,origin_enu=origin))
            else:send_bodyrate_thrust(link,cmd[0],cmd[1])
            if time.monotonic()-beat>.4:send_gcs_heartbeat(link);beat=time.monotonic()
            sending.append((time.perf_counter()-tick)*1000);time.sleep(max(.001,.01-(time.perf_counter()-tick)))
    thread=threading.Thread(target=sender,daemon=True);thread.start()
    processes=[];paths=[]
    for i,topic in enumerate(CONTACT_TOPICS):
        path=args.outdir/f'gate_{i+1:02d}_contacts.pbtxt';f=path.open('w')
        proc=subprocess.Popen(['docker','exec','-e',f'GZ_PARTITION={PARTITION}',CONTAINER,'gz','topic','-e','-t',topic],stdout=f,stderr=subprocess.DEVNULL)
        processes.append((proc,f));paths.append(path)
    shim=SimpleNamespace(windows=track.windows,signs=model.signs,cfg=SimpleNamespace(radius=.38594472))
    convert=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    body=np.diag([1.,-1.,-1.]);rows=[];solves=[];crossings=[];gate=0;completed=False;failure=None
    now=0.;pos=np.array(track.start);vel=np.zeros(3);acc=np.zeros(3);last_pos=pos.copy();last_t=0.;last_v=None
    thrust=np.full(4,2.0643076923076924*9.8066/4);last_update=None;next_solve=0.;last_row=-1
    last_omega=None;last_omega_time=None;angular_acc=np.zeros(3)
    inertia=np.array([.02383948,.02394241,.04399995])
    mixing=np.array([[1.,1.,1.,1.],[-.174,.174,.174,-.174],[-.174,.174,-.174,.174],[-.016,-.016,.016,.016]])
    watchdog=StallWatchdog()
    try:
        time.sleep(1.25);link.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,6<<16)
        deadline=time.monotonic()+12;last_arm=0.;armed=False
        while time.monotonic()<deadline:
            receive()
            if heartbeat is not None and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:armed=True;break
            if time.monotonic()-last_arm>1:
                link.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0);last_arm=time.monotonic()
            time.sleep(.002)
        if not armed:raise RuntimeError('arm failed')
        while clock.value<30:receive();time.sleep(.002)
        while clock.value<30+args.max_flight_time:
            receive();now=clock.value-30
            if watchdog.stalled(clock.value):failure='clock_stalled';break
            pos=np.array([latest.y,latest.x,-latest.z])+origin;vel=np.array([latest.vy,latest.vx,-latest.vz])
            if last_update is None:last_update=now;last_v=vel.copy()
            if now-last_update>=.02:
                raw=(vel-last_v)/(now-last_update);acc=.8*acc+.2*np.clip(raw,-12,12);last_update=now;last_v=vel.copy()
            passed,cross=advance_gate(shim,gate,last_pos,pos,last_t,now)
            if passed:gate+=1;crossings.append(cross);print('GP PASS',gate,now,flush=True)
            last_pos=pos.copy();last_t=now
            if now>=next_solve:
                tick=time.perf_counter()
                rotation=convert@Rotation.from_quat([attitude.q2,attitude.q3,attitude.q4,attitude.q1]).as_matrix()@body
                q=Rotation.from_matrix(rotation).as_quat();omega=body@np.array([attitude.rollspeed,attitude.pitchspeed,attitude.yawspeed])
                measured=np.clip(2.0643076923076924*np.dot(acc+[0,0,9.8066],rotation[:,2]),1.,34.19432)
                if last_omega is not None and now>last_omega_time:
                    angular_acc=.8*angular_acc+.2*(omega-last_omega)/(now-last_omega_time)
                last_omega=omega.copy();last_omega_time=now
                torque=inertia*angular_acc+np.cross(omega,inertia*omega)
                thrust=np.clip(np.linalg.solve(mixing,np.r_[measured,torque]),0,8.54858)
                if motors is not None:
                    if motors.thrust is None or abs(clock.value-motors.last_time)>.15:
                        raise RuntimeError('stale PX4 actuator telemetry')
                    thrust=motors.thrust.copy()
                state=np.r_[pos,vel,q[3],q[:3],omega,thrust]
                control,pred,ms=model.solve(state,now,gate)
                requested_rates=body@pred[1,10:13]
                normalized=collective_thrust_to_px4(float(pred[1,13:17].sum()))
                if args.input_mode=='ctbr':
                    requested_rates=body@control[1:4]
                    normalized=collective_thrust_to_px4(control[0])
                elapsed=1000*(time.perf_counter()-tick)
                accepted=elapsed<100 and np.all(np.isfinite(pred))
                solves.append(dict(time=now,cycle_ms=elapsed,accepted=bool(accepted),gate=gate,status='independent_gp_mppi'))
                if accepted:shared['command']=(requested_rates,normalized);thrust=pred[1,13:17].copy()
                else:failure='controller_deadline_or_numeric_failure';break
                next_solve=now+.03
            if now-last_row>=.018:
                rows.append([now,*pos,*vel,*acc,*pos,gate,0.]);last_row=now
            if pos[2]<.3 or pos[2]>15 or abs(pos[0])>32 or pos[1]>26 or pos[1]<-34:
                failure='workspace_abort';break
            if gate==7 and np.linalg.norm(pos-track.goal)<.55 and np.linalg.norm(vel)<1:
                completed=True;break
            time.sleep(.001)
        if not completed and failure is None:failure='timeout'
    except Exception as exc:failure=repr(exc)
    finally:
        shared['command']=None;shared['hold']=pos.copy();time.sleep(.5);shared['stop']=True;thread.join(timeout=1)
        clock.close()
        if motors is not None:motors.close()
        for proc,f in processes:
            proc.terminate()
            try:proc.wait(timeout=2)
            except subprocess.TimeoutExpired:proc.kill()
            f.close()
    contacts=[parse_contacts(path) for path in paths]
    subprocess.run(['docker','exec',CONTAINER,'/opt/px4-gazebo/bin/px4-logger','stop'],capture_output=True)
    logs=subprocess.run(['docker','exec',CONTAINER,'bash','-lc','ls -t /opt/px4-gazebo/log/*/*.ulg'],capture_output=True,text=True)
    if logs.returncode==0 and logs.stdout.strip():subprocess.run(['docker','cp',f'{CONTAINER}:{logs.stdout.splitlines()[0]}',str(args.outdir/'flight.ulg')],check=True)
    with (args.outdir/'telemetry.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['t','x','y','z','vx','vy','vz','ax','ay','az','rx','ry','rz','gate','error']);writer.writerows(rows)
    result=dict(method='independent_gp_mppi_ctbr',completed=completed,failure=failure,flight_time_s=now if completed else None,
        last_time_s=now,collision_detected=any(c['contact_pair_count']>0 for c in contacts),contacts=contacts,crossings=crossings,
        config=dict(radius=.38594472,**asdict(model.cfg)),parameters=parameters,solves=solves,build_s=build,
        phase=args.phase,layout=0,motion_rate=args.motion_rate,seed=args.seed,motor_telemetry=args.motor_telemetry,origin_enu_m=origin.tolist(),
        goal_error_m=float(np.linalg.norm(pos-track.goal)),goal_speed_mps=float(np.linalg.norm(vel)),
        source_sha256=source_hashes,
        evidence='Independent paper-based implementation with dynamic-gate constraints and PX4 CTBR adaptation; NOT official author code')
    (args.outdir/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['solves','contacts','source_sha256']},indent=2),flush=True)


if __name__=='__main__':main()
