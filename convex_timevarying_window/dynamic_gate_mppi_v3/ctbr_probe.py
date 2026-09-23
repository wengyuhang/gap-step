"""Independent PX4 CTBR response probe; no gate-racing claim."""
import argparse
from collections import deque
import csv
import json
import threading
import time
import subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_dynamic_seven_window_gazebo.online_safe_mppi_px4_experiment import (
    CONTAINER, ClockReader, set_parameter, mavutil)
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import (
    send_bodyrate_thrust,send_setpoint,send_gcs_heartbeat,enu_to_ned)
from convex_timevarying_window.receding_gate_racing.scenario import launch_variant,vary_track
from 复现.waypoint_mppi.gp_px4 import MotorEstimate
from .ctbr import encode_ctbr,CTBRRollout
from .rollout import KF
from .state_consistency import PositionVelocityObserver


class Session:
    """Single telemetry owner, coherent snapshots, independent feedback sender."""
    def __init__(self,out,track,consistent_velocity=False):
        self.out=out;self.track=track
        out.mkdir(parents=True,exist_ok=False)
        launch_variant(track,Path('convex_dynamic_seven_window_gazebo').resolve(),out)
        import os
        os.environ['MAVLINK20']='1';mavutil.set_dialect('common')
        set_parameter('MAV_PROTO_VER',2.)
        self.link=mavutil.mavlink_connection('udpin:0.0.0.0:14550',source_system=254,source_component=190)
        send_gcs_heartbeat(self.link)
        if self.link.wait_heartbeat(timeout=10) is None:raise RuntimeError('no heartbeat')
        for message in (31,32,375):
            self.link.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,message,10000,0,0,0,0,0)
        self.clock=ClockReader();self.lock=threading.Lock();self.stop=False
        self.snapshot=None;self.feedback=None;self.failure=None;self.failure_snapshot=None;self.sent=[];self.armed=False
        self.measurements=deque(maxlen=8192)
        self.origin=None;self.hold=track.start.copy();self._latest=None;self._attitude=None
        self._motors=MotorEstimate();self._stamps={};self._outputs=None
        self.velocity_observer=PositionVelocityObserver() if consistent_velocity else None
        self.observer_log=[]
        self._receive_thread=threading.Thread(target=self._receive,daemon=True)
        self._receive_thread.start()
        deadline=time.monotonic()+10
        while self.snapshot is None and time.monotonic()<deadline:time.sleep(.01)
        if self.snapshot is None:raise RuntimeError('missing initial telemetry')
        self._send_thread=threading.Thread(target=self._send,daemon=True);self._send_thread.start()

    def _receive(self):
        convert=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
        body=np.diag([1.,-1.,-1.])
        while not self.stop:
            msg=self.link.recv_match(blocking=False)
            if msg is None:time.sleep(.001);continue
            kind=msg.get_type();self._stamps[kind]=time.monotonic()
            if kind=='HEARTBEAT':self.armed=bool(msg.base_mode&mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if kind=='LOCAL_POSITION_NED':self._latest=msg
            if kind=='ATTITUDE_QUATERNION':self._attitude=msg
            if kind=='ACTUATOR_OUTPUT_STATUS':
                self._outputs=np.array(msg.actuator[:4]);self._motors.update(msg.time_usec*1e-6,self._outputs)
            p,q=self._latest,self._attitude
            if p is None or q is None or self._motors.speed is None or self.clock.value is None:continue
            if self.origin is None:self.origin=np.array([-16.-p.y,4.-p.x,-5.76+p.z])
            position=np.array([p.y,p.x,-p.z])+self.origin;velocity=np.array([p.vy,p.vx,-p.vz])
            if self.velocity_observer is not None:
                raw_velocity=velocity.copy()
                velocity=self.velocity_observer.update(p.time_boot_ms*.001,position,velocity)
                if self.snapshot is None or p.time_boot_ms*.001-30.>self.snapshot[0]:
                    self.observer_log.append((p.time_boot_ms*.001-30.,*raw_velocity,*velocity))
            rot=convert@Rotation.from_quat([q.q2,q.q3,q.q4,q.q1]).as_matrix()@body
            omega=body@np.array([q.rollspeed,q.pitchspeed,q.yawspeed])
            # State time belongs to the position measurement, not whichever
            # clock/heartbeat happened to wake the receiver. Align attitude
            # by measured angular velocity over the small message skew.
            skew=(p.time_boot_ms-q.time_boot_ms)*.001
            if abs(skew)>.1:continue
            rot=rot@Rotation.from_rotvec(omega*skew).as_matrix()
            quat=Rotation.from_matrix(rot).as_quat()
            x=np.r_[position,velocity,quat[3],quat[:3],omega,self._motors.speed]
            age=max(time.monotonic()-self._stamps.get(k,0.) for k in
                    ('LOCAL_POSITION_NED','ATTITUDE_QUATERNION','ACTUATOR_OUTPUT_STATUS'))
            with self.lock:
                stamp=p.time_boot_ms*.001-30.
                if self.snapshot is None or stamp>self.snapshot[0]:self.measurements.append((stamp,x.copy()))
                self.snapshot=(stamp,x,age,time.monotonic())

    def read(self):
        with self.lock:
            t,x,age,stamp=self.snapshot
            return t,x.copy(),age+time.monotonic()-stamp

    def drain_measurements(self):
        with self.lock:
            values=list(self.measurements);self.measurements.clear()
        return values

    def _send(self):
        beat=0.
        while not self.stop:
            wall=time.monotonic();now,x,age=self.read();fn=self.feedback
            try:
                if fn is None:
                    send_setpoint(self.link,enu_to_ned(self.hold,position=True,origin_enu=self.origin))
                else:
                    if age>.1:raise RuntimeError('stale_full_state')
                    u=fn(x,now);rate,thrust=encode_ctbr(u)
                    send_bodyrate_thrust(self.link,rate,thrust)
                    self.sent.append((now,*x,*u,1000*(time.monotonic()-wall)))
                if wall-beat>.4:send_gcs_heartbeat(self.link);beat=wall
            except Exception as exc:
                self.failure=repr(exc);self.failure_snapshot=(now,x.copy(),age)
                self.hold=x[:3].copy();self.feedback=None
            time.sleep(max(.001,.01-(time.monotonic()-wall)))

    def prepare(self):
        time.sleep(1.25)
        self.link.mav.set_mode_send(1,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,6<<16)
        deadline=time.monotonic()+12;arm=0.
        while not self.armed and time.monotonic()<deadline:
            if time.monotonic()-arm>1:
                self.link.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
                arm=time.monotonic()
            time.sleep(.01)
        if not self.armed:raise RuntimeError('arm failed')
        while self.read()[0]<0:time.sleep(.01)

    def close(self):
        _,x,_=self.read();self.hold=x[:3].copy();self.feedback=None
        time.sleep(.5);self.stop=True
        self._receive_thread.join(1);self._send_thread.join(1);self.clock.close()
        np.savez_compressed(self.out/'execution.npz',rows=np.asarray(self.sent))
        if self.failure_snapshot is not None:
            at,state,age=self.failure_snapshot
            np.savez_compressed(self.out/'sender_failure.npz',time=at,state=state,age=age)
        if self.velocity_observer is not None:np.savez_compressed(self.out/'velocity_observer.npz',rows=np.asarray(self.observer_log))
        subprocess.run(['docker','exec',CONTAINER,'/opt/px4-gazebo/bin/px4-logger','stop'],capture_output=True)
        p=subprocess.run(['docker','exec',CONTAINER,'bash','-lc','ls -t /opt/px4-gazebo/log/*/*.ulg'],capture_output=True,text=True)
        if p.returncode==0 and p.stdout.strip():
            subprocess.run(['docker','cp',f'{CONTAINER}:{p.stdout.splitlines()[0]}',str(self.out/'flight.ulg')],check=True,capture_output=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--outdir',type=Path,required=True);a=p.parse_args()
    track,_=build_track();vary_track(track,.4);model=CTBRRollout(track)
    s=Session(a.outdir,track);rows=[];failure=None
    try:
        s.prepare();origin=s.read()[0]
        def feedback(x,now):
            t=now-origin;u=model.feedback(x,track.start,np.zeros(3),np.zeros(3))
            # Symmetric pulses injected into the live stabilizing controller.
            for axis in range(3):
                local=t-(2+axis*3)
                if 0<=local<.2:u[axis+1]+=.2
                if 1<=local<1.2:u[axis+1]-=.2
            return u
        s.feedback=feedback
        while s.read()[0]-origin<12:
            t,x,age=s.read()
            if s.failure:raise RuntimeError(s.failure)
            tilt=np.degrees(np.arccos(np.clip(1-2*(x[7]**2+x[8]**2),-1,1)))
            if tilt>42 or np.linalg.norm(x[:3]-track.start)>2:raise RuntimeError('probe_envelope_exceeded')
            rows.append((t,*x));time.sleep(.01)
    except Exception as exc:failure=repr(exc)
    finally:s.close()
    q=np.asarray(rows)
    result=dict(evidence='CTBR no-gate response probe; NOT racing',failure=failure,
                duration=0 if not len(q) else float(q[-1,0]-q[0,0]),
                max_position_error=None if not len(q) else float(np.linalg.norm(q[:,1:4]-track.start,axis=1).max()))
    (a.outdir/'probe.json').write_text(json.dumps(result,indent=2)+'\n');print(result)


if __name__=='__main__':main()
