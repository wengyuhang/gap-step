"""Measured-input/time alignment, no Gazebo ground-truth feedback."""
from collections import deque
import json
import subprocess
import numpy as np
from convex_timevarying_window.single_gate_stop_racing.metric_session import Session as MetricSession


class MotorEstimate:
    """Causal zero-order hold: the NEW target cannot act over the PAST interval."""
    def __init__(self):
        self.speed=None; self.target=None; self.last_time=None; self.thrust=None
        self.history=deque(maxlen=1024)

    @staticmethod
    def advance(speed,target,dt):
        tau=np.where(target>speed,.0125,.025)
        return target+(speed-target)*np.exp(-max(0.,dt)/tau)

    def update(self,stamp,values):
        values=np.asarray(values,float)
        if not np.isfinite(values).all() or (self.last_time is not None and stamp<=self.last_time): return
        target=np.clip(values,0.,1000.)
        self.speed=target.copy() if self.speed is None else self.advance(self.speed,self.target,stamp-self.last_time)
        self.target=target; self.last_time=stamp; self.thrust=8.54858e-6*self.speed**2
        self.history.append((stamp,self.speed.copy(),target.copy()))

    def at(self,stamp):
        eligible=[r for r in list(self.history) if r[0]<=stamp+1e-9]
        if not eligible: raise RuntimeError('missing_time_aligned_motor_telemetry')
        time,speed,target=eligible[-1]
        if stamp-time>.04: raise RuntimeError('stale_time_aligned_motor_telemetry')
        return self.advance(speed,target,stamp-time)


class Session(MetricSession):
    # Raw interval-averaged IMU gyro is distinct from the low-pass rate in
    # ATTITUDE_QUATERNION. HIGHRES_IMU carries its own sample timestamp.
    telemetry_intervals=((31,10000),(32,10000),(375,4000),(105,4000))
    motor_estimator_class=MotorEstimate

    def __init__(self,*args,**kwargs):
        self.gyro_history=deque(maxlen=1024); self.sensor_alignment=[]
        super().__init__(*args,**kwargs)
        # Observation only: remove 10/50 Hz ULog decimation so first receiver
        # updates and actuator targets can be audited at the 4 ms plant clock.
        container='convex_seven_togt_experiment'
        for args in [('stop',),('start','-f','-r','250','-c','25','-b','256')]:
            subprocess.run(['docker','exec',container,'/opt/px4-gazebo/bin/px4-logger',*args],
                           check=True,capture_output=True,timeout=8)
        (self.out/'logging_protocol.json').write_text(json.dumps(dict(rate_hz=250,interval_factor=25,
            buffer_KiB=256,physics_and_flight_parameters_changed=False),indent=2))

    def on_message(self,message):
        if message.get_type()=='HIGHRES_IMU':
            self.gyro_history.append((message.time_usec*1e-6-30.,
                np.array([message.xgyro,-message.ygyro,-message.zgyro])))

    def control_state(self,state,stamp):
        samples=list(self.gyro_history)
        if not samples or abs(samples[-1][0]-stamp)>.04:
            raise RuntimeError('missing_or_stale_raw_gyro')
        times=np.array([v[0] for v in samples]); rates=np.array([v[1] for v in samples])
        x=state.copy(); x[10:13]=[np.interp(stamp,times,rates[:,i]) for i in range(3)]
        x[13:17]=self._motors.at(stamp+30.)
        self.sensor_alignment.append([stamp,samples[-1][0],self._motors.last_time-30.,*state[10:13],*x[10:13]])
        return x

    def close(self):
        try: super().close()
        finally: np.savez_compressed(self.out/'sensor_alignment.npz',rows=np.asarray(self.sensor_alignment))
