"""Same aligned PX4/MPC session with the two single-gate evidence streams."""
import json
import subprocess
import time
import numpy as np
from convex_timevarying_window.togt_style_mpc.aligned_session import Session
from . import scenario as world
from .planning import save

CONTAINER = 'convex_seven_togt_experiment'
PARTITION = CONTAINER+'_partition'


def launch(track, assets, out):
    world.launch(track, assets, out)
    subprocess.run(['docker', 'update', '--cpuset-cpus', '4-23', CONTAINER],
        check=True, capture_output=True)
    save(out/'execution_cpu_groups.json', dict(controller_cpus=[0,1,2,3], plant_cpus=list(range(4,24)),
        state_guard_s=.1, state_guard_unchanged=True))


class RecordedSession(Session):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_health = []
        save(self.out/'reference_activation.json', dict(activated=False))
        self.native_recorders = []
        topics = [('/world/convex_seven_dynamic_px4_togt/model/x500_0/link/base_link/sensor/magnetometer_sensor/magnetometer', 'magnetometer_native.pbtxt'),
                  ('/world/convex_seven_dynamic_px4_togt/pose/info', 'gate_pose.pbtxt')]
        for topic, name in topics:
            stream = (self.out/name).open('w')
            reader = subprocess.Popen(['docker', 'exec', '-e', 'GZ_PARTITION='+PARTITION,
                CONTAINER, 'gz', 'topic', '-e', '-t', topic], stdout=stream, stderr=subprocess.DEVNULL)
            self.native_recorders.append((reader, stream))
        started = time.perf_counter()
        script = """import json,sys,subprocess
from concurrent.futures import ThreadPoolExecutor
def info(topic):
    result=subprocess.run(['gz','topic','-i','-t',topic],capture_output=True,text=True,check=True,timeout=8)
    return topic,result.stdout
with ThreadPoolExecutor(max_workers=8) as pool:
    print(json.dumps(dict(pool.map(info,json.loads(sys.argv[1])))))
"""
        queried = subprocess.run(['docker', 'exec', '-e', 'GZ_PARTITION='+PARTITION, CONTAINER,
            'python3', '-c', script, json.dumps(world.contact_topics(self.track))],
            capture_output=True, text=True, check=True, timeout=15)
        publishers = json.loads(queried.stdout)
        if len(publishers) != 2 or not all('Publishers [Address, Message Type]:' in text and 'gz.msgs.Contacts' in text for text in publishers.values()):
            raise RuntimeError('missing_native_contact_publisher')
        save(self.out/'contact_publishers.json', publishers)
        save(self.out/'contact_manifest.json', dict(sensor_topics=world.contact_topics(self.track),
            gate_sensor_groups=tuple((topic,) for topic in world.contact_topics(self.track)), sensors=2,
            each_sensor_covers_entire_historic_sleeve_mesh=True))
        save(self.out/'publisher_check_timing.json', dict(elapsed_s=time.perf_counter()-started, channels=2))

    def prepare(self):
        super().prepare()
        at, state, age = self.read()
        ready = dict(state_time_s=float(at), position_m=state[:3].tolist(),
            start_error_m=float(np.linalg.norm(state[:3]-self.track.start)),
            speed_mps=float(np.linalg.norm(state[3:6])), age_s=float(age))
        ready['ready_at_common_race_epoch'] = bool(at <= .1 and ready['start_error_m'] < .55 and ready['speed_mps'] < 1.)
        save(self.out/'race_start_audit.json', ready)
        if not ready['ready_at_common_race_epoch']:
            raise RuntimeError('initialization_not_ready_at_race_epoch')
        save(self.out/'reference_activation.json', dict(activated=True))

    def read(self):
        value = super().read()
        if hasattr(self, 'read_health'):
            at, state, age = value
            self.read_health.append((float(at), float(age), bool(np.isfinite(state).all())))
        return value

    def close(self):
        try:
            super().close()
        finally:
            for reader, stream in getattr(self, 'native_recorders', []):
                reader.terminate()
                try:
                    reader.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    reader.kill()
                    reader.wait()
                stream.close()
            self.native_recorders = []
            health = getattr(self, 'read_health', [])
            save(self.out/'state_health.json', dict(reads=len(health),
                maximum_age_s=max((row[1] for row in health), default=None),
                over_100ms=sum(row[1] > .1 for row in health),
                nonfinite=sum(not row[2] for row in health)))


