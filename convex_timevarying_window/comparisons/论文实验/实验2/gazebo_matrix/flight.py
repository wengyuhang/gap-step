"""Same frozen-reference tracker and PX4 transport for all five methods."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import numpy as np
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from convex_timevarying_window.global_gate_minco_probe.flight import Worker, Scheduler, Progress
from convex_timevarying_window.single_gate_stop_racing.planner import Config
from convex_timevarying_window.togt_style_mpc.aligned_session import Session
from convex_timevarying_window.comparisons.single_gate_swing.contacts import parse_contacts
from .reference import Reference, MPC
from .audit import audit
from ..basic_group.whole_course_difficulty import scenario
from ..basic_group import scenario as world

CONTAINER = 'convex_seven_togt_experiment'
PARTITION = CONTAINER+'_partition'


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


class RecordedSession(Session):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
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
        if len(publishers) != 8 or not all('Publishers [Address, Message Type]:' in text and 'gz.msgs.Contacts' in text for text in publishers.values()):
            raise RuntimeError('missing_native_contact_publisher')
        save(self.out/'contact_publishers.json', publishers)
        save(self.out/'contact_manifest.json', dict(sensor_topics=world.contact_topics(self.track),
            gate_sensor_groups=world.contact_groups(self.track), sensors=8,
            each_sensor_covers_entire_historic_sleeve_mesh=True))
        save(self.out/'publisher_check_timing.json', dict(elapsed_s=time.perf_counter()-started, channels=8))

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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference-dir', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    reference_dir, out = args.reference_dir.resolve(), args.outdir.resolve()
    record = json.loads((reference_dir/'manifest.json').read_text())
    case = scenario.Case(level=record['level'])
    track, _ = scenario.build_case(case)
    source = np.load(reference_dir/'reference.npz')
    plan = Plan(0., 0, source['durations'], source['coefficients'], np.empty(0), 0., 0., 'frozen_reference', 7)
    cfg = replace(Config(), horizon=7, radius=.3859447197013662,
        full_task_error_recovery=False, workspace_lower=(-100., -120., .65), workspace_upper=(150., 120., 20.))
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight, px4
    references = []

    def reference_factory(start, count):
        reference = Reference(start, plan)
        references.append(reference)
        return reference

    flight.Worker, flight.Scheduler, flight.Reference = Worker, Scheduler, reference_factory
    flight.variant_config = lambda variant: cfg
    flight.build_case = lambda unused: (track, None)
    flight.launch = world.launch
    flight.manifest = lambda unused: scenario.manifest(case)
    flight.MAX_FLIGHT_TIME = 180
    px4.MPC, px4.Progress, px4.Session = MPC, Progress, RecordedSession
    px4.CONTACT_TOPICS, px4.parse_contacts = world.contact_topics(track), parse_contacts
    px4.WORKSPACE_LOWER, px4.WORKSPACE_UPPER = cfg.workspace_lower, cfg.workspace_upper
    px4.FINISH_POSITION_TOLERANCE, px4.FINISH_SPEED_TOLERANCE = .55, 1.
    old_replace = px4.replace
    px4.replace = lambda value, **changes: replace(value, **dict(changes, horizon=7))
    sys.argv = [flight.__name__, '--method', 'ours', '--frequency', '1', '--amplitude', '0', '--phase', '0', '--outdir', str(out)]
    try:
        flight.main()
    finally:
        px4.replace = old_replace
        subprocess.run(['docker', 'exec', '-e', 'GZ_PARTITION='+PARTITION, CONTAINER, 'gz', 'service',
            '-s', '/world/convex_seven_dynamic_px4_togt/control', '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', 'pause: true'], capture_output=True, timeout=5)
        if out.exists():
            save(out/'reference_activation.json', dict(activated=bool(references and references[0].active is not None)))
        if (out/'result.json').is_file():
            raw = json.loads((out/'result.json').read_text())
            raw.update(experiment_case=dict(layout='togt_whole', level=record['level']),
                experiment_variant=record['variant'], reference_manifest=record,
                nominal_time_s=float(plan.durations.sum()), planning_scheduler='none_frozen_reference',
                tracking_error_replan_enabled=False, reference_retiming=False,
                reference_attitude='north tilt/heading pi/2', global_planning_s=record['planning_total_s'])
            raw['contacts'] = world.parse_all_contacts(out, track, 30.+raw['last_time_s'])
            raw['collision_detected'] = any(c['contact_pair_count'] for c in raw['contacts'])
            save(out/'result.json', raw)
            shutil.copyfile(reference_dir/'reference.npz', out/'reference_trajectory.npz')
            if raw['last_time_s'] > .01:
                print('AUDIT', json.dumps(audit(out), ensure_ascii=False), flush=True)
            else:
                save(out/'comparison_audit.json', dict(passed=False, verdict='startup_failure',
                    flight_time_s=None, collision_detected=None, failure=raw['failure'], actual_gate_count=0))


if __name__ == '__main__':
    main()
