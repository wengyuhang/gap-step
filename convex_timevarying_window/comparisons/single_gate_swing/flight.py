import argparse
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
from . import scenario
from .planner import Config
from .worker import Worker
from .contacts import parse_contacts
from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC as TrackingMPC
from convex_timevarying_window.comparisons.single_gate_constant_velocity.flight import Scheduler


def main():
    parser = argparse.ArgumentParser()
    for key in ('frequency','amplitude'):
        parser.add_argument('--'+key,type=float,required=True)
    parser.add_argument('--method',choices=('ours','center'),required=True)
    parser.add_argument('--outdir',type=Path,required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight, px4
    MPC=TrackingMPC
    from convex_timevarying_window.togt_style_mpc.aligned_session import Session
    from convex_timevarying_window.single_gate_stop_racing.events import Progress
    case = scenario.Case(args.frequency,args.amplitude,0.)
    flight.Worker = Worker
    flight.Scheduler = Scheduler
    flight.variant_config = lambda variant: Config(free_points=variant=='ours',full_task_error_recovery=False)
    flight.build_case = lambda unused: scenario.build_case(case)
    flight.launch = scenario.launch
    flight.manifest = lambda unused: scenario.manifest(case)
    flight.MAX_FLIGHT_TIME = 40
    class CheckedSession(Session):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            publishers = {}
            for topic in (*px4.CONTACT_TOPICS[:1],scenario.GROUND_TOPIC):
                info = subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
                    'convex_seven_togt_experiment','gz','topic','-i','-t',topic],
                    check=True,capture_output=True,text=True,timeout=8)
                publishers[topic] = info.stdout
                if 'Publishers [Address, Message Type]:' not in info.stdout or 'gz.msgs.Contacts' not in info.stdout:
                    self.close()
                    raise RuntimeError('contact_sensor_missing_publisher:'+topic)
            (self.out/'contact_publishers.json').write_text(json.dumps(publishers,indent=2)+'\n')
    px4.MPC, px4.Session, px4.Progress = MPC, CheckedSession, Progress
    px4.CONTACT_TOPICS = (*px4.CONTACT_TOPICS[:1],scenario.GROUND_TOPIC)
    px4.parse_contacts = parse_contacts
    sys.argv = [flight.__name__,'--method',args.method,'--frequency',str(args.frequency),
                '--amplitude',str(args.amplitude),'--phase','0','--outdir',str(out)]
    try:
        flight.main()
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000',
            '--req','pause: true'],capture_output=True,timeout=5)
        if (out/'result.json').exists():
            raw = json.loads((out/'result.json').read_text())
            contacts = [parse_contacts(out/f'gate_{i:02d}_contacts.pbtxt',30.+raw['last_time_s']) for i in (1,2)]
            raw.update(case=asdict(case),method='swing_hard_'+args.method,contacts=contacts,
                collision_detected=any(c['contact_pair_count'] for c in contacts),
                tracker_variant=('single-shooting' if raw.get('tracker_metadata',{}).get('implementation')=='execution_aligned_single_shooting_ctbr_mpc' else 'execution-aligned'),
                tracking_error_replan_enabled=False,
                post_trajectory_validation_enabled=True,closed_loop_certificate=False,
                hard_dynamics='17 nodes per piece; not continuous certification',
                local_safety='floating-point Bernstein/sine bounds',physics_step_s=scenario.PHYSICS_STEP,
                planning_scheduler='initial event and rejection retries only')
            (out/'result.json').write_text(json.dumps(raw,indent=2)+'\n')
            from .audit import audit
            print('AUDIT',json.dumps(audit(out)),flush=True)


if __name__=='__main__':
    main()
