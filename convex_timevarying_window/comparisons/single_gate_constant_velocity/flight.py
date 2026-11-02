import argparse
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
from . import scenario
from .planner import Config
from .worker import Worker
from convex_timevarying_window.event_triggered_mpc.scheduler import Scheduler as ParentScheduler


class Scheduler(ParentScheduler):
    def observe(self,now,gate,exited,error):
        # Single gate plus fixed terminal: keep the accepted full trajectory.
        # No error-triggered replanning or post-exit change of the goal leg.
        if not self.started:
            self.started=True;self.trigger('initial',now,gate)


def main():
    p=argparse.ArgumentParser();p.add_argument('--speed',type=float,required=True)
    p.add_argument('--method',choices=('ours','center'),required=True)
    p.add_argument('--outdir',type=Path,required=True);a=p.parse_args();out=a.outdir.resolve()
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight,px4
    from convex_timevarying_window.togt_style_mpc.aligned_controller import MPC
    from convex_timevarying_window.togt_style_mpc.aligned_session import Session
    from convex_timevarying_window.single_gate_stop_racing.events import Progress
    flight.Worker=Worker;flight.Scheduler=Scheduler
    flight.variant_config=lambda variant:Config(free_points=variant=='ours',full_task_error_recovery=False)
    case=scenario.Case(a.speed)
    flight.build_case=lambda unused:scenario.build_case(case)
    flight.launch=scenario.launch;flight.manifest=lambda unused:scenario.manifest(case)
    flight.MAX_FLIGHT_TIME=40
    px4.MPC=MPC;px4.Session=Session;px4.Progress=Progress
    px4.CONTACT_TOPICS=px4.CONTACT_TOPICS[:1]
    sys.argv=[flight.__name__,'--method',a.method,'--frequency','1','--amplitude','0',
              '--phase','0','--outdir',str(out)]
    try:flight.main()
    finally:
        subprocess.run(['docker','exec','-e','GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment','gz','service','-s','/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000',
            '--req','pause: true'],capture_output=True,timeout=5)
        if (out/'result.json').exists():
            raw=json.loads((out/'result.json').read_text())
            raw.update(case=asdict(case),method='constant_velocity_hard_'+a.method,
                tracker_variant='execution-aligned',tracking_error_replan_enabled=False,
                post_trajectory_validation_enabled=True,closed_loop_certificate=False,
                hard_dynamics='17 nodes per piece; not continuous certification',
                local_safety='floating-point Bernstein/affine bounds',
                planning_scheduler='initial event and rejection retries only')
            (out/'result.json').write_text(json.dumps(raw,indent=2)+'\n')
            from .audit import audit
            print('AUDIT',json.dumps(audit(out)),flush=True)


if __name__=='__main__':main()
