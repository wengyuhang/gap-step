import json
from pathlib import Path
import sys
from . import scenario
from .joint_planner import Config,Worker
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import MPC


def main():
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight,px4
    flight.Worker=Worker;flight.variant_config=lambda variant:Config(free_points=variant=='ours')
    flight.build_case=scenario.build_case;flight.launch=scenario.launch;flight.manifest=scenario.manifest
    flight.MAX_FLIGHT_TIME=60;px4.MPC=MPC
    output=Path(sys.argv[sys.argv.index('--outdir')+1]).resolve()
    try:flight.main()
    finally:
        path=output/'result.json'
        if path.exists():
            raw=json.loads(path.read_text())
            raw.update(method='two_gate_joint_hard_v2',joint_two_gate_optimization=True,
                gate_count=2,closed_loop_safety_certificate=False,
                evidence='PX4 MPC closed loop, measured crossing/body-exit events and Gazebo contacts; no independent truth audit or safety certificate')
            path.write_text(json.dumps(raw,indent=2)+'\n')


if __name__=='__main__':main()
