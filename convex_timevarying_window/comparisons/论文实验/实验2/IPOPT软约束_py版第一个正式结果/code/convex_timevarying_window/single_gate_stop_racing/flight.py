"""Horizontally spread fixed-plane seven-gate course; one gate per replan."""
import argparse
import json
from pathlib import Path
import sys
from .planner import Config
from .worker import Worker
from .events import Progress
from .scheduler import Scheduler
from .metric_session import Session,metric_scale


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument('--tracker',choices=('togt-style','px4-model','single-shooting'),default='togt-style')
    selected,remaining=parser.parse_known_args()
    sys.argv=[sys.argv[0],*remaining]
    session_type=Session
    if selected.tracker=='single-shooting':
        from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC
        from convex_timevarying_window.togt_style_mpc.aligned_session import Session as session_type
    elif selected.tracker=='togt-style':
        from convex_timevarying_window.togt_style_mpc.controller import MPC
    else:
        from .px4_mpc import MPC
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight,px4
    from . import scenario
    flight.Worker=Worker;flight.Scheduler=Scheduler
    flight.variant_config=lambda variant:Config(free_points=variant=='ours')
    flight.build_case=scenario.build_case;flight.launch=scenario.launch;flight.manifest=scenario.manifest
    px4.WORKSPACE_LOWER,px4.WORKSPACE_UPPER=scenario.workspace_bounds()
    flight.MAX_FLIGHT_TIME=120;px4.MPC=MPC;px4.Progress=Progress;px4.Session=session_type
    out=Path(sys.argv[sys.argv.index('--outdir')+1]).resolve()
    try:flight.main()
    finally:
        path=out/'result.json'
        if path.exists():
            import xml.etree.ElementTree as ET
            latitude=float(ET.parse(out/'world.sdf').getroot().findtext('./world/spherical_coordinates/latitude_deg'))
            raw=json.loads(path.read_text());raw.update(method='single_gate_stop_racing_'+selected.tracker,
                tracker_variant=selected.tracker,
                position_coordinate_mapping='PX4_spherical_GPS_map_to_Gazebo_WGS84_metres',
                position_scale_enu=metric_scale(latitude).tolist(),
                mapping_datum_latitude_deg=latitude,
                ground_truth_feedback_enabled=False,
                prediction_model=('x500_13state_rotor_thrust_rigidbody' if selected.tracker=='togt-style'
                                  else 'PX483c4f4e_PID_normalized_allocation_x500_drag_moments'),
                first_gate_sideways_shift_m=scenario.FIRST_GATE_SHIFT_M,
                horizontal_spacing_scale=scenario.HORIZONTAL_SPACING_SCALE,
                other_gate_avoidance_enabled=False,
                tracking_error_replan_enabled=False,
                closed_loop_safety_certificate=False,return_to_start_required=True,
                last_exit_replan_enabled=False,
                evidence='PX4/Gazebo body events and physical contacts; independent audit reported separately')
            path.write_text(json.dumps(raw,indent=2)+'\n')
        if out.exists():
            import hashlib
            import shutil
            root=Path(__file__).resolve().parents[2]
            files=list(Path(__file__).parent.glob('*.py'))
            if selected.tracker in ('togt-style','single-shooting'):
                files+=list((root/'convex_timevarying_window/togt_style_mpc').glob('*.py'))
            hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
            (out/'method_source_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
            for p in files:
                dst=out/'source_snapshot'/p.relative_to(root);dst.parent.mkdir(parents=True,exist_ok=True)
                shutil.copyfile(p,dst)


if __name__=='__main__':main()
