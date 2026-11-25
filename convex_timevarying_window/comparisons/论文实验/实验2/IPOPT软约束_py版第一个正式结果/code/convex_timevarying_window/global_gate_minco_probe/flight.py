"""Track a frozen global trajectory; planner and physical course stay unchanged."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
import numpy as np
from .phase_descent import load_plan
from .run import save
from convex_timevarying_window.wang_free_state_mpc.reference import Reference as Timeline
from convex_timevarying_window.single_gate_stop_racing import scenario
from convex_timevarying_window.single_gate_stop_racing.planner import Config, route_signs
from convex_timevarying_window.single_gate_stop_racing.scheduler import Scheduler as EventScheduler
from convex_timevarying_window.single_gate_stop_racing.events import Progress as LocalProgress
from convex_timevarying_window.single_gate_stop_racing.run import sources


class Reference(Timeline):
    def __init__(self, start, plan):
        super().__init__(start)
        self.frozen_plan = plan
        self.contract_log = []

    def advance(self, now, gate=None, phase=None):
        with self.lock:
            if gate is not None: self.gate = gate
            if phase is not None: self.phase = phase
            # Do not preview/accelerate before the race clock starts.
            if now >= 0. and self.active is None:
                self.active = self.frozen_plan
            return self.active


class Scheduler(EventScheduler):
    def observe(self, *args): pass
    def due(self, now): return False


class Worker:
    def __init__(self, track, cfg):
        self.geometry = SimpleNamespace(windows=track.windows, signs=route_signs(track))
        self.metrics = []; self.busy = False
        self.startup = {'offline_precomputed_global_plan': True}
    def poll(self): return None
    def close(self): pass


class Progress(LocalProgress):
    """A remote crossing of an infinite plane is not a finite gate attempt."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.outside_plane_transits = []

    def update(self, p0, p1, t0, t1):
        if self.gate < len(self.windows) and not self.crossed:
            w = self.windows[self.gate]; sign = self.signs[self.gate]
            a = sign*w.world_to_local(p0, t0)[1]
            b = sign*w.world_to_local(p1, t1)[1]
            if a >= 0. and b < 0.:
                fraction = a/(a-b); at = t0+fraction*(t1-t0)
                q, _ = w.world_to_local((1-fraction)*np.asarray(p0)+fraction*np.asarray(p1), at)
                distance = float(w.aperture.boundary_distance(q))
                if not w.aperture.contains(q) and distance > self.radius+self.thickness:
                    self.outside_plane_transits.append(dict(gate=self.gate, time=float(at),
                        local_point=q.tolist(), distance_to_frame_m=distance))
                    return False
        return super().update(p0, p1, t0, t1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan-dir', required=True, type=Path)
    parser.add_argument('--outdir', required=True, type=Path)
    parser.add_argument('--native-camera', action='store_true', help='GUI without the separate camera anchor helper')
    parser.add_argument('--tracker', choices=('togt-style','px4-model','execution-aligned','single-shooting'), default='togt-style',
                        help='Rotor NMPC to CTBR (default), or the archived 33-state tracker')
    args = parser.parse_args(); out = args.outdir.resolve(); src = args.plan_dir.resolve()
    result = json.loads((src/'result.json').read_text())
    protocol = json.loads((src/'protocol.json').read_text())
    checked = json.loads((src/'independent_audit.json').read_text())
    if not result['found'] or not checked['ordered_single_forward_crossings']:
        raise ValueError('Require an accepted and independently event-audited global plan')
    if out.exists(): raise FileExistsError(out)
    plan = load_plan(result); case = scenario.Case(**protocol['case'])
    from convex_timevarying_window.simple_two_gate_soft_only_mpc import flight, px4
    if args.tracker == 'single-shooting':
        from convex_timevarying_window.togt_style_mpc.shooting_controller import MPC
    elif args.tracker == 'execution-aligned':
        from convex_timevarying_window.togt_style_mpc.aligned_controller import MPC
    elif args.tracker == 'togt-style':
        from convex_timevarying_window.togt_style_mpc.controller import MPC
    else:
        from convex_timevarying_window.single_gate_stop_racing.px4_mpc import MPC
    from convex_timevarying_window.single_gate_stop_racing.metric_session import Session, metric_scale
    if args.tracker in ('execution-aligned','single-shooting'):
        from convex_timevarying_window.togt_style_mpc.aligned_session import Session
    from convex_timevarying_window.single_gate_stop_racing.audit import audit
    flight.Worker = Worker; flight.Scheduler = Scheduler
    flight.Reference = lambda start, count: Reference(start, plan)
    flight.variant_config = lambda variant: Config()
    flight.build_case = scenario.build_case
    flight.launch = lambda track, assets, outdir: scenario.launch(track, assets, outdir, camera_helper=not args.native_camera)
    flight.manifest = scenario.manifest
    flight.MAX_FLIGHT_TIME = 120
    px4.WORKSPACE_LOWER, px4.WORKSPACE_UPPER = scenario.workspace_bounds()
    progress_instances = []
    def progress_factory(*args, **kwargs):
        progress = Progress(*args, **kwargs); progress_instances.append(progress); return progress
    px4.MPC = MPC; px4.Progress = progress_factory; px4.Session = Session
    root = Path(__file__).resolve().parents[2]; frozen = sources(root)
    sys.argv = [flight.__name__, '--method', 'ours', '--frequency', str(case.frequency),
                '--amplitude', str(case.amplitude), '--phase', str(case.phase), '--outdir', str(out)]
    try:
        flight.main()
    finally:
        subprocess.run(['docker', 'exec', '-e', 'GZ_PARTITION=convex_seven_togt_experiment_partition',
            'convex_seven_togt_experiment', 'gz', 'service', '-s', '/world/convex_seven_dynamic_px4_togt/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
            '--req', 'pause: true'], capture_output=True, timeout=5)
        if out.exists():
            for rel in frozen:
                dst = out/'source_snapshot'/rel; dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(root/rel, dst)
            save(out/'frozen_sources.json', frozen)
            save(out/'verification.json', dict(sources_unchanged=sources(root)==frozen))
        if (out/'result.json').exists():
            import xml.etree.ElementTree as ET
            latitude = float(ET.parse(out/'world.sdf').getroot().findtext('./world/spherical_coordinates/latitude_deg'))
            raw = json.loads((out/'result.json').read_text())
            full_state = protocol.get('independent_gate_acceleration_jerk', False)
            raw.update(method='global_pvaj_minco_offline_mpc' if full_state else 'global_pv_minco_offline_mpc',
                global_plan_directory=str(src), planning_horizon_windows=plan.gate_count,
                independent_gate_acceleration_jerk=full_state,
                camera_helper_enabled=not args.native_camera,
                tracker_variant=args.tracker,
                planning_thrust_fraction=protocol.get('planning_thrust_fraction',1.),
                outside_plane_transits=progress_instances[0].outside_plane_transits if progress_instances else [],
                reference_trajectory_sha256=hashlib.sha256((src/'trajectory.npz').read_bytes()).hexdigest(),
                nominal_flight_time_s=float(plan.durations.sum()), planning_scheduler='none_global_frozen',
                offline_planning_s=result['solve_s'], first_gate_sideways_shift_m=scenario.FIRST_GATE_SHIFT_M,
                horizontal_spacing_scale=scenario.HORIZONTAL_SPACING_SCALE, return_to_start_required=True,
                tracking_error_replan_enabled=False, closed_loop_safety_certificate=False,
                position_coordinate_mapping='PX4_spherical_GPS_map_to_Gazebo_WGS84_metres',
                position_scale_enu=metric_scale(latitude).tolist(), ground_truth_feedback_enabled=False)
            save(out/'result.json', raw)
            shutil.copyfile(src/'trajectory.npz', out/'reference_trajectory.npz')
            audited = audit(out)
            truth = np.load(out/'truth_trace.npz')
            audited['actual_distance_m'] = float(np.linalg.norm(np.diff(truth['positions'],axis=0),axis=1).sum())
            save(out/'comparison_audit.json', audited)
            print('PHYSICAL_AUDIT', json.dumps(audited), flush=True)


if __name__ == '__main__': main()
