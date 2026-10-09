"""Bind every post-flight check to the actual fixed-plane course."""
import argparse
import json
from pathlib import Path

from .scenario import Case, build_case, audit_scene, check_world
from convex_timevarying_window.comparisons.checked_soft_racing.run import analyze_run, save
from convex_timevarying_window.comparisons.checked_soft_racing.adjudicate import assess
from convex_timevarying_window.comparisons.checked_soft_mpc_racing.run import inspect
from convex_timevarying_window.comparisons.wang_free_state_mpc.run import apply_passage
from convex_timevarying_window.gs_mppi.direct_ctbr.compare_targets import body_dynamics
from convex_timevarying_window.wang_free_state_mpc.passage_audit import audit_run


def inspect_run(path, method):
    path = Path(path)
    raw = json.loads((path/'result.json').read_text())
    case = Case(**json.loads((path/'scenario.json').read_text())['case'])
    if raw.get('scenario_adapter') != 'fixed_plane_wang' or raw['scenario']['case'] != vars(case):
        raise ValueError('Missing or inconsistent fixed-plane scene provenance')
    parity = check_world(build_case(case)[0], path/'world.sdf')
    row = dict(infrastructure_failure=False, physical_success=False, realtime_success=False,
               flight_time_s=raw.get('flight_time_s'), last_time_s=raw.get('last_time_s'),
               failure=raw.get('failure'), collision=raw.get('collision_detected', False))
    with audit_scene(case):
        if method == 'ours':
            row.update(inspect(path))
        elif method == 'wang':
            row['infrastructure_failure'] = raw.get('failure') == "RuntimeError('arm failed')"
            if not row['infrastructure_failure']:
                try:
                    row.update(analyze_run(path))
                    maximum = raw.get('sender_max_ms')
                    row['sender_max_ms'] = maximum
                    row['controller_deadline_passed'] = maximum is not None and maximum < 100.
                    row['realtime_success'] &= row['controller_deadline_passed']
                    row['dynamics'] = body_dynamics(path, raw['last_time_s'])
                    row['strict_dynamics_success'] = bool(row['realtime_success'] and
                        row['dynamics']['satisfies_direct_dynamics_limits'])
                except Exception as exc:
                    row.update(physical_success=False, realtime_success=False, analysis_error=repr(exc))
                try:
                    row['preparation_assessment'] = assess(path)
                    row['infrastructure_failure'] = row['preparation_assessment']['setup_failure']
                except Exception as exc:
                    row['preparation_assessment'] = dict(setup_failure=False, error=repr(exc))
        else:
            raise ValueError(method)
        try:
            passage = audit_run(path)
        except Exception as exc:
            passage = dict(passed=False, violations=['audit_error'], error=repr(exc))
        save(path/'passage_audit.json', passage)
        row = apply_passage(row, passage)
    row['scene_parity'] = parity
    save(path/'scene_audit.json', dict(case=vars(case), method=method, **row))
    return row


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--method', choices=('ours','wang'), required=True)
    a = p.parse_args()
    print(json.dumps(inspect_run(a.run, a.method), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
