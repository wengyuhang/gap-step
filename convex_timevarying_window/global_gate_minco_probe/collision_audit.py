"""Recover evidence from an interrupted flight; never fabricate a normal result.json."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from .phase_descent import load_plan
from .run import save
from convex_dynamic_seven_window_gazebo.px4_togt_nmpc_experiment import parse_contacts
from convex_timevarying_window.single_gate_stop_racing.scenario import Case, build_case
from convex_timevarying_window.single_gate_stop_racing.events import Progress
from convex_timevarying_window.single_gate_stop_racing.planner import route_signs
from convex_timevarying_window.three_phase_mppi.report import continuous_quaternions


def audit(out, source):
    protocol = json.loads((source/'protocol.json').read_text())
    plan = load_plan(json.loads((source/'result.json').read_text()))
    cfg = json.loads((source/'build.json').read_text())['config']
    track, _ = build_case(Case(**protocol['case']))
    contacts = [parse_contacts(out/f'gate_{i:02d}_contacts.pbtxt') for i in range(1, 8)]
    first = min(c['first_contact_sim_time_s'] for c in contacts if c['contact_pair_count'])-30.
    u = ULog(str(out/'flight_recovered_after_interrupt.ulg'))
    d = u.get_dataset('vehicle_local_position_groundtruth').data
    a = u.get_dataset('vehicle_attitude_groundtruth').data
    t = d['timestamp']*1e-6
    selected = (t >= 30.) & (t < 30.+first)
    t = t[selected]
    aq = np.column_stack([a[k] for k in ('q[1]', 'q[2]', 'q[3]', 'q[0]')])
    valid = np.isfinite(aq).all(axis=1) & (np.linalg.norm(aq, axis=1) > .5)
    q = continuous_quaternions(aq[valid])
    qs = np.column_stack([np.interp(t, a['timestamp'][valid]*1e-6, q[:, i]) for i in range(4)])
    p = np.column_stack([d[k][selected] for k in ('x', 'y', 'z')])
    p += Rotation.from_quat(qs).apply(np.tile([0., 0., -.24], (len(t), 1)))
    p = p[:, [1, 0, 2]]*np.array([1., 1., -1.]); t -= 30.
    ref = plan.sample_many(t)
    err = np.linalg.norm(p-ref, axis=1)
    progress = Progress(list(track.windows), route_signs(track), cfg['radius'], cfg['frame_thickness'])
    for at, pos in zip(t, p): progress.observe(pos, float(at))
    gate = next(i for i, c in enumerate(contacts) if c['contact_pair_count'])
    w = track.windows[gate]
    rows = []
    for at in (first-.1, first-.05, first-.02, first-.01, t[-1]):
        i = int(np.argmin(abs(t-at))); at = float(t[i])
        actual_q, actual_z = w.world_to_local(p[i], at)
        nominal_q, nominal_z = w.world_to_local(ref[i], at)
        eps = 1e-5
        gate_v = (w.state_at(at+eps)[0]-w.state_at(at-eps)[0])/(2*eps)
        rows.append(dict(time_s=at, error_m=float(err[i]), actual=p[i].tolist(), reference=ref[i].tolist(),
            actual_local=[*actual_q.tolist(), float(actual_z)], nominal_local=[*nominal_q.tolist(), float(nominal_z)],
            actual_frame_centerline_distance_m=float(np.hypot(w.aperture.boundary_distance(actual_q), actual_z)),
            nominal_frame_centerline_distance_m=float(np.hypot(w.aperture.boundary_distance(nominal_q), nominal_z)),
            gate_velocity_mps=gate_v.tolist()))
    logs = [json.loads(line) for line in (out/'mpc_live.jsonl').read_text().splitlines()]
    result = dict(completed=False, collision_detected=True, flight_time_s=None,
        first_contact_race_time_s=first, collision_gate=gate+1, contacts=contacts,
        safe_body_exits_before_first_contact=len(progress.exits), pre_contact_crossings=progress.crossings,
        pre_contact_body_exits=progress.exits, reference_time_s=float(plan.durations.sum()),
        actual_distance_before_contact_m=float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
        pre_contact_truth_error_m=dict(median=float(np.median(err)), p95=float(np.quantile(err, .95)), maximum=float(err.max())),
        last_pre_contact_samples=rows, required_nominal_distance_m=cfg['radius']+cfg['frame_thickness']+cfg['passage_margin'],
        mpc_pre_contact_max_ms=max(v['cycle_ms'] for v in logs if v['time'] < first),
        frozen_sources_unchanged=json.loads((out/'verification.json').read_text())['sources_unchanged'],
        reference_sha256=hashlib.sha256((source/'trajectory.npz').read_bytes()).hexdigest(),
        normal_result_missing_reason='SIGINT during stalled logger stop after collision and invalid quaternion; raw ULog manually recovered',
        evidence='native-rate truth strictly before first contact plus raw Gazebo frame contacts; not a continuous safety certificate')
    save(out/'interrupted_collision_audit.json', result)
    np.savez_compressed(out/'pre_contact_truth.npz', times=t, positions=p, reference=ref)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    parser.add_argument('--plan-dir', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.outdir, args.plan_dir), indent=2))
