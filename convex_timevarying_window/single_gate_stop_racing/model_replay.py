"""Recorded-input model validation; never changes a saved flight or its result."""
import argparse
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from scipy.spatial.transform import Rotation
from .px4_model import dynamics, initial_state, KF, BODY
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import dynamics as old_dynamics


def series(ulog, name, fields):
    d = ulog.get_dataset(name).data
    return d['timestamp']*1e-6-30., np.column_stack([d[k] for k in fields])


def interpolate(times, values, at):
    return np.array([np.interp(at, times, values[:, i]) for i in range(values.shape[1])])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--flight', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True); args = ap.parse_args()
    ul = ULog(str(args.flight/'flight.ulg'))
    e = np.load(args.flight/'execution.npz')['rows']
    _, ind = np.unique(e[:, 0][::-1], return_index=True); e = e[np.sort(len(e)-1-ind)]
    def physical(t):
        x = interpolate(e[:, 0], e[:, 1:18], t); x[6:10] /= np.linalg.norm(x[6:10]); return x
    ct, cv = series(ul, 'vehicle_rates_setpoint', ['roll', 'pitch', 'yaw', 'thrust_body[2]'])
    controls = np.c_[4*KF*(150+850*(-cv[:, 3]))**2, cv[:, :3]*BODY]
    tt, tv = series(ul, 'vehicle_torque_setpoint', ['xyz[0]', 'xyz[1]', 'xyz[2]'])
    mt, mv = series(ul, 'actuator_outputs', [f'output[{i}]' for i in range(4)])
    it, iv = series(ul, 'rate_ctrl_status', ['rollspeed_integ', 'pitchspeed_integ', 'yawspeed_integ'])
    tick, step, _, allocate = dynamics(); _, old_step, _ = old_dynamics()
    # Check normalized allocation independently of rate response and state bias.
    alloc_errors = []
    for t, actual in zip(mt, mv):
        if not 1. <= t < 40.:
            continue
        k = np.searchsorted(ct, t, side='right')-1
        j = np.searchsorted(tt, t, side='right')-1
        predicted = np.asarray(allocate(np.r_[-cv[k, 3], tv[j]*BODY])).ravel()
        alloc_errors.append(predicted-actual)
    alloc_errors = np.asarray(alloc_errors)
    # Internal-state observer from the beginning, corrected by measured state.
    # No logged integrator or future measurement is injected into this observer.
    observer = initial_state(physical(max(-1.5, e[0, 0])))
    start = max(-1.5, e[0, 0]); snapshots = {}
    targets = np.array([2., 5., 10., 15., 20., 25., 30., 33., 36., 37., 37.5, 38., 38.5, 39.])
    while start < targets[-1]+1e-8:
        for target in targets:
            if abs(start-target) < .00201:
                snapshots[float(target)] = observer.copy()
        end = min(start+.004, targets[-1]+.004)
        k = np.searchsorted(ct, start, side='right')-1
        observer = np.asarray(tick(observer, controls[max(k, 0)], end-start)).ravel()
        observer[:17] = physical(end); start = end
    rows = []
    for start in targets:
        stop = start+1.; actual = physical(stop)
        for mode in ('old', 'aligned_observer', 'aligned_logged_integral'):
            x = physical(start) if mode == 'old' else snapshots[float(start)].copy()
            if mode == 'aligned_logged_integral':
                x[17:20] = interpolate(it, iv, start)*BODY
            # Both models use identical 4 ms command sampling. The gyro biquad
            # must not be re-clocked at irregular received-command intervals.
            knots = np.r_[np.arange(start, stop-1e-8, .004), stop]
            for left, right in zip(knots[:-1], knots[1:]):
                k = np.searchsorted(ct, left+1e-9, side='right')-1
                fn = old_step if mode == 'old' else tick
                x = np.asarray(fn(x, controls[max(k, 0)], right-left)).ravel()
            rp = Rotation.from_quat(x[[7, 8, 9, 6]]).as_matrix()
            ra = Rotation.from_quat(actual[[7, 8, 9, 6]]).as_matrix()
            rows.append(dict(start_s=float(start), model=mode,
                position_drift_m=float(np.linalg.norm(x[:3]-actual[:3])),
                velocity_drift_mps=float(np.linalg.norm(x[3:6]-actual[3:6])),
                thrust_axis_drift_deg=float(np.degrees(np.arccos(np.clip(rp[:, 2]@ra[:, 2], -1., 1.))))))
    result = dict(source=str(args.flight), duration_s=1., rollouts=rows,
        allocation_motor_error_rad_s=dict(median_abs=float(np.median(np.abs(alloc_errors))),
            p95_abs=float(np.quantile(np.abs(alloc_errors), .95)), max_abs=float(np.max(np.abs(alloc_errors)))),
        observer_integral_comparison=[dict(time_s=float(t), estimated=snapshots[float(t)][17:20].tolist(),
            logged=(interpolate(it, iv, t)*BODY).tolist()) for t in targets],
        limitations='Measured physical states and estimated motor states; sensor bias and timing remain. Not a flight or safety claim.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__': main()
