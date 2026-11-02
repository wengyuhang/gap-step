"""Read frozen planning records; draw optimizer points, NOT flight paths."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from shapely.geometry import Polygon

from convex_timevarying_window.dynamic_passage_velocity_minco.hard_dynamics import flat_actuators
from convex_timevarying_window.dynamic_passage_velocity_minco.allocation_mpc import KF
from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
from .scenario import VERTICES, LEFT_INITIAL_POINT, Case, build_case
from .run import save_json


def audit_plan(folder, case, radius):
    stored = np.load(folder / 'plan.npz')
    plan = Plan(0., 0, stored['durations'], stored['coefficients'], stored['x'],
                0., 0., 'offline_posthoc_audit', 1)
    t = np.linspace(0., plan.durations.sum(), int(np.ceil(plan.durations.sum() / .001)) + 1)
    states = [plan.sample_many(t, d) for d in (0, 1, 2, 3, 4)]
    F, rates, motors = flat_actuators.map(len(t))(*[v.T for v in states[1:]])
    motors, rates = np.asarray(motors), np.asarray(rates)
    w = build_case(case).windows[0]
    centers = w.center0 + np.sin(2*np.pi*t[:, None]/w.motion.translation_period) * w.motion.translation_amplitude
    q = (states[0] - centers) @ w.state_at(0.)[1]
    clearance = np.hypot(w.aperture.boundary_distance(q[:, :2]), q[:, 2]) - radius
    low, high = KF * 150**2, KF * 1000**2
    return dict(role='posthoc_1ms_sampling_not_continuous_dynamics_certificate',
                dt_max_s=float(np.diff(t).max()), sample_count=len(t),
                minimum_frame_clearance_after_all_margins_m=float(clearance.min()),
                minimum_rotor_thrust_N=float(motors.min()), maximum_rotor_thrust_N=float(motors.max()),
                rotor_bounds_N=[low, high],
                maximum_relative_rotor_upper_violation=max(0., float(motors.max()/high-1)),
                maximum_relative_rotor_lower_violation=max(0., float((low-motors.min())/high)),
                maximum_abs_body_rates_rad_s=np.max(abs(rates), axis=1).tolist(),
                body_rate_limits_rad_s=[3.8, 3.8, 3.49])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).parent / 'results')
    parser.add_argument('--control-name', default='right_seed_control_v2_20261012')
    parser.add_argument('--analysis-name', default='comparison_v2_20261012')
    args = parser.parse_args()
    out = args.root / args.analysis_name
    if out.exists():
        raise FileExistsError('Preserve previous analyses')
    out.mkdir()
    primary = args.root / 'left_seed_v1_20261012'
    manifest = json.loads((primary / 'manifest.json').read_text())
    static = json.loads((primary / 'f3_a0/result.json').read_text())
    dynamic = json.loads((primary / 'f3_a2/result.json').read_text())
    normalized = json.loads((args.root / 'left_normalized_diagnostic_v1_20261012/f3_a2/result.json').read_text())
    control_folder = args.root / args.control_name
    control = json.loads((control_folder / 'result.json').read_text())
    rho = manifest['check_radius_m']
    membership = {}
    for name, data in [('static', static), ('dynamic_left', dynamic),
                       ('normalized_left', normalized), ('right_control', control)]:
        ap = build_case().windows[0].aperture
        membership[name] = sum(not ap.contains(v['q'], tolerance=1e-7) for v in data['evaluations'])
    save_json(out / 'posthoc_audit.json', dict(
        static=audit_plan(primary / 'f3_a0', Case(3., 0.), rho),
        right_control=audit_plan(control_folder, Case(**dynamic['case']), rho),
        number_of_SC_evaluations_outside_physical_polygon=membership))
    erosion = Polygon(VERTICES).buffer(-rho)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.2), gridspec_kw={'width_ratios': [1.45, 1]})
    outline = np.vstack([VERTICES, VERTICES[0]])
    for ax in axes:
        ax.fill(outline[:, 0], outline[:, 1], color='#f0f0f0')
        ax.plot(outline[:, 0], outline[:, 1], 'k-', lw=1.6)
        for i, part in enumerate(erosion.geoms):
            q = np.asarray(part.exterior.coords)
            ax.fill(q[:, 0], q[:, 1], color='#94cfa2', alpha=.65,
                    label='Body + frame + margin feasible at plane' if i == 0 else None)
        ax.scatter(*LEFT_INITIAL_POINT, s=180, marker='*', color='#2166ac', zorder=6,
                   label='Requested left initial point')
        for i in range(4):
            trace = [v for v in dynamic['stats']['trace'] if v['seed'] == i]
            if trace:
                points = np.array([v['q'] for v in trace])
                ax.plot(points[:, 0], points[:, 1], '.-', color='#d95f02', alpha=.6, lw=.6,
                        markersize=3, label='Moving gate: left-seeded iterates' if i == 0 else None)
        q = static['final']['q']
        ax.scatter(*q, s=55, marker='s', color='#7570b3', zorder=7,
                   label='Static accepted point (left)')
        ax.grid(alpha=.2)
        ax.set_aspect('equal')
        ax.set_xlabel('Gate-local horizontal coordinate (m)')
        ax.set_ylabel('Gate-local vertical coordinate (m)')
    q = control['final']['q']
    axes[0].scatter(*q, s=80, marker='o', color='#17803b', zorder=7,
                    label='Right-initialized control: accepted (NOT migration)')
    axes[0].add_patch(Circle(q, rho, edgecolor='#17803b', fill=False, lw=1.2))
    axes[0].scatter(*control['initial_q'], s=65, marker='D', color='#17803b', zorder=5,
                    label='Right control initial point')
    axes[0].set_title('3 Hz / 2 m: primary left-seeded solve FAILED\nRight seed is a separate feasibility control')
    axes[0].set_xlim(-1.85, 3.45)
    axes[0].set_ylim(-1.85, 1.85)
    axes[0].legend(loc='upper center', bbox_to_anchor=(.55, -.17), fontsize=7)
    axes[1].set_title('Zoom: every primary time seed remains LEFT\nThese lines are optimizer iterates, NOT flight')
    axes[1].set_xlim(-1.15, -.85)
    axes[1].set_ylim(-.10, .17)
    fig.tight_layout()
    fig.subplots_adjust(bottom=.28)
    fig.savefig(out / 'result_overview.png', dpi=190, bbox_inches='tight')
    plt.close(fig)
    print(str(out), flush=True)


if __name__ == '__main__':
    main()
