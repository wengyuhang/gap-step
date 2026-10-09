"""Read-only flight analysis; figures and derived evidence never change raw logs."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def collect(path):
    audit = json.loads((path/'comparison_audit.json').read_text())
    raw = json.loads((path/'result.json').read_text())
    data = np.load(path/'mechanism_trace.npz')
    crossing = audit['crossings'][0]['time_s']
    contacts = [c['first_contact_sim_time_s']-30 for c in raw['contacts']
                if c['first_contact_sim_time_s'] is not None]
    return dict(path=str(path), audit=audit, raw=raw, data=data, crossing=crossing,
                first_contact=min(contacts) if contacts else None)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args(); out = args.outdir.resolve()
    root = Path(__file__).resolve().parents[2]
    old = root/'convex_timevarying_window/togt_point_only_mpc/results/contact_sweep_s4_no_inset_v2_20261009'
    items = [collect(old/'single_frequency_ours_f3.5_a2_p000'),
             collect(out/'passage_ours_f3.5_a2_p000'), collect(out/'passage_center_f3.5_a2_p000')]
    # Verify physical scene and controller match even for the historical flight.
    fields = ('world_sha256', 'mpc_config', 'px4_parameters')
    assert all(all(x['audit'][k] == items[0]['audit'][k] for x in items) for k in fields)
    frozen = json.loads((out/'protocol.json').read_text())['source_sha256']
    unchanged = all(hashlib.sha256((root/rel).read_bytes()).hexdigest() == digest
                    for rel, digest in frozen.items())
    assert unchanged, 'A frozen source was modified'
    metadata = dict(analysis_added_after_flights=True, frozen_flight_sources_unchanged=unchanged,
                    world_and_mpc_and_px4_matched=True,
                    historical_baseline_not_reflown=True, repeated_trial_success_rate_not_estimated=True)
    rows = []
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'axes.unicode_minus': False})
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.3), sharex=True, sharey=True)
    labels = ('历史自由点：仅安全软代价', '本次自由点：动态穿越约束', '本次强制中心：动态穿越约束')
    for ax, item, label in zip(axes, items, labels):
        audit, raw, data = item['audit'], item['raw'], item['data']
        t = data['times']; dt = t-item['crossing']; local = data['local_positions']
        radius = raw['config']['radius']+raw['config']['frame_thickness']
        mask = (dt >= -.09) & (dt <= .09)
        overlap = np.abs(local[:, 2]) < radius
        half_width = 1.15+raw['config']['frame_offset']
        allowable = half_width-np.sqrt(np.maximum(radius**2-local[:, 2]**2, 0.))
        ax.axvspan(audit['entry_time_s']-item['crossing'],
                   audit['exit_time_s']-item['crossing'], color='.9', label='机体与窗平面重叠')
        ax.plot(dt[mask], local[mask, 1], color='#1766a3', lw=2.3, label='实际横向相对位置')
        selected = mask & overlap
        ax.plot(dt[selected], allowable[selected], color='#c43f3f', lw=1.8, label='物理球代理允许范围')
        ax.plot(dt[selected], -allowable[selected], color='#c43f3f', lw=1.8)
        ax.axvline(0., color='.3', ls=':', lw=1.)
        ax.axhline(0., color='.5', ls=':', lw=.8)
        q = np.array(audit['crossings'][0]['point_local_m'])
        ax.plot(0., q[1], 'o', color='#1766a3')
        if item['first_contact'] is not None:
            at = item['first_contact']; dx = at-item['crossing']
            ax.plot(dx, np.interp(at, t, local[:, 1]), 'x', color='red', ms=10, mew=2.5,
                    label='真实旋翼接触')
            ax.axvspan(dx, .09, color='#edc8c8', alpha=.3)
        gate_v = 2*np.pi*3.5*2*np.cos(2*np.pi*3.5*item['crossing'])
        ax.set_title(label+'\n'+f'横向穿越偏置 {q[1]:+.3f} m；窗速 {gate_v:+.2f} m/s', fontsize=10)
        ax.set_xlabel('相对机体中心过平面的时间 / s')
        ax.set_xlim(-.09, .09); ax.set_ylim(-1.6, 2.1); ax.grid(alpha=.2)
        attempts = raw['solves']
        rows.append(dict(method=label, source=item['path'], passed=audit['passed'],
            collision=audit['collision_detected'], completed=audit['completed'],
            flight_time_s=audit['flight_time_s'] if audit['passed'] else None,
            first_contact_time_s=item['first_contact'], crossing_time_s=item['crossing'],
            crossing_point_local_m=q.tolist(), gate_lateral_velocity_mps=float(gate_v),
            relative_lateral_velocity_mps=audit['crossing_relative_lateral_speed_mps'],
            body_plane_overlap_duration_s=audit['exit_time_s']-audit['entry_time_s'],
            requests=len(attempts), accepted_requests=sum(s['accepted'] for s in attempts),
            planning_max_end_to_end_ms=max(s['cycle_ms'] for s in attempts),
            mpc_max_cycle_ms=max(s['cycle_ms'] for s in raw['mpc_cycles']),
            offline_planner_build_s=raw['build_s'],
            diagnostic_proxy_clearance_m=audit['crossing_min_proxy_clearance_m']))
    axes[0].set_ylabel('无人机相对窗口中心的横向位置 / m')
    axes[0].legend(fontsize=8, loc='lower left')
    axes[1].legend(fontsize=8, loc='lower left')
    fig.suptitle('同一原尺寸单窗：3.5 Hz / 2 m。是否碰撞以 Gazebo 接触为准\n'
                 '历史图红色背景为接触后数据，不作为正常跟踪性能', fontsize=12)
    fig.tight_layout(); fig.savefig(out/'relative_passage.png', dpi=160); plt.close(fig)
    metadata['methods'] = rows
    a, b = rows[1]['flight_time_s'], rows[2]['flight_time_s']
    metadata.update(free_minus_center_time_s=a-b, free_improvement_percent=100*(b-a)/b)
    (out/'mechanism_analysis.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(metadata, ensure_ascii=False, indent=2))


if __name__ == '__main__': main()
