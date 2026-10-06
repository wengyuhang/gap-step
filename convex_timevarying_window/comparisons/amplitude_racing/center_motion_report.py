"""Summarize the checked-method center-motion matrix without hiding failures."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track


RATES = (1, 3, 6, 10)
AMPLITUDES = (1, 2, 4)


def center_motion_metrics(rate: int, amplitude: int) -> dict:
    track, _ = build_track()
    vary_track(track, motion_rate=rate, translation_scale=amplitude, rotation_scale=1)
    peak_speed = 0.0
    greatest_axis_travel = 0.0
    for window in track.windows:
        motion = window.motion
        axis_amplitude = np.asarray(motion.translation_amplitude, float)
        omega = 2 * np.pi / motion.translation_period
        phase = np.linspace(0, 2 * np.pi, 4097)
        velocity = omega * axis_amplitude * np.cos(phase[:, None] + np.array([0., .7, 1.4]))
        peak_speed = max(peak_speed, float(np.linalg.norm(velocity, axis=1).max()))
        greatest_axis_travel = max(greatest_axis_travel, float(2 * np.abs(axis_amplitude).max()))
    return {'max_sampled_center_speed_mps': peak_speed,
            'greatest_axis_peak_to_peak_m': greatest_axis_travel}


def plot_motion(root: Path) -> None:
    t = np.linspace(0, 3, 901)
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 3.7), layout='constrained')
    for amplitude in AMPLITUDES:
        track, _ = build_track()
        vary_track(track, motion_rate=10, translation_scale=amplitude, rotation_scale=1)
        w = track.windows[0]
        x = w.motion.translation_amplitude[0] * np.sin(2 * np.pi * t / w.motion.translation_period + w.motion.phase)
        axes[0].plot(t, x, label=f'{amplitude}× displacement')
    for rate in RATES:
        track, _ = build_track()
        vary_track(track, motion_rate=rate, translation_scale=4, rotation_scale=1)
        w = track.windows[0]
        x = w.motion.translation_amplitude[0] * np.sin(2 * np.pi * t / w.motion.translation_period + w.motion.phase)
        axes[1].plot(t, x, label=f'{rate}× frequency')
    for ax in axes:
        ax.set(xlabel='Race time (s)', ylabel='W1 center x displacement (m)')
        ax.axhline(0, color='0.4', linewidth=.7)
        ax.grid(alpha=.22)
        ax.legend(fontsize=8)
    axes[0].set_title('Frequency 10×: varying center travel')
    axes[1].set_title('Travel 4×: varying frequency')
    fig.savefig(root / 'center_motion.png', dpi=180)
    plt.close(fig)


def report(root: Path) -> None:
    protocol = json.loads((root / 'protocol.json').read_text())
    if protocol.get('design') != 'center_motion_grid' or not protocol.get('checked'):
        raise ValueError('This report requires the checked center-motion grid')
    rows = json.loads((root / 'summary.json').read_text()) if (root / 'summary.json').exists() else []
    by_key = {(int(row['rate']), int(row['translation_scale']), int(row['phase_id'])): row
              for row in rows if not row.get('infrastructure_failure')}
    motion = {(rate, amplitude): center_motion_metrics(rate, amplitude)
              for rate in RATES for amplitude in AMPLITUDES}
    plot_motion(root)

    fig, ax = plt.subplots(figsize=(10.5, 4.4), layout='constrained')
    success = np.full((len(RATES), len(AMPLITUDES)), np.nan)
    for i, rate in enumerate(RATES):
        for j, amplitude in enumerate(AMPLITUDES):
            available = [by_key[(rate, amplitude, phase)] for phase in (0, 1)
                         if (rate, amplitude, phase) in by_key]
            if available:
                success[i, j] = sum(bool(row.get('realtime_success')) for row in available)
            good_times = [row['flight_time_s'] for row in available
                          if row.get('realtime_success') and row.get('flight_time_s') is not None]
            label = f"{int(success[i,j])}/{len(available)}" if available else 'pending'
            if good_times:
                label += '\n' + ' / '.join(f'{value:.1f}' for value in good_times) + ' s'
            ax.text(j, i, label, ha='center', va='center', fontsize=10)
    ax.imshow(success, vmin=0, vmax=2, cmap='Blues', origin='upper', alpha=.55,
              aspect='auto')
    ax.set_xticks(range(len(AMPLITUDES)), [f'{value}×' for value in AMPLITUDES])
    ax.set_yticks(range(len(RATES)), [f'{value}×' for value in RATES])
    ax.set(xlabel='Center displacement amplitude', ylabel='Motion frequency',
           title='Checked PX4/Gazebo: strict passes / completed runs\nand individual strict-pass flight times')
    fig.savefig(root / 'center_motion_results.png', dpi=180)
    plt.close(fig)

    lines = ['# Current checked method: center-motion matrix', '',
             'Frequency scales translation and rotation periods together; only translation amplitude changes. '
             'Two fixed phases (0 and 120°) per condition are not random repetitions.', '',
             '| Frequency | Center amplitude | Largest axis travel | Peak center speed* | Strict passes | Strict-pass flight time | Worst online cycle | Negative results |',
             '|---:|---:|---:|---:|---:|---:|---:|---|']
    def failure_label(row: dict) -> str:
        reason = row.get('failure') or row.get('analysis_error')
        if not reason and row.get('min_clearance_m', 0) < 0:
            reason = f"sampled clearance {row['min_clearance_m']:.3f} m"
        return f"p{row['phase_id']}: {reason or 'failed audit'}"
    for rate in RATES:
        for amplitude in AMPLITUDES:
            group = [by_key[(rate, amplitude, phase)] for phase in (0, 1)
                     if (rate, amplitude, phase) in by_key]
            passed = [row for row in group if row.get('realtime_success')]
            times = ', '.join(f"{row['flight_time_s']:.3f}" for row in passed) if passed else '—'
            cycles = [row['cycle_ms']['max'] for row in group if isinstance(row.get('cycle_ms'), dict)]
            failures = ', '.join(failure_label(row) for row in group
                                 if not row.get('realtime_success')) or '—'
            m = motion[(rate, amplitude)]
            lines.append(f"| {rate}× | {amplitude}× | {m['greatest_axis_peak_to_peak_m']:.2f} m | "
                         f"{m['max_sampled_center_speed_mps']:.3f} m/s | {len(passed)}/{len(group)} | "
                         f"{times} s | {max(cycles):.2f} ms | {failures} |" if cycles else
                         f"| {rate}× | {amplitude}× | {m['greatest_axis_peak_to_peak_m']:.2f} m | "
                         f"{m['max_sampled_center_speed_mps']:.3f} m/s | {len(passed)}/{len(group)} | "
                         f"{times} s | — | {failures} |")
    lines.extend(['', '*Numerical sampling over one motion period, not a continuous maximum certificate.',
                  'Strict pass means ordered seven-gate completion and goal return, zero Gazebo frame contacts, '
                  '10 ms sampled true sphere clearance and room bounds passed, and every full planning cycle under 100 ms.',
                  'No continuous-time or closed-loop safety certificate is claimed.',
                  '', 'See `summary.json` and each run directory for all failures, raw logs, and audits.'])
    completed = [row for row in rows if row.get('realtime_success')]
    if completed:
        slowest = max(completed, key=lambda row: row['flight_time_s'])
        run = json.loads((Path(slowest['run']) / 'result.json').read_text())
        accepted = [solve for solve in run['solves'] if solve['accepted']]
        if accepted and run['crossings']:
            lines.extend(['', '## Slowest completed run', '',
                          f"Rate {slowest['rate']}×, center amplitude {slowest['translation_scale']}×, "
                          f"phase p{slowest['phase_id']}: {slowest['flight_time_s']:.3f} s. "
                          f"First accepted local plan at {accepted[0]['time']:.3f} s and first gate at "
                          f"{run['crossings'][0]['time']:.3f} s. The planner did not accept a plan before "
                          'that time; this does not prove that earlier safe flight was physically impossible.'])
    (root / 'CENTER_MOTION_REPORT.md').write_text('\n'.join(lines) + '\n')
    (root / 'center_motion_metrics.json').write_text(json.dumps([
        {'rate': rate, 'translation_scale': amplitude, **motion[(rate, amplitude)]}
        for rate in RATES for amplitude in AMPLITUDES], indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    report(parser.parse_args().root)
