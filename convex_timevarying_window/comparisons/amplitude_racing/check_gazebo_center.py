"""Check a live Gazebo gate center against the exact scenario used by a run."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track


CONTAINER = 'convex_seven_togt_experiment'
PARTITION = 'convex_seven_togt_experiment_partition'
TOPIC = '/world/convex_seven_dynamic_px4_togt/pose/info'


def sample_pose(gate_name: str) -> tuple[float, np.ndarray]:
    raw = subprocess.check_output([
        'docker', 'exec', '-e', f'GZ_PARTITION={PARTITION}', CONTAINER,
        'gz', 'topic', '-e', '-t', TOPIC, '-n', '1'], text=True, timeout=10)
    stamp = re.search(r'stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)', raw)
    block = re.search(r'pose\s*\{\s*name: "' + re.escape(gate_name) + r'"(.*?)orientation',
                      raw, re.S)
    if stamp is None or block is None:
        raise RuntimeError('Gazebo pose/info did not contain the gate pose and timestamp')
    instant = int(stamp.group(1)) + int(stamp.group(2)) * 1e-9
    position = np.array([float(re.search(r'\b' + axis + r':\s*([+-]?[\d.eE+-]+)',
                                         block.group(1)).group(1)) for axis in 'xyz'])
    return instant, position


def main(run_dir: Path, count: int = 10, delay: float = .16) -> None:
    result = json.loads((run_dir / 'result.json').read_text())
    track, _ = build_track()
    vary_track(track, result['phase'], result['layout'], result['motion_rate'],
               result['translation_scale'], result['rotation_scale'])
    window = track.windows[0]
    gate_name = 'gate_01_W1_rectangle'
    samples = []
    for _ in range(count):
        instant, actual = sample_pose(gate_name)
        race_time = max(0., instant - 30.)
        predicted = window.state_at(race_time)[0]
        samples.append({'sim_time_s': instant, 'race_time_s': race_time,
                        'actual_xyz_m': actual.tolist(), 'predicted_xyz_m': predicted.tolist(),
                        'position_error_m': float(np.linalg.norm(actual - predicted))})
        time.sleep(delay)
    positions = np.array([sample['actual_xyz_m'] for sample in samples])
    record = {'run': str(run_dir.resolve()), 'gate': gate_name,
              'observed_samples': count,
              'sampled_xyz_peak_to_peak_m': np.ptp(positions, axis=0).tolist(),
              'max_model_discrepancy_m': max(sample['position_error_m'] for sample in samples),
              'samples': samples,
              'evidence': 'Gazebo model pose/info compared with planned center query; sampled, not continuous'}
    (run_dir / 'gazebo_center_pose_check.json').write_text(json.dumps(record, indent=2) + '\n')
    t = np.array([sample['sim_time_s'] for sample in samples]); t -= t[0]
    fig, ax = plt.subplots(figsize=(7.5, 3.7), layout='constrained')
    for axis, label in enumerate('xyz'):
        ax.plot(t, positions[:, axis] - window.center0[axis], 'o-', label=f'Gazebo {label}')
        ax.plot(t, np.array([sample['predicted_xyz_m'][axis] for sample in samples])
                - window.center0[axis], ':', color=ax.lines[-1].get_color(), linewidth=2)
    ax.set(xlabel='Time since first pose sample (s)', ylabel='W1 center displacement (m)',
           title=f'Gazebo center motion: frequency {result["motion_rate"]:g}×, '
                 f'centering amplitude {result["translation_scale"]:g}×')
    ax.grid(alpha=.2); ax.legend(ncol=3)
    fig.savefig(run_dir / 'gazebo_center_pose_check.png', dpi=180)
    plt.close(fig)
    print(json.dumps({key: value for key, value in record.items() if key != 'samples'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--count', type=int, default=10)
    parser.add_argument('--delay', type=float, default=.16)
    args = parser.parse_args()
    main(args.run_dir, args.count, args.delay)
