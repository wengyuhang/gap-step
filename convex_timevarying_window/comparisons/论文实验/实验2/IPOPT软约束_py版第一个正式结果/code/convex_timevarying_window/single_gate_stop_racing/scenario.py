"""Spread the horizontal course about its unchanged start; keep real gate motion."""
import numpy as np
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane import scenario as original

Case = original.Case
FIRST_GATE_SHIFT_M = -5.
HORIZONTAL_SPACING_SCALE = 2.


def launch(track, assets, outdir, *, camera_helper=True):
    import os
    import subprocess
    from pathlib import Path
    from .presentation import clear_view
    previous = original.write_world

    def write_world(track, assets, outdir):
        path = previous(track, assets, outdir)
        clear_view(path)
        return path

    original.write_world = write_world
    try:
        result = original.launch(track, assets, outdir)
    finally:
        original.write_world = previous
    if os.environ.get('GZ_SHOW_GUI') == '1' and camera_helper:
        container = 'convex_seven_togt_experiment'
        source = Path(__file__).with_name('camera_view.py')
        subprocess.run(['docker', 'cp', str(source), container+':/tmp/camera_view.py'], check=True)
        subprocess.run(['docker', 'exec', '-d', container, 'bash', '-lc',
                        'exec python3 /tmp/camera_view.py >/tmp/camera_view.log 2>&1'], check=True)
    return result


def workspace_bounds(scale=HORIZONTAL_SPACING_SCALE):
    origin=np.array([-16.,4.])
    lower=np.array([-28.,-30.,.65]);upper=np.array([28.,22.,12.])
    lower[:2]=origin+scale*(lower[:2]-origin)
    upper[:2]=origin+scale*(upper[:2]-origin)
    return tuple(lower),tuple(upper)


def build_case(case, first_gate_shift_m=FIRST_GATE_SHIFT_M,
               horizontal_spacing_scale=HORIZONTAL_SPACING_SCALE):
    track, cfg = original.build_case(case)
    if horizontal_spacing_scale!=1.:
        for window in track.windows:
            window.center0=window.center0.copy()
            window.center0[:2]=track.start[:2]+horizontal_spacing_scale*(window.center0[:2]-track.start[:2])
    window = track.windows[0]
    window.center0 = window.center0 + first_gate_shift_m * window.state_at(0.)[1][:, 1]
    return track, cfg


def manifest(case):
    result = original.manifest(case)
    track, _ = build_case(case)
    result.update(anchors=[w.center0.tolist() for w in track.windows],
                  original_positions_unchanged=False,
                  original_planes_unchanged=False,
                  original_orientations_unchanged=True,
                  fixed_planes=True,
                  horizontal_spacing_scale=HORIZONTAL_SPACING_SCALE,
                  layout_scale_origin=track.start.tolist(),
                  workspace_lower=workspace_bounds()[0],workspace_upper=workspace_bounds()[1],
                  first_gate_sideways_shift_m=FIRST_GATE_SHIFT_M,
                  first_gate_shift_world=(FIRST_GATE_SHIFT_M * track.windows[0].state_at(0.)[1][:, 1]).tolist(),
                  presentation='Open-air visual scene; no decorative walls/roof; ground-level finish marker; physical course unchanged',
                  scene_adaptation='All horizontal gate anchors spread 2x about the unchanged start; W1 additionally shifted -5m sideways; height, orientation, aperture, physical motion and phase unchanged')
    return result
