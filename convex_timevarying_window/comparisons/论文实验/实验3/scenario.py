"""One original rectangular sleeve, prescribed horizontal sinusoidal motion."""
from dataclasses import asdict, dataclass
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import apertures
from ..实验2.basic_group import scenario as shared
from ..实验2.static_calibration import scenario as historic

ROOT = shared.ROOT
ASSETS = shared.ASSETS
pose = shared.pose
START = np.array([-11., 9.9, 4.64])
CENTER = START + [8., 0., 0.]
GOAL = START + [16., 0., 0.]
ROTATION = np.array([[0., 0., 1.], [0., -1., 0.], [1., 0., 0.]])


@dataclass(frozen=True)
class Case:
    level: int = 0

    def __post_init__(self):
        if not isinstance(self.level, int) or self.level < 0:
            raise ValueError('The single-gate level must be a nonnegative integer')


def build_case(case):
    frequency = float(case.level) if case.level else 1.
    window = historic.Window(0, CENTER.copy(), ROTATION.copy(), frequency,
        float(case.level), apertures()[0])
    window.motion = shared.Motion('translation', ROTATION.copy(), frequency, float(case.level))
    return SCWindowTrack(f'single_translate_{case.level}', START.copy(), GOAL.copy(),
        (window,), (0,)), None


def manifest(case):
    track, _ = build_case(case)
    window = track.windows[0]
    return dict(case=asdict(case), start=track.start.tolist(), goal=track.goal.tolist(),
        center0=window.center0.tolist(), rotation=window.R.tolist(), gate_count=1,
        pieces=2, physical_vertices=window.aperture.physical_vertices.tolist(),
        opening_up_m=2.90, opening_horizontal_m=2.30, amplitude_m=float(case.level),
        frequency_hz=0. if case.level == 0 else float(case.level), initial_phase_rad=0.,
        peak_center_speed_mps=2.*np.pi*case.level**2, epoch_sim_s=30.,
        start_normal_distance_m=8., goal_normal_distance_m=8.,
        physics_step_s=.001, contact_hz=1000, endpoint_vaj='zero',
        motion='center0 + world-y A sin(2 pi f t), fixed frame orientation',
        sleeve_mesh=str(ASSETS/'meshes/gate_01_rectangle_sleeve.stl'))


def contact_topics(track):
    return historic.contact_topics(track)


def parse_all_contacts(path, track, end_sim_s):
    rows = []
    for index in range(1, len(track.windows)+2):
        file = Path(path)/f'gate_{index:02d}_contacts.pbtxt'
        text = file.read_text(errors='replace') if file.exists() else ''
        text = re.sub(r'(stamp\s*\{\s*sec:\s*\d+)(\s*\})', r'\1\n nsec: 0\2', text)
        class View:
            name = file.name
            def exists(self): return file.exists()
            def read_text(self, **unused): return text
        record = historic.legacy_contacts(View(), end_sim_s)
        rows.append(dict(raw_logs=[record['raw_log']], sensor_records=[record],
            **{key:value for key,value in record.items() if key != 'raw_log'}))
    return rows


def write_world(track, assets, out):
    # The historical writer changes only models paired with track.windows.
    target = historic.write_world(track, assets, out)
    tree = ET.parse(target)
    world = tree.find('world')
    for model in list(world.findall('model')):
        if model.get('name', '').startswith('gate_') and model.get('name') != 'gate_01_rectangle':
            world.remove(model)
    gate = world.find("model[@name='gate_01_rectangle']")
    plugin = gate.find("plugin[@name='paper_experiment::MixedGateMotion']")
    plugin.find('axis0').text = ' '.join(map(str, -ROTATION[:, 1]))
    plugin.find('axis1').text = ' '.join(map(str, ROTATION[:, 0]))
    tree.write(target, encoding='unicode')
    return target


def launch(track, assets, out):
    historic.prepare_assets()
    common = historic.launch_common
    previous = common.write_variant_world
    common.write_variant_world = write_world
    try:
        return common.launch_variant(track, ASSETS, out)
    finally:
        common.write_variant_world = previous
