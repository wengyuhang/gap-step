from dataclasses import dataclass, asdict
import xml.etree.ElementTree as ET
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation import scenario as original
from convex_timevarying_window.receding_gate_racing import scenario as common


@dataclass(frozen=True)
class Case:
    speed: float


@dataclass(frozen=True)
class LinearMotion:
    velocity: np.ndarray
    anchor_time: float
    translation_period: float = 1.
    phase: float = 0.
    rotation_period: float = 1.
    translation_enabled: bool = True
    rotation_enabled: bool = False

    @property
    def translation_amplitude(self): return np.zeros(3)
    @property
    def rotation_amplitude(self): return np.zeros(3)
    def translation(self, t): return self.velocity*(t-self.anchor_time), self.velocity.copy()
    def rotation(self, t): return np.zeros(3), np.zeros(3)
    def scale(self, t): return 1., 0.


def build_case(case):
    track, cfg = original.build_case(original.Case(1., 0., 0.))
    w = track.windows[0]
    # Fixed geometry-only timing rule, common to both variants and all speeds.
    anchor = max(.8, np.linalg.norm(w.center0-track.start)/6.5+.5)
    w.motion = LinearMotion(case.speed*w.state_at(0.)[1][:, 1], float(anchor))
    return track, cfg


def manifest(case):
    track, _ = build_case(case); w = track.windows[0]
    return dict(case=asdict(case), start=track.start.tolist(), goal=track.goal.tolist(),
        gate_anchor=w.center0.tolist(), anchor_time_s=w.motion.anchor_time,
        velocity_enu_mps=w.motion.velocity.tolist(), base_rpy=w.angles0_rpy.tolist(),
        aperture_vertices=w.aperture.physical_vertices.tolist(),
        motion='c(t)=anchor+velocity*(t-anchor_time); fixed plane; external race clock',
        race_epoch_sim_s=30., gate_count=1)


def write_world(track, assets, outdir):
    static, _ = original.build_case(original.Case(1., 0., 0.))
    path = original.write_world(static, assets, outdir)
    tree = ET.parse(path)
    model = next(m for m in tree.findall('.//world/model') if m.get('name','').startswith('gate_'))
    w = track.windows[0]; plugin = model.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
    ET.SubElement(plugin, 'linear_translation').text = 'true'
    ET.SubElement(plugin, 'linear_velocity').text = ' '.join(map(str,w.motion.velocity))
    ET.SubElement(plugin, 'linear_anchor_time').text = str(w.motion.anchor_time)
    model.find('pose').text = ' '.join(map(str,np.r_[w.state_at(0.)[0],w.angles0_rpy]))
    tree.write(path, encoding='unicode')
    return path


def launch(track, assets, outdir):
    old = common.write_variant_world; common.write_variant_world = write_world
    try: return common.launch_variant(track, original.ASSETS, outdir)
    finally: common.write_variant_world = old
