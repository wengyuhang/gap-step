"""Fixed plane, horizontal straight motion; never through the drone start."""
from dataclasses import dataclass, asdict
import xml.etree.ElementTree as ET
import numpy as np
from convex_timevarying_window.comparisons.single_gate_translation import scenario as original
from convex_timevarying_window.comparisons.single_gate_swing import scenario as swing
from convex_timevarying_window.comparisons.single_gate_constant_velocity.scenario import LinearMotion
from convex_timevarying_window.receding_gate_racing import scenario as common

PHYSICS_STEP = swing.PHYSICS_STEP
GROUND_TOPIC = swing.GROUND_TOPIC
INITIAL_LATERAL_OFFSET = 30.


@dataclass(frozen=True)
class Case:
    speed: float  # positive: initially approaching; negative: receding
    offset: float = INITIAL_LATERAL_OFFSET


def build_case(case):
    track, cfg = original.build_case(original.Case(1., 0., 0.))
    w = track.windows[0]
    lateral = w.state_at(0.)[1][:, 1]
    shift = float((track.start-w.center0)@lateral)-case.offset
    w.center0 = w.center0+shift*lateral
    # The fixed finish lies on the start's lateral coordinate, six metres
    # beyond the unchanged plane; it never moves with a method or speed.
    track.goal[:] += float((track.start-track.goal)@lateral)*lateral
    w.motion = LinearMotion(case.speed*lateral, 0.)
    return track, cfg


def manifest(case):
    track, _ = build_case(case); w = track.windows[0]
    return dict(case=asdict(case),start=track.start.tolist(),goal=track.goal.tolist(),
        gate_anchor=w.center0.tolist(),velocity_enu_mps=w.motion.velocity.tolist(),
        base_rpy=w.angles0_rpy.tolist(),aperture_vertices=w.aperture.physical_vertices.tolist(),
        initial_lateral_offset_m=case.offset,
        closest_to_start_time_s=case.offset/case.speed if case.speed>0 else None,
        motion='c(t)=c0+v*t; fixed plane; horizontal in-plane translation; never stops or reverses',
        race_epoch_sim_s=30.,physics_step_s=PHYSICS_STEP,gate_count=1)


def write_world(track, assets, outdir):
    static, _ = original.build_case(original.Case(1., 0., 0.))
    path = swing.write_world(static, assets, outdir)
    tree = ET.parse(path)
    model = next(m for m in tree.findall('.//world/model') if m.get('name','').startswith('gate_'))
    w = track.windows[0]
    plugin = model.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
    plugin.find('center0').text = ' '.join(map(str,w.center0))
    ET.SubElement(plugin,'linear_translation').text = 'true'
    ET.SubElement(plugin,'linear_velocity').text = ' '.join(map(str,w.motion.velocity))
    ET.SubElement(plugin,'linear_anchor_time').text = '0'
    model.find('pose').text = ' '.join(map(str,np.r_[w.center0,w.angles0_rpy]))
    tree.write(path,encoding='unicode')
    return path


def launch(track, assets, outdir):
    previous = common.write_variant_world
    common.write_variant_world = write_world
    try: return common.launch_variant(track,original.ASSETS,outdir)
    finally: common.write_variant_world = previous
