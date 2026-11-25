"""Increase pendulum rise while retaining horizontal amplitude and frequency."""
from dataclasses import replace
import xml.etree.ElementTree as ET

import numpy as np

from ..basic_group.whole_course_difficulty import scenario as original
from ..basic_group import scenario as world

ROOT = original.ROOT
ASSETS = original.ASSETS
Case = original.Case
LEVELS = original.LEVELS
pose = original.pose
SWING_LENGTH = 2.20
VARIANT = 'higher_swing_length_2p20'
_original_writer = world.write_world
_original_launch = world.launch


def build_case(case):
    track, _ = original.build_case(case)
    for window in track.windows:
        if window.motion.kind == 'swing':
            if window.motion.amplitude >= SWING_LENGTH:
                raise ValueError('Pendulum length must exceed horizontal amplitude')
            window.motion = replace(window.motion, length=SWING_LENGTH)
    return replace(track, name=track.name+'_'+VARIANT), None


def manifest(case):
    result = original.manifest(case)
    track, _ = build_case(case)
    result.update(scenario_variant=VARIANT, swing_length_m=SWING_LENGTH,
        modification='shorter circular pendulum arc, same horizontal amplitude and frequency')
    for row, window in zip(result['windows'], track.windows):
        motion = window.motion
        if motion.kind == 'swing':
            row.update(swing_length_m=motion.length,
                pivot=(window.center0+motion.length*window.R[:, 0]).tolist(),
                swing_angle_rad=motion.angle,
                maximum_vertical_rise_m=float(motion.length-np.sqrt(motion.length**2-motion.amplitude**2)),
                peak_center_speed_mps=float(2*np.pi*motion.frequency*motion.length*motion.angle))
    speeds = [row['peak_center_speed_mps'] for row in result['windows']]
    result['peak_center_speed_range_mps'] = [min(speeds), max(speeds)]
    return result


def write_world(track, assets, out):
    target = _original_writer(track, assets, out)
    tree = ET.parse(target)
    models = [model for model in tree.findall('world/model')
        if model.get('name', '').startswith('gate_')]
    for model, window in zip(models, track.windows):
        plugin = model.find("plugin[@name='paper_experiment::MixedGateMotion']")
        plugin.find('length').text = str(window.motion.length)
    tree.write(target, encoding='unicode')
    return target


def launch(track, assets, out):
    writer = world.write_world
    world.write_world = write_world
    try:
        return _original_launch(track, assets, out)
    finally:
        world.write_world = writer
