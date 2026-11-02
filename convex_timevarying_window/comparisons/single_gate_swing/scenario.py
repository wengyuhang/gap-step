"""Original W1 aperture, exogenous in-plane sinusoid, finer contact clock."""
from dataclasses import asdict
import xml.etree.ElementTree as ET
from convex_timevarying_window.comparisons.single_gate_translation import scenario as original
from convex_timevarying_window.receding_gate_racing import scenario as common

Case = original.Case
build_case = original.build_case
PHYSICS_STEP = .001
GROUND_TOPIC = '/world/convex_seven_dynamic_px4_togt/model/ground/link/ground_link/sensor/ground_contact/contact'


def manifest(case):
    result = original.manifest(case)
    result.update(physics_step_s=PHYSICS_STEP, race_epoch_sim_s=30.,
                  ground_contact_topic=GROUND_TOPIC, phase_restriction=False)
    return result


def write_world(track, assets, outdir):
    path = original.write_world(track, assets, outdir)
    tree = ET.parse(path)
    world = tree.find('world')
    world.find('physics/max_step_size').text = str(PHYSICS_STEP)
    for sensor in world.findall("model/link/sensor[@type='contact']"):
        sensor.find('update_rate').text = '1000'
    link = world.find("model[@name='ground']/link")
    sensor = ET.SubElement(link, 'sensor', name='ground_contact', type='contact')
    ET.SubElement(sensor, 'always_on').text = 'true'
    ET.SubElement(sensor, 'update_rate').text = '1000'
    ET.SubElement(sensor, 'topic').text = GROUND_TOPIC
    ET.SubElement(ET.SubElement(sensor, 'contact'), 'collision').text = 'ground_collision'
    tree.write(path, encoding='unicode')
    return path


def launch(track, assets, outdir):
    previous = common.write_variant_world
    common.write_variant_world = write_world
    try:
        return common.launch_variant(track, original.ASSETS, outdir)
    finally:
        common.write_variant_world = previous
