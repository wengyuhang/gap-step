"""Presentation-only world changes; never alter physical course elements."""
import xml.etree.ElementTree as ET

DECORATIONS = ('back_wall', 'left_wall', 'right_wall', 'lab_ceiling')


def decorative(name):
    return name in DECORATIONS or name.startswith('ceiling_panel_')


def clear_view(path):
    tree = ET.parse(path)
    world = tree.getroot().find('world')
    removed = []
    for model in list(world.findall('model')):
        if decorative(model.get('name', '')):
            if model.findall('.//collision') or model.findall('.//sensor'):
                raise ValueError('Refusing to remove physical/sensor-bearing decoration')
            removed.append(model.get('name'))
            world.remove(model)
    world.find('scene/ambient').text = '0.8 0.8 0.8 1'
    world.find('scene/background').text = '0.72 0.82 0.94 1'
    world.find('scene/shadows').text = 'false'
    world.find("light[@name='sun']/cast_shadows").text = 'false'
    # Enlarge only the drawn floor, not its collision or world datum.
    world.find("model[@name='ground']/link/visual/geometry/plane/size").text = '220 180'
    # The large floating disk obscured the whole drone at the finish point.
    marker = world.find("model[@name='start_finish_marker']/pose")
    pose = marker.text.split()
    pose[2] = '-5.975'
    marker.text = ' '.join(pose)
    tree.write(path, encoding='unicode')
    return removed
