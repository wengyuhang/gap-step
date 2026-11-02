from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from convex_timevarying_window.single_gate_stop_racing.presentation import clear_view, decorative
from convex_timevarying_window.single_gate_stop_racing.camera_view import body_offset, OFFSET


def test_cleanup_preserves_all_physical_models_and_gate_motion(tmp_path):
    source = Path('convex_dynamic_seven_window_gazebo/convex_seven_dynamic_px4_togt.sdf')
    target = tmp_path/'world.sdf'
    target.write_bytes(source.read_bytes())
    before = ET.parse(target).getroot().find('world')
    removed = clear_view(target)
    after = ET.parse(target).getroot().find('world')
    assert len(removed) == 16
    assert not any(decorative(m.get('name')) for m in after.findall('model'))
    for m in before.findall('model'):
        name = m.get('name')
        if name.startswith('gate_'):
            assert ET.tostring(m) == ET.tostring(after.find(f"model[@name='{name}']"))
    assert after.find("model[@name='start_finish_marker']/pose").text.split()[:3] == ['-16', '4', '-5.975']
    assert [ET.tostring(c) for c in before.findall('.//collision')] == [ET.tostring(c) for c in after.findall('.//collision')]
    for key in ('physics', 'gravity', 'spherical_coordinates'):
        assert ET.tostring(before.find(key)) == ET.tostring(after.find(key))


def test_refuses_to_remove_collision(tmp_path):
    path = tmp_path/'world.sdf'
    path.write_text('<sdf><world><model name="lab_ceiling"><link><collision/></link></model></world></sdf>')
    with pytest.raises(ValueError):
        clear_view(path)


def test_chase_offset_stays_in_world_frame_for_aggressive_attitudes():
    for rotation in Rotation.random(100, random_state=4):
        np.testing.assert_allclose(rotation.apply(body_offset(rotation.as_quat())), OFFSET, atol=1e-13)
