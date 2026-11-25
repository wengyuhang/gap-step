from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from convex_timevarying_window.two_gate_hard_mpc.scenario import Case,build_case,write_world


def test_staggered_scene_and_world(tmp_path):
    track,_=build_case(Case(1.,2.,0.))
    a,b=track.windows
    assert np.linalg.norm(a.center0-b.center0)>23
    assert abs(b.center0[1]-a.center0[1])>11
    assert np.linalg.norm(np.cross(a.center0-track.start,b.center0-a.center0))>1
    path=write_world(track,Path('convex_dynamic_seven_window_gazebo'),tmp_path)
    models=[m for m in ET.parse(path).findall('.//world/model') if m.get('name','').startswith('gate_')]
    assert len(models)==2
    assert [m.find('.//sensor/topic').text for m in models]==['/convex_seven/gate_01/contacts','/convex_seven/gate_02/contacts']
    for t in (0.,.13,.41):
        assert abs(a.state_at(t)[0][0]-a.center0[0])<1e-9
        assert abs(b.state_at(t)[0][0]-b.center0[0])<1e-9


