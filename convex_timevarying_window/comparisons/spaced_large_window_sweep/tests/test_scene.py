import numpy as np
import xml.etree.ElementTree as ET
from ..scenario import build_case,Case,large,spread,layout_check,write_world,LOWER,UPPER


def test_spacing_not_motion_or_vehicle():
    old,_=large.build_case(Case(2,3,0));new,_=build_case(Case(2,3,0))
    assert np.array_equal(old.start,new.start)
    assert np.array_equal(old.goal,new.goal)
    for a,b in zip(old.windows,new.windows):
        assert np.allclose(b.center0,spread(a.center0))
        assert b.center0[2]==a.center0[2]
        for t in [0,.173,2.32]:
            assert np.allclose(a.state_at(t)[0]-a.center0,b.state_at(t)[0]-b.center0)
            assert np.allclose(a.state_at(t)[1],b.state_at(t)[1])
        assert np.allclose(a.aperture.boundary_points(),b.aperture.boundary_points())


def test_swept_separation_and_bounds():
    check=layout_check()
    assert check['minimum_gate_swept_sphere_gap_m']>12
    assert check['straight_return_vs_other_gates_min_gap_m']>16
    assert np.allclose(spread([-28,-30,.65]),LOWER)
    assert np.allclose(spread([28,22,12]),UPPER)


def test_world_centers_match_planner(tmp_path):
    track,_=build_case(Case(1,2,0));path=write_world(track,large.ASSETS,tmp_path)
    tree=ET.parse(path);models=[m for m in tree.findall('.//world/model') if m.get('name','').startswith('gate_')]
    for model,w in zip(models,track.windows):
        plugin=model.find("plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
        assert np.allclose(np.fromstring(plugin.findtext('center0'),sep=' '),w.center0)
        assert np.allclose(np.fromstring(model.findtext('pose'),sep=' ')[:3],w.state_at(0)[0])
    ground=tree.find(".//world/model[@name='ground']")
    assert np.allclose(np.fromstring(ground.findtext('.//plane/size'),sep=' '),[240,210])
