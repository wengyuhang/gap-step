from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from ..scenario import build_case,Case,base,write_world,ASSETS


def test_scaled_apertures_same_motion_and_absolute_margin():
    case=Case(2.,3.,0.);original,_=base.build_case(case);large,_=build_case(case)
    for a,b in zip(original.windows,large.windows):
        assert np.array_equal(a.center0,b.center0)
        for t in (0.,.1,2.17):
            assert np.allclose(a.state_at(t)[0],b.state_at(t)[0])
            assert np.allclose(a.state_at(t)[1],b.state_at(t)[1])
        if a.aperture.kind=='circle':assert b.aperture.radius==3*a.aperture.radius
        else:
            assert np.allclose(b.aperture.physical_vertices,3*a.aperture.physical_vertices)
            assert np.allclose(b.aperture.physical_vertices-b.aperture.safe_vertices,
                               a.aperture.physical_vertices-a.aperture.safe_vertices)
        assert a.aperture.margin==b.aperture.margin


def test_world_meshes_match_enlarged_frame(tmp_path):
    import trimesh
    track,_=build_case(Case(2.,3.,0.));path=write_world(track,ASSETS,tmp_path)
    tree=ET.parse(path)
    models=[m for m in tree.findall('.//world/model') if m.get('name','').startswith('gate_')]
    assert len(models)==7
    for model,w in zip(models,track.windows):
        uri=model.find('.//collision/geometry/mesh/uri').text
        mesh=trimesh.load(ASSETS/uri.removeprefix('/workspace/'),force='mesh')
        assert np.isclose(mesh.bounds[1,2],.085,atol=1e-7)
        assert np.isclose(mesh.bounds[0,2],-.085,atol=1e-7)
        if w.aperture.kind=='circle':
            # Radius of the frame centerline plus its unchanged tube radius.
            assert np.allclose(mesh.bounds[1,:2],w.aperture.radius+.15,atol=.002)
    again=tmp_path/'again';again.mkdir()
    assert path.read_bytes()==write_world(track,ASSETS,again).read_bytes()


def test_flight_only_changes_scene():
    from convex_timevarying_window.simple_two_gate_soft_only_mpc.flight import variant_config
    from dataclasses import asdict
    a,b=map(asdict,(variant_config('ours'),variant_config('center')))
    assert a.pop('free_points') and not b.pop('free_points')
    assert a==b


def test_free_point_box_covers_large_window_and_center_is_fixed():
    from ..planner import crossing_bounds
    track,_=build_case(Case(1.,2.,0.))
    for w in track.windows:
        lo,hi=crossing_bounds(w.aperture,True)
        boundary=w.aperture.boundary_points()
        assert np.all(boundary>=lo-1e-10) and np.all(boundary<=hi+1e-10)
        assert np.max(hi)>2.
        assert all(np.array_equal(x,np.zeros(2)) for x in crossing_bounds(w.aperture,False))


def test_adjudication_never_accepts_collision(tmp_path):
    from ..run import adjudicate
    row=dict(directory=str(tmp_path),completed=True,body_exit_count=7,
             ordered_once=True,complete_body_exits=True,flight_time_s=30.,collision_detected=True)
    assert adjudicate(row)['simulation_race_time_s'] is None
    row['collision_detected']=False
    assert adjudicate(row)['simulation_race_time_s']==30.
    row['ordered_once']=False
    assert adjudicate(row)['simulation_race_time_s'] is None
