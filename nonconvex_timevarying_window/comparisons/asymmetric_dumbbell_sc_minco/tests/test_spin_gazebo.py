import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.spin.scenario import build_case
from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.spin.gazebo_scene import write_world, ASSETS, FRAME_TOPIC


def test_exact_nonconvex_capsule_geometry_and_spin_parameters(tmp_path):
    track=build_case(2.)
    world=ET.parse(write_world(track,ASSETS,tmp_path)).find('world')
    gates=[m for m in world.findall('model') if m.get('name','').startswith('gate_')]
    assert len(gates)==1
    model=gates[0];link=model.find('link')
    assert len(link.findall('collision'))==24
    sensor=link.find('sensor')
    assert sensor.findtext('topic')==FRAME_TOPIC
    assert len(sensor.findall('contact/collision'))==24
    vertices=track.windows[0].aperture.physical_vertices
    for i,(a,b) in enumerate(zip(vertices,np.roll(vertices,-1,axis=0))):
        c=link.find(f"collision[@name='edge_{i:02d}']")
        pose=np.fromstring(c.findtext('pose'),sep=' ')
        axis=Rotation.from_euler('xyz',pose[3:]).apply([0,0,1])
        length=float(c.findtext('geometry/cylinder/length'))
        np.testing.assert_allclose(pose[:3]-axis*length/2,np.r_[a,0],atol=1e-12)
        np.testing.assert_allclose(pose[:3]+axis*length/2,np.r_[b,0],atol=1e-12)
    plugin=model.find('plugin')
    assert float(plugin.findtext('spin_hz'))==2
    assert float(plugin.findtext('frequency'))==3
    assert float(plugin.findtext('motion_start_time'))==30
    assert float(world.findtext('physics/max_step_size'))==.001


def test_rigid_scene_alignment_keeps_local_coordinates_and_derivatives():
    from convex_timevarying_window.dynamic_passage_velocity_minco.trajectory import Plan
    from nonconvex_timevarying_window.comparisons.asymmetric_dumbbell_sc_minco.spin.flight import align_with_launch_origin, WORLD_TRANSLATION
    original=build_case(2.);track=build_case(2.)
    rng=np.random.default_rng(42)
    plan=Plan(0.,0,np.array([1.,2.]),rng.normal(size=(2,8,3)),np.zeros(7),0,0,'test',1)
    track,shifted=align_with_launch_origin(track,plan)
    np.testing.assert_allclose(track.start,[-16,4,3.2])
    for t in np.linspace(0,3,11):
        np.testing.assert_allclose(shifted.sample(t),plan.sample(t)+WORLD_TRANSLATION,atol=1e-12)
        for d in (1,2,3): np.testing.assert_allclose(shifted.sample(t,d),plan.sample(t,d))
        q0=original.windows[0].world_to_local(plan.sample(t),t)
        q1=track.windows[0].world_to_local(shifted.sample(t),t)
        np.testing.assert_allclose(q0[0],q1[0],atol=1e-12)
        np.testing.assert_allclose(q0[1],q1[1],atol=1e-12)
