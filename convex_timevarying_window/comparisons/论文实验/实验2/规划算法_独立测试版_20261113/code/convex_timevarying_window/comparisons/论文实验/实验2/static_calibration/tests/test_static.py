import json
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import apertures,WINDOW_MARGIN
from convex_timevarying_window.comparisons.论文实验.实验2.static_calibration import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.static_calibration.planner import Objective


@pytest.mark.parametrize('layout',scenario.LAYOUTS)
def test_historic_shapes_sizes_and_safe_domains_are_unchanged(layout):
    track,_=scenario.build_case(scenario.Case(layout,'static'));rng=np.random.default_rng(4)
    for w,source in zip(track.windows,apertures()):
        ap=w.aperture;assert ap.margin==source.margin==WINDOW_MARGIN
        assert ap.radius==source.radius
        if ap.kind=='polygon':
            np.testing.assert_array_equal(ap.physical_vertices,source.physical_vertices)
            np.testing.assert_array_equal(ap.safe_vertices,source.safe_vertices)
        for _ in range(10):
            d=rng.normal(size=ap.dimension);q,jac=ap.point_and_jacobian(d);expected,expected_jac=source.point_and_jacobian(d)
            np.testing.assert_array_equal(q,expected);np.testing.assert_array_equal(jac,expected_jac)
            assert ap.contains(q,safe=True)


@pytest.mark.parametrize('center',[False,True])
@pytest.mark.parametrize('level',['static','high'])
def test_reused_dynamic_time_and_safe_point_gradient(center,level):
    tr,_=scenario.build_case(scenario.Case('slalom',level));o=Objective(tr,center);x=o.initial_guess();cost,grad=o.value_and_gradient(x)
    assert len(x)==(8 if center else 33)
    for i in ([0,3,7] if center else [0,3,7,8,13,18,25]):
        e=np.eye(len(x))[i]*2e-6;fd=(o.value_and_gradient(x+e)[0]-o.value_and_gradient(x-e)[0])/(4e-6)
        np.testing.assert_allclose(grad[i],fd,rtol=4e-3,atol=.04)
    f=o.forward(x)
    for w,t,q,P in zip(tr.windows,f.traversal_times,f.local_points,f.waypoints):
        assert w.aperture.contains(q,safe=True)
        if center:np.testing.assert_allclose(P,w.state_at(t)[0],atol=1e-10)


def test_world_has_unchanged_complete_historic_meshes_and_eight_sensors(tmp_path):
    tr,_=scenario.build_case(scenario.Case('original','static'));tree=ET.parse(scenario.write_world(tr,scenario.ASSETS,tmp_path));world=tree.find('world')
    original=ET.parse(scenario.HISTORIC/'convex_seven_dynamic_px4_togt.sdf').find('world')
    old=[m for m in original.findall('model') if m.get('name','').startswith('gate_')]
    new=[m for m in world.findall('model') if m.get('name','').startswith('gate_')]
    assert len(old)==len(new)==7
    for a,b in zip(old,new):assert ET.tostring(a.find('link/collision'))==ET.tostring(b.find('link/collision'))
    sensors=world.findall('model/link/sensor');assert len(sensors)==8
    assert all(len(s.findall('contact/collision'))==1 and s.findtext('update_rate')=='1000' for s in sensors)
    assert world.findtext('physics/max_step_size')=='.001'


def test_native_zero_nanosecond_field_omission_is_counted(tmp_path):
    tr,_=scenario.build_case(scenario.Case('height','static'))
    (tmp_path/'gate_01_contacts.pbtxt').write_text('header { stamp { sec: 31 } }\ncontact { collision1 { name: "gate_01::frame" } collision2 { name: "x500_0::rotor_0" } }\n')
    rows=scenario.parse_all_contacts(tmp_path,tr,32.)
    assert len(rows)==8 and rows[0]['contact_pair_count']==1 and rows[0]['first_contact_sim_time_s']==31.
