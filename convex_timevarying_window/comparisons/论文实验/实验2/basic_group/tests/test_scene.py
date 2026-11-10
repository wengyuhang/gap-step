import json
import xml.etree.ElementTree as ET
import casadi as ca
import numpy as np
import pytest
from convex_timevarying_window.comparisons.论文实验.实验2.basic_group import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.compact_calibration import scenario as compact
from convex_timevarying_window.comparisons.论文实验.实验2.static_calibration.planner import Objective


def test_static_manifest_geometry_and_world_are_identical(tmp_path):
    case=scenario.Case('original','static');a,_=scenario.build_case(case);b,_=compact.build_case(case)
    assert json.loads(json.dumps(scenario.manifest(case)))==json.loads(json.dumps(compact.manifest(case)))
    (tmp_path/'a').mkdir();(tmp_path/'b').mkdir()
    pa=scenario.write_world(a,scenario.ASSETS,tmp_path/'a');pb=compact.write_world(b,scenario.ASSETS,tmp_path/'b')
    assert pa.read_bytes()==pb.read_bytes()


@pytest.mark.parametrize('level',['medium','high'])
def test_symbolic_numeric_motion_bounds_and_plugin_match(level,tmp_path):
    track,_=scenario.build_case(scenario.Case('original',level));t=ca.MX.sym('t')
    world=ET.parse(scenario.write_world(track,scenario.ASSETS,tmp_path))
    models=[m for m in world.findall('world/model') if m.get('name','').startswith('gate_')]
    for w,model in zip(track.windows,models):
        fn=ca.Function('pose',[t],list(scenario.pose(w,t)));plugin=model.find('plugin')
        axis0=np.fromstring(plugin.findtext('axis0'),sep=' ');axis1=np.fromstring(plugin.findtext('axis1'),sep=' ')
        assert abs(axis0[2])<1e-14 and axis1[2]>.999
        times=np.linspace(.023,.481,151);pos,vel=w.motion.local(times);lo,hi=w.motion.local_bounds(times[0],times[-1])
        assert np.all(pos>=lo-1e-12) and np.all(pos<=hi+1e-12)
        for at in times[::15]:
            c,R,v,_=w.state_at(at);numeric=np.array(fn(at)[0]).ravel()
            np.testing.assert_allclose(numeric,c,atol=1e-12)
            s=np.sin(2*np.pi*w.motion.frequency*at)
            if w.motion.kind=='translation':expected=w.center0+w.motion.amplitude*s*axis0
            else:
                theta=w.motion.angle*s;expected=w.center0+w.motion.length*np.sin(theta)*axis0+w.motion.length*(1-np.cos(theta))*axis1
            np.testing.assert_allclose(c,expected,atol=1e-12)
            np.testing.assert_allclose((w.state_at(at+1e-7)[0]-w.state_at(at-1e-7)[0])/2e-7,v,rtol=1e-6,atol=1e-6)
        assert w.motion.kind==('translation' if int(w.name[:2])%2 else 'swing')


@pytest.mark.parametrize('center',[False,True])
def test_togt_dynamic_gradient(center):
    track,_=scenario.build_case(scenario.Case('original','high'));o=Objective(track,center);x=o.initial_guess();value,g=o.value_and_gradient(x)
    for i in ([0,3,7] if center else [0,3,7,8,14,22]):
        step=np.eye(len(x))[i]*2e-6
        fd=(o.value_and_gradient(x+step)[0]-o.value_and_gradient(x-step)[0])/4e-6
        np.testing.assert_allclose(g[i],fd,rtol=5e-3,atol=.06)


@pytest.mark.parametrize('free',[False,True])
def test_pvaj_symbolic_crossing_points_use_the_same_motion(free):
    from convex_timevarying_window.comparisons.论文实验.实验2 import planner
    track,_=scenario.build_case(scenario.Case('original','high'));previous=planner.pose;planner.pose=scenario.pose
    try:o=planner.Planner(track,free)
    finally:planner.pose=previous
    x=o.seeds()[0];times=np.cumsum(np.exp(x[:8]))[:-1];points=np.array(o.point_function(x)).T
    for w,t,p in zip(track.windows,times,points):np.testing.assert_allclose(p,w.state_at(t)[0],atol=1e-10)


def test_missing_contact_evidence_is_not_zero_contacts(tmp_path):
    from convex_timevarying_window.comparisons.论文实验.实验2.basic_group.validate import NativeContactFile
    with pytest.raises(FileNotFoundError):NativeContactFile(tmp_path/'missing.pbtxt').open()
