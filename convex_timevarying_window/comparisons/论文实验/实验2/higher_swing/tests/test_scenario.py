import xml.etree.ElementTree as ET

import numpy as np

from convex_timevarying_window.comparisons.论文实验.实验2.higher_swing import scenario
from convex_timevarying_window.comparisons.论文实验.实验2.soft_dynamics import best_center_release


def test_only_pendulum_length_changes_and_rise_is_larger():
    case = scenario.Case(level='a75_f6')
    original, _ = scenario.original.build_case(case)
    track, _ = scenario.build_case(case)
    for old, new in zip(original.windows, track.windows):
        np.testing.assert_array_equal(old.center0, new.center0)
        np.testing.assert_array_equal(old.R, new.R)
        assert old.motion.amplitude==new.motion.amplitude and old.motion.frequency==new.motion.frequency
        np.testing.assert_allclose(new.state_at(0.)[0], old.state_at(0.)[0], atol=1e-12)
        if new.motion.kind=='swing':
            old_rise = old.motion.length-np.sqrt(old.motion.length**2-old.motion.amplitude**2)
            new_rise = new.motion.length-np.sqrt(new.motion.length**2-new.motion.amplitude**2)
            assert new.motion.length==2.2 and new_rise>2.5*old_rise
        else:
            np.testing.assert_array_equal(new.motion.local(.137)[0],old.motion.local(.137)[0])


def test_motion_pose_derivatives_and_bounds_agree():
    track, _ = scenario.build_case(scenario.Case(level='a75_f6'))
    for window in track.windows:
        for time in (.03,.117,.4):
            expected = scenario.pose(window,time)[0]
            position = window.state_at(time)[0]
            np.testing.assert_allclose(np.array(expected).ravel(),position,atol=1e-12)
            step=1e-7
            derivative=(window.state_at(time+step)[0]-window.state_at(time-step)[0])/(2*step)
            np.testing.assert_allclose(derivative,window.state_at(time)[2],rtol=1e-7,atol=1e-6)
        for left,right in ((0.,.07),(.15,.4),(.02,1.)):
            lower,upper=window.motion.local_bounds(left,right)
            points=window.motion.local(np.linspace(left,right,501))[0]
            assert np.all(points>=lower-1e-10) and np.all(points<=upper+1e-10)


def test_world_plugin_matches_the_analytic_motion_and_keeps_physical_geometry(tmp_path):
    case=scenario.Case(level='a75_f6')
    track,_=scenario.build_case(case)
    old_dir=tmp_path/'old';new_dir=tmp_path/'new';old_dir.mkdir();new_dir.mkdir()
    old_track,_=scenario.original.build_case(case)
    old=ET.parse(scenario._original_writer(old_track,scenario.ASSETS,old_dir))
    new=ET.parse(scenario.write_world(track,scenario.ASSETS,new_dir))
    old_models=[m for m in old.findall('world/model') if m.get('name','').startswith('gate_')]
    new_models=[m for m in new.findall('world/model') if m.get('name','').startswith('gate_')]
    for window,original,model in zip(track.windows,old_models,new_models):
        assert ET.tostring(original.find('link/collision'))==ET.tostring(model.find('link/collision'))
        plugin=model.find("plugin[@name='paper_experiment::MixedGateMotion']")
        length=float(plugin.findtext('length'));amplitude=float(plugin.findtext('amplitude'));frequency=float(plugin.findtext('frequency'))
        assert length==window.motion.length
        axis0=np.fromstring(plugin.findtext('axis0'),sep=' ');axis1=np.fromstring(plugin.findtext('axis1'),sep=' ')
        for time in (.01,.1,.23):
            if window.motion.kind=='swing':
                angle=np.arcsin(amplitude/length)*np.sin(2*np.pi*frequency*time)
                expected=window.center0+axis0*length*np.sin(angle)+axis1*length*(1-np.cos(angle))
            else:
                expected=window.center0+axis0*amplitude*np.sin(2*np.pi*frequency*time)
            np.testing.assert_allclose(expected,window.state_at(time)[0],atol=1e-12)


def test_matched_center_initial_curves_remain_valid_for_the_new_motion():
    track,_=scenario.build_case(scenario.Case(level='a75_f6'))
    free=best_center_release.make_planner(track,True)
    center=best_center_release.make_planner(track,False)
    for first,second in zip(free.seeds(),center.seeds()):
        np.testing.assert_allclose(free.curve_values(first),center.curve_values(second),atol=1e-8)
        for point,window,at in zip(np.asarray(free.point_function(first)).T,track.windows,np.cumsum(np.exp(first[:8]))[:-1]):
            np.testing.assert_allclose(point,window.state_at(at)[0],atol=1e-12)
