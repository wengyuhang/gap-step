import numpy as np
import pytest
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.receding_gate_racing.scenario import vary_track


def test_motion_rate_is_time_scaling():
    original,_=build_track()
    fast,_=build_track()
    vary_track(fast,motion_rate=3.)
    for a,b in zip(original.windows,fast.windows):
        for t in (0.,.37,2.1):
            pa,ra,*_=a.state_at(3*t)
            pb,rb,*_=b.state_at(t)
            np.testing.assert_allclose(pa,pb,atol=1e-12)
            np.testing.assert_allclose(ra,rb,atol=1e-12)


@pytest.mark.parametrize('rate',[0.,-1.,float('nan'),float('inf')])
def test_invalid_rate(rate):
    track,_=build_track()
    with pytest.raises(ValueError):
        vary_track(track,motion_rate=rate)


def test_amplitudes_and_export_match(tmp_path):
    from pathlib import Path
    import xml.etree.ElementTree as ET
    from convex_timevarying_window.receding_gate_racing.scenario import write_variant_world
    original,_=build_track();track,_=build_track()
    vary_track(track,phase=.4,motion_rate=10,translation_scale=4,rotation_scale=2)
    for a,b in zip(original.windows,track.windows):
        np.testing.assert_allclose(b.motion.translation_amplitude,4*a.motion.translation_amplitude)
        np.testing.assert_allclose(b.motion.rotation_amplitude,2*a.motion.rotation_amplitude)
        assert b.motion.translation_period==a.motion.translation_period/10
        np.testing.assert_allclose(b.center0,a.center0)
    world=write_variant_world(track,Path('convex_dynamic_seven_window_gazebo'),tmp_path)
    plugins=ET.parse(world).findall(".//plugin[@name='convex_dynamic_course::PeriodicGateMotion']")
    assert len(plugins)==7
    for plugin,w in zip(plugins,track.windows):
        np.testing.assert_allclose(np.fromstring(plugin.findtext('translation_amplitude'),sep=' '),w.motion.translation_amplitude)
        np.testing.assert_allclose(np.fromstring(plugin.findtext('rotation_amplitude'),sep=' '),w.motion.rotation_amplitude)


@pytest.mark.parametrize('scale',[-1.,float('nan'),float('inf')])
def test_invalid_amplitude(scale):
    track,_=build_track()
    with pytest.raises(ValueError):vary_track(track,translation_scale=scale)
    with pytest.raises(ValueError):vary_track(track,rotation_scale=scale)
