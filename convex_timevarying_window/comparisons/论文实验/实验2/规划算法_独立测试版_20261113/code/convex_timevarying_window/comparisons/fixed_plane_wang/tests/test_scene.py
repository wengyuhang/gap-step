import math
from pathlib import Path
import xml.etree.ElementTree as ET

import casadi as ca
import numpy as np
import pytest

from convex_timevarying_window.comparisons.fixed_plane_wang.scenario import (
    Case, build_case, original_track, CENTERS, NORMALS, MOTION_AXES,
    write_world, check_world, neutral_vary, audit_scene)
from convex_timevarying_window.comparisons.fixed_plane_wang.run import conditions, selected_rows, source_hashes
from convex_timevarying_window.receding_gate_racing.planner import symbolic_pose

ROOT = Path(__file__).resolve().parents[4]


def test_matrix_and_held_out_pilot():
    cases = conditions()
    assert len(cases)==len(set(cases))==12
    assert set(cases).isdisjoint(conditions(True))


@pytest.mark.parametrize('case',[Case(*c) for c in conditions()])
def test_fixed_planes_and_exact_motion(case):
    track,_ = build_case(case)
    for i,w in enumerate(track.windows):
        for t in np.linspace(0,2.43,23):
            c,R,_,_ = w.state_at(t)
            angle = 2*math.pi*case.frequency_hz*t+case.phase+.9*i
            displacement = np.zeros(3)
            displacement[MOTION_AXES[i]] = case.amplitude_m*np.sin(angle)
            np.testing.assert_allclose(c,CENTERS[i]+displacement,atol=1e-12)
            np.testing.assert_allclose(R[:,2],NORMALS[i],atol=1e-12)
            assert abs(NORMALS[i]@(c-CENTERS[i]))<1e-12
            _,v = w.motion.translation(t)
            assert v[MOTION_AXES[i]]==pytest.approx(case.amplitude_m*2*math.pi*case.frequency_hz*np.cos(angle))


def test_symbolic_and_numeric_pose_agree():
    track,_ = build_case(Case(1.,2.,.61))
    t = ca.SX.sym('t')
    for w in track.windows:
        fn = ca.Function('pose',[t],list(symbolic_pose(w,t)))
        for time in (0.,.137,.95,1.31):
            c,R = fn(time)
            expected = w.state_at(time)
            np.testing.assert_allclose(c.full().ravel(),expected[0],atol=1e-10)
            np.testing.assert_allclose(R.full(),expected[1],atol=1e-10)


def test_forward_layout_and_original_apertures():
    track,_ = build_case(Case(1.,2.,0.))
    old,_ = original_track()
    np.testing.assert_equal(track.start,old.start)
    np.testing.assert_equal(track.goal,old.goal)
    for i,w in enumerate(track.windows):
        # All possible independent phases at predecessor/successor extremes.
        for offset in (-2.,2.):
            prev = track.start.copy() if i==0 else CENTERS[i-1].copy()
            nxt = track.goal.copy() if i==6 else CENTERS[i+1].copy()
            if i>0: prev[MOTION_AXES[i-1]] += offset
            if i<6: nxt[MOTION_AXES[i+1]] += offset
            assert NORMALS[i]@(prev-CENTERS[i]) < -1.
            assert NORMALS[i]@(nxt-CENTERS[i]) > 1.
        assert w.name == old.windows[i].name
        assert w.aperture.kind == old.windows[i].aperture.kind
        assert w.aperture.radius == old.windows[i].aperture.radius
        np.testing.assert_equal(w.aperture.physical_vertices,old.windows[i].aperture.physical_vertices)


def test_world_serialization(tmp_path):
    track,_ = build_case(Case(1.,2.,4*math.pi/3))
    path = write_world(track,ROOT/'convex_dynamic_seven_window_gazebo',tmp_path)
    assert check_world(track,path)['max_position_error_m']<1e-10
    models = [m for m in ET.parse(path).findall('.//world/model') if m.get('name','').startswith('gate_')]
    for model,w in zip(models,track.windows):
        pose = np.fromstring(model.findtext('pose'),sep=' ')
        np.testing.assert_allclose(pose[:3],w.state_at(0)[0])
        np.testing.assert_allclose(pose[3:],w.angles0_rpy)


def test_audit_binding_restored_even_on_error():
    import convex_timevarying_window.receding_gate_racing.analyze as module
    before = module.build_track
    case = Case(.6,1.2,0.)
    with pytest.raises(RuntimeError):
        with audit_scene(case):
            assert module.build_track()[0].name.startswith('fixed_plane_')
            raise RuntimeError('exercise cleanup')
    assert module.build_track is before


def test_reject_double_scaling():
    track,_ = build_case(Case(1.,2.,0.))
    assert neutral_vary(track) is track
    with pytest.raises(ValueError):
        neutral_vary(track,motion_rate=10)


def test_retry_selection_is_method_aware_and_not_outcome_based():
    rows = [dict(method=m,condition_id=[1,2,0],attempt=1,infrastructure_failure=False) for m in ('ours','wang')]
    assert len(selected_rows(rows))==2
    retry = dict(rows[0],attempt=2)
    with pytest.raises(ValueError): selected_rows(rows+[retry])
    rows[0]['infrastructure_failure']=True
    assert selected_rows(rows+[retry])[0]['attempt']==2


def test_original_algorithm_hashes_unchanged():
    assert len(source_hashes())>63
