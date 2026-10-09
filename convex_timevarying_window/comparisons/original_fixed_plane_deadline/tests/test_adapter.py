import numpy as np
from convex_timevarying_window.comparisons.original_fixed_plane_deadline import adapter
from convex_timevarying_window.comparisons.original_fixed_plane_checked.tests.test_scene import test_all_gate_queries_match_scene
from convex_timevarying_window.wang_free_state_mpc import planner, events
from convex_timevarying_window.togt_minco_mpc import planner as geometry
from convex_timevarying_window.wang_progress_mpc.planner import Planner
from 复现.polynomial_dynamic_racing_2023.px4_fixed_plane.scenario import Case,build_case


def test_install_preserves_deadline_method_and_inherited_geometry(monkeypatch):
    original_solve=Planner.solve;original_build=Planner._build
    with monkeypatch.context() as patch:
        patch.setattr(planner,'symbolic_pose',planner.symbolic_pose)
        patch.setattr(planner,'symbolic_clearance_sq',planner.symbolic_clearance_sq)
        patch.setattr(geometry,'frame_clearances',geometry.frame_clearances)
        patch.setattr(events,'local_coordinates',events.local_coordinates)
        adapter.install()
        assert Planner.solve is original_solve and Planner._build is original_build
        assert geometry.frame_clearances is adapter.scene.frame_clearances
        track,_=build_case(Case(1.,2.,0.));tt=np.linspace(0.,8.,61)
        for w in track.windows:
            pp=np.array([w.state_at(float(t))[0]+[.1,.2,.3] for t in tt])
            expected=np.array([np.r_[*w.world_to_local(p,float(t))] for p,t in zip(pp,tt)])
            np.testing.assert_allclose(events.local_coordinates(w,pp,tt),expected,atol=1e-10)
