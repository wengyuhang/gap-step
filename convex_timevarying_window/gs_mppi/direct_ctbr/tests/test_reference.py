import numpy as np
from convex_dynamic_seven_window_gazebo.x500_togt.experiment import build_track
from convex_timevarying_window.gs_mppi.direct_ctbr.reference import crossing_point_track
from convex_timevarying_window.dynamic_gate_mppi_v3.config import Config


def test_point_inset_does_not_change_physical_aperture_or_known_motion():
    track,_=build_track();planned=crossing_point_track(track);cfg=Config()
    inset=cfg.radius+cfg.error_margin+.02+.10
    for w,p in zip(track.windows,planned.windows):
        for t in [0.,.5,3.]:
            assert all(np.array_equal(a,b) for a,b in zip(w.state_at(t),p.state_at(t)))
        assert np.array_equal(w.aperture.boundary_points(),p.aperture.boundary_points())
        a=p.aperture
        if a.kind=='circle':assert np.isclose(a.margin/2,inset)
        else:
            assert np.all(w.aperture.boundary_distance(a.safe_vertices)>=inset-1e-9)
            assert all(w.aperture.contains(q) for q in a.safe_vertices)
        # Still a free crossing point, not a centre-only planner.
        d=a.initial_d();q0,_=a.point_and_jacobian(d)
        d[0]+=.2;q1,_=a.point_and_jacobian(d)
        assert np.linalg.norm(q1-q0)>1e-4
