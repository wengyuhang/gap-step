import numpy as np

from mdg.disk_tracking import build_gate_tracks

from conftest import static_gate


def test_hungarian_tracks_are_continuous_and_densely_safe(fast_config):
    gate = static_gate("wave")
    tracks = build_gate_tracks(gate, fast_config, method="mdg_free", horizon=4.0)
    assert tracks
    for track in tracks:
        assert np.all(np.diff(track.times) > 0.0)
        grid = np.linspace(track.times[0], track.times[-1], 101)
        for time in grid:
            if not track.active_at(float(time)):
                continue
            center, radius, center_dot, radius_dot = track.evaluate(float(time))
            assert np.all(np.isfinite(center_dot))
            assert np.isfinite(radius_dot)
            assert radius >= fast_config.disks.min_radius
            safe = gate.safe_polygon(float(time), fast_config.safety.safety_radius)
            assert safe.covers(
                __import__("shapely").geometry.Point(*center).buffer(
                    radius, quad_segs=32
                )
            )
