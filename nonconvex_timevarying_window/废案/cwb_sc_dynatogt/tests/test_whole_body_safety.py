from __future__ import annotations

import numpy as np

from nonconvex_timevarying_window.cwb_sc_dynatogt.body_model import CuboidBody
from nonconvex_timevarying_window.cwb_sc_dynatogt.config import WholeBodySafetyConfig
from nonconvex_timevarying_window.cwb_sc_dynatogt.whole_body_safety import (
    VerificationStatus,
    verify_whole_body_trajectory,
)
from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import ForwardPass

from .conftest import LinearTrajectory, StaticXPlaneWindow


def _forward() -> ForwardPass:
    trajectory = LinearTrajectory()
    return ForwardPass(
        k=np.zeros(1), d=np.zeros((1, 2)), durations=np.array([4.0]),
        traversal_times=np.array([2.0]), waypoints=np.array([[2.0, 0.0, 0.0]]),
        local_points=np.zeros((1, 2)), waypoint_jacobians=(np.zeros((3, 2)),),
        waypoint_time_derivatives=np.zeros((1, 3)), trajectory=trajectory,
    )


def _config() -> WholeBodySafetyConfig:
    return WholeBodySafetyConfig(
        half_extents=(0.3, 0.1, 0.1), sc_safe_radius=0.9,
        time_tolerance=0.01, lambda_tolerance=0.01,
        max_interval_depth=18, interval_scan_steps=64,
    )


def test_complete_section_is_numerically_verified(square_map) -> None:
    window = StaticXPlaneWindow(square_map)
    track = SCWindowTrack("safe", np.zeros(3), np.ones(3), (window,), (0,))
    report = verify_whole_body_trajectory(
        forward=_forward(), track=track,
        body=CuboidBody(np.array([0.3, 0.1, 0.1])), config=_config(),
    )
    assert report.status is VerificationStatus.NUMERICALLY_VERIFIED
    assert report.status is not VerificationStatus.CERTIFIED


def test_centroid_safe_but_body_section_outside_is_unsafe(square_map) -> None:
    window = StaticXPlaneWindow(square_map)
    track = SCWindowTrack("unsafe", np.zeros(3), np.ones(3), (window,), (0,))
    report = verify_whole_body_trajectory(
        forward=_forward(), track=track,
        body=CuboidBody(np.array([0.3, 1.2, 0.1])), config=_config(),
    )
    assert report.status is VerificationStatus.UNSAFE
    witness = report.windows[0].witnesses[0]
    assert witness.outside_sc_domain
